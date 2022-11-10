
#include "stdafx.h"
#include "TiePoints.h"
using namespace pcl;
using namespace std;

//�ݹ�Ѱ�ҵ���ͬ����
void GetTiePoint(int **_nTiePointNum, struct TiePoint*** _pairTiePoints, struct TagMatchPoint& _matchedPoints, int* _nCloudID, int _nCloudIdx, int _nCloudNum, float _ix, float _iy, float _iz)
{
	int nCloudID = _nCloudID[_nCloudIdx];//����id
	bool bFound = false;
	struct TagPoint* ptCloud = _matchedPoints.PointList;
	while (ptCloud != NULL)
	{
		if (nCloudID == ptCloud->nCloudID)
		{
			bFound = true;
			break;
		}
		ptCloud = ptCloud->pNext;
	}
	if (bFound == false)
	{
		struct TagPoint* ptiCloud = new struct TagPoint;
		ptiCloud->pNext = _matchedPoints.PointList;
		ptiCloud->nCloudID = nCloudID;
		ptiCloud->x = _ix;
		ptiCloud->y = _iy;
		ptiCloud->z = _iz;
		_matchedPoints.PointList = ptiCloud;
	}
	int i = _nCloudIdx;
	for (int j = i + 1; j < _nCloudNum; j++)
	{
		int nTiePointNum = _nTiePointNum[i][j];
		if (nTiePointNum == 0)
			continue;
		struct TiePoint* pTiePointsij = _pairTiePoints[i][j];
		for (int m = 0; m < nTiePointNum; m++)
		{
			if (fabsf(pTiePointsij[m].TargetX - _ix) < 1e-3&&fabsf(pTiePointsij[m].TargetY - _iy) < 1e-3 &&fabsf(pTiePointsij[m].TargetZ - _iz) < 1e-3)
				GetTiePoint(_nTiePointNum, _pairTiePoints, _matchedPoints, _nCloudID, j, _nCloudNum, pTiePointsij[m].SourceX, pTiePointsij[m].SourceY, pTiePointsij[m].SourceZ);
		}
	}
}

//�������еĵ��ƶ�tie�ļ���������tie�ļ�
TagMatchPoint* GetWholeTie(int _nCloudNum, char* _strMiddlePath, int* _nCloudID, int** _nTiePointNum, TiePoint*** _PairTiePoints)
{
	TagMatchPoint MatchedPoints;//Target�������ƥ���
	int nAllPointList = 0;
	for (int i = 0; i < _nCloudNum; ++i)
		for (int j = 0; j < _nCloudNum; ++j)
			nAllPointList += _nTiePointNum[i][j];
	TagMatchPoint* AllPoints = new TagMatchPoint[nAllPointList];//��������ļ�������ƥ���
	TagPoint* PointTemp;
	int iter = 0;
	for (int i = 0; i < _nCloudNum - 1; ++i)
	{
		for (int j = i + 1; j < _nCloudNum; ++j)
		{
			for (int k = 0; k < _nTiePointNum[i][j]; ++k)
			{
				MatchedPoints.PointList = NULL;
				GetTiePoint(_nTiePointNum, _PairTiePoints, MatchedPoints, _nCloudID, i, _nCloudNum, _PairTiePoints[i][j][k].TargetX, _PairTiePoints[i][j][k].TargetY, _PairTiePoints[i][j][k].TargetZ);
				PointTemp = MatchedPoints.PointList;
				//if (PointTemp->pNext->pNext != NULL)//�޳����ȵ�
				{
					AllPoints[iter].PointList = new TagPoint;
					AllPoints[iter].nPointID = iter;
					AllPoints[iter].nMark = 1;
					AllPoints[iter].PointList = PointTemp;
					while (PointTemp->pNext != NULL)
					{
						AllPoints[iter].nMark++;
						PointTemp = PointTemp->pNext;
					}
					iter++;
				}
			}
		}
	}
	return AllPoints;
}

//��һ�����ת���������ת��
TagPoint* TransformPoint(TagPoint* _Source, registration::TransformationEstimationSVD<PointXYZ, PointXYZ, float>::Matrix4 _TransMatrix)
{
	TagPoint* Ans = new TagPoint;
	PointCloud<PointXYZ>::Ptr  Cloud1(new PointCloud<PointXYZ>);
	PointCloud<PointXYZ>::Ptr  Cloud2(new PointCloud<PointXYZ>);
	PointXYZ Point1(_Source->x, _Source->y, _Source->z);
	Cloud1->push_back(Point1);
	transformPointCloud(*Cloud1, *Cloud2, _TransMatrix, true);
	Ans->x = Cloud2->at(0).x;
	Ans->y = Cloud2->at(0).y;
	Ans->z = Cloud2->at(0).z;
	return Ans;
}

//����CBA�����eoi�����޸�����tie�ļ��ĵ������
void TransformFinalTie(TagMatchPoint* _AllPoints, int _nPointsNum, char* _strEoiName, int* _nMatchedCloudIdx, int _nMatchedCloudNum, int _nMatchingCloudIdx)
{
	//��ȡcba.eoi�ļ���ȡ����任����
	FILE* fpEoi;
	fopen_s(&fpEoi, _strEoiName, "r");
	if (fpEoi == NULL)
	{
		cout << "failed to open eoi file.";
		return;
	}
	int nCloudID1, nCloudID2;
	registration::TransformationEstimationSVD<PointXYZ, PointXYZ, float>::Matrix4 TransMatrix1;
	registration::TransformationEstimationSVD<PointXYZ, PointXYZ, float>::Matrix4 TransMatrix2;
	fscanf(fpEoi, "%d%f%f%f%f\n", &nCloudID1, &TransMatrix1(3, 3), &TransMatrix1(0, 3), &TransMatrix1(1, 3), &TransMatrix1(2, 3));
	fscanf(fpEoi, "%f%f%f%f%f\n", &TransMatrix1(0, 0), &TransMatrix1(0, 1), &TransMatrix1(0, 2), &TransMatrix1(1, 0), &TransMatrix1(1, 1));
	fscanf(fpEoi, "%f%f%f%f\n", &TransMatrix1(1, 2), &TransMatrix1(2, 0), &TransMatrix1(2, 1), &TransMatrix1(2, 2));
	fscanf(fpEoi, "%d%f%f%f%f\n", &nCloudID2, &TransMatrix2(3, 3), &TransMatrix2(0, 3), &TransMatrix2(1, 3), &TransMatrix2(2, 3));
	fscanf(fpEoi, "%f%f%f%f%f\n", &TransMatrix2(0, 0), &TransMatrix2(0, 1), &TransMatrix2(0, 2), &TransMatrix2(1, 0), &TransMatrix2(1, 1));
	fscanf(fpEoi, "%f%f%f%f\n", &TransMatrix2(1, 2), &TransMatrix2(2, 0), &TransMatrix2(2, 1), &TransMatrix2(2, 2));
	for (int i = 0; i < 3; ++i)
	{
		TransMatrix1(3, i) = 0;
		TransMatrix2(3, i) = 0;
	}
	//���¶�Ӧ���б�
	TagPoint* PointTemp;
	for (int i = 0; i < _nPointsNum; ++i)
	{
		if (_AllPoints[i].PointList->nCloudID == nCloudID1)
			_AllPoints[i].PointList = TransformPoint(_AllPoints[i].PointList, TransMatrix1);
		else if (_AllPoints[i].PointList->nCloudID == nCloudID2)
			_AllPoints[i].PointList = TransformPoint(_AllPoints[i].PointList, TransMatrix2);
		PointTemp = _AllPoints[i].PointList->pNext;
		while (PointTemp != NULL)
		{
			if (PointTemp->nCloudID == nCloudID1)
				PointTemp = TransformPoint(PointTemp, TransMatrix1);
			else if (PointTemp->nCloudID == nCloudID2)
				PointTemp = TransformPoint(PointTemp, TransMatrix2);
			PointTemp = PointTemp->pNext;
		}
	}
	fclose(fpEoi);
}

//����Ӧ���б�д�뱾��
void WriteAllTiePoints(TagMatchPoint* _AllPoints, int _nPointsNum, char* _strFinalTieName)
{
	FILE* fpTie;
	fopen_s(&fpTie, _strFinalTieName, "w");
	if (fpTie == NULL) return;
	TagPoint* PointTemp;
	for (int i = 0; i < _nPointsNum; ++i)
	{
		fprintf(fpTie, "%d %d ", _AllPoints[i].nPointID, _AllPoints[i].nMark);
		fprintf(fpTie, "%d %.4f %.4f %.4f   ", _AllPoints[i].PointList->nCloudID, _AllPoints[i].PointList->x, _AllPoints[i].PointList->y, _AllPoints[i].PointList->z);
		PointTemp = _AllPoints[i].PointList->pNext;
		while (PointTemp != NULL)
		{
			fprintf(fpTie, "%d %.4f %.4f %.4f   ", PointTemp->nCloudID, PointTemp->x, PointTemp->y, PointTemp->z);
			PointTemp = PointTemp->pNext;
		}
		fprintf(fpTie, "\n");
	}
	fclose(fpTie);
}

void LoadTieFiles(int _nCloudNum, char* _strMiddlePath, char*** _strTieName, int** _nNeiborMatrix, int** _nTiePointNum, TiePoint*** _PairTiePoints)
{
	int nTargetID, nSourceID;
	for (int i = 0; i < _nCloudNum - 1; ++i)
	{
		for (int j = i + 1; j < _nCloudNum; ++j)
		{
			if (_nNeiborMatrix[i][j] == 0)
			{
				_nTiePointNum[i][j] = 0;
				_PairTiePoints[i][j] = NULL;
			}
			else
			{
				FILE* fpTie;
				char* strMiddlePathTemp = new char[50];
				strcpy(strMiddlePathTemp, _strMiddlePath);
				fopen_s(&fpTie, strcat(strMiddlePathTemp, _strTieName[i][j]), "r");
				if (fpTie == NULL)
				{
					cout << "failed to open final tie file." << endl;
					return;
				}
				fscanf(fpTie, "%d %d %d\n", &_nTiePointNum[i][j], &nTargetID, &nSourceID);
				_PairTiePoints[i][j] = new TiePoint[_nTiePointNum[i][j]];//��������Ϣ		
				for (int k = 0; k < _nTiePointNum[i][j]; ++k)
					fscanf(fpTie, "%f %f %f   %f %f %f\n", &_PairTiePoints[i][j][k].TargetX, &_PairTiePoints[i][j][k].TargetY, &_PairTiePoints[i][j][k].TargetZ,
						&_PairTiePoints[i][j][k].SourceX, &_PairTiePoints[i][j][k].SourceY, &_PairTiePoints[i][j][k].SourceZ);
				delete[] strMiddlePathTemp;
				strMiddlePathTemp = NULL;
				fclose(fpTie);
			}
		}
	}
}