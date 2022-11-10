#include "stdafx.h"
#include "ImFile.h"

#ifdef _SECURE_SCL
#undef _SECURE_SCL
#define _SECURE_SCL 0
#else
#define _SECURE_SCL 0
#endif /*_SECURE_SCL*/


#include <vector>

using namespace std;

//根据CBA结果的eoi矩阵，修改PairTiePoint点对匹配结果的坐标
void TransformPairTiePoints(TiePoint*** _PairTiePoints, int** _nTiePointNum, int _nTransIdx, registration::TransformationEstimationSVD<PointXYZ, PointXYZ, float>::Matrix4 _TransMatrix, int _nCloudNum)
{
	//更新PairTiePoints
	TagPoint* PointTemp = new TagPoint;
	for (int i = 0; i < _nTransIdx; ++i)
	{
		for (int k = 0; k < _nTiePointNum[i][_nTransIdx]; ++k)
		{
			PointTemp->x = _PairTiePoints[i][_nTransIdx][k].SourceX;
			PointTemp->y = _PairTiePoints[i][_nTransIdx][k].SourceY;
			PointTemp->z = _PairTiePoints[i][_nTransIdx][k].SourceZ;
			PointTemp = TransformPoint(PointTemp, _TransMatrix);
			_PairTiePoints[i][_nTransIdx][k].SourceX = PointTemp->x;
			_PairTiePoints[i][_nTransIdx][k].SourceY = PointTemp->y;
			_PairTiePoints[i][_nTransIdx][k].SourceZ = PointTemp->z;
		}
	}
	for (int i = _nTransIdx + 1; i < _nCloudNum; ++i)
	{
		for (int k = 0; k < _nTiePointNum[_nTransIdx][i]; ++k)
		{
			PointTemp->x = _PairTiePoints[_nTransIdx][i][k].TargetX;
			PointTemp->y = _PairTiePoints[_nTransIdx][i][k].TargetY;
			PointTemp->z = _PairTiePoints[_nTransIdx][i][k].TargetZ;
			PointTemp = TransformPoint(PointTemp, _TransMatrix);
			_PairTiePoints[_nTransIdx][i][k].TargetX = PointTemp->x;
			_PairTiePoints[_nTransIdx][i][k].TargetY = PointTemp->y;
			_PairTiePoints[_nTransIdx][i][k].TargetZ = PointTemp->z;
		}
	}
	delete[] PointTemp;
	PointTemp = NULL;
}

//写入初始两组点云的im文件
void WriteImFileFirstTwo(TiePoint*** _PairTiePoint, TagMatchPoint* _AllTiePoints, int _nTgtIdx, int _nSrcIdx, char* _strImFile, int* _nCloudID, int** _nTiePointNum, int _nCloudNum)
{
	int nTiePointNum = 0;//tie文件内点链表个数
	for (int i = 0; i < _nCloudNum; ++i)
		for (int j = 0; j < _nCloudNum; ++j)
			nTiePointNum += _nTiePointNum[i][j];
	nTiePointNum /= 2;
	FILE* fpImFile;
	fopen_s(&fpImFile, _strImFile, "w");
	if (fpImFile == NULL)
	{
		cout << "failed to open im file.";
		return;
	}
	int nPairTiePointNum = _nTiePointNum[_nTgtIdx][_nSrcIdx];
	Eigen::Matrix4f fTransRT;//刚体转换矩阵R T
	Eigen::Matrix<float, 3, Eigen::Dynamic> CloudTgt(3, nPairTiePointNum);
	Eigen::Matrix<float, 3, Eigen::Dynamic> CloudSrc(3, nPairTiePointNum);
	for (int i = 0; i < nPairTiePointNum; ++i)
	{
		CloudTgt(0, i) = _PairTiePoint[_nTgtIdx][_nSrcIdx][i].TargetX;
		CloudTgt(1, i) = _PairTiePoint[_nTgtIdx][_nSrcIdx][i].TargetY;
		CloudTgt(2, i) = _PairTiePoint[_nTgtIdx][_nSrcIdx][i].TargetZ;
		CloudSrc(0, i) = _PairTiePoint[_nTgtIdx][_nSrcIdx][i].SourceX;
		CloudSrc(1, i) = _PairTiePoint[_nTgtIdx][_nSrcIdx][i].SourceY;
		CloudSrc(2, i) = _PairTiePoint[_nTgtIdx][_nSrcIdx][i].SourceZ;
	}
	fTransRT = pcl::umeyama(CloudSrc, CloudTgt, true);
	//写入.im文件
	fprintf(fpImFile, "%14d%20.7lf%20.7lf%20.7lf\n", 2, 0.0, 0.0, 0.0);
	fprintf(fpImFile, "%14d\n", 1);
	fprintf(fpImFile, "%15d%15d\n", _nCloudID[_nTgtIdx], 0);
	fprintf(fpImFile, "%25.7lf%25.7lf%25.7lf\n", 0.0, 0.0, 0.0);
	fprintf(fpImFile, "%25.12lf%25.12lf%25.12lf\n", 1.0, 0.0, 0.0);
	fprintf(fpImFile, "%25.12lf%25.12lf%25.12lf\n", 0.0, 1.0, 0.0);
	fprintf(fpImFile, "%25.12lf%25.12lf%25.12lf\n\n", 0.0, 0.0, 1.0);
	fprintf(fpImFile, "%15d%15d\n", _nCloudID[_nSrcIdx], 0);
	fprintf(fpImFile, "%25.7lf%25.7lf%25.7lf\n", fTransRT(0, 3), fTransRT(1, 3), fTransRT(2, 3));
	fprintf(fpImFile, "%25.12lf%25.12lf%25.12lf\n", fTransRT(0, 0), fTransRT(0, 1), fTransRT(0, 2));
	fprintf(fpImFile, "%25.12lf%25.12lf%25.12lf\n", fTransRT(1, 0), fTransRT(1, 1), fTransRT(1, 2));
	fprintf(fpImFile, "%25.12lf%25.12lf%25.12lf\n\n", fTransRT(2, 0), fTransRT(2, 1), fTransRT(2, 2));
	int nImPoint = 0; //im文件内点的数量
	int nImPointOb = 0;//im文件内点观测值的数量
	vector<TagPoint*> vTagPointForMatch;
	TagPoint* TagPointTemp;
	for (int i = 0; i < nTiePointNum; ++i)
	{
		vTagPointForMatch.clear();
		TagPointTemp = _AllTiePoints[i].PointList;
		for (int j = 0; j < _AllTiePoints[i].nMark; ++j)
		{
			if (TagPointTemp->nCloudID == _nCloudID[_nTgtIdx] || TagPointTemp->nCloudID == _nCloudID[_nSrcIdx])
				vTagPointForMatch.push_back(TagPointTemp);
			TagPointTemp = TagPointTemp->pNext;
		}
		if (vTagPointForMatch.size() == 2)
			nImPoint++;
	}
	nImPointOb = nImPoint * 2;
	fprintf(fpImFile, "%14d%25d\n", nImPoint, nImPointOb);
	for (int i = 0; i < nTiePointNum; i++)
	{
		vTagPointForMatch.clear();
		TagPointTemp = _AllTiePoints[i].PointList;
		for (int j = 0; j < _AllTiePoints[i].nMark; ++j)
		{
			if (TagPointTemp->nCloudID == _nCloudID[_nTgtIdx] || TagPointTemp->nCloudID == _nCloudID[_nSrcIdx])
				vTagPointForMatch.push_back(TagPointTemp);
			TagPointTemp = TagPointTemp->pNext;
		}
		if (vTagPointForMatch.size() == 2)
		{
			TagPoint* TagPointTgt;
			if (vTagPointForMatch.at(0)->nCloudID == _nCloudID[_nTgtIdx])
				TagPointTgt = vTagPointForMatch.at(0);
			else
				TagPointTgt = vTagPointForMatch.at(1);
			fprintf(fpImFile, "%15d%25.4lf%25.4lf%25.4lf%15d\n", _AllTiePoints[i].nPointID, TagPointTgt->x, TagPointTgt->y, TagPointTgt->z, 2);
			fprintf(fpImFile, "%15d%25.4lf%25.4lf%25.4lf\n", vTagPointForMatch[0]->nCloudID, vTagPointForMatch[0]->x, vTagPointForMatch[0]->y, vTagPointForMatch[0]->z);
			fprintf(fpImFile, "%15d%25.4lf%25.4lf%25.4lf\n", vTagPointForMatch[1]->nCloudID, vTagPointForMatch[1]->x, vTagPointForMatch[1]->y, vTagPointForMatch[1]->z);
		}
	}
	fprintf(fpImFile, "\n%15d", 0);
	fclose(fpImFile);
}

//写入初始两组点云之后的点云im文件
void WriteImFile(TiePoint*** _PairTiePoint, TagMatchPoint* _AllTiePoints, registration::TransformationEstimationSVD<PointXYZ, PointXYZ, float>::Matrix4* _TransMatrix, int* _nTgtIdx, int _nSrcIdx, char* _strImFile, int* _nCloudID, int** _nTiePointNum, int _nCloudNum, int _nMatchedCloud)
{
	TiePoint*** PairTiePointTransed = new TiePoint**[_nCloudNum];
	for (int i = 0; i < _nCloudNum; ++i)
		PairTiePointTransed[i] = new TiePoint*[_nCloudNum];
	for (int i = 0; i < _nCloudNum - 1; ++i)
	{
		for (int j = i + 1; j < _nCloudNum; ++j)
		{
			PairTiePointTransed[i][j] = new TiePoint[_nTiePointNum[i][j]];
			for (int k = 0; k < _nTiePointNum[i][j]; ++k)
			{
				PairTiePointTransed[i][j][k].TargetX = _PairTiePoint[i][j][k].TargetX;
				PairTiePointTransed[i][j][k].TargetY = _PairTiePoint[i][j][k].TargetY;
				PairTiePointTransed[i][j][k].TargetZ = _PairTiePoint[i][j][k].TargetZ;
				PairTiePointTransed[i][j][k].SourceX = _PairTiePoint[i][j][k].SourceX;
				PairTiePointTransed[i][j][k].SourceY = _PairTiePoint[i][j][k].SourceY;
				PairTiePointTransed[i][j][k].SourceZ = _PairTiePoint[i][j][k].SourceZ;
			}
		}
	}
	for (int i = 0; i < _nCloudNum; ++i)
		TransformPairTiePoints(PairTiePointTransed, _nTiePointNum, i, _TransMatrix[i], _nCloudNum);
	FILE* fpImFile;
	fopen_s(&fpImFile, _strImFile, "w");
	if (fpImFile == NULL)
	{
		cout << "failed to open im file.";
		return;
	}
	int nPointNumSum = 0;
	for (int i = 0; i < _nMatchedCloud; ++i) nPointNumSum += _nTiePointNum[_nTgtIdx[i]][_nSrcIdx];
	Eigen::Matrix4f fTransRT;//刚体转换矩阵R T
	Eigen::Matrix<float, 3, Eigen::Dynamic> CloudTgtTransed(3, nPointNumSum);
	Eigen::Matrix<float, 3, Eigen::Dynamic> CloudSrcTransed(3, nPointNumSum);
	int nPrevPointNum = 0;
	for (int i = 0; i < _nMatchedCloud; ++i)
	{
		for (int j = 0; j < _nTiePointNum[_nTgtIdx[i]][_nSrcIdx]; ++j)
		{
			if (_nTgtIdx[i] < _nSrcIdx)
			{
				CloudTgtTransed(0, nPrevPointNum + j) = PairTiePointTransed[_nTgtIdx[i]][_nSrcIdx][j].TargetX;
				CloudTgtTransed(1, nPrevPointNum + j) = PairTiePointTransed[_nTgtIdx[i]][_nSrcIdx][j].TargetY;
				CloudTgtTransed(2, nPrevPointNum + j) = PairTiePointTransed[_nTgtIdx[i]][_nSrcIdx][j].TargetZ;
				CloudSrcTransed(0, nPrevPointNum + j) = PairTiePointTransed[_nTgtIdx[i]][_nSrcIdx][j].SourceX;
				CloudSrcTransed(1, nPrevPointNum + j) = PairTiePointTransed[_nTgtIdx[i]][_nSrcIdx][j].SourceY;
				CloudSrcTransed(2, nPrevPointNum + j) = PairTiePointTransed[_nTgtIdx[i]][_nSrcIdx][j].SourceZ;
			}
			else
			{//转换source和target
				CloudTgtTransed(0, nPrevPointNum + j) = PairTiePointTransed[_nSrcIdx][_nTgtIdx[i]][j].SourceX;
				CloudTgtTransed(1, nPrevPointNum + j) = PairTiePointTransed[_nSrcIdx][_nTgtIdx[i]][j].SourceY;
				CloudTgtTransed(2, nPrevPointNum + j) = PairTiePointTransed[_nSrcIdx][_nTgtIdx[i]][j].SourceZ;
				CloudSrcTransed(0, nPrevPointNum + j) = PairTiePointTransed[_nSrcIdx][_nTgtIdx[i]][j].TargetX;
				CloudSrcTransed(1, nPrevPointNum + j) = PairTiePointTransed[_nSrcIdx][_nTgtIdx[i]][j].TargetY;
				CloudSrcTransed(2, nPrevPointNum + j) = PairTiePointTransed[_nSrcIdx][_nTgtIdx[i]][j].TargetZ;
			}
		}
		nPrevPointNum += _nTiePointNum[_nTgtIdx[i]][_nSrcIdx];
	}
	fTransRT = pcl::umeyama(CloudSrcTransed, CloudTgtTransed, false);
	int nTiePointNum = 0;
	for (int i = 0; i < _nCloudNum; ++i)
		for (int j = 0; j < _nCloudNum; ++j)
			nTiePointNum += _nTiePointNum[i][j];
	nTiePointNum /= 2;
	//写入.im文件
	fprintf(fpImFile, "%14d%20.7lf%20.7lf%20.7lf\n", _nMatchedCloud + 1, 0.0, 0.0, 0.0);
	fprintf(fpImFile, "%14d\n", 1);
	for (int i = 0; i < _nMatchedCloud; ++i)
	{
		fprintf(fpImFile, "%15d%15d\n", _nCloudID[_nTgtIdx[i]], 0);
		fprintf(fpImFile, "%25.7lf%25.7lf%25.7lf\n", _TransMatrix[_nTgtIdx[i]](0, 3), _TransMatrix[_nTgtIdx[i]](1, 3), _TransMatrix[_nTgtIdx[i]](2, 3));
		fprintf(fpImFile, "%25.12lf%25.12lf%25.12lf\n", _TransMatrix[_nTgtIdx[i]](0, 0), _TransMatrix[_nTgtIdx[i]](0, 1), _TransMatrix[_nTgtIdx[i]](0, 2));
		fprintf(fpImFile, "%25.12lf%25.12lf%25.12lf\n", _TransMatrix[_nTgtIdx[i]](1, 0), _TransMatrix[_nTgtIdx[i]](1, 1), _TransMatrix[_nTgtIdx[i]](1, 2));
		fprintf(fpImFile, "%25.12lf%25.12lf%25.12lf\n\n", _TransMatrix[_nTgtIdx[i]](2, 0), _TransMatrix[_nTgtIdx[i]](2, 1), _TransMatrix[_nTgtIdx[i]](2, 2));
	}
	fprintf(fpImFile, "%15d%15d\n", _nCloudID[_nSrcIdx], 0);
	fprintf(fpImFile, "%25.7lf%25.7lf%25.7lf\n", fTransRT(0, 3), fTransRT(1, 3), fTransRT(2, 3));
	fprintf(fpImFile, "%25.12lf%25.12lf%25.12lf\n", fTransRT(0, 0), fTransRT(0, 1), fTransRT(0, 2));
	fprintf(fpImFile, "%25.12lf%25.12lf%25.12lf\n", fTransRT(1, 0), fTransRT(1, 1), fTransRT(1, 2));
	fprintf(fpImFile, "%25.12lf%25.12lf%25.12lf\n\n", fTransRT(2, 0), fTransRT(2, 1), fTransRT(2, 2));
	int nImPoint = 0; //im文件内点的数量
	int nImPointOb = 0;//im文件内点观测值的数量
	vector<TagPoint*> vTagPointForMatch;
	TagPoint* TagPointTemp;
	for (int i = 0; i < nTiePointNum; ++i)
	{
		vTagPointForMatch.clear();
		TagPointTemp = _AllTiePoints[i].PointList;
		for (int j = 0; j < _AllTiePoints[i].nMark; ++j)
		{
			for (int k = 0; k < _nMatchedCloud; ++k)
			{
				if (TagPointTemp->nCloudID == _nCloudID[_nTgtIdx[k]])
					vTagPointForMatch.push_back(TagPointTemp);
			}
			if (TagPointTemp->nCloudID == _nCloudID[_nSrcIdx])
				vTagPointForMatch.push_back(TagPointTemp);
			TagPointTemp = TagPointTemp->pNext;
		}
		float fTgtX(0.0), fTgtY(0.0), fTgtZ(0.0);
		int nMatchedPoint = 0;
		if (vTagPointForMatch.size() > 1)//写入im文件的tie点
		{
			nImPoint++;
			nImPointOb += vTagPointForMatch.size();
		}
	}
	fprintf(fpImFile, "%14d%25d\n", nImPoint, nImPointOb);
	for (int i = 0; i < nTiePointNum; ++i)
	{
		vTagPointForMatch.clear();
		TagPointTemp = _AllTiePoints[i].PointList;
		for (int j = 0; j < _AllTiePoints[i].nMark; ++j)
		{
			for (int k = 0; k < _nMatchedCloud; ++k)
			{
				if (TagPointTemp->nCloudID == _nCloudID[_nTgtIdx[k]])
					vTagPointForMatch.push_back(TagPointTemp);
			}
			if (TagPointTemp->nCloudID == _nCloudID[_nSrcIdx])
				vTagPointForMatch.push_back(TagPointTemp);
			TagPointTemp = TagPointTemp->pNext;
		}
		float fTgtX(0.0), fTgtY(0.0), fTgtZ(0.0);
		int nMatchedPoint = 0;
		if (vTagPointForMatch.size() > 1)//写入im文件的tie点
		{
			for (int j = 0; j < vTagPointForMatch.size(); ++j)//已平差点坐标求平均作为参考值
			{
				for (int k = 0; k < _nMatchedCloud; ++k)
				{
					if (vTagPointForMatch[j]->nCloudID == _nCloudID[_nTgtIdx[k]])
					{
						TagPointTemp = TransformPoint(vTagPointForMatch[j], _TransMatrix[_nTgtIdx[k]]);
						fTgtX += TagPointTemp->x;
						fTgtY += TagPointTemp->y;
						fTgtZ += TagPointTemp->z;
						nMatchedPoint++;
					}
				}
			}
			fTgtX /= nMatchedPoint;
			fTgtY /= nMatchedPoint;
			fTgtZ /= nMatchedPoint;
			fprintf(fpImFile, "%15d%25.4lf%25.4lf%25.4lf%15d\n", _AllTiePoints[i].nPointID, fTgtX, fTgtY, fTgtZ, vTagPointForMatch.size());
			for (int j = 0; j < vTagPointForMatch.size(); ++j)//写入点的观测值
				fprintf(fpImFile, "%15d%25.4lf%25.4lf%25.4lf\n", vTagPointForMatch[j]->nCloudID, vTagPointForMatch[j]->x, vTagPointForMatch[j]->y, vTagPointForMatch[j]->z);
		}
	}
	fprintf(fpImFile, "\n%15d", 0);
	fclose(fpImFile);
	for (int i = 0; i < _nCloudNum; ++i)
	{
		delete[] PairTiePointTransed[i];
		PairTiePointTransed[i] = NULL;
	}
}

void WriteAllCloudImFile(TagMatchPoint* _AllOriginMatchPoint, TagMatchPoint* _AllMatchPoint, int _nPointsNum, char* _strFinalTieName, int _nCloudNum, int* _nCloudID)
{
	FILE* fpTie;
	fopen_s(&fpTie, _strFinalTieName, "w");
	if (fpTie == NULL) return;
	TagPoint* PointTemp;
	fprintf(fpTie, "%14d%20.7lf%20.7lf%20.7lf\n", _nCloudNum, 0.0, 0.0, 0.0);
	fprintf(fpTie, "%14d\n", 1);
	for (int i = 0; i < _nCloudNum; ++i)
	{
		fprintf(fpTie, "%15d%15d\n", _nCloudID[i], 0);
		fprintf(fpTie, "%25.7lf%25.7lf%25.7lf\n", 0.0, 0.0, 0.0);
		fprintf(fpTie, "%25.12lf%25.12lf%25.12lf\n", 1.0, 0.0, 0.0);
		fprintf(fpTie, "%25.12lf%25.12lf%25.12lf\n", 0.0, 1.0, 0.0);
		fprintf(fpTie, "%25.12lf%25.12lf%25.12lf\n\n", 0.0, 0.0, 1.0);
	}
	fprintf(fpTie, "%14d%25d\n", _nPointsNum, _nPointsNum * 2);
	for (int i = 0; i < _nPointsNum; ++i)
	{
		double fAvgX = 0.0, fAvgY = 0.0, fAvgZ = 0.0;
		int nP = 1;
		fAvgX += _AllMatchPoint[i].PointList->x;
		fAvgY += _AllMatchPoint[i].PointList->y;
		fAvgZ += _AllMatchPoint[i].PointList->z;
		PointTemp = _AllMatchPoint[i].PointList->pNext;
		while (PointTemp != NULL)
		{
			fAvgX += PointTemp->x;
			fAvgY += PointTemp->y;
			fAvgZ += PointTemp->z;
			nP++;
			PointTemp = PointTemp->pNext;
		}
		fAvgX /= nP;
		fAvgY /= nP;
		fAvgZ /= nP;

		fprintf(fpTie, "%15d%25.4lf%25.4lf%25.4lf%15d\n", i + 1, fAvgX, fAvgY, fAvgZ, nP);
		fprintf(fpTie, "%15d%25.4lf%25.4lf%25.4lf\n", _AllOriginMatchPoint[i].PointList->nCloudID, _AllOriginMatchPoint[i].PointList->x, _AllOriginMatchPoint[i].PointList->y, _AllOriginMatchPoint[i].PointList->z);
		PointTemp = _AllOriginMatchPoint[i].PointList->pNext;
		while (PointTemp != NULL)
		{
			fprintf(fpTie, "%15d%25.4lf%25.4lf%25.4lf\n", PointTemp->nCloudID, PointTemp->x, PointTemp->y, PointTemp->z);
			PointTemp = PointTemp->pNext;
		}
	}
	fclose(fpTie);
}

//从eoi文件更新点云转换矩阵数组
void UpdateCloudTransMatrixByEoiFile(registration::TransformationEstimationSVD<PointXYZ, PointXYZ, float>::Matrix4* _CloudTransMatrix, char* _strEoiFile, int _nCloudNum, int _nMatrixNum, int* _nCloudID)
{
	//读取cba.eoi文件获取变换矩阵
	FILE* fpEoi;
	fopen_s(&fpEoi, _strEoiFile, "r");
	if (fpEoi == NULL)
	{
		cout << "failed to open eoi file.";
		return;
	}
	registration::TransformationEstimationSVD<PointXYZ, PointXYZ, float>::Matrix4 TransMatrixTemp;
	int nEoiCloudID;
	for (int i = 0; i < _nMatrixNum; ++i)
	{
		fscanf(fpEoi, "%d%f%f%f%f\n", &nEoiCloudID, &TransMatrixTemp(3, 3), &TransMatrixTemp(0, 3), &TransMatrixTemp(1, 3), &TransMatrixTemp(2, 3));
		fscanf(fpEoi, "%f%f%f%f%f\n", &TransMatrixTemp(0, 0), &TransMatrixTemp(0, 1), &TransMatrixTemp(0, 2), &TransMatrixTemp(1, 0), &TransMatrixTemp(1, 1));
		fscanf(fpEoi, "%f%f%f%f\n", &TransMatrixTemp(1, 2), &TransMatrixTemp(2, 0), &TransMatrixTemp(2, 1), &TransMatrixTemp(2, 2));
		for (int i = 0; i < 3; ++i)
			TransMatrixTemp(3, i) = 0;
		TransMatrixTemp(3, 3) = 1;
		for (int j = 0; j < _nCloudNum; ++j)
			if (nEoiCloudID == _nCloudID[j])
				_CloudTransMatrix[j] = TransMatrixTemp;	
	}
	fclose(fpEoi);
}