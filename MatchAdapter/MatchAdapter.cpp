// MatchAdapter.cpp : �������̨Ӧ�ó������ڵ㡣
//
#include "stdafx.h"
#include "TiePoints.h"
#include "ImFile.h"
#include "VisualizeCloud.h"
#include <atlstr.h>
#define MAX_LINE 100 //�ļ���ȡһ����󳤶�

using namespace pcl;
using namespace std;

//����cmd����pairmatchadapter.exe
void PairMatch(int _nCloudNum, int** _nNeiborMatrix, char* _strMiddlePath, char** _strCloudName, char*** _strTieName)
{
	char** strPoiName = new char*[_nCloudNum];//������������poi�ļ�������
	for (int i = 0; i < _nCloudNum; ++i)
		strPoiName[i] = new char[MAX_LINE];
	char* strMiddlePathTemp = new char[50];
	for (int i = 0; i < _nCloudNum; ++i)
	{
		strcpy(strMiddlePathTemp, _strMiddlePath);
		char *p = strrchr(_strCloudName[i], '\\') + 1;
		strcpy(strPoiName[i], strcat(strMiddlePathTemp, p));
		strPoiName[i][strlen(strPoiName[i]) - 3] = 'p';
		strPoiName[i][strlen(strPoiName[i]) - 2] = 'o';
		strPoiName[i][strlen(strPoiName[i]) - 1] = 'i';
	}
	for (int i = 0; i < _nCloudNum - 1; ++i)
	{
		for (int j = i + 1; j < _nCloudNum; ++j)
		{
			char* strFileNameEx = strrchr(_strCloudName[i], '\\') + 1;
			char* strFileName = new char[strlen(strFileNameEx) - 4];
			strncpy_s(strFileName, strlen(strFileNameEx) - 3, strFileNameEx, strlen(strFileNameEx) - 4);
			strcpy(_strTieName[i][j], strFileName);
			strFileNameEx = strrchr(_strCloudName[j], '\\') + 1;
			strFileName = new char[strlen(strFileNameEx) - 3];
			strncpy_s(strFileName, strlen(strFileNameEx) - 3, strFileNameEx, strlen(strFileNameEx) - 4);
			strcat(_strTieName[i][j], "_"); 
			strcat(_strTieName[i][j], strFileName);
			strcat(_strTieName[i][j], ".tie");
			//cout << _strTieName[i][j] << endl;
		}
	}
	char* strPairMatchCommand = new char[200];
	for (int i = 0; i < _nCloudNum - 1; ++i)
	{
		for (int j = i + 1; j < _nCloudNum; ++j)
		{
			if (_nNeiborMatrix[i][j] == 1)
			{//��������ƥ��
				strcpy(strPairMatchCommand, "e:\\OfficeE\\PairMatchAdapter.exe " /*"PairMatchAdapter.exe "*/);
				strcat(strPairMatchCommand, strPoiName[i]);
				strcat(strPairMatchCommand, " ");
				strcat(strPairMatchCommand, strPoiName[j]);
				strcat(strPairMatchCommand, " ");
				strcat(strPairMatchCommand, _strMiddlePath);
				strcat(strPairMatchCommand, _strTieName[i][j]);
				//system(strPairMatchCommand);//����cmd����PairMatchAdapter.exe 
			}
		}
	}
	for (int i = 0; i < _nCloudNum; ++i)
	{
		delete[] strPoiName[i];
		strPoiName[i] = NULL;
	}
	delete[] strMiddlePathTemp;
	strMiddlePathTemp = NULL;
	delete[] strPairMatchCommand;
	strPairMatchCommand = NULL;
}
//��ȡ��n��������ǿ�ĵ���id
int GetNthCloud(int** _nTiePointNum, /*int* _nCloudID,*/ int _n, int _nCloudNum)
{
	if (_n < 1 || _n >= _nCloudNum) return -1;
	int* nCloudLeftIdx = new int[_nCloudNum];
	for (int i = 0; i < _nCloudNum; ++i) nCloudLeftIdx[i] = i;
	int* nMaxMatchIdx = new int[_nCloudNum];
	int nMaxCorrs = -1;
	//if (n == 1)
	{
		for (int i = 0; i < _nCloudNum - 1; ++i)
		{
			for (int j = i + 1; j < _nCloudNum; ++j)
			{
				if (nMaxCorrs < _nTiePointNum[i][j])
				{
					nMaxCorrs = _nTiePointNum[i][j];
					nMaxMatchIdx[0] = i;
					nMaxMatchIdx[1] = j;
				}
			}
		}
	}
	nCloudLeftIdx[nMaxMatchIdx[0]] = -1;
	nCloudLeftIdx[nMaxMatchIdx[1]] = -1;
	int flag = 0;
	for (int itor = 1; itor < _n; ++itor)
	{
		nMaxCorrs = -1;
		for (int i = 0; i < _nCloudNum; ++i)
		{
			int nMaxCorrsTemp = 0;
			if (nCloudLeftIdx[i] != -1)
			{
				for (int j = 0; j < _nCloudNum; ++j)
				{
					if (nCloudLeftIdx[j] == -1)
						nMaxCorrsTemp += _nTiePointNum[i][j];
				}
				if (nMaxCorrs < nMaxCorrsTemp)
				{
					nMaxMatchIdx[itor + 1] = i;
					flag = i;
					nMaxCorrs = nMaxCorrsTemp;
				}
			}
		}
		nCloudLeftIdx[flag] = -1;
	}
	int ans = nMaxMatchIdx[_n];
	delete[] nMaxMatchIdx;
	nMaxMatchIdx = NULL;
	return ans;
}

void CalculateCBA(char* _strMiddlePath)
{
	SYSTEM_INFO SystemInfo;//��ȡCPU�߳���
	GetSystemInfo(&SystemInfo);
	CString strThreadNum;
	strThreadNum.Format(_T("-num_threads=%d"), SystemInfo.dwNumberOfProcessors);
	CString cstrCbaCommand;
	CString strCbaFileName;
	CString cstrMiddlePath = _strMiddlePath;
	strCbaFileName.Format(_T("%scba.exe"), cstrMiddlePath);
	CString strCBAFilePath;
	strCBAFilePath.Format(_T("%scba.im"), cstrMiddlePath);
	cstrCbaCommand.Format(_T("%s %s -input=\"%s\" -robustify=true -rigorous_spacial_trans=true"), strCbaFileName, strThreadNum, strCBAFilePath);
	USES_CONVERSION;
	char* strCbaCommand = T2A(cstrCbaCommand);
	system(strCbaCommand);
}
 

int main(int argc, char *argv[])
{
	argv[1] = "e:\\OfficeG\\OfficeG_MatchAdapter1-80.txt";
	if (argv[1] == NULL) return 0;
	char* strFeatureAdapterName = new char[100];
	strcpy(strFeatureAdapterName, /*"e:\\Office\\FeatureAdapter.exe "*/ "FeatureAdapter.exe ");
	strcat(strFeatureAdapterName, argv[1]);
	//system(strFeatureAdapterName);//����cmd���������ļ�.poi
	FILE* fpMatchAdpt;
	fopen_s(&fpMatchAdpt, argv[1], "r");//��ȡMatchAdapter�ļ�
	if (fpMatchAdpt == NULL) { cout << "failed to open adapterfile."; return 0; }
	char strBuffer[MAX_LINE] = {};//������
	char* strMiddlePath = new char[50];//�м��ļ�·��
	char* strFinalTieName = new char[50];//����tie�ļ���
	int nCloudNum = 0;//�����ļ�����
	int len = 0;
	fgets(strMiddlePath, MAX_LINE, fpMatchAdpt);
	strMiddlePath[strlen(strMiddlePath) - 1] = 0;
	fgets(strFinalTieName, MAX_LINE, fpMatchAdpt);
	strFinalTieName[strlen(strFinalTieName) - 1] = 0;
	fgets(strBuffer, 10, fpMatchAdpt);
	nCloudNum = atoi(strBuffer);
	int* nCloudID = new int[nCloudNum];//�����ļ�id����
	char** strCloudName = new char*[nCloudNum];//�����ļ�������
	for (int i = 0; i < nCloudNum; ++i)
		strCloudName[i] = new char[MAX_LINE];
	for (int i = 0; i < nCloudNum; ++i)
	{
		fgets(strBuffer, 5, fpMatchAdpt);
		nCloudID[i] = atoi(strBuffer);
		fgets(strBuffer, MAX_LINE, fpMatchAdpt);
		strBuffer[strlen(strBuffer) - 1] = 0;
		strcpy(strCloudName[i], strBuffer);
	}
	for (int i = 0; i < nCloudNum; ++i)
	{//ȥ�������ļ���strCloudName[]�׿ո�
		char *	start;
		int len = strlen(strCloudName[i]);
		start = strCloudName[i];
		while (*start && isspace(*start))
			start++;
		strcpy(strCloudName[i], start);
	}
	int **nNeiborMatrix = new int*[nCloudNum];//��ȡ�ڽӾ���
	for (int i = 0; i < nCloudNum; ++i)
		nNeiborMatrix[i] = new int[nCloudNum];
	int ch;
	int i = 0, j = 0;
	while ((ch = fgetc(fpMatchAdpt)) != EOF)
	{
		if (ch != '\n'&&ch != ' ')
		{
			nNeiborMatrix[i][j] = ch - '0';
			j++;
			if (j == nCloudNum)
			{
				i++;
				j = 0;
			}
		}
	}
	//���ƶ�ƥ��
	char*** strTieName = new char**[nCloudNum];//������tie�ļ����Ķ�ά�ַ�������
	for (int i = 0; i < nCloudNum; ++i)
		strTieName[i] = new char*[nCloudNum];
	for (int i = 0; i < nCloudNum; ++i)
		for (int j = 0; j < nCloudNum; ++j)
			strTieName[i][j] = new char[MAX_LINE];
	PairMatch(nCloudNum, nNeiborMatrix, strMiddlePath, strCloudName, strTieName);
	//��������ȡ����tie�ļ��ڵĵ㣬������Ϣ��nTiePointNum, �����Ϣ��PairTiePoints
	int** nTiePointNum = new int*[nCloudNum];
	for (int i = 0; i < nCloudNum; ++i)
		nTiePointNum[i] = new int[nCloudNum];
	for (int i = 0; i < nCloudNum; ++i)
		for (int j = 0; j < nCloudNum; ++j)
			nTiePointNum[i][j] = 0;
	TiePoint*** PairTiePoints = new TiePoint**[nCloudNum];
	for (int i = 0; i < nCloudNum; ++i)
		PairTiePoints[i] = new TiePoint*[nCloudNum];
	LoadTieFiles(nCloudNum, strMiddlePath, strTieName, nNeiborMatrix, nTiePointNum, PairTiePoints);
	for (int i = 1; i < nCloudNum; ++i)
		for (int j = 0; j < i; ++j)
			nTiePointNum[i][j] = nTiePointNum[j][i];//��Գ�����
	registration::TransformationEstimationSVD<PointXYZ, PointXYZ, float>::Matrix4* CloudTransMatrix = new registration::TransformationEstimationSVD<PointXYZ, PointXYZ, float>::Matrix4[nCloudNum];
		for (int i = 0; i < nCloudNum; ++i)
		CloudTransMatrix[i].setIdentity();
	//UpdateCloudTransMatrixByEoiFile(CloudTransMatrix, "e:\\officee\\middle\\finaltransmatrix.eoi", nCloudNum, nCloudNum, nCloudID);
	//TransformedCloudVisualizer(strCloudName, CloudTransMatrix, nCloudNum);//ƴ�Ӻ���ƿ��ӻ�

	int nPointNum = 0;//��Ӧ������ 
	for (int i = 0; i < nCloudNum; ++i)
		for (int j = 0; j < nCloudNum; ++j)
			nPointNum += nTiePointNum[i][j];
	TagMatchPoint* AllTiePointsOrigin = GetWholeTie(nCloudNum, strMiddlePath, nCloudID, nTiePointNum, PairTiePoints);//����tie�ļ��ݹ�Ѱ��ͬ����,����ԭʼ�����tie
	//WriteAllTiePoints(AllTiePointsOrigin, nPointNum / 2, "e:\\NewOffice\\Middle\\TiePoints.tie");
	int nReferenceTgtIdx, nReferenceSrcIdx, nMaxCorrs = -1;//�ҵ���������ܵ�������������������
	for (int i = 0; i < nCloudNum; ++i)
	{
		for (int j = 0; j < nCloudNum; ++j)
		{
			if (nTiePointNum[i][j] > nMaxCorrs)
			{
				nReferenceTgtIdx = i;
				nReferenceSrcIdx = j;
				nMaxCorrs = nTiePointNum[i][j];
			}
		}
	}
	//����.im�ļ�
	char* strImFile = new char[MAX_LINE];
	strcpy(strImFile, strMiddlePath);
	strcat(strImFile, "cba.im");
	WriteImFileFirstTwo(PairTiePoints, AllTiePointsOrigin, nReferenceTgtIdx, nReferenceSrcIdx, strImFile, nCloudID, nTiePointNum, nCloudNum);
	//����CBA
	CalculateCBA(strMiddlePath);
	char* strEoiName = new char[MAX_LINE]; 
	strcpy(strEoiName, strMiddlePath);
	strcat(strEoiName, "cba.eoi");
	UpdateCloudTransMatrixByEoiFile(CloudTransMatrix, strEoiName, nCloudNum, 2, nCloudID);
	int* nMatchedCloudIdx = new int[nCloudNum];//����ɵ�����ת���ĵ�������
	nMatchedCloudIdx[0] = nReferenceTgtIdx;
	nMatchedCloudIdx[1] = nReferenceSrcIdx;
	for (int itor = 2; itor < nCloudNum; ++itor)//ʣ��ÿ�������ļ�ѭ��һ��
	{
		int nMatchingCloudIdx = GetNthCloud(nTiePointNum, nCloudID, itor, nCloudNum);//�ô�ѭ����������Ϊ��nThisCloud������
		WriteImFile(PairTiePoints, AllTiePointsOrigin, CloudTransMatrix, nMatchedCloudIdx, nMatchingCloudIdx, strImFile, nCloudID, nTiePointNum, nCloudNum, itor);//д��ô�ѭ����im�ļ�
		CalculateCBA(strMiddlePath);//��cba����eoi����
		UpdateCloudTransMatrixByEoiFile(CloudTransMatrix, strEoiName, nCloudNum, itor + 1, nCloudID);
		nMatchedCloudIdx[itor] = nMatchingCloudIdx;//����ǰ������ӵ�����ɼ���ĵ����б�
	}
	TransformedCloudVisualizer(strCloudName, CloudTransMatrix, nCloudNum);//ƴ�Ӻ���ƿ��ӻ�
	//c
	{
		for (int i = 0; i < nCloudNum; ++i)
		{
			delete[] PairTiePoints[i];
			PairTiePoints[i] = NULL;
		}
		delete[] CloudTransMatrix;
		CloudTransMatrix = NULL;
		delete[] nMatchedCloudIdx;
		nMatchedCloudIdx = NULL;
		delete[] strEoiName;
		strEoiName = NULL;
		for (int i = 0; i < nCloudNum; ++i)
		{
			delete[] nTiePointNum[i];
			nTiePointNum[i] = NULL;
		}
		for (int i = 0; i < nCloudNum; ++i)
		{
			for (int j = 0; j < nCloudNum; ++j)
			{
				delete[] strTieName[i][j];
				strTieName[i][j] = NULL;
			}
			delete[] strTieName[i];
			strTieName[i] = NULL;
		}
		delete[] strFeatureAdapterName;
		strFeatureAdapterName = NULL;
		fclose(fpMatchAdpt);
		delete[] strMiddlePath;
		strMiddlePath = NULL;
		delete[] strFinalTieName;
		strFinalTieName = NULL;
		delete[] nCloudID;
		nCloudID = NULL;
		for (int i = 0; i < nCloudNum; ++i)
		{
			delete[] strCloudName[i];
			strCloudName[i] = NULL;
		}
		for (int i = 0; i < nCloudNum; ++i)
		{
			delete[] nNeiborMatrix[i];
			nNeiborMatrix[i] = NULL;
		}
		delete[] strImFile;
		strImFile = NULL;
	}
	return 0;
}