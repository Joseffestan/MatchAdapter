#pragma once
#include "stdafx.h"
#include "TiePoints.h"

//����CBA�����eoi�����޸�PairTiePoint���ƥ����������
extern void TransformPairTiePoints(TiePoint*** _PairTiePoints, int** _nTiePointNum, int _nTransIdx, registration::TransformationEstimationSVD<PointXYZ, PointXYZ, float>::Matrix4 _TransMatrix, int _nCloudNum);

//д���ʼ������Ƶ�im�ļ�
extern void WriteImFileFirstTwo(TiePoint*** _PairTiePoint, TagMatchPoint* _AllTiePoints, int _nTgtIdx, int _nSrcIdx, char* _strImFile, int* _nCloudID, int** _nTiePointNum, int _nCloudNum);

//д���ʼ�������֮��ĵ���im�ļ�
extern void WriteImFile(TiePoint*** _PairTiePoint, TagMatchPoint* _AllTiePoints, registration::TransformationEstimationSVD<PointXYZ, PointXYZ, float>::Matrix4* _TransMatrix, int* _nTgtIdx, int _nSrcIdx, char* _strImFile, int* _nCloudID, int** _nTiePointNum, int _nCloudNum, int _nMatchedCloud);

//�����е�������д��ͬһim�ļ�(������)
extern void WriteAllCloudImFile(TagMatchPoint* _AllOriginMatchPoint, TagMatchPoint* _AllMatchPoint, int _nPointsNum, char* _strFinalTieName, int _nCloudNum, int* _nCloudID);

//��eoi�ļ����µ���ת����������
extern void UpdateCloudTransMatrixByEoiFile(registration::TransformationEstimationSVD<PointXYZ, PointXYZ, float>::Matrix4* _CloudTransMatrix, char* _strEoiFile, int _nCloudNum, int _nMatrixNum, int* _nCloudID);