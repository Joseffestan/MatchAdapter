#pragma once

#include "stdafx.h"
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/common/transforms.h>
#include <pcl/common/eigen.h>
#include "PointDataType.h"
using namespace pcl;



//�ݹ�Ѱ�ҵ���ͬ����
extern void GetTiePoint(int **_nTiePointNum, struct TiePoint*** _pairTiePoints, struct TagMatchPoint& _matchedPoints, int* _nCloudID, int _nCloudIdx, int _nCloudNum, float _ix, float _iy, float _iz);

//�������еĵ��ƶ�tie�ļ���������tie�ļ�
extern TagMatchPoint* GetWholeTie(int _nCloudNum, char* _strMiddlePath, int* _nCloudID, int** _nTiePointNum, TiePoint*** _PairTiePoints);

//��һ�����ת���������ת��
extern TagPoint* TransformPoint(TagPoint* _Source, registration::TransformationEstimationSVD<PointXYZ, PointXYZ, float>::Matrix4 _TransMatrix);

//����CBA�����eoi�����޸�����tie�ļ��ĵ������
extern void TransformFinalTie(TagMatchPoint* _AllPoints, int _nPointsNum, char* _strEoiName, int* _nMatchedCloudIdx, int _nMatchedCloudNum, int _nMatchingCloudIdx);

//����Ӧ���б�д�뱾��
extern void WriteAllTiePoints(TagMatchPoint* _AllPoints, int _nPointsNum, char* _strFinalTieName);

//��������ȡ����tie�ļ��ڵĵ㣬������Ϣ��_nTiePointNum, �����Ϣ��_PairTiePoints
extern void LoadTieFiles(int _nCloudNum, char* _strMiddlePath, char*** _strTieName, int** _nNeiborMatrix, int** _nTiePointNum, TiePoint*** _PairTiePoints);
