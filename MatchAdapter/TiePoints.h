#pragma once

#include "stdafx.h"
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/common/transforms.h>
#include <pcl/common/eigen.h>
#include "PointDataType.h"
using namespace pcl;



//递归寻找点云同名点
extern void GetTiePoint(int **_nTiePointNum, struct TiePoint*** _pairTiePoints, struct TagMatchPoint& _matchedPoints, int* _nCloudID, int _nCloudIdx, int _nCloudNum, float _ix, float _iy, float _iz);

//根据所有的点云对tie文件构建完整tie文件
extern TagMatchPoint* GetWholeTie(int _nCloudNum, char* _strMiddlePath, int* _nCloudID, int** _nTiePointNum, TiePoint*** _PairTiePoints);

//将一点根据转换矩阵进行转换
extern TagPoint* TransformPoint(TagPoint* _Source, registration::TransformationEstimationSVD<PointXYZ, PointXYZ, float>::Matrix4 _TransMatrix);

//根据CBA结果的eoi矩阵，修改最终tie文件的点对坐标
extern void TransformFinalTie(TagMatchPoint* _AllPoints, int _nPointsNum, char* _strEoiName, int* _nMatchedCloudIdx, int _nMatchedCloudNum, int _nMatchingCloudIdx);

//将对应点列表写入本地
extern void WriteAllTiePoints(TagMatchPoint* _AllPoints, int _nPointsNum, char* _strFinalTieName);

//遍历并读取所有tie文件内的点，点数信息到_nTiePointNum, 点对信息到_PairTiePoints
extern void LoadTieFiles(int _nCloudNum, char* _strMiddlePath, char*** _strTieName, int** _nNeiborMatrix, int** _nTiePointNum, TiePoint*** _PairTiePoints);
