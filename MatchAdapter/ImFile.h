#pragma once
#include "stdafx.h"
#include "TiePoints.h"

//根据CBA结果的eoi矩阵，修改PairTiePoint点对匹配结果的坐标
extern void TransformPairTiePoints(TiePoint*** _PairTiePoints, int** _nTiePointNum, int _nTransIdx, registration::TransformationEstimationSVD<PointXYZ, PointXYZ, float>::Matrix4 _TransMatrix, int _nCloudNum);

//写入初始两组点云的im文件
extern void WriteImFileFirstTwo(TiePoint*** _PairTiePoint, TagMatchPoint* _AllTiePoints, int _nTgtIdx, int _nSrcIdx, char* _strImFile, int* _nCloudID, int** _nTiePointNum, int _nCloudNum);

//写入初始两组点云之后的点云im文件
extern void WriteImFile(TiePoint*** _PairTiePoint, TagMatchPoint* _AllTiePoints, registration::TransformationEstimationSVD<PointXYZ, PointXYZ, float>::Matrix4* _TransMatrix, int* _nTgtIdx, int _nSrcIdx, char* _strImFile, int* _nCloudID, int** _nTiePointNum, int _nCloudNum, int _nMatchedCloud);

//将所有点云坐标写入同一im文件(测试用)
extern void WriteAllCloudImFile(TagMatchPoint* _AllOriginMatchPoint, TagMatchPoint* _AllMatchPoint, int _nPointsNum, char* _strFinalTieName, int _nCloudNum, int* _nCloudID);

//从eoi文件更新点云转换矩阵数组
extern void UpdateCloudTransMatrixByEoiFile(registration::TransformationEstimationSVD<PointXYZ, PointXYZ, float>::Matrix4* _CloudTransMatrix, char* _strEoiFile, int _nCloudNum, int _nMatrixNum, int* _nCloudID);