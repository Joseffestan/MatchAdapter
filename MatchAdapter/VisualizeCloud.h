#pragma once

#include "stdafx.h"
#include <pcl/io/pcd_io.h> 
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/radius_outlier_removal.h>

using namespace pcl;

extern void TXTtoPCDConvertor(char* _strTxtCloudName, PointCloud<PointXYZRGB>::Ptr _PcdCloud);

extern void PcdVisualizer(std::vector<PointCloud<PointXYZRGB>::Ptr> Clouds, int _nCloudNum);

extern void CloudVisualizer(char** _strTxtCloudName, int _nCloudNum);

extern void TransformedCloudVisualizer(char** _strTxtCloudName, registration::TransformationEstimationSVD<PointXYZ, PointXYZ, float>::Matrix4* CloudTransMatrix, int _nCloudNum);