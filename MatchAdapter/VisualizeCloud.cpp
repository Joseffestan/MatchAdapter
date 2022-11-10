
#include "stdafx.h"
#include "VisualizeCloud.h"
#include <time.h>
#include <pcl/visualization/pcl_visualizer.h>

using namespace pcl;
using namespace std;

bool IsXYZCorrect(float _x, float _y, float _z, int _nRef)
{
	if (_x > 30000 || _x < -30000)
		return false;
	if (_y > 30000 || _y < -30000)
		return false;
	if (_z > 10000 || _z < -10000)
		return false;
	if (_nRef > 256 || _nRef < 5)
		return false;
	return true;
}
void TXTtoPCDConvertor(char* _strTxtCloudName, PointCloud<PointXYZRGB>::Ptr _PcdCloud)
{
	// 加载txt数据
	FILE *fpTxt;
	PointXYZRGB point;
	int nRef;
	uint64_t timestamp = 0;
	fopen_s(&fpTxt, _strTxtCloudName, "r");
	if (fpTxt == NULL)
	{
		cout << "failed to open txt cloud file.";
		return;
	}
	char strLine[50];
	while (!feof(fpTxt))
	{
		fgets(strLine, 50, fpTxt);
		sscanf(strLine, "%f,%f,%f,%f,%lld,\n", &point.x, &point.y, &point.z, &point.rgb, &timestamp);
		point.x *= 1000;
		point.y *= 1000;
		point.z *= 1000;
		if (IsXYZCorrect(point.x, point.y, point.z, point.rgb) == true)
			_PcdCloud->push_back(point);
	}
	fclose(fpTxt);
}
 

void PcdVisualizer(vector<PointCloud<PointXYZRGB>::Ptr> _Clouds, int _nCloudNum)
{
	visualization::PCLVisualizer viewer("registration Viewer");
	srand((unsigned int)time(NULL));
	for (int i = 0; i < _nCloudNum; ++i)
	{
		visualization::PointCloudColorHandlerGenericField<PointXYZRGB> fildColor(_Clouds[i], "rgb");
		/*int* nColorRGB = new int[3];
		for (int j = 0; j < 3; ++j)
			nColorRGB[j] = rand() % (255 - 0 + 1) + 1;//设置范围 0-255
		visualization::PointCloudColorHandlerCustom<PointXYZRGB> handler(_Clouds[i], nColorRGB[0], nColorRGB[1], nColorRGB[2]);*/
 
		char szCloudID[5];
		itoa(i, szCloudID, 4);
		if (i % 2 == 0)
			viewer.addPointCloud(_Clouds[i], fildColor, szCloudID);
	}
	
	while (!viewer.wasStopped())
	{
		viewer.spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
}


void CloudVisualizer(char** _strTxtCloudName, int _nCloudNum)
{
	vector<PointCloud<PointXYZRGB>::Ptr> Clouds;
	for (int i = 0; i < _nCloudNum; ++i) 
	{
		PointCloud<PointXYZRGB>::Ptr Cloud(new PointCloud<PointXYZRGB>);
		TXTtoPCDConvertor(_strTxtCloudName[i], Cloud);
		Clouds.push_back(Cloud);
	}
	PcdVisualizer(Clouds,_nCloudNum);
}
void TransformedCloudVisualizer(char** _strTxtCloudName, registration::TransformationEstimationSVD<PointXYZ, PointXYZ, float>::Matrix4* _CloudTransMatrix, int _nCloudNum)
{
	vector<PointCloud<PointXYZRGB>::Ptr> TransedClouds;
	for (int i = 0; i < _nCloudNum; ++i)
	{
	 	PointCloud<PointXYZRGB>::Ptr Cloud(new PointCloud<PointXYZRGB>);
		TXTtoPCDConvertor(_strTxtCloudName[i], Cloud);
		PointCloud<PointXYZRGB>::Ptr TransedCloud(new PointCloud<PointXYZRGB>);
		for (int j = 0; j < 3; ++j)
			_CloudTransMatrix[i](j, 3) *= 1000;
		transformPointCloud(*Cloud, *TransedCloud, _CloudTransMatrix[i], true);
		//PointCloud<PointXYZRGB>::Ptr CloudFiltered(new PointCloud<PointXYZRGB>);
		//RadiusOutlierRemoval<PointXYZRGB> OutlierFilter;
		//OutlierFilter.setInputCloud(TransedCloud);
		//OutlierFilter.setRadiusSearch(30);// 搜索半径
		//OutlierFilter.setMinNeighborsInRadius(15);//最少的邻居数目
		//OutlierFilter.filter(*CloudFiltered);
		TransedClouds.push_back(TransedCloud);
	}
	PcdVisualizer(TransedClouds, _nCloudNum);
}