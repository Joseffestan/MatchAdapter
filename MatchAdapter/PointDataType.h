#pragma once

struct TiePoint {
	float TargetX;
	float TargetY;
	float TargetZ;
	float SourceX;
	float SourceY;
	float SourceZ;
};
struct TagPoint {
	int nCloudID;
	float x;
	float y;
	float z;
	struct TagPoint* pNext;
};
struct TagMatchPoint {
	int nPointID;
	int nMark;//PointList�ڵ������
	struct TagPoint* PointList;
};