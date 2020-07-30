#pragma once
// pch.cpp: ��Ԥ�����ͷ��Ӧ��Դ�ļ�


#ifdef SIMPLE_CLASS_EXPORT

#define SIMPLE_CLASS_EXPORT __declspec(dllexport)

#else

#define SIMPLE_CLASS_EXPORT __declspec(dllimport)

#endif

#ifndef ANGLE_NUM
#define ANGLE_NUM 18
#endif // !ANGLE_NUM

#ifndef BODU_POINT_NUM
#define BODU_POINT_NUM 32
#endif
typedef struct
{
	float fAngle[ANGLE_NUM];
	float *fJoint[BODU_POINT_NUM];
	uchar* cmatData;
	int sizeMat;
	int sizeRows;
	int sizeCols;
	/*cv::Mat cMat;*/
}ReturnKinct;

SIMPLE_CLASS_EXPORT int recordStart();
SIMPLE_CLASS_EXPORT int recordStop();
SIMPLE_CLASS_EXPORT ReturnKinct recordOne();