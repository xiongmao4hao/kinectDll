#pragma once
// pch.cpp: 与预编译标头对应的源文件


#ifdef SIMPLE_CLASS_EXPORT

#define SIMPLE_CLASS_EXPORT __declspec(dllexport)

#else

#define SIMPLE_CLASS_EXPORT __declspec(dllimport)

#endif

#ifndef ANGLE_NUM
#define ANGLE_NUM 18
#endif // !ANGLE_NUM


typedef struct
{
	float fJointsAngle[ANGLE_NUM];
}ReturnKinct;

class Kinct
{
public:
	bool  bOnePicture;
	Kinct();
	~Kinct();
	int init();
	int del();
	int capThread();
private:
	k4a_device_t* dev;
	k4a_calibration_t* sensorCalibration;
	ReturnKinct* returnKinect;
	cv::Mat* colorFrame;
	thread* tids;
	uint32_t uintNum;
	mutex Flagangle1to1;
	int  iFlagAngle1to1, iFlagMaster;
	uint32_t iMasterNum;
	bool bInitFlag, bDel;
	float fAngelUsing[ANGLE_NUM];
	//bool esc = false;
	void cap(k4a_device_t& dev, cv::Mat& colorFrame, const int i, k4a_calibration_t& sensorCalibration, \
		float(&fAngelUsing)[ANGLE_NUM], ReturnKinct& returnKinect);  //普通的函数，用来执行线程
	void onePicture(const int i, cv::Mat colorFrame, k4a_image_t colorImage, k4a_capture_t sensor_capture, k4abt_tracker_t tracker);
protected:
};


SIMPLE_CLASS_EXPORT int recordStart();
SIMPLE_CLASS_EXPORT int recordStop();
SIMPLE_CLASS_EXPORT int recordOne();