
#pragma once

#include <stdlib.h>

#include <k4a/k4a.hpp>
#include "windows_thread.h"
#include "windows_time.h"
#include "kinect_cv_dk.h"
#include "kinect_angle.h"

#include <array>
#include <iostream>
#include <map>
#include <vector>

#include <cmath>
#include <time.h>

#include <math.h>


#define KEY_DOWN(VK_NONAME) ((GetAsyncKeyState(VK_NONAME) & 0x8001) ? 1:0)//自对GetAsyncKeyState函数的上一次调用以来，如键已被按过，则位0设为1；否则设为0。如键目前处于按下状态，则位15设为1；如抬起，则为0。详见有道云
//定义的报错显示程序
#define VERIFY(result, error)                                                                            \
    if(result != K4A_RESULT_SUCCEEDED)                                                                   \
    {                                                                                                    \
        printf("%s \n - (File: %s, Function: %s, Line: %d)\n", error, __FILE__, __FUNCTION__, __LINE__); \
        exit(1);                                                                                         \
    }   

#ifndef ANGLE_NUM
#define ANGLE_NUM 18
#endif // !ANGLE_NUM


class Kinct
{
public:
	static volatile bool  bOnePicture;
	Kinct();
	int init();
	int del();
	int capThread();
	int getAngle(float (&fAngle)[ANGLE_NUM]);
	int getJoint(float *(&fJoint)[BODU_POINT_NUM]);
	int getCmat(cv::Mat&);

private:
	k4a_device_t* dev;
	k4a_calibration_t* sensorCalibration;
	cv::Mat* colorFrame;
	std::thread* tids;
	std::mutex* onePictureFlag;
	uint32_t uintNum;
	//时间戳
	uint64_t timeStamp;
	//保存关节点坐标
	k4abt_skeleton_t skeleton;
	std::mutex Flagangle1to1;
	int  iFlagAngle1to1, iFlagMaster, iMasterNum;
	bool bInitFlag, bDel;
	float fAngelUsing[ANGLE_NUM], fAmgleALL[ANGLE_NUM];
	//bool esc = false;
	void cap(k4a_device_t& dev, cv::Mat& colorFrame, const int i, k4a_calibration_t& sensorCalibration, \
		float(&fAngelUsing)[ANGLE_NUM]);  //普通的函数，用来执行线程
	int onePicture(const int i, k4a_capture_t sensor_capture, float(&fAngelUsing)[ANGLE_NUM], k4abt_tracker_t& tracker);
protected:
};

