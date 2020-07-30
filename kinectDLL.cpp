// pch.cpp: 与预编译标头对应的源文件
#include "pch.h"

// 当使用预编译的头时，需要使用此源文件，编译才能成功。

#include "kinect_record.h"
#include "kinectDLL.h"

extern Kinct kinct;
extern cv::Mat cmatData;

using namespace cv;
using namespace std;

//回调用函数
int recordStart()
{
	recordStop();
	VERIFY(kinct.init(), "kinect inint failed!");//初始化,启动相机
	kinct.capThread();//创建捕获线程
	return 0;
}

int recordStop()
{
	VERIFY(kinct.del(), "kinect del failed!");//关闭相机捕获
	return 0;
}

ReturnKinct recordOne()
{
	kinct.bOnePicture = true;
	ReturnKinct returnKinct;
	while (1)
	{
		if (!kinct.bOnePicture)
		{
			cout << "getting" << endl;
			VERIFY(kinct.getAngle(returnKinct.fAngle), " angle fail....");
			VERIFY(kinct.getJoint(returnKinct.fJoint), " joint fail....");
			kinct.getCmat(cmatData);
			returnKinct.sizeMat = cmatData.rows * cmatData.cols;
			returnKinct.cmatData = cmatData.data;
			returnKinct.sizeCols = cmatData.cols;
			returnKinct.sizeRows = cmatData.rows;
			/*returnKinct.cMat = cmatDate;*/
			break;
		}
	}
	return returnKinct;
}