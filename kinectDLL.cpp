// pch.cpp: ��Ԥ�����ͷ��Ӧ��Դ�ļ�
#include "pch.h"

// ��ʹ��Ԥ�����ͷʱ����Ҫʹ�ô�Դ�ļ���������ܳɹ���

#include "kinect_record.h"
#include "kinectDLL.h"

extern Kinct kinct;
extern cv::Mat cmatData;

using namespace cv;
using namespace std;

//�ص��ú���
int recordStart()
{
	recordStop();
	VERIFY(kinct.init(), "kinect inint failed!");//��ʼ��,�������
	kinct.capThread();//���������߳�
	return 0;
}

int recordStop()
{
	VERIFY(kinct.del(), "kinect del failed!");//�ر��������
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