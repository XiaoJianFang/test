#include <iostream>  
#include <fstream>  
#include <vector>  
#include <math.h> 
#include <opencv2/opencv.hpp>

#include "../include/PacketMap.h"
#include "../include/opencv/cv.h"
#include "../include/opencv/highgui.h"
#define _DEBUG

#include "../include/Visualization/VisualizationBasicApi.h"
#include "../include/Visualization/VisualizationConfigure.h"

#ifdef _DEBUG
#pragma comment(lib, "../lib/visualization/Visualization2016d.lib")
#else
#pragma comment(lib, "../lib/visualization/Visualization2016.lib")
#endif

#ifdef _DEBUG 
#pragma comment(lib, "../lib/cv/x64/opencv_world310d.lib")
#else
#pragma comment(lib, "../lib/cv/x64/opencv_world310.lib")
#endif
using namespace cv;
using namespace std;

VisualMouse  view;
CvFont font;

bool  isLdown;
bool  isWheel;
float oldx;
float oldy;
float initScale;
float g_offsetxdown;
float g_offsetydown;
float offsetx;
float offsety;
float curx;
float cury;
float mousex;
float mousey;


float DistanceTwo;
float pointTwo[8] = { 0.f };

void CloneData(vector<float>&datax, vector<float>&datay, vector<float>&showx, vector<float>&showy, float offsetx, float offsety);

void mouseHandler(int event, int x, int y, int flags, void* param)
{
	cvInitFont(&font, CV_FONT_HERSHEY_PLAIN, 1.5, 1.5, 0, 2, 8);
	mousex = (x / view.g_winScale) - view.gwinX - view.g_offsetx;
	mousey = ((view.windowhigh - y) / view.g_winScale) + view.gwinBkwd - view.g_offsety;
	sprintf_s(view.labeldst, "(%d, %d)cm", (int)(mousex * 100), (int)(mousey * 100));

	if (event == CV_EVENT_LBUTTONDOWN)
	{
		isLdown = true;
		oldx = (float)x;
		oldy = (float)(view.windowhigh - y);
		g_offsetxdown = view.g_offsetx;
		g_offsetydown = view.g_offsety;
	}
	else if (event == cv::EVENT_MOUSEMOVE)
	{
		if (isLdown)
		{
			offsetx = x - oldx;
			offsety = view.windowhigh - y - oldy;
			view.g_offsetx = (g_offsetxdown + offsetx / view.g_winScale);
			view.g_offsety = (g_offsetydown + offsety / view.g_winScale);
		}
	}
	else if (event == cv::EVENT_LBUTTONUP)
	{
		isLdown = false;
		offsetx = 0;
		offsety = 0;
	}
	else if (event == CV_EVENT_RBUTTONDOWN)
	{
		curx = (mousex + view.gwinX) * initScale;
		cury = (mousey - view.gwinBkwd)*initScale;
		//cvPutText(dst, labeldst, cvPoint(curx, cury), &font, CV_RGB(255, 0, 0));
		//cvShowImage("dst", dst);
		//ReserveMousex.push_back(mousex);
		//ReserveMousey.push_back(mousey);
		if (view.Distanceflag == 1)
		{
			pointTwo[0] = mousex;
			pointTwo[1] = mousey;
			view.Distanceflag = 2;
		}
		else if (view.Distanceflag == 2)
		{
			pointTwo[2] = mousex;
			pointTwo[3] = mousey;
			view.Distanceflag = 3;
		}
	}
	else if (event == cv::EVENT_MOUSEWHEEL)
	{
		if (flags > 0)
			view.g_winScale *= 1.2f;
		else if (flags < 0)
			view.g_winScale *= 0.8f;
	}

	if (view.Distanceflag == 3)
	{
		DistanceTwo = sqrt((pointTwo[0] - pointTwo[2])*(pointTwo[0] - pointTwo[2]) + (pointTwo[1] - pointTwo[3])*(pointTwo[1] - pointTwo[3]));
		sprintf_s(view.labeldistance, "Diatance=%d cm", (int)(DistanceTwo * 100));
	}
	else
	{
		sprintf_s(view.labeldistance, "Diatance=%d cm", 0);
	}

	
}

//void ShowDatas(ICPVisiualData& ICPData)
//{
//	visual::VisualizationBasicApi vsb;
//	vsb.setwinX(50);
//	vsb.setwinFwd(70);
//	vsb.setwinBkwd(-20);
//	vsb.setwinScale(9.0f);
//	IplImage* ICPVisual = cvCreateImage(cvSize(int((float)vsb.getwinX() * 2.f * vsb.getwinScale()), int(((float)vsb.getwinFwd() - (float)vsb.getwinBkwd())*vsb.getwinScale())), IPL_DEPTH_64F, 3);
//	ICPVisual->origin = IPL_ORIGIN_BL;
//	vector<float> showx, showy;
//
//	cvShowImage("ICP", ICPVisual);
//	cvSetMouseCallback("ICP", mouseHandler, (void*)ICPVisual);
//
//	view.windowhigh = (float)vsb.getwinFwd() - (float)vsb.getwinBkwd();
//	view.gwinX = vsb.getwinX();
//	view.gwinBkwd = vsb.getwinBkwd();
//	view.g_winScale = vsb.getwinScale();
//	view.gwinFwd = vsb.getwinFwd();
//	view.initScale = vsb.getwinScale();
//	view.g_offsetx = 0.f;
//	view.g_offsety = 0.f;
//	view.Distanceflag = 0;
//
//	while (1)
//	{
//		vsb.setwinScale(view.g_winScale);
//
//		vsb.clearImage(ICPVisual, CV_RGB(192, 192, 192));
//
//		CloneData(ICPData.MapCloudx, ICPData.MapCloudy, showx, showy, view.g_offsetx, view.g_offsety);
//		vsb.showPoints(ICPVisual, showx, showy, 2, CV_RGB(250, 0, 0), -1);
//
//		CloneData(ICPData.CurrentCloudx,ICPData.CurrentCloudy, showx, showy, view.g_offsetx, view.g_offsety);
//		vsb.showPoints(ICPVisual, showx, showy, 2, CV_RGB(0, 250, 0), -1);
//
//		CloneData(ICPData.TransfomationCloudx,ICPData.TransfomationCloudy, showx, showy, view.g_offsetx, view.g_offsety);
//		vsb.showPoints(ICPVisual, showx, showy, 2, CV_RGB(0, 0, 250), -1);
//
//		cvPutText(ICPVisual, view.labeldst, cvPoint(20, view.windowhigh*view.initScale - 20), &font, CV_RGB(255, 0, 0));
//		cvPutText(ICPVisual, view.labeldistance, cvPoint(20, view.windowhigh*view.initScale - 50), &font, CV_RGB(255, 0, 0));
//
//		cvShowImage("ICP", ICPVisual);
//
//		int Keynum = 0;
//		Keynum = cvWaitKey(10);
//		if (Keynum == 27)
//		{
//			break;
//		}
//		else if (Keynum == 68)//D
//		{
//			view.Distanceflag = 1;
//			cout << "Measure Distance" << endl;
//		}
//	}
//}

void ShowDatas(vector<float> data1x, vector<float> data1y, vector<float> data2x, vector<float> data2y )
{
	visual::VisualizationBasicApi vsb;
	vsb.setwinX(300);
	vsb.setwinFwd(800);
	vsb.setwinBkwd(0);
	vsb.setwinScale(1.0f);
	IplImage* ICPVisual = cvCreateImage(cvSize(int((float)vsb.getwinX() * 2.f * vsb.getwinScale()), int(((float)vsb.getwinFwd() - (float)vsb.getwinBkwd())*vsb.getwinScale())), IPL_DEPTH_64F, 3);
	ICPVisual->origin = IPL_ORIGIN_BL;
	vector<float> showx, showy;
	vector<float> showx1, showy1;
	vector<float> showx2, showy2;
	vector<float> showx3, showy3;

	cvShowImage("ICP", ICPVisual);
	cvSetMouseCallback("ICP", mouseHandler, (void*)ICPVisual);

	view.windowhigh = (float)vsb.getwinFwd() - (float)vsb.getwinBkwd();
	view.gwinX = vsb.getwinX();
	view.gwinBkwd = vsb.getwinBkwd();
	view.g_winScale = vsb.getwinScale();
	view.gwinFwd = vsb.getwinFwd();
	view.initScale = vsb.getwinScale();
	view.g_offsetx = 0.f;
	view.g_offsety = 0.f;
	view.Distanceflag = 0;

	//for (int i = -50; i < 50; i++)
	//{
	//	showx1.push_back(i);
	//	showy1.push_back(2);
	//	showx2.push_back(i);
	//	showy2.push_back(-2);
	//}
	//showx3.push_back(0);
	//showy3.push_back(0);
	while (1)
	{
		vsb.setwinScale(view.g_winScale);

		vsb.clearImage(ICPVisual, CV_RGB(192, 192, 192));

		CloneData(data1x, data1y, showx, showy, view.g_offsetx, view.g_offsety);
		vsb.showPoints(ICPVisual, showx, showy, 3, CV_RGB(250, 0, 0), -1);

		/*CloneData(showx1, showy1, showx, showy, view.g_offsetx, view.g_offsety);
		vsb.showPoints(ICPVisual, showx, showy, 2, CV_RGB(250, 0, 0), -1);

		CloneData(showx2, showy2, showx, showy, view.g_offsetx, view.g_offsety);
		vsb.showPoints(ICPVisual, showx, showy, 2, CV_RGB(250, 0, 0), -1);*/

		CloneData(data2x, data2y, showx, showy, view.g_offsetx, view.g_offsety);
		vsb.showPoints(ICPVisual, showx, showy, 2, CV_RGB(0, 250, 0), -1);

		//CloneData(showx3, showx3, showx, showy, view.g_offsetx, view.g_offsety);
		//vsb.showPoints(ICPVisual, showx, showy, 4, CV_RGB(0, 0, 250), -1);

		cvPutText(ICPVisual, view.labeldst, cvPoint(20, view.windowhigh - 20), &font, CV_RGB(255, 0, 0));
		cvPutText(ICPVisual, view.labeldistance, cvPoint(20, view.windowhigh - 50), &font, CV_RGB(255, 0, 0));

		cvShowImage("ICP", ICPVisual);

		int Keynum = 0;
		Keynum = cvWaitKey(10);
		if (Keynum == 27)
		{
			break;
		}
		else if (Keynum == 68)//D
		{
			view.Distanceflag = 1;
			cout << "Measure Distance" << endl;
		}
	}
}

void ShowDatas(ShowData& Data)
{
	visual::VisualizationBasicApi vsb;
	vsb.setwinX(100);
	vsb.setwinFwd(400);
	vsb.setwinBkwd(0);
	vsb.setwinScale(2.5f);
	IplImage* VisualImage = cvCreateImage(cvSize(int((float)vsb.getwinX() * 2.f * vsb.getwinScale()), int(((float)vsb.getwinFwd() - (float)vsb.getwinBkwd())*vsb.getwinScale())), IPL_DEPTH_64F, 3);
	VisualImage->origin = IPL_ORIGIN_BL;
	
	cvShowImage(Data.Name.c_str(), VisualImage);
	cvSetMouseCallback(Data.Name.c_str(), mouseHandler, (void*)VisualImage);

	view.windowhigh = (float)vsb.getwinFwd() - (float)vsb.getwinBkwd();
	view.gwinX = vsb.getwinX();
	view.gwinBkwd = vsb.getwinBkwd();
	view.g_winScale = vsb.getwinScale();
	view.gwinFwd = vsb.getwinFwd();
	view.initScale = vsb.getwinScale();
	view.g_offsetx = 0.f;
	view.g_offsety = 0.f;
	view.Distanceflag = 0;
	
	vector<float> showx, showy;

	while (1)
	{
		vsb.setwinScale(view.g_winScale);

		vsb.clearImage(VisualImage, CV_RGB(192, 192, 192));

		for (int i = 0; i < Data.Data.size(); i++)
		{
			CloneData(Data.Data[i].x, Data.Data[i].y, showx, showy, view.g_offsetx, view.g_offsety);
			if (Data.Data[i].Style == "Point")
			{
				vsb.showPoints(VisualImage, showx, showy, Data.Data[i].size, CV_RGB(Data.Data[i].R, Data.Data[i].G, Data.Data[i].B), -1);
			}
			else if (Data.Data[i].Style == "Line")
			{
				vsb.showLines(VisualImage, showx, showy, CV_RGB(Data.Data[i].R, Data.Data[i].G, Data.Data[i].B), 1);
			}
			
		}
		
		cvPutText(VisualImage, view.labeldst, cvPoint(20, view.windowhigh - 20), &font, CV_RGB(255, 0, 0));
		cvPutText(VisualImage, view.labeldistance, cvPoint(20, view.windowhigh - 50), &font, CV_RGB(255, 0, 0));

		cvShowImage(Data.Name.c_str(), VisualImage);

		int Keynum = 0;
		Keynum = cvWaitKey(10);
		if (Keynum == 27)
		{
			break;
		}
		else if (Keynum == 68)//D
		{
			view.Distanceflag = 1;
			cout << "Measure Distance" << endl;
		}
	}
}

void CloneData(vector<float>&datax, vector<float>&datay, vector<float>&showx, vector<float>&showy, float offsetx, float offsety)
{
	showx.clear(); showy.clear();
	for (size_t i = 0; i < datax.size(); i++)
	{
		showx.push_back(datax[i] + offsetx);
		showy.push_back(datay[i] + offsety);
	}
}