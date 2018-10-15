//#pragma once

#ifndef _VISUAL_CONF_H_
#define _VISUAL_CONF_H_

#include <windows.h>
#ifndef PI
#define PI				3.14159265358979323846f
#endif
#define deltadisy		-3
#define SLAMDATAMAXLEN	5000
#define GPSMAXLEN		10000
#define SCALE			10								//zoom	
#define OPTREGIONHEIGHT	200								//Output Region: 300 px
#define WINDOWX			30								//x:[-30m, 30m]
#define WINDOWFORWARD	50								//y <= 50m
#define WINDOWBACKWARD	(-25-OPTREGIONHEIGHT/SCALE)		//y >= -25m
#define VW				1.6f								//Vehicle Width 1.7m
#define VL				3.6f								//Vehicle Height 4.2m
#define CDOFFSET		3.1f							//正常坐标原点在车头中间，平移距离，向下为正
// #define DATAXMIN		-50
// #define DATAXMAX		50
// #define DATAYMIN		-20
// #define DATAYMAX		80
// #define MOVINGVX		0.1f
// #define MOVINGVY		0.1f	


class VisualizationConfigure
{
public:
	VisualizationConfigure() 
	{
		severIp = L"127.0.0.1";
		showFreq = 33;
		deltaDisY = 0;
		winX = 30;
		winFwd = 80;
		winBkwd = -20;
		optHeight = 200;
		winScale = 20.f;
	}
	VisualizationConfigure(LPCWSTR _severIp, int _showFreq, int _deltaDisY, int _winX, int _winFwd, int _winBkwd, int _optHeight, float _winScale) 
	{ 
		severIp = _severIp;
		showFreq = _showFreq;
		deltaDisY = _deltaDisY;
		winX = _winX;
		winFwd = _winFwd;
		winBkwd = _winBkwd;
		optHeight = _optHeight;
		winScale = _winScale;
	}
	void setseverIp(LPCWSTR data)	{ severIp = data; }
	void setshowFreq(int data)		{ showFreq = data; }
	void setdeltaDisY(int data)		{ deltaDisY = data; }
	void setwinX(int data)			{ winX = data; }
	void setwinFwd(int data)		{ winFwd = data; }
	void setwinBkwd(int data)		{ winBkwd = data; }
	void setoptHeight(int data)		{ optHeight = data; }
	void setwinScale(float data)		{ winScale = data; }
	LPCWSTR getseverIp() { return severIp; }
	int getshowFreq() { return showFreq; }
	int getdeltaDisY() { return deltaDisY; }
	int getwinX() { return winX; }
	int getwinFwd() { return winFwd; }
	int getwinBkwd() { return winBkwd; }
	int getoptHeight() { return optHeight; }
	float getwinScale() { return winScale; }
	~VisualizationConfigure() {};

protected:
	LPCWSTR severIp;
	int showFreq;
	int deltaDisY;
	int winX;
	int winFwd;
	int winBkwd;
	int optHeight;
	float winScale;

private:
};

// extern 
// extern 
// extern 
// extern 
// extern 
// extern 
// extern 
// extern 

#endif