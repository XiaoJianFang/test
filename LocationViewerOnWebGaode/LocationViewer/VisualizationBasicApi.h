#ifndef _VISUALIZE_BASIC_H_
#define _VISUALIZE_BASIC_H_

#include <windows.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <math.h>
#include "../include/opencv/cv.h"
#include "../include/opencv/highgui.h"
#include "VisualizationConfigure.h"

//#ifdef _DEBUG 
//#pragma comment(lib, "../lib/cv/x64/opencv_world310d.lib")
//#else
//#pragma comment(lib, "../lib/cv/x64/opencv_world310.lib")
//#endif

using namespace std;

namespace visual
{

	class VisualizationBasicApi : public VisualizationConfigure
	{
	public:
		VisualizationBasicApi() {};
		~VisualizationBasicApi() {};
		void showPoint(IplImage* src, CvPoint2D32f& pt, int radius, CvScalar color, int thickness);
		void showCircle(IplImage* src, CvPoint2D32f& pt, float radius, CvScalar color, int thickness);
		void showLine(IplImage* src, CvPoint2D32f& pt0, CvPoint2D32f& pt1, CvScalar color, int thickness);
		void showPoints(IplImage* src, vector<float>& vecx, vector<float>& vecy, int radius, CvScalar color, int thickness, bool output = false);
		void showPoints(IplImage* src, vector<float>& vecx, vector<float>& vecy, int radius, vector<CvScalar>& veccolor, int thickness);
		void showPoints(IplImage* src, vector<float>& vecx, vector<float>& vecy, int startIndex, int endIndex, int radius, CvScalar color, int thickness);
		void showSonic(IplImage* src, vector<float>& vecx, vector<float>& vecy, int radius, CvScalar color, int thickness, int inter);
		void showLines(IplImage* src, vector<float>& vecx, vector<float>& vecy, CvScalar color, int thickness);
		void showLines(IplImage* src, vector<float>& vecx, vector<float>& vecy, int startIndex, int endIndex, CvScalar color, int thickness);
		void showRectangles(IplImage* src, vector<float>& vecx, vector<float>& vecy, vector<float>& vecw, vector<float>& vecl, CvScalar color, int thickness);
		void showRectangles(IplImage* src, vector<float>& vecx, vector<float>& vecy, vector<float>& vecw, vector<float>& vecl, vector<float>& vecvabx, vector<float>& vecvaby, CvScalar color, int thickness);

		void clearImage(IplImage* src, CvScalar color);
		void showGrid(IplImage* src, int* x, int* y, int lx, int ly, CvScalar colorx, CvScalar colory);
		void showBox(IplImage* src, int w, int l, CvScalar color);
		void showCoordinate(IplImage* src, CvScalar colorx, CvScalar colory);
		void iniOptRegion(IplImage* src);

		void optText(IplImage* src, char* text, CvPoint pos, CvScalar color);
		void optText(IplImage* src, char* text, float xscale, float yscale, int thickness, CvPoint pos, CvScalar color);
		void optText(IplImage* src, char* text, float xscale, float yscale, CvPoint pos, CvScalar color);

		void rotateImage(IplImage* src, IplImage* dst, CvPoint2D32f center, int degree);

	private:

	};

	template<class T1, class T2> /*extern*/ void cvtData2Vector(T1* data, int length, vector<T2>& vec, float a, float b)
	{
		vec.clear();
		T2 _t = 0;
		for (int i = 0; i < length; i++)
		{
			_t = (T2)data[i] * (T2)a + (T2)b;
			vec.push_back(_t);
		}
	}

	template<class T1, class T2> /*extern*/ void cvtData2VectorForVeld(T1* datax, T1* datay, int length, vector<T2>& vecx, vector<T2>& vecy)
	{
		vecx.clear();
		vecy.clear();
		T2 _t = 0;
		for (int i = 0; i < length; i++)
		{
			if (datax[i] != 999 /*|| datax[i] || datay[i]*/)
			{
				_t = (T2)datax[i] * (T2)-1;
				vecx.push_back(_t);
				_t = (T2)datay[i];
				vecy.push_back(_t);
			}
		}
	}

	template<class T1, class T2> /*extern*/ void cvtData2Vector(T1* datax, T1* datay, int length, vector<T2>& vecx, vector<T2>& vecy, float a, float b, bool removeZero)
	{
		vecx.clear();
		vecy.clear();
		T2 _t = 0;
		for (int i = 0; i < length; i++)
		{
			if (!removeZero || datax[i] || datay[i])
			{
				_t = (T2)datax[i] * (T2)a + (T2)b;
				vecx.push_back(_t);
				_t = (T2)datay[i] * (T2)a + (T2)b;
				vecy.push_back(_t);
			}
		}
	}

}



#endif