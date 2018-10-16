#ifndef _FIT_H_
#define _FIT_H_

#include <iostream>
#include <fstream>
#include <vector>
#include <list>
#include <math.h>
#include <time.h>
#include <algorithm>
#include "../include/gsl/gsl_fit.h"
#include "../include/gsl/gsl_multifit.h"
#include "../include/gsl/gsl_poly.h"
#include "../include/opencv/cv.h"
#include "../include/opencv/highgui.h"
#include "../include/client/Packet.h"

#pragma comment(lib, "../lib/gsl/libgsl-0.lib")
#pragma comment(lib, "../lib/gsl/libgslcblas-0.lib")
#pragma comment(lib, "../lib/client/Cocktail.lib")

using namespace std;
extern bool fitPoints2(vector<float>& inx, vector<float>& iny, vector<float>& fitInx, vector<float>& fitIny, int nPoints, int order);
extern bool fitPoints(vector<float>& inx, vector<float>& iny, vector<float>& fitInx, vector<float>& fitIny, int nPoints, int order);
extern void fitPoints(vector<float>& inx, vector<float>& iny, double* c, double& chisq, int length, int order);
extern bool ransacFitPoints(vector<float>& inx, vector<float>& iny, vector<float>& fitInx, vector<float>& fitIny, int nPoints, int order);

#endif 

