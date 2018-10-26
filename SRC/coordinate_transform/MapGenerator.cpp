
#include "MapGenerator.h"
#include "../include/client/Packet.h"
#include "../include/Visualization/VisualizationBasicApi.h"
#include "../include/Visualization/VisualizationConfigure.h"
#include <opencv.hpp>
#include  <numeric>
#include "opencv2/video/tracking.hpp" 
#include "opencv2/highgui/highgui.hpp" 
#include "fitting.h"

#include   "datum.h"
#include   "utm.h"
//extern int threshold;
//extern int kalmanflag;

//////************************************/////////
const double EPSINON = 0.0000000000001;
vector <GPS> TargetGps;
vector<odometer> GPSMap;
vector<float> ReserveMousex;
vector<float> ReserveMousey;
RecordIBEO  RectordData;
//文件读写
void OpenFile(std::string& fileName)
{
	TCHAR szBuffer[300] = { 0 };
	OPENFILENAME ofn = { 0 };
	ofn.lStructSize = sizeof(ofn);
	ofn.hwndOwner = NULL;
	ofn.lpstrFilter = "txt(*.txt)\0*.txt\0";
	ofn.lpstrInitialDir = "E:\\Intelligent vehicle\\UGVProject2016\\UGV2016.part\\UGVFinal";
	ofn.lpstrFile = szBuffer;
	ofn.nMaxFile = sizeof(szBuffer) / sizeof(*szBuffer);
	ofn.nFilterIndex = 0;
	ofn.Flags = OFN_PATHMUSTEXIST | OFN_FILEMUSTEXIST | OFN_EXPLORER;
	BOOL bSel = GetOpenFileName(&ofn);
	fileName = std::string(szBuffer);
}
//字符提取
int split(const std::string& str, std::vector<std::string>& ret_, std::string sep = ",")
{
	if (str.empty()) return 0;

	ret_.clear();
	std::string tmp;
	std::string::size_type pos_begin = str.find_first_not_of(sep);
	std::string::size_type comma_pos = 0;
	while (pos_begin != std::string::npos)
	{
		comma_pos = str.find(sep, pos_begin);
		if (comma_pos != std::string::npos)
		{
			tmp = str.substr(pos_begin, comma_pos - pos_begin);
			pos_begin = comma_pos + sep.length();
		}
		else
		{
			tmp = str.substr(pos_begin);
			pos_begin = comma_pos;
		}

		if (!tmp.empty())
		{
			ret_.push_back(tmp);
			tmp.clear();
		}
	}
	return 0;
}
//计算数据点边界
void calcMapWidthandHeight(vector<float>& vecx, vector<float>& vecy, float& maxX, float& maxY, float& minX, float& minY)
{
	if (vecx.size() != vecy.size()) return;
	maxX = -99999999.f;
	maxY = -99999999.f;
	minX = 99999999.f;
	minY = 99999999.f;
	for (size_t i = 0; i < vecx.size(); i++)
	{
		maxX = max(maxX, vecx[i]);
		maxY = max(maxY, vecy[i]);
		minX = min(minX, vecx[i]);
		minY = min(minY, vecy[i]);
	}
}
void calcMapWidthandHeight(
	vector<float>& vecx0, vector<float>& vecy0,
	vector<float>& vecx1, vector<float>& vecy1,
	vector<float>& vecx2, vector<float>& vecy2,
	vector<float>& vecx3, vector<float>& vecy3,
	vector<float>& vecx4, vector<float>& vecy4,
	vector<float>& vecx5, vector<float>& vecy5,
	float& maxX, float& maxY, float& minX, float& minY)
{
	maxX = -99999999.f;
	maxY = -99999999.f;
	minX = 99999999.f;
	minY = 99999999.f;
	for (size_t i = 0; i < vecx0.size(); i++)
	{
		maxX = max(maxX, vecx0[i]);
		maxY = max(maxY, vecy0[i]);
		minX = min(minX, vecx0[i]);
		minY = min(minY, vecy0[i]);
	}
	for (size_t i = 0; i < vecx1.size(); i++)
	{
		maxX = max(maxX, vecx1[i]);
		maxY = max(maxY, vecy1[i]);
		minX = min(minX, vecx1[i]);
		minY = min(minY, vecy1[i]);
	}
	for (size_t i = 0; i < vecx2.size(); i++)
	{
		maxX = max(maxX, vecx2[i]);
		maxY = max(maxY, vecy2[i]);
		minX = min(minX, vecx2[i]);
		minY = min(minY, vecy2[i]);
	}
	for (size_t i = 0; i < vecx3.size(); i++)
	{
		maxX = max(maxX, vecx3[i]);
		maxY = max(maxY, vecy3[i]);
		minX = min(minX, vecx3[i]);
		minY = min(minY, vecy3[i]);
	}
	for (size_t i = 0; i < vecx4.size(); i++)
	{
		maxX = max(maxX, vecx4[i]);
		maxY = max(maxY, vecy4[i]);
		minX = min(minX, vecx4[i]);
		minY = min(minY, vecy4[i]);
	}
	for (size_t i = 0; i < vecx5.size(); i++)
	{
		maxX = max(maxX, vecx5[i]);
		maxY = max(maxY, vecy5[i]);
		minX = min(minX, vecx5[i]);
		minY = min(minY, vecy5[i]);
	}
}
//排序
int  my_cmp(pair<float, float> p1, pair<float, float>  p2)
{
	return p1.second < p2.second;
}
void pairsort(vector <float> &x, vector <float> &y)
{
	if ((!x.size()) || (x.size() != y.size())) return;
	vector<pair<float, float> > line;
	for (int i = 0; i < x.size(); i++)
	{
		line.push_back(make_pair(x[i], y[i]));
	}
	sort(line.begin(), line.end(), my_cmp);
	for (int i = 0; i < x.size(); i++)
	{
		x[i] = line[i].first;
		y[i] = line[i].second;
	}
}
//数据筛选
void inline selectData(vector<float>& vecx, vector<float>& vecy, float xdmin, float xdmax, float ydmin, float ydmax, int maxlength)
{
	int lenx = (int)vecx.size();
	int leny = (int)vecy.size();
	vector<float>::iterator vx, vy;
	if (lenx && lenx == leny)
	{
		if (lenx > maxlength)
		{
			vecx.erase(vecx.begin(), vecx.begin() + lenx - maxlength);
			vecy.erase(vecy.begin(), vecy.begin() + lenx - maxlength);
		}
		for (vx = vecx.begin(), vy = vecy.begin(); vx != vecx.end() && vy != vecy.end();)
		{
			if (*vx < xdmin || *vx > xdmax || *vy < ydmin || *vy > ydmax || (!*vx || !*vy))
			{
				vx = vecx.erase(vx);
				vy = vecy.erase(vy);
			}
			else
			{
				++vx;
				++vy;
			}
		}
	}
	else if (lenx)
		return;
}
void inline selectData2(vector<float>& vecx, vector<float>& vecy, float xdmin, float xdmax, float ydmin, float ydmax, int maxlength)
{
	vector<CvPoint2D32f> ctrpts;
	for (int i = 0; i < vecx.size(); ++i)
		ctrpts.push_back(cvPoint2D32f(vecx[i], vecy[i]));
	sort(ctrpts.begin(), ctrpts.end(), less_y);
	vector<CvPoint2D32f>::iterator vpt, tvpt = ctrpts.begin();
	for (vpt = ctrpts.begin(); vpt != ctrpts.end(); ++vpt)
	{
		if ((*vpt).y < ydmin)
			tvpt = vpt;
		else
			break;
	}
	ctrpts.erase(ctrpts.begin(), tvpt);
	tvpt = ctrpts.begin();
	for (vpt = ctrpts.begin(); vpt != ctrpts.end(); ++vpt)
	{
		if ((*vpt).y > ydmax)
			break;
		tvpt = vpt;
	}
	ctrpts.erase(tvpt, ctrpts.end());
	sort(ctrpts.begin(), ctrpts.end(), less_x);
	tvpt = ctrpts.begin();
	for (vpt = ctrpts.begin(); vpt != ctrpts.end(); ++vpt)
	{
		if ((*vpt).x < xdmin)
			tvpt = vpt;
		else
			break;
	}
	ctrpts.erase(ctrpts.begin(), tvpt);
	tvpt = ctrpts.begin();
	for (vpt = ctrpts.begin(); vpt != ctrpts.end(); ++vpt)
	{
		if ((*vpt).x > xdmax)
			break;
		tvpt = vpt;
	}
	ctrpts.erase(tvpt, ctrpts.end());
	vecx.clear();
	vecy.clear();
	for (int i = 0; i < ctrpts.size(); ++i)
	{
		vecx.push_back(ctrpts[i].x);
		vecy.push_back(ctrpts[i].y);
	}
}
//栅格化地图
void rasterizeData(vector<float>& vecx, vector<float>& vecy, float scale)
{
	if (vecx.empty())
		return;
	vector<CvPoint2D32f> ctrpts;
	for (int i = 0; i < vecx.size(); ++i)
		ctrpts.push_back(cvPoint2D32f(vecx[i], vecy[i]));
	sort(ctrpts.begin(), ctrpts.end(), less_yx32f);
	if (scale == 1000)
		ctrpts.erase(unique(ctrpts.begin(), ctrpts.end(), equpt100), ctrpts.end());
	else if (scale == 100)
		ctrpts.erase(unique(ctrpts.begin(), ctrpts.end(), equpt100), ctrpts.end());
	else if (scale == 10)
		ctrpts.erase(unique(ctrpts.begin(), ctrpts.end(), equpt10), ctrpts.end());
	else if (scale == 1)
		ctrpts.erase(unique(ctrpts.begin(), ctrpts.end(), equpt1), ctrpts.end());
	else
		ctrpts.erase(unique(ctrpts.begin(), ctrpts.end(), equpt10), ctrpts.end());

	vecx.clear();
	vecy.clear();
	for (size_t i = 0; i < ctrpts.size(); ++i)
	{
		vecx.push_back(ctrpts[i].x);
		vecy.push_back(ctrpts[i].y);
	}
}
void rasterizeData(vector<float>& vecx, vector<float>& vecy, float scale, int threshold)
{
	//static int vote = 1;
	if (vecx.empty())
		return;
	vector<CvPoint2D32f> ctrpts;
	vector<int> vote;
	int voteCount = 1;
	for (int i = 0; i < vecx.size(); ++i)
		ctrpts.push_back(cvPoint2D32f(vecx[i], vecy[i]));
	
	 if (scale == 1000)
		sort(ctrpts.begin(), ctrpts.end(), less_yx1000);
	else if (scale == 100)
		sort(ctrpts.begin(), ctrpts.end(), less_yx100);
	else if (scale == 10)
		sort(ctrpts.begin(), ctrpts.end(), less_yx10);
	else if (scale == 1)
		sort(ctrpts.begin(), ctrpts.end(), less_yx1);
	else
		sort(ctrpts.begin(), ctrpts.end(), less_yx10);

	for (vector<CvPoint2D32f>::iterator it = ctrpts.begin(); it != (ctrpts.end() - 1); it++)
	{
		if ((int((*it).x*scale) == int((*(it + 1)).x*scale)) && (int((*it).y*scale) == int((*(it + 1)).y*scale)))
			voteCount++;
		else
		{
			vote.push_back(voteCount);
			voteCount = 1;
		}
	}
	if (vote.size() > 3)
	{
		if ((int(ctrpts[ctrpts.size() - 2].x*scale) == int(ctrpts[ctrpts.size() - 1].x*scale))
			&& (int(ctrpts[ctrpts.size() - 2].y*scale) == int(ctrpts[ctrpts.size() - 1].y*scale)))
		{
			vote.push_back(1);

		}

	}

	if (scale == 1000)
		ctrpts.erase(unique(ctrpts.begin(), ctrpts.end(), equpt100), ctrpts.end());
	else if (scale == 100)
		ctrpts.erase(unique(ctrpts.begin(), ctrpts.end(), equpt100), ctrpts.end());
	else if (scale == 10)
		ctrpts.erase(unique(ctrpts.begin(), ctrpts.end(), equpt10), ctrpts.end());
	else if (scale == 1)
		ctrpts.erase(unique(ctrpts.begin(), ctrpts.end(), equpt1), ctrpts.end());

	else
		ctrpts.erase(unique(ctrpts.begin(), ctrpts.end(), equpt10), ctrpts.end());

	//	cout << "voteSize:" << vote.size() << endl;
	//	cout << "uniqueSize:" << ctrpts.size() << endl;

	vector<CvPoint2D32f>::iterator it_ctrpts = ctrpts.begin();
	for (vector<int>::iterator it_vote = vote.begin(); it_vote != vote.end();)
	{

		if (*it_vote < threshold)
		{
			it_vote = vote.erase(it_vote);
			it_ctrpts = ctrpts.erase(it_ctrpts);
		}
		else
		{
			it_vote++;
			it_ctrpts++;
		}
	}

	vecx.clear();
	vecy.clear();
	for (size_t i = 0; i < ctrpts.size(); ++i)
	{
		vecx.push_back(ctrpts[i].x);
		vecy.push_back(ctrpts[i].y);
	}
}
void rasterizeDataGrid(vector<float>& vecx, vector<float>& vecy, float scale, int threshold)
{
	//static int vote = 1;
	if (vecx.empty())
		return;
	vector<CvPoint2D32f> ctrpts;
	vector<int> vote;
	vector<vector<float>> cloudx;
	vector<vector<float>> cloudX;
	vector<float> cloudy;
	int voteCount = 1;
	for (int i = 0; i < vecx.size(); ++i)
		ctrpts.push_back(cvPoint2D32f(vecx[i], vecy[i]));
	
	if (scale == 5000)
		sort(ctrpts.begin(), ctrpts.end(), less_yx5000);
	else if (scale == 2000)
		sort(ctrpts.begin(), ctrpts.end(), less_yx2000);
	else if (scale == 1000)
		sort(ctrpts.begin(), ctrpts.end(), less_yx1000);
	else if (scale == 100)
		sort(ctrpts.begin(), ctrpts.end(), less_yx100);
	else if (scale == 60)
		sort(ctrpts.begin(), ctrpts.end(), less_yx60);
	else if (scale == 40)
		sort(ctrpts.begin(), ctrpts.end(), less_yx40);
	else if (scale == 20)
		sort(ctrpts.begin(), ctrpts.end(), less_yx20);
	else if (scale == 10)
		sort(ctrpts.begin(), ctrpts.end(), less_yx10);
	else if (scale == 5)
		sort(ctrpts.begin(), ctrpts.end(), less_yx5);
	else if (scale == 1)
		sort(ctrpts.begin(), ctrpts.end(), less_yx1);
	else
		sort(ctrpts.begin(), ctrpts.end(), less_yx10);

	float scale_;
	
	scale_ =scale/100.0;
	size_t NN = 0;
	for (vector<CvPoint2D32f>::iterator it = ctrpts.begin(); it != (ctrpts.end() - 1); it++)
	{
		vector<float> pointx, pointy;
		int flag = 0;
		while ((it != (ctrpts.end() - 2))&&(int((*it).y / scale_) == int((*(it + 1)).y / scale_)))
		{
			flag = 1;
			pointx.push_back((*it).x);
			pointy.push_back((*it).y);
			it++;
		}
		
		pointx.push_back((*it).x);
		pointy.push_back((*it).y);

		if (it == (ctrpts.end() - 2))
		{
			if (int((*it).y / scale_) == int((*(it + 1)).y / scale_))
			{
				pointx.push_back((*(it+1)).x);
				pointy.push_back((*(it+1)).y);
			}
		}
		pairsort(pointy, pointx);
		
		
		cloudx.push_back(pointx);
		cloudy.push_back(int((*it).y / scale_)*scale_);
		size_t jj = 0;
		while (pointx.size()>0)
		{
			(*(it- jj)).x = pointx.back();
			jj++;
			pointx.pop_back();
			NN++;
		}
		if (it == (ctrpts.end() - 2))
		{
			if (int((*it).y / scale_) != int((*(it + 1)).y / scale_))
			{
				pointx.clear();
				pointx.push_back((*(it + 1)).x);
				cloudx.push_back(pointx);
				cloudy.push_back(int((*(it+1)).y / scale_)*scale_);
				NN++;
			}
		}
		
	}
   int 	VoteCount = 1;
   vector<int> Vote;
		size_t index = 0;
		size_t cloudnum = ctrpts.size();
		for (size_t i = 0; i < cloudy.size(); i++)
		{
			vector<float> X;
			vector<float> XX;
			X = cloudx[i];
			for (size_t j = 0; j < X.size()-1; j++)
			{
				index++;
				
				if  (int(X[j] / scale_) == int(X[j + 1] / scale_))
				{
					VoteCount++;
					if (j == (X.size() - 2))
					{
						index++;
						VoteCount++;
					}
				}
				else
				{
					if (VoteCount>=threshold)
					{
						XX.push_back(int(X[j] / scale_)*scale_);
					}
					else
					{
						while (VoteCount > 0)
						{
							if ((index - VoteCount) != (cloudnum - 1))
							{
								ctrpts.erase(ctrpts.begin()+index - VoteCount);
								index--;
								VoteCount--;
							}
							
						}
					}
					VoteCount = 1;
					if (j == (X.size() - 2)) 
					{
						index++;
						if (index <= (cloudnum - 1))
						{
							ctrpts.erase(ctrpts.begin() + index);
							index--;
						}

					}
				}
				
			}
			
			cloudX.push_back(XX);
		}
	
		vecx.clear();
		vecy.clear();
		for (size_t i = 0; i < cloudy.size(); i++)
		{
			vector <float> x;
			x = cloudX[i];
			for (size_t j = 0; j < x.size(); j++)
			{
				vecx.push_back(x[j]);
				vecy.push_back(cloudy[i]);
			}
		}
}
void rasterizeData2(vector<float>& vecx, vector<float>& vecy, vector<float>& rasterizedvecx, vector<float>& rasterizedvecy, float scalex, float scaley)
{
	if (vecx.empty())
		return;
	vector<CvPoint> ctrpts;
	for (int i = 0; i < vecx.size(); ++i)
		ctrpts.push_back(cvPoint((int)(vecx[i] * scalex), (int)(vecy[i] * scaley)));
	sort(ctrpts.begin(), ctrpts.end(), less_yx);
	ctrpts.erase(unique(ctrpts.begin(), ctrpts.end(), equpt), ctrpts.end());
	rasterizedvecx.clear();
	rasterizedvecy.clear();
	for (size_t i = 0; i < ctrpts.size(); ++i)
	{
		rasterizedvecx.push_back((float)ctrpts[i].x / scalex);
		rasterizedvecy.push_back((float)ctrpts[i].y / scaley);
	}
}
//去除噪声
void rasterizeDataNoise(vector<float>& vecx, vector<float>& vecy, float scale, int threshold)
{
	//static int vote = 1;
	if (vecx.empty())
		return;
	vector<CvPoint2D32f> ctrpts;
	vector<int> vote;
	vector<vector<float>> cloudx;
	vector<vector<float>> cloudX;
	vector<float> cloudy;
	int voteCount = 1;
	for (int i = 0; i < vecx.size(); ++i)
		ctrpts.push_back(cvPoint2D32f(vecx[i], vecy[i]));
	if (scale == 1000)
		sort(ctrpts.begin(), ctrpts.end(), less_yx100);
	else if (scale == 100)
		sort(ctrpts.begin(), ctrpts.end(), less_yx100);
	else if (scale == 60)
		sort(ctrpts.begin(), ctrpts.end(), less_yx60);
	else if (scale == 40)
		sort(ctrpts.begin(), ctrpts.end(), less_yx40);
	else if (scale == 20)
		sort(ctrpts.begin(), ctrpts.end(), less_yx20);
	else if (scale == 10)
		sort(ctrpts.begin(), ctrpts.end(), less_yx10);
	else if (scale == 5)
		sort(ctrpts.begin(), ctrpts.end(), less_yx5);
	else if (scale == 1)
		sort(ctrpts.begin(), ctrpts.end(), less_yx1);
	else
		sort(ctrpts.begin(), ctrpts.end(), less_yx10);

	float scale_;

	scale_ = scale / 100.0;
	size_t NN = 0;
	for (vector<CvPoint2D32f>::iterator it = ctrpts.begin(); it != (ctrpts.end() - 1); it++)
	{
		vector<float> pointx, pointy;
		int flag = 0;
		while ((it != (ctrpts.end() - 2)) && (int((*it).y / scale_) == int((*(it + 1)).y / scale_)))
		{
			flag = 1;
			pointx.push_back((*it).x);
			pointy.push_back((*it).y);
			it++;
		}

		pointx.push_back((*it).x);
		pointy.push_back((*it).y);

		if (it == (ctrpts.end() - 2))
		{
			if (int((*it).y / scale_) == int((*(it + 1)).y / scale_))
			{
				pointx.push_back((*(it + 1)).x);
				pointy.push_back((*(it + 1)).y);
			}
		}
		pairsort(pointy, pointx);


		cloudx.push_back(pointx);
		cloudy.push_back(int((*it).y / scale_)*scale_);
		size_t jj = 0;
		while (pointx.size()>0)
		{
			(*(it - jj)).x = pointx.back();
			jj++;
			pointx.pop_back();
			NN++;
		}
		if (it == (ctrpts.end() - 2))
		{
			if (int((*it).y / scale_) != int((*(it + 1)).y / scale_))
			{
				pointx.clear();
				pointx.push_back((*(it + 1)).x);
				cloudx.push_back(pointx);
				cloudy.push_back(int((*(it + 1)).y / scale_)*scale_);
				NN++;
			}
		}

	}
	int 	VoteCount = 1;
	vector<int> Vote;
	size_t index = 0;
	size_t cloudnum = ctrpts.size();
	for (size_t i = 0; i < cloudy.size(); i++)
	{
		vector<float> X;
		vector<float> XX;
		X = cloudx[i];
		for (size_t j = 0; j < X.size() - 1; j++)
		{
			index++;

			if (int(X[j] / scale_) == int(X[j + 1] / scale_))
			{
				VoteCount++;
				if (j == (X.size() - 2))
				{
					index++;
					VoteCount++;
				}
			}
			else
			{
				if (VoteCount >= threshold)
				{
					XX.push_back(int(X[j] / scale_)*scale_);
				}
				else
				{
					while (VoteCount > 0)
					{
						if ((index - VoteCount) != (cloudnum - 1))
						{
							ctrpts.erase(ctrpts.begin() + index - VoteCount);
							index--;
							VoteCount--;
						}

					}
				}
				VoteCount = 1;
				if (j == (X.size() - 2))
				{
					index++;
					if (index <= (cloudnum - 1))
					{
						ctrpts.erase(ctrpts.begin() + index);
						index--;
					}

				}
			}

		}

		cloudX.push_back(XX);
	}

	vecx.clear();
	vecy.clear();
	for (size_t i = 0; i < ctrpts.size(); i++)
	{
		vecx.push_back(ctrpts[i].x);
		vecy.push_back(ctrpts[i].y);
	}
}
void RasterizeibeoMapData(mapdata& Map, int resolution, int threshold)
{
	rasterizeDataGrid(Map.ibeox, Map.ibeoy, resolution, threshold);
}
void RasterizelaneMapData(mapdata& Map, int resolution, int threshold)
{
	rasterizeDataGrid(Map.l4x, Map.l4y, resolution, threshold);
}
//数据存储
void saveData(std::ofstream& out, vector<float>& vecx, vector<float>& vecy, int precision, std::string atrribute)
{
	if (vecx.empty() && vecy.empty())
	{
		out << "[" + atrribute + "]\t" << "99";
		out << std::endl;
		out << "[" + atrribute + "]\t" << "99";
		out << std::endl;
	}
	else
	{
		out << setiosflags(ios::fixed) << setprecision(precision);
		out << "[" + atrribute + "]\t";
		for (size_t i = 0; i < vecx.size(); i++)
			out << vecx[i] << "\t";
		out << std::endl << "[" + atrribute + "]\t";
		for (size_t i = 0; i < vecx.size(); i++)
			out << vecy[i] << "\t";
		out << std::endl;
	}
}
void saveGPSData(std::ofstream& out, long double lat, long double lon, long double heading, float x, float y)
{
	out << setiosflags(ios::fixed) << setprecision(15);
	out << 0 << "\t";
	out << "[GPS]\t" << 0 << "\t" << lat << "\t" << lon << "\t" << heading << "\t" << setprecision(3);
	out << x << "\t" << y << "\t" << 0 << "\t" << 0 << "\t" << std::endl;
}
void saveGPSData(std::ofstream& out, GPS& gps)
{
	out << setiosflags(ios::fixed) << setprecision(15);
	out << gps.flag << "\t";
	out << "[GPS]\t" << 0 << "\t" << gps.lat << "\t" << gps.lon << "\t" << gps.head << "\t" << setprecision(3);
	out << gps.x << "\t" << gps.y << "\t" << 0 << "\t" << 0 << "\t" << std::endl;
}
void saveGPSRoadData(std::ofstream& out, int flag, long double lat, long double lon, long double heading)
{
	lat = lat * (long double)180 / (long double)PI64;
	lon = lon * (long double)180 / (long double)PI64;
	heading = heading * (long double)180 / (long double)PI64;
	out << setiosflags(ios::fixed) << setprecision(15);
	out << flag << "\t";
	out << "[GPS]\t" << 0 << "\t" << lat << "\t" << lon << "\t" << heading << "\t";
	out << 0 << "\t" << 0 << "\t" << 0 << "\t" << 0 << "\t" << std::endl;
}
void saveGPSRoadData(std::ofstream& out, int flag, long double lat, long double lon, long double heading, float x, float y)
{
	lat = lat * (long double)180 / (long double)PI64;
	lon = lon * (long double)180 / (long double)PI64;
	heading = heading * (long double)180 / (long double)PI64;
	out << setiosflags(ios::fixed) << setprecision(15);
	out << flag << "\t";
	out << "[GPS]\t" << 0 << "\t" << lat << "\t" << lon;
	out<<setprecision(2)<< "\t" << heading << "\t";
	out << x << "\t" << y << "\t" << 0 << "\t" << 0 << "\t" << std::endl;
}
void saveDataVELD(std::ofstream& out, vector<float>& vecx, vector<float>& vecy, int precision, std::string atrribute)
{
	if (vecx.empty() && vecy.empty())
	{
		out << 0 << "\t";
		out << "[" + atrribute + "]\t" << "99";
		out << std::endl;
	}
	else
	{
		out << 0 << "\t";
		out << setiosflags(ios::fixed) << setprecision(precision);
		out << "[" + atrribute + "]\t";
		for (size_t i = 0; i < vecx.size(); i++)
			out << vecx[i] << "\t";
		out << std::endl;
	}
}
void saveDataFlag(std::ofstream& out, vector<int>& vecx, std::string atrribute)
{
	if (vecx.empty())
	{
		out << 0 << "\t";
		out << "[" + atrribute + "]\t" << "0";
		out << std::endl;
	}
	else
	{
		out << 0 << "\t";
		out << "[" + atrribute + "]\t";
		for (size_t i = 0; i < vecx.size(); i++)
			out << vecx[i] << "\t";
		out << std::endl;
	}
}
void saveDataVELD2(std::ofstream& out, vector<float>& vecx, vector<float>& vecy, int precision, std::string atrribute)
{
	if (vecx.empty() && vecy.empty())
	{
		out << 0 << "\t";
		out << setiosflags(ios::fixed) << setprecision(precision);
		out << "[" + atrribute + "]\t";
		for (size_t i = 0; i < LENGTHOFLANE; i++)
			out << 999 << "\t";
		out << std::endl;

		out << 0 << "\t";
		out << setiosflags(ios::fixed) << setprecision(precision);
		out << "[" + atrribute + "]\t";
		for (size_t i = 0; i <LENGTHOFLANE; i++)
			out << 999 << "\t";
		out << std::endl;
	}
	else
	{
		out << 0 << "\t";
		out << setiosflags(ios::fixed) << setprecision(precision);
		out << "[" + atrribute + "]\t";
		for (size_t i = 0; i < vecx.size(); i++)
			out << vecx[i] << "\t";
		out << std::endl;

		out << 0 << "\t";
		out << setiosflags(ios::fixed) << setprecision(precision);
		out << "[" + atrribute + "]\t";
		for (size_t i = 0; i < vecy.size(); i++)
			out << vecy[i] << "\t";
		out << std::endl;
	}
}
void saveDataIBEO(std::ofstream& out, vector<float>& vecx, vector<float>& vecy, int precision, std::string atrribute)
{
	if (vecx.empty() && vecy.empty())
	{
		for (int i = 0; i < 11; i++)
		{
			out << 0 << "\t";
			out << "[" + atrribute + "]\t" << "9999";
			out << std::endl;
		}
	}
	else
	{
		for (int i = 0; i < 9; i++)
		{
			out << 0 << "\t";
			out << "[" + atrribute + "]\t" << "9999";
			out << std::endl;
		}
		out << 0 << "\t";
		out << setiosflags(ios::fixed) << setprecision(precision);
		out << "[" + atrribute + "]\t";
		for (size_t i = 0; i < vecx.size(); i++)
			out << vecx[i] << "\t";
		out << std::endl;

		out << 0 << "\t";
		out << "[" + atrribute + "]\t";
		for (size_t i = 0; i < vecx.size(); i++)
			out << vecy[i] << "\t";
		out << std::endl;
	}
}
void saveOccupyGridData(std::ofstream& out, vector<float>& vecx, vector<float>& vecy, vector<int>& style, float scale)
{
	string x = "[X]\t";
	string y = "[Y]\t";
	string value = "[STYLE]\t";

	out << x;
	for (size_t i = 0; i < vecx.size(); i++)
		out << int(vecx[i] / scale) << "\t";
	out << std::endl;

	out << y;
	for (size_t i = 0; i < vecx.size(); i++)
		out << (int)(vecy[i] / scale) << "\t";
	out << std::endl;

	out << value;
	for (size_t i = 0; i < vecx.size(); i++)
		out << style[i] << "\t";
	out << std::endl;
}
//坐标变换
void inline CalculateVehicleState(long double lat, long double l_lat, long double lon, long double l_lon, long double head, long double l_head, float& x0, float& y0, float& dhead)
{
	long double dis = 6378.140 * 1000 * 2 * asin(sqrt((sin((lat - l_lat) / 2)) * (sin((lat - l_lat) / 2)) + cos(lat) * cos(l_lat) * (sin((lon - l_lon) / 2)) * (sin((lon - l_lon) / 2))))/** 10000 / 10000*/;
	long double angle = atan2((lon - l_lon)*cos(l_lat), lat - l_lat);
	if (angle > PI64)
		angle = angle - 2 * PI64;
	long double dheadangle = l_head - angle;
	dhead = (float)(head - l_head);

	if (dhead >= PI)
		dhead = dhead - PI * 2;
	else if (dhead < -PI)
		dhead = dhead + PI * 2;

	if (dheadangle >= PI)
		dheadangle = dheadangle - 2 * PI;
	else if (dheadangle >= PI / 2 && dheadangle < PI)
		dheadangle = dheadangle - PI;
	else if (dheadangle < -PI / 2 && dheadangle >= -PI)
		dheadangle = dheadangle + PI;
	else if (dheadangle < -PI)
		dheadangle = dheadangle + 2 * PI;

	x0 = (float)(-dis * sin(dheadangle));
	y0 = (float)(dis * cos(dheadangle));
}
void inline CalculateSensorData(vector<float>& vecx, vector<float>& vecy, double x0, double y0, float dhead)
{
	for (int i = 0; i < vecx.size(); i++)
	{
		float _vecx = vecx[i];
		float _vecy = vecy[i];
		vecx[i] = (float)((_vecx - x0) * cos(dhead) - (_vecy - y0) * sin(dhead));
		vecy[i] = (float)((_vecx - x0) * sin(dhead) + (_vecy - y0) * cos(dhead));
	}
}
void inline CalculateSensorData(float& vecx, float& vecy, float x0, float y0, float dhead)
{
	float _vecx = vecx;
	float _vecy = vecy;
	vecx = (_vecx - x0) * cos(dhead) - (_vecy - y0) * sin(dhead);
	vecy = (_vecx - x0) * sin(dhead) + (_vecy - y0) * cos(dhead);
}
float ComputeGPSDistance( long double lat1, long double lon1, long double lat2, long double lon2)
{
	float dis;
	dis = (float)(6378.140 * 1000 * 2 * asin(sqrt((sin((lat1 - lat2) / 2))*(sin((lat1 - lat2) / 2)) + cos(lat1)*
		cos(lat2)*(sin((lon1 - lon2) / 2))*(sin((lon1 - lon2) / 2)))));
	dis = (float)floor(dis * 10000 + 0.5) / 10000;
	return dis;
}
float Computedistance(float x1, float y1, float x2, float y2)
{
	return sqrt((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2));
}
void  Global2_Local(double &goalPx, double &goalPy, /*目标点GPS*/long double latGoal, long double lonGoal, /*当前GPS*/long double lat, long double lon, long double head)
{
	double angle;
	float dis, disla, dislo;
	dis=ComputeGPSDistance(latGoal, lonGoal, lat, lon);
	disla=ComputeGPSDistance(latGoal, lonGoal, lat, lonGoal);
	dislo=ComputeGPSDistance(lat, lonGoal, lat, lon);

	if ((lon - lonGoal) > EPSINON)
	{
		if ((lat - latGoal) > EPSINON)
		{
			angle = 180 * atan((dislo) / (disla)) / PI;
		}
		else if ((lat - latGoal) < (-EPSINON))
		{
			angle = (180 + 180 * atan((dislo) / (-disla)) / PI);
		}
		else
		{
			angle = 90;
		}
	}
	else if ((lon - lonGoal)<(-EPSINON))
	{
		if ((lat - latGoal)>EPSINON)
		{
			angle = 180 * atan((-dislo) / (disla)) / PI;
		}
		else if ((lat - latGoal) < (-EPSINON))
		{
			angle = (-180 + 180 * atan((dislo) / (disla)) / PI);
		}
		else
			angle = -90;
	}
	else
	{
		//angle = 0;
		if ((lat - latGoal)>EPSINON)
		{
			angle = 0;
		}
		else if ((lat - latGoal) < (-EPSINON))
		{
			angle = 180;
		}
		else
			angle = 0;
	}
	double dHeadAngle;
	angle = angle / (long double)180 * PI;
	dHeadAngle = (angle - head);
	goalPx = -1 * dis*sin(dHeadAngle);
	goalPy = -1 * dis*cos(dHeadAngle);
}
void  CalculateCoordinate(double&x, double&y, GPS currentGPS)
{
	static odometer GPSFrame;
	static int startflag = 0;
	double tempx = 0, tempy = 0, dhead = 0;

	if (!startflag)
	{
		x = 0;
		y = 0;
		GPSFrame.gps.push_back(currentGPS);
		GPSFrame.x.push_back((float)x);
		GPSFrame.y.push_back((float)y);
		startflag++;
	}
	else
	{
		Global2_Local(tempx, tempy, currentGPS.lat, currentGPS.lon, GPSFrame.gps.back().lat, GPSFrame.gps.back().lon, GPSFrame.gps.back().head);

		dhead = GPSFrame.gps.back().head - GPSFrame.gps[0].head;
		float x0, y0;
		x0 = (float)tempx;  y0 = (float)tempy;
		CalculateSensorData(x0, y0, 0.f, 0.f, (float)(-dhead));//旋转
		x = GPSFrame.x.back() + x0;
		y = GPSFrame.y.back() + y0;
		if (fabs(tempx) > 1 || fabs(tempy) > 1)
		{
			GPSFrame.gps.push_back(currentGPS);
			GPSFrame.x.push_back((float)x);
			GPSFrame.y.push_back((float)y);
		}
	}

}
void  ChangeMapCoordinate(GPS gps, mapdata& map, mapdata& newmap)
{
	size_t indexzero = 0;
	float x0 = 0, y0 = 0, dhead = 0;
	double x, y;
	Global2_Local(x, y, gps.lat, gps.lon, map.gps[0].lat, map.gps[0].lon, map.gps[0].head);

	x0 = x - map.x[indexzero];
	y0 = y - map.y[indexzero];
	dhead = (float)(gps.head - map.gps[indexzero].head);
	newmap = map;
	CalculateSensorData(newmap.x, newmap.y, x0, y0, dhead);
	CalculateSensorData(newmap.ibeox, newmap.ibeoy, x0, y0, dhead);
	CalculateSensorData(newmap.l4x, newmap.l4y, x0, y0, dhead);
}
//kmean聚类
void kmeansdeal(vector<float>& linex, vector<float>& liney, vector<float>& linexsave, vector<float>& lineysave)
{
	const int MAX_CLUSTERS = 15;

	cv::Mat img(600, 300, CV_8UC3);
	int clusterCount = 1;
	int sampleCount = (int)(linex.size() - 1);
	cv::Mat points(sampleCount, 1, CV_32FC2), labels;
	cv::Mat centers;
	if (linex.size() < 10)
	{
		return;
	}

	for (int i = 0; i < linex.size(); i++)
	{
		cv::Point p;
		p.x = (int)linex[i];
		p.y = (int)liney[i];
		points.at<cv::Point2f>(i) = p;
	}
	kmeans(points, clusterCount, labels,
		cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 100, 1.0),
		3, cv::KMEANS_PP_CENTERS, centers);

	vector <classpoint> mapveld(clusterCount);
	img = cv::Scalar::all(0);
	for (int i = 0; i < sampleCount; i++)
	{
		int clusterIdx = labels.at<int>(i);
		int clusterIdx_ = clusterIdx;
		cv::Point ipt = points.at<cv::Point2f>(i);
		ipt.x += 100;
		ipt.y += 300;
		//circle(img, ipt, 1, colorTab[clusterIdx_], cv::FILLED, cv::LINE_4);
		mapveld[clusterIdx].x.push_back((float)(ipt.x - 100));
		mapveld[clusterIdx].y.push_back((float)(ipt.y - 300));
	}
	imshow("clusters", img);

	vector <float> fitx, fity;
	vector <float> lines;
	int veldLineOrder = 3;
	cv::Mat img2 = img;

	for (int i = 0; i < clusterCount; i++)
	{
		bool ret2 = fitPoints(mapveld[i].x, mapveld[i].y, fitx, fity, 10, veldLineOrder);
		for (int i = 0; i < fitx.size(); i++)
		{
			cv::Point ipt;
			ipt.x = (int)fitx[i] + 100; 
			ipt.y = (int)fity[i] + 300;
			//circle(img2, ipt, 1, colorTab[0], cv::FILLED, cv::LINE_4);
		}
		linexsave.insert(linexsave.end(), fitx.begin(), fitx.end());
		lineysave.insert(lineysave.end(), fity.begin(), fity.end());
		fitx.clear();
		fity.clear();

	}
	imshow("fit", img2);
	//waitKey(1);
}
//偏移操作
void xOffset(vector<float>& vecx, float offset)
{
	for (size_t i = 0; i < vecx.size(); i++)
	{
		vecx[i] += offset;
	}
}
void OffSET(vector<float> x, vector<float>&off, int offset)
{
	for (size_t i = 0; i < x.size(); i++)
	{
		off.push_back(x[i] + offset);
	}
}
void OffSET(vector<float> x, vector<float>&off, float offset)
{
	for (size_t i = 0; i < x.size(); i++)
	{
		off.push_back(x[i] + offset);
	}
}
void OffSET(vector<double> x, vector<double>&off, int offset)
{
	for (size_t i = 0; i < x.size(); i++)
	{
		off.push_back(x[i] + offset);
	}
}
//滤波算法
void KalmanFilter(vector<float>&x, vector<float>&y)
{
	if (x.size()<10 || (x.size() != y.size())) return;
	vector<float> filterx, filtery;
	//1.kalman filter setup  
	const int stateNum = 4;    //状态值4×1向量(x,y,△x,△y)  
	const int measureNum = 2;   //测量值2×1向量(x,y)    
	cv::KalmanFilter KF(stateNum, measureNum, 0);

	KF.transitionMatrix = (cv::Mat_<float>(4, 4) << 1, 0, 1, 0, 0, 1, 0, 1, 0, 0, 1, 0, 0, 0, 0, 1);  //转移矩阵A  
	cv::setIdentity(KF.measurementMatrix);                     //测量矩阵H  
	cv::setIdentity(KF.processNoiseCov, cv::Scalar::all(1e-15));  //系统噪声方差矩阵Q  
	cv::setIdentity(KF.measurementNoiseCov, cv::Scalar::all(1e-13));   //测量噪声方差矩阵R  
	cv::setIdentity(KF.errorCovPost, cv::Scalar::all(1));            //后验错误估计协方差矩阵P  
	
	KF.statePost.at<float>(0) = x[0];
	KF.statePost.at<float>(1) = y[0];

	filterx.push_back(x[0]);
	filtery.push_back(y[0]);
	cv::Mat measurement = cv::Mat::zeros(measureNum, 1, CV_32F);      //初始测量值x'(0)，因为后面要更新这个值，所以必须先定义  

	for (int ii = 1; ii < x.size(); ii++)
	{
		cv::Mat prediction = KF.predict();
		cv::Point predict_pt = cv::Point(prediction.at<int>(0), prediction.at<int>(1));   //预测值(x',y')  
		measurement.at<float>(0) = x[ii];
		measurement.at<float>(1) = y[ii];
		KF.correct(measurement);
		filterx.push_back(KF.statePost.at<float>(0));
		filtery.push_back(KF.statePost.at<float>(1));
	}
	x = filterx;
	y = filtery;
}
void MeanFilter(vector<float>&x, vector<float>&y)
{
	vector <float> seedsx_, seedsy_;
	int filternum = 5;

	if (x.size() <=(2*filternum)) return;
	
	for (int i = 0; i < filternum; i++)
	{
		seedsx_.push_back(x[i]);
		seedsy_.push_back(y[i]);
	}
	for (int i = filternum; i < x.size() - filternum; i++)
	{
		float sumx = 0, sumy = 0;
		for (int j = i -filternum; j < i + filternum; j++)
		{
			sumx += x[j];
			sumy += y[j];
		}
		seedsx_.push_back((float)(0.5*sumx / filternum));
		seedsy_.push_back((float)(0.5*sumy / filternum));
	}
	for (int i = (int)x.size() - filternum; i < x.size(); i++)
	{
		seedsx_.push_back(x[i]);
		seedsy_.push_back(y[i]);
	}
	x = seedsx_;
	y = seedsy_;
}
//建图
void Buildmap2(mapdata& Map, string dir)
{
	static int mapnumber;
	PacketVELD m_PVeld;
	PacketRoad m_PRoad;
	PacketTraffic m_PTraffic;
	PacketStopLine m_PStopLine;
	PacketIbeo m_PIbeo;
	visual::VisualizationBasicApi vsb;
	vector <GPS> gpsdata;
	vector <float> GPSfliterheading;
	float t_raw_lane1y[LENGTHOFLANE] = { 0.0f };
	float t_raw_lane2y[LENGTHOFLANE] = { 0.0f };
	float t_raw_lane3y[LENGTHOFLANE] = { 0.0f };
	float t_raw_lane4y[LENGTHOFLANE] = { 0.0f };

	long double raw_lat = 0; long double raw_lon = 0; long double raw_head = 0;
	long double l_lat = 0, l_lon = 0, l_head = 0;
	vector<float> raw_ibeocontourx, raw_ibeocontoury;
	vector<float> ibctrx, ibctry,originx, originy, l1x, l1y, l2x, l2y, l3x, l3y, l4x, l4y;
	vector<float> raw_lane1x;	vector<float> raw_lane2x;	vector<float> raw_lane3x;	vector<float> raw_lane4x;
	vector<float> raw_lane1y;	vector<float> raw_lane2y;	vector<float> raw_lane3y;	vector<float> raw_lane4y;

	float distanceofall = 0;
	int KalmanInit = 0;
	const int stateNum = 4;    //状态值6×1向量(x,y,head,△x,△y,△head)  
	const int measureNum = 2;   //测量值3×1向量(x,y,head)    
	cv::KalmanFilter KF(stateNum, measureNum, 0);
	cv::Mat measurement = cv::Mat::zeros(measureNum, 1, CV_32F);  //初始测量值x'(0)，因为后面要更新这个值，所以必须先定义  
	float dT = 0.03f;

	for (int i = 0; i < 40; i++)
	{
		t_raw_lane1y[i] = (float)i + 2.f;
		t_raw_lane2y[i] = (float)i + 2.f;
		t_raw_lane3y[i] = (float)i + 2.f;
		t_raw_lane4y[i] = (float)i + 2.f;
	}

	string headingname = "GPSheading.txt";
	int gpsfilterflag = 0;
	std::string line;
	int flag = 0;
	float Dhead;
	double GPSx, GPSy;
	float  oldhead, headfilter, headnofilter, ww;
	vector<float> Savex, Savey, Savexkalman, Saveykalman;
	vector<float> headangle, headangle2, gyroscope;
	PacketGPS  RecordGPS;
	RecordIBEO RecordIBEO;
	int gpsi = 0;
	ifstream  in(dir.c_str());
	if (!in)
	{
		cout << "file does not exist." << endl;
		return;
	}
	while (in.peek() != EOF)
	{
	
		std::vector<std::string> lineSplit;
		std::getline(in, line);
		split(line, lineSplit, "\t");
		if (lineSplit.empty()) break;
		if (lineSplit[1] == "[GPS]")
		{
			raw_lat = stringToNum<long double>(lineSplit[3]);
			raw_lon = stringToNum<long double>(lineSplit[4]);
			raw_head = stringToNum<long double>(lineSplit[5]);
			RecordGPS.lat = raw_lat;
			RecordGPS.lon = raw_lon;
			RecordGPS.heading = raw_head;
			RecordGPS.speed = stringToNum<float>(lineSplit[6]);
			RecordGPS.vx = stringToNum<float>(lineSplit[7]);
			RecordGPS.vy = stringToNum<float>(lineSplit[8]);
			RecordGPS.wz = stringToNum<float>(lineSplit[9]);
			ww = RecordGPS.wz;
			raw_lat = raw_lat / (long double)180 * (long double)PI64;
			raw_lon = raw_lon / (long double)180 * (long double)PI64;
			raw_head = raw_head / (long double)180 * (long double)PI64;

			GPS rawgps;
			rawgps.lat = raw_lat;
			rawgps.lon = raw_lon;
			rawgps.head = raw_head;
			gpsdata.push_back(rawgps);

			if (flag == 0)
			{
				l_lat = raw_lat;
				l_lon = raw_lon;
				l_head = raw_head;
				headfilter = (float)raw_head;
				headnofilter =(float) raw_head;
				oldhead = (float)raw_head;
				flag++;
			}
			if (raw_head - oldhead < -PI64/2)
			{
				Dhead = (float)(raw_head - oldhead + 2 * PI64);
			}
			else if (raw_head - oldhead>PI64/2)
			{
				Dhead = (float)(raw_head - oldhead - 2 * PI64);
			}
			else
			{
				Dhead = (float)(raw_head - oldhead);
			}
			headnofilter = oldhead + Dhead;
			oldhead = headnofilter;


			gpsi++;
			if (!gpsfilterflag)
			{
				headfilter = headnofilter;
				Dhead = (float)(headfilter - l_head);
			}
			else
			{
				headfilter = GPSfliterheading[gpsi];//高斯过程拟合航向角
				Dhead = (float)(headfilter - l_head);
			}
			//CalculateCoordinate(GPSx, GPSy, rawgps);//计算轨迹全局坐标

			//double x, y;
			Global2_Local(GPSx, GPSy, raw_lat, raw_lon, l_lat, l_lon, l_head);
			if (Savex.size() > 2)
			{
				distanceofall += Computedistance(GPSx, GPSy, Savex.back(), Savey.back());
			}
			Savex.push_back((float)GPSx);
			Savey.push_back((float)GPSy);
			
			//kalmanflag = 0;
			//if (!KalmanInit)
			//{
			//	KF.transitionMatrix = (cv::Mat_<float>(4, 4) <<
			//		1, 0, dT, 0,
			//		0, 1, 0, dT,
			//		0, 0, 1, 0,
			//		0, 0, 0, 1);   //转移矩阵A  

			//	cv::setIdentity(KF.measurementMatrix);                     //测量矩阵H  
			//	cv::setIdentity(KF.processNoiseCov, cv::Scalar::all(1e-2));  //系统噪声方差矩阵Q  
			//	cv::setIdentity(KF.measurementNoiseCov, cv::Scalar::all(1e-3));   //测量噪声方差矩阵R  
			//	cv::setIdentity(KF.errorCovPost, cv::Scalar::all(1));            //后验错误估计协方差矩阵P  
			//	KF.statePost.at<float>(0) = (float)GPSx;
			//	KF.statePost.at<float>(1) = (float)GPSy;
			//	KalmanInit = 1;
			//}
			//else
			//{
			//	cv::Mat prediction = KF.predict();
			//	cv::Point predict_pt = cv::Point(prediction.at<int>(0), prediction.at<int>(1));   //预测值(x',y')  
			//	measurement.at<float>(0) = (float)GPSx;
			//	measurement.at<float>(1) = (float)GPSy;
			//	KF.correct(measurement);
			//	if (kalmanflag==1)
			//	{
			//		GPSx = KF.statePost.at<float>(0);
			//		GPSy = KF.statePost.at<float>(1);
			//	}
			//}
			Savexkalman.push_back((float)GPSx);
			Saveykalman.push_back((float)GPSy);
			headangle.push_back(headfilter);
			headangle2.push_back(headnofilter);
			gyroscope.push_back(ww);
			
			originx.push_back((float)GPSx);
			originy.push_back((float)GPSy);
		}
		else if (lineSplit[1] == "[VELD]")
		{
			if (!flag) continue;
			if (lineSplit.size() < 10)
			{
				std::getline(in, line);
				split(line, lineSplit, "\t");
			}

			for (int i = 0, j = 2; j < lineSplit.size(); i++, j++)
				m_PVeld.Lane1x[i] = stringToNum<float>(lineSplit[j]);
			std::getline(in, line);
			split(line, lineSplit, "\t");
			for (int i = 0, j = 2; j < lineSplit.size(); i++, j++)
				m_PVeld.Lane2x[i] = stringToNum<float>(lineSplit[j]);
			std::getline(in, line);
			split(line, lineSplit, "\t");
			for (int i = 0, j = 2; j < lineSplit.size(); i++, j++)
				m_PVeld.Lane3x[i] = stringToNum<float>(lineSplit[j]);
			std::getline(in, line);
			split(line, lineSplit, "\t");
			for (int i = 0, j = 2; j < lineSplit.size(); i++, j++)
				m_PVeld.Lane4x[i] = stringToNum<float>(lineSplit[j]);
			for (int i = 0; i < LENGTHOFLANE; i++)
			{
				if (i>15)
				{
					m_PVeld.Lane1x[i] = 999;
					m_PVeld.Lane2x[i] = 999;
					m_PVeld.Lane3x[i] = 999;
					m_PVeld.Lane4x[i] = 999;
				}
			}
			visual::cvtData2VectorForVeld(m_PVeld.Lane1x, t_raw_lane1y, LENGTHOFLANE, raw_lane1x, raw_lane1y);
			visual::cvtData2VectorForVeld(m_PVeld.Lane2x, t_raw_lane2y, LENGTHOFLANE, raw_lane2x, raw_lane2y);
			visual::cvtData2VectorForVeld(m_PVeld.Lane3x, t_raw_lane3y, LENGTHOFLANE, raw_lane3x, raw_lane3y);
			visual::cvtData2VectorForVeld(m_PVeld.Lane4x, t_raw_lane4y, LENGTHOFLANE, raw_lane4x, raw_lane4y);

			float x0 = 0.f, y0 = 0.f, angle = 0;
			//CalculateSensorData(raw_ibeocontourx, raw_ibeocontoury, x0, y0, angle);
			//x0 = 0.f, y0 = 0.0f, angle = (float)(parameter/100.f/ 180.f*PI64);

			CalculateSensorData(raw_lane1x, raw_lane1y, 0, 0, -Dhead);//旋转
			CalculateSensorData(raw_lane1x, raw_lane1y, -GPSx, -GPSy, 0);    //平移
			CalculateSensorData(raw_lane2x, raw_lane2y, 0, 0, -Dhead);//旋转
			CalculateSensorData(raw_lane2x, raw_lane2y, -GPSx, -GPSy, 0);    //平移
			CalculateSensorData(raw_lane3x, raw_lane3y, 0, 0, -Dhead);//旋转
			CalculateSensorData(raw_lane3x, raw_lane3y, -GPSx, -GPSy, 0);    //平移
			CalculateSensorData(raw_lane4x, raw_lane4y, 0, 0, -Dhead);//旋转
			CalculateSensorData(raw_lane4x, raw_lane4y, -GPSx, -GPSy, 0);    //平移

			l1x.insert(l1x.end(), raw_lane1x.begin(), raw_lane1x.end());
			l1y.insert(l1y.end(), raw_lane1y.begin(), raw_lane1y.end());
			l2x.insert(l2x.end(), raw_lane2x.begin(), raw_lane2x.end());
			l2y.insert(l2y.end(), raw_lane2y.begin(), raw_lane2y.end());
			l3x.insert(l3x.end(), raw_lane3x.begin(), raw_lane3x.end());
			l3y.insert(l3y.end(), raw_lane3y.begin(), raw_lane3y.end());
			l4x.insert(l4x.end(), raw_lane4x.begin(), raw_lane4x.end());
			l4y.insert(l4y.end(), raw_lane4y.begin(), raw_lane4y.end());
		}
		else if (lineSplit[1] == "[ROAD]")
		{
			for (int i = 0, j = 2; j < lineSplit.size(); i++, j++)
				m_PRoad.leftx[i] = stringToNum<int>(lineSplit[j]);
			std::getline(in, line);
			split(line, lineSplit, "\t");
			for (int i = 0, j = 2; j < lineSplit.size(); i++, j++)
				m_PRoad.lefty[i] = stringToNum<int>(lineSplit[j]);
			std::getline(in, line);
			split(line, lineSplit, "\t");
			for (int i = 0, j = 2; j < lineSplit.size(); i++, j++)
				m_PRoad.rightx[i] = stringToNum<int>(lineSplit[j]);
			std::getline(in, line);
			split(line, lineSplit, "\t");
			for (int i = 0, j = 2; j < lineSplit.size(); i++, j++)
				m_PRoad.righty[i] = stringToNum<int>(lineSplit[j]);
		}
		else if (lineSplit[1] == "[STOP]")
		{
			m_PStopLine.stopline = stringToNum<int>(lineSplit[2]);
		}
		else if (lineSplit[1] == "[TRFL]")
		{
			m_PTraffic.trafficlight = stringToNum<int>(lineSplit[2]);
		}
		else if (lineSplit[1] == "[IBEO]")
		{
			if (!flag) continue;
			int ibeostyle = 0;
			if (lineSplit.size() < 5)
			{
				ibeostyle = stringToNum<int>(lineSplit[2]);
				std::getline(in, line);
				split(line, lineSplit, "\t");
			}
			for (int i = 0, j = 2; j < lineSplit.size(); i++, j++)
			m_PIbeo.ibx[i] = stringToNum<float>(lineSplit[j]);
			std::getline(in, line);
			split(line, lineSplit, "\t");
			for (int i = 0, j = 2; j < lineSplit.size(); i++, j++)
				m_PIbeo.iby[i] = stringToNum<float>(lineSplit[j]);
			std::getline(in, line);
			split(line, lineSplit, "\t");
			for (int i = 0, j = 2; j < lineSplit.size(); i++, j++)
				m_PIbeo.ibvx[i] = stringToNum<float>(lineSplit[j]);
			std::getline(in, line);
			split(line, lineSplit, "\t");
			for (int i = 0, j = 2; j < lineSplit.size(); i++, j++)
				m_PIbeo.ibvy[i] = stringToNum<float>(lineSplit[j]);
			std::getline(in, line);
			split(line, lineSplit, "\t");
			for (int i = 0, j = 2; j < lineSplit.size(); i++, j++)
				m_PIbeo.ibw[i] = stringToNum<float>(lineSplit[j]);
			std::getline(in, line);
			split(line, lineSplit, "\t");
			for (int i = 0, j = 2; j < lineSplit.size(); i++, j++)
				m_PIbeo.ibl[i] = stringToNum<float>(lineSplit[j]);
			std::getline(in, line);
			split(line, lineSplit, "\t");
			for (int i = 0, j = 2; j < lineSplit.size(); i++, j++)
				m_PIbeo.ibvabx[i] = stringToNum<float>(lineSplit[j]);
			std::getline(in, line);
			split(line, lineSplit, "\t");
			for (int i = 0, j = 2; j < lineSplit.size(); i++, j++)
				m_PIbeo.ibvaby[i] = stringToNum<float>(lineSplit[j]);
			std::getline(in, line);
			split(line, lineSplit, "\t");
			for (int i = 0, j = 2; j < lineSplit.size(); i++, j++)
				m_PIbeo.ibcontournum[i] = stringToNum<int>(lineSplit[j]);

			//if (!(ibeostyle % 2))
			if ((ibeostyle % 2))
			{
				std::getline(in, line);
				std::getline(in, line);
				continue;
			}

			raw_ibeocontourx.clear();
			raw_ibeocontoury.clear();
			std::getline(in, line);
			split(line, lineSplit, "\t");
			for (int i = 0, j = 2; j < lineSplit.size(); i++, j++)
				//m_PIbeo.ibcontourx[i] = stringToNum<float>(lineSplit[j]);
				raw_ibeocontourx.push_back(stringToNum<float>(lineSplit[j]));
			std::getline(in, line);
			split(line, lineSplit, "\t");
			for (int i = 0, j = 2; j < lineSplit.size(); i++, j++)
				//m_PIbeo.ibcontoury[i] = stringToNum<float>(lineSplit[j]);
			   raw_ibeocontoury.push_back(stringToNum<float>(lineSplit[j]));

			
			//visual::cvtData2Vector(m_PIbeo.ibcontourx, m_PIbeo.ibcontoury, LENGTHOFCONTOUR, raw_ibeocontourx, raw_ibeocontoury, 1.f, 0, true);
			
			float x0 = 0.f, y0 = 2.85f, angle = 0;
			//CalculateSensorData(raw_ibeocontourx, raw_ibeocontoury, x0, y0, angle);
			////CalculateSensorData(raw_ibeocontourx, raw_ibeocontoury, x0, y0, angle);//旋转
			//x0 = 0.f, y0 = 0.0f, angle = (float)(1.3 / 180.0*(long double)PI64);
			//CalculateSensorData(raw_ibeocontourx, raw_ibeocontoury, x0, y0, angle);//旋转

			for (int i = 0; i < raw_ibeocontoury.size(); i++)
			{
				if (raw_ibeocontoury[i]>50.f)
				{
					raw_ibeocontoury.erase(raw_ibeocontoury.begin() + i);
					raw_ibeocontourx.erase(raw_ibeocontourx.begin() + i);
					i--;
				}
			}

			/*GPSIBEO IBEOdata;
			if (gpsdata.size())
			{
				IBEOdata.gps = RecordGPS;
				IBEOdata.x = raw_ibeocontourx;
				IBEOdata.y = raw_ibeocontoury;
				RecordIBEO.data.push_back(IBEOdata);
			}*/

			CalculateSensorData(raw_ibeocontourx, raw_ibeocontoury, 0, 0, -Dhead);//旋转
			CalculateSensorData(raw_ibeocontourx, raw_ibeocontoury, -GPSx, -GPSy, 0);    //平移
			ibctrx.insert(ibctrx.end(), raw_ibeocontourx.begin(), raw_ibeocontourx.end());
			ibctry.insert(ibctry.end(), raw_ibeocontoury.begin(), raw_ibeocontoury.end());
		}

	}
	cout<<"dis="<<distanceofall << endl;
	/*string dirr;
	std::ofstream outFile;
	dirr = "GPSheadingnofilter.txt";
	outFile.open(dirr);
	for (int i = 0; i < headangle2.size(); i++)
	{
		outFile << headangle[i] << '\t' << headangle2[i] << '\t' << gyroscope[i] << endl;
	}
	outFile.close();*/
	/*string dirr2;
	std::ofstream outFile2;
	dirr2 = "GPSXY.txt";
	outFile2.open(dirr2);
	for (int i = 0; i < Savex.size(); i++)
	{
		outFile2 << Savex[i] << '\t' << Savey[i] << '\t' << Savexkalman[i] << '\t' << Saveykalman[i] << endl;
	}
	outFile2.close();*/
	l1x.clear();
	l1y.clear();
	l4x.clear();
	l4y.clear();
	Map.gps = gpsdata;
	Map.ibeox = ibctrx;
	Map.ibeoy = ibctry;
	Map.l1x = l1x; Map.l2x = l2x; Map.l3x = l3x; 
	Map.l1y = l1y; Map.l2y = l2y; Map.l3y = l3y; 

	Map.l4x.insert(Map.l4x.end(), Map.l2x.begin(), Map.l2x.end());
	Map.l4x.insert(Map.l4x.end(), Map.l3x.begin(), Map.l3x.end());

	Map.l4y.insert(Map.l4y.end(), Map.l2y.begin(), Map.l2y.end());
	Map.l4y.insert(Map.l4y.end(), Map.l3y.begin(), Map.l3y.end());

	Map.x = originx;
	Map.y = originy;
	Map.coordinatezero = 0;
	mapnumber++;
	SaveGenerateGPSMap(Map, mapnumber);
	//SaveRecordIBEOData(RecordIBEO);
}

void BuildmapUTM(mapdata& Map, string dir)
{
	static int mapnumber;
	PacketVELD m_PVeld;
	PacketRoad m_PRoad;
	PacketTraffic m_PTraffic;
	PacketStopLine m_PStopLine;
	PacketIbeo m_PIbeo;
	visual::VisualizationBasicApi vsb;
	vector <GPS> gpsdata;
	vector <float> GPSfliterheading;
	float t_raw_lane1y[LENGTHOFLANE] = { 0.0f };
	float t_raw_lane2y[LENGTHOFLANE] = { 0.0f };
	float t_raw_lane3y[LENGTHOFLANE] = { 0.0f };
	float t_raw_lane4y[LENGTHOFLANE] = { 0.0f };

	 double raw_lat = 0; long double raw_lon = 0; long double raw_head = 0;
	 double l_lat = 0, l_lon = 0, l_head = 0;
	vector<float> raw_ibeocontourx, raw_ibeocontoury;
	vector<float> ibctrx, ibctry, originx, originy, l1x, l1y, l2x, l2y, l3x, l3y, l4x, l4y;
	vector<float> raw_lane1x;	vector<float> raw_lane2x;	vector<float> raw_lane3x;	vector<float> raw_lane4x;
	vector<float> raw_lane1y;	vector<float> raw_lane2y;	vector<float> raw_lane3y;	vector<float> raw_lane4y;

	float distanceofall = 0;
	int KalmanInit = 0;
	const int stateNum = 4;    //状态值6×1向量(x,y,head,△x,△y,△head)  
	const int measureNum = 2;   //测量值3×1向量(x,y,head)    
	cv::KalmanFilter KF(stateNum, measureNum, 0);
	cv::Mat measurement = cv::Mat::zeros(measureNum, 1, CV_32F);  //初始测量值x'(0)，因为后面要更新这个值，所以必须先定义  
	float dT = 0.03f;

	const my_Ellipse* e = standard_ellipse(ELLIPSE_WGS84);
	GridZone zone = GRID_AUTO;
	Hemisphere hemi = HEMI_AUTO;
	double xUTM_init, yUTM_init;
	//geographic_to_grid(e->a, e->e2, lat_ini*deg2rad, lon_ini*deg2rad, &zone, &hemi, &X_ini, &Y_ini);


	for (int i = 0; i < 40; i++)
	{
		t_raw_lane1y[i] = (float)i + 2.f;
		t_raw_lane2y[i] = (float)i + 2.f;
		t_raw_lane3y[i] = (float)i + 2.f;
		t_raw_lane4y[i] = (float)i + 2.f;
	}

	string headingname = "GPSheading.txt";
	int gpsfilterflag = 0;
	std::string line;
	int flag = 0;
	float Dhead;
	double GPSx, GPSy;
	float  oldhead, headfilter, headnofilter, ww;
	vector<float> Savex, Savey, Savexkalman, Saveykalman;
	vector<float> headangle, headangle2, gyroscope;
	PacketGPS  RecordGPS;
	RecordIBEO RecordIBEO;
	int gpsi = 0;
	ifstream  in(dir.c_str());
	if (!in)
	{
		cout << "file does not exist." << endl;
		return;
	}
	while (in.peek() != EOF)
	{

		std::vector<std::string> lineSplit;
		std::getline(in, line);
		split(line, lineSplit, "\t");
		if (lineSplit.empty()) break;
		if (lineSplit[1] == "[GPS]")
		{
			raw_lat = stringToNum<long double>(lineSplit[3]);
			raw_lon = stringToNum<long double>(lineSplit[4]);
			raw_head = stringToNum<long double>(lineSplit[5]);
			RecordGPS.lat = raw_lat;
			RecordGPS.lon = raw_lon;
			RecordGPS.heading = raw_head;
			RecordGPS.speed = stringToNum<float>(lineSplit[6]);
			RecordGPS.vx = stringToNum<float>(lineSplit[7]);
			RecordGPS.vy = stringToNum<float>(lineSplit[8]);
			RecordGPS.wz = stringToNum<float>(lineSplit[9]);
			ww = RecordGPS.wz;
			raw_lat = raw_lat / (long double)180 * (long double)PI64;
			raw_lon = raw_lon / (long double)180 * (long double)PI64;
			raw_head = raw_head / (long double)180 * (long double)PI64;

			GPS rawgps;
			rawgps.lat = raw_lat;
			rawgps.lon = raw_lon;
			rawgps.head = raw_head;
			gpsdata.push_back(rawgps);

			if (flag == 0)
			{
				l_lat = raw_lat;
				l_lon = raw_lon;
				l_head = 0;// raw_head;
				headfilter = (float)raw_head;
				headnofilter = (float)raw_head;
				oldhead = (float)raw_head;
				flag++;
				geographic_to_grid(e->a, e->e2, l_lat, l_lon, &zone, &hemi, &yUTM_init, &xUTM_init);
				yUTM_init = 3463220.5963258;
				xUTM_init = 329241.61917934;
				cout << "x=" << xUTM_init << "  y=" << yUTM_init << endl;
			}
			
			
			Dhead =  raw_head;// (float)(raw_head - oldhead);
	

			gpsi++;
		
			//double x, y;
			//Global2_Local(GPSx, GPSy, raw_lat, raw_lon, l_lat, l_lon, l_head);
			geographic_to_grid(e->a, e->e2, raw_lat, raw_lon, &zone, &hemi, &GPSy, &GPSx);
			GPSx -= xUTM_init;
			GPSy -= yUTM_init;
			//cout << "x=" << GPSx << "  y=" << GPSy << endl;

			if (Savex.size() > 2)
			{
				distanceofall += Computedistance(GPSx, GPSy, Savex.back(), Savey.back());
			}
			Savex.push_back((float)GPSx);
			Savey.push_back((float)GPSy);

			Savexkalman.push_back((float)GPSx);
			Saveykalman.push_back((float)GPSy);
			gyroscope.push_back(ww);

			originx.push_back((float)GPSx);
			originy.push_back((float)GPSy);

		}
		else if (lineSplit[1] == "[SLPS]")
		{
			float slamposx = 0, slamposy = 0, slamposz = 0, slamposheading = 0;
			slamposx = stringToNum<float>(lineSplit[3]);
			slamposy = stringToNum<float>(lineSplit[4]);
			slamposz = stringToNum<float>(lineSplit[5]);
			slamposheading = stringToNum<float>(lineSplit[6]);
			slamposheading = slamposheading / (long double)180 * (long double)PI64;;

			if (flag == 0)
			{
				flag++;
				oldhead = slamposheading;
			}
			GPSx = slamposx;
			GPSy = slamposy;
		 
			{
				Dhead = (float)(slamposheading - oldhead);
			}
			//oldhead = slamposheading;
			GPS rawgps;
			rawgps.x = GPSx;
			rawgps.y = GPSy;
			gpsdata.push_back(rawgps);

			originx.push_back((float)GPSx);
			originy.push_back((float)GPSy);

			if (Savex.size() > 2)
			{
				distanceofall += Computedistance(GPSx, GPSy, Savex.back(), Savey.back());
			}
			Savex.push_back((float)GPSx);
			Savey.push_back((float)GPSy);

		}
		else if (lineSplit[1] == "[VELD]")
		{
		//old offline
		 if(1)
		 {
	    	if (!flag) continue;
			if (lineSplit.size() < 10)
			{
				std::getline(in, line);
				split(line, lineSplit, "\t");
			}

			for (int i = 0, j = 2; j < lineSplit.size(); i++, j++)
				m_PVeld.Lane1x[i] = stringToNum<float>(lineSplit[j]);
			std::getline(in, line);
			split(line, lineSplit, "\t");
			for (int i = 0, j = 2; j < lineSplit.size(); i++, j++)
				m_PVeld.Lane2x[i] = stringToNum<float>(lineSplit[j]);
			std::getline(in, line);
			split(line, lineSplit, "\t");
			for (int i = 0, j = 2; j < lineSplit.size(); i++, j++)
				m_PVeld.Lane3x[i] = stringToNum<float>(lineSplit[j]);
			std::getline(in, line);
			split(line, lineSplit, "\t");
			for (int i = 0, j = 2; j < lineSplit.size(); i++, j++)
				m_PVeld.Lane4x[i] = stringToNum<float>(lineSplit[j]);
			for (int i = 0; i < LENGTHOFLANE; i++)
			{
				if (i > 15)
				{
					m_PVeld.Lane1x[i] = 999;
					m_PVeld.Lane2x[i] = 999;
					m_PVeld.Lane3x[i] = 999;
					m_PVeld.Lane4x[i] = 999;
				}
			}
			visual::cvtData2VectorForVeld(m_PVeld.Lane1x, t_raw_lane1y, LENGTHOFLANE, raw_lane1x, raw_lane1y);
			visual::cvtData2VectorForVeld(m_PVeld.Lane2x, t_raw_lane2y, LENGTHOFLANE, raw_lane2x, raw_lane2y);
			visual::cvtData2VectorForVeld(m_PVeld.Lane3x, t_raw_lane3y, LENGTHOFLANE, raw_lane3x, raw_lane3y);
			visual::cvtData2VectorForVeld(m_PVeld.Lane4x, t_raw_lane4y, LENGTHOFLANE, raw_lane4x, raw_lane4y);

		}
			else
			{
			raw_lane1x.clear();
			raw_lane1y.clear();
			raw_lane2x.clear();
			raw_lane2y.clear();
			raw_lane3x.clear();
			raw_lane3y.clear();
			raw_lane4x.clear();
			raw_lane4y.clear();

			std::getline(in, line);
			split(line, lineSplit, "\t");
			for (int j = 2; j < lineSplit.size(); j++)
				raw_lane1x.push_back(stringToNum<float>(lineSplit[j]));
			std::getline(in, line);
			split(line, lineSplit, "\t");
			for (int j = 2; j < lineSplit.size(); j++)
				raw_lane1y.push_back(stringToNum<float>(lineSplit[j]));

			std::getline(in, line);
			split(line, lineSplit, "\t");
			for (int j = 2; j < lineSplit.size(); j++)
				raw_lane2x.push_back(stringToNum<float>(lineSplit[j]));
			std::getline(in, line);
			split(line, lineSplit, "\t");
			for (int j = 2; j < lineSplit.size(); j++)
				raw_lane2y.push_back(stringToNum<float>(lineSplit[j]));

			std::getline(in, line);
			split(line, lineSplit, "\t");
			for (int j = 2; j < lineSplit.size(); j++)
				raw_lane3x.push_back(stringToNum<float>(lineSplit[j]));
			std::getline(in, line);
			split(line, lineSplit, "\t");
			for (int j = 2; j < lineSplit.size(); j++)
				raw_lane3y.push_back(stringToNum<float>(lineSplit[j]));

			std::getline(in, line);
			split(line, lineSplit, "\t");
			for (int j = 2; j < lineSplit.size(); j++)
				raw_lane4x.push_back(stringToNum<float>(lineSplit[j]));
			std::getline(in, line);
			split(line, lineSplit, "\t");
			for (int j = 2; j < lineSplit.size(); j++)
				raw_lane4y.push_back(stringToNum<float>(lineSplit[j]));
			float error_x = 0;
			for (int i = 0; i < raw_lane3y.size(); i++)
			{
				error_x += raw_lane3x[i];
				if (raw_lane3y[i] > 10)
				{
					raw_lane3y.erase(raw_lane3y.begin() + i);
					raw_lane3x.erase(raw_lane3x.begin() + i);
					i--;
				}
			}

			for (int i = 0; i < raw_lane2y.size(); i++)
			{
				error_x += raw_lane3x[i];

				if (raw_lane2y[i] > 10)
				{
					raw_lane2y.erase(raw_lane2y.begin() + i);
					raw_lane2x.erase(raw_lane2x.begin() + i);
					i--;
				}
			}
			if (error_x == 0)
			{
				raw_lane1x.clear();
				raw_lane1y.clear();
				raw_lane2x.clear();
				raw_lane2y.clear();
				raw_lane3x.clear();
				raw_lane3y.clear();
				raw_lane4x.clear();
				raw_lane4y.clear();
			}

		}
			float x0 = 0.f, y0 = 0.f, angle = 0;
			//CalculateSensorData(raw_ibeocontourx, raw_ibeocontoury, x0, y0, angle);
			//x0 = 0.f, y0 = 0.0f, angle = (float)(parameter/100.f/ 180.f*PI64);

			CalculateSensorData(raw_lane1x, raw_lane1y, 0, 0, -Dhead);//旋转
			CalculateSensorData(raw_lane1x, raw_lane1y, -GPSx, -GPSy, 0);    //平移
			CalculateSensorData(raw_lane2x, raw_lane2y, 0, 0, -Dhead);//旋转
			CalculateSensorData(raw_lane2x, raw_lane2y, -GPSx, -GPSy, 0);    //平移
			CalculateSensorData(raw_lane3x, raw_lane3y, 0, 0, -Dhead);//旋转
			CalculateSensorData(raw_lane3x, raw_lane3y, -GPSx, -GPSy, 0);    //平移
			CalculateSensorData(raw_lane4x, raw_lane4y, 0, 0, -Dhead);//旋转
			CalculateSensorData(raw_lane4x, raw_lane4y, -GPSx, -GPSy, 0);    //平移

			l1x.insert(l1x.end(), raw_lane1x.begin(), raw_lane1x.end());
			l1y.insert(l1y.end(), raw_lane1y.begin(), raw_lane1y.end());
			l2x.insert(l2x.end(), raw_lane2x.begin(), raw_lane2x.end());
			l2y.insert(l2y.end(), raw_lane2y.begin(), raw_lane2y.end());
			l3x.insert(l3x.end(), raw_lane3x.begin(), raw_lane3x.end());
			l3y.insert(l3y.end(), raw_lane3y.begin(), raw_lane3y.end());
			l4x.insert(l4x.end(), raw_lane4x.begin(), raw_lane4x.end());
			l4y.insert(l4y.end(), raw_lane4y.begin(), raw_lane4y.end());
		}
		else if (lineSplit[1] == "[ROAD]")
		{
			for (int i = 0, j = 2; j < lineSplit.size(); i++, j++)
				m_PRoad.leftx[i] = stringToNum<int>(lineSplit[j]);
			std::getline(in, line);
			split(line, lineSplit, "\t");
			for (int i = 0, j = 2; j < lineSplit.size(); i++, j++)
				m_PRoad.lefty[i] = stringToNum<int>(lineSplit[j]);
			std::getline(in, line);
			split(line, lineSplit, "\t");
			for (int i = 0, j = 2; j < lineSplit.size(); i++, j++)
				m_PRoad.rightx[i] = stringToNum<int>(lineSplit[j]);
			std::getline(in, line);
			split(line, lineSplit, "\t");
			for (int i = 0, j = 2; j < lineSplit.size(); i++, j++)
				m_PRoad.righty[i] = stringToNum<int>(lineSplit[j]);
		}
		else if (lineSplit[1] == "[STOP]")
		{
			m_PStopLine.stopline = stringToNum<int>(lineSplit[2]);
		}
		else if (lineSplit[1] == "[TRFL]")
		{
			m_PTraffic.trafficlight = stringToNum<int>(lineSplit[2]);
		}
		else if (lineSplit[1] == "[IBEO]")
		{
			if (!flag) continue;
			int ibeostyle = 0;
			if (lineSplit.size() < 5)
			{
				ibeostyle = stringToNum<int>(lineSplit[2]);
				std::getline(in, line);
				split(line, lineSplit, "\t");
			}
			for (int i = 0, j = 2; j < lineSplit.size(); i++, j++)
				m_PIbeo.ibx[i] = stringToNum<float>(lineSplit[j]);
			std::getline(in, line);
			split(line, lineSplit, "\t");
			for (int i = 0, j = 2; j < lineSplit.size(); i++, j++)
				m_PIbeo.iby[i] = stringToNum<float>(lineSplit[j]);
			std::getline(in, line);
			split(line, lineSplit, "\t");
			for (int i = 0, j = 2; j < lineSplit.size(); i++, j++)
				m_PIbeo.ibvx[i] = stringToNum<float>(lineSplit[j]);
			std::getline(in, line);
			split(line, lineSplit, "\t");
			for (int i = 0, j = 2; j < lineSplit.size(); i++, j++)
				m_PIbeo.ibvy[i] = stringToNum<float>(lineSplit[j]);
			std::getline(in, line);
			split(line, lineSplit, "\t");
			for (int i = 0, j = 2; j < lineSplit.size(); i++, j++)
				m_PIbeo.ibw[i] = stringToNum<float>(lineSplit[j]);
			std::getline(in, line);
			split(line, lineSplit, "\t");
			for (int i = 0, j = 2; j < lineSplit.size(); i++, j++)
				m_PIbeo.ibl[i] = stringToNum<float>(lineSplit[j]);
			std::getline(in, line);
			split(line, lineSplit, "\t");
			for (int i = 0, j = 2; j < lineSplit.size(); i++, j++)
				m_PIbeo.ibvabx[i] = stringToNum<float>(lineSplit[j]);
			std::getline(in, line);
			split(line, lineSplit, "\t");
			for (int i = 0, j = 2; j < lineSplit.size(); i++, j++)
				m_PIbeo.ibvaby[i] = stringToNum<float>(lineSplit[j]);
			std::getline(in, line);
			split(line, lineSplit, "\t");
			for (int i = 0, j = 2; j < lineSplit.size(); i++, j++)
				m_PIbeo.ibcontournum[i] = stringToNum<int>(lineSplit[j]);

			//if (!(ibeostyle % 2))//save velo
			if ((ibeostyle % 2))//save ibeo
				//if ((ibeostyle == 3))
				//if ((ibeostyle==1) || (ibeostyle ==0))
			{
				std::getline(in, line);
				std::getline(in, line);
				continue;
			}

			raw_ibeocontourx.clear();
			raw_ibeocontoury.clear();
			std::getline(in, line);
			split(line, lineSplit, "\t");
			for (int i = 0, j = 2; j < lineSplit.size(); i++, j++)
				//m_PIbeo.ibcontourx[i] = stringToNum<float>(lineSplit[j]);
				raw_ibeocontourx.push_back(stringToNum<float>(lineSplit[j]));
			std::getline(in, line);
			split(line, lineSplit, "\t");
			for (int i = 0, j = 2; j < lineSplit.size(); i++, j++)
				//m_PIbeo.ibcontoury[i] = stringToNum<float>(lineSplit[j]);
				raw_ibeocontoury.push_back(stringToNum<float>(lineSplit[j]));


			//visual::cvtData2Vector(m_PIbeo.ibcontourx, m_PIbeo.ibcontoury, LENGTHOFCONTOUR, raw_ibeocontourx, raw_ibeocontoury, 1.f, 0, true);

			float x0 = 0.f, y0 = 2.85f, angle = 0;
			//CalculateSensorData(raw_ibeocontourx, raw_ibeocontoury, x0, y0, angle);
			////CalculateSensorData(raw_ibeocontourx, raw_ibeocontoury, x0, y0, angle);//旋转
			//x0 = 0.f, y0 = 0.0f, angle = (float)(1.3 / 180.0*(long double)PI64);
			//CalculateSensorData(raw_ibeocontourx, raw_ibeocontoury, x0, y0, angle);//旋转

			for (int i = 0; i < raw_ibeocontoury.size(); i++)
			{
				if (raw_ibeocontoury[i]>15.f || raw_ibeocontoury[i] < -15.f || raw_ibeocontourx[i]>20.f || raw_ibeocontourx[i] < -20.f)
				{
					raw_ibeocontoury.erase(raw_ibeocontoury.begin() + i);
					raw_ibeocontourx.erase(raw_ibeocontourx.begin() + i);
					i--;
				}
			}
		
			CalculateSensorData(raw_ibeocontourx, raw_ibeocontoury, 0, 0, -Dhead);//旋转
			CalculateSensorData(raw_ibeocontourx, raw_ibeocontoury, -GPSx, -GPSy, 0);    //平移
			ibctrx.insert(ibctrx.end(), raw_ibeocontourx.begin(), raw_ibeocontourx.end());
			ibctry.insert(ibctry.end(), raw_ibeocontoury.begin(), raw_ibeocontoury.end());
		}

	}
	cout << "dis=" << distanceofall << endl;
	/*string dirr;
	std::ofstream outFile;
	dirr = "GPSheadingnofilter.txt";
	outFile.open(dirr);
	for (int i = 0; i < headangle2.size(); i++)
	{
	outFile << headangle[i] << '\t' << headangle2[i] << '\t' << gyroscope[i] << endl;
	}
	outFile.close();*/
	/*string dirr2;
	std::ofstream outFile2;
	dirr2 = "GPSXY.txt";
	outFile2.open(dirr2);
	for (int i = 0; i < Savex.size(); i++)
	{
	outFile2 << Savex[i] << '\t' << Savey[i] << '\t' << Savexkalman[i] << '\t' << Saveykalman[i] << endl;
	}
	outFile2.close();*/
	l1x.clear();
	l1y.clear();
	l4x.clear();
	l4y.clear();
	Map.gps = gpsdata;
	Map.ibeox = ibctrx;
	Map.ibeoy = ibctry;
	Map.l1x = l1x; Map.l2x = l2x; Map.l3x = l3x;
	Map.l1y = l1y; Map.l2y = l2y; Map.l3y = l3y;

	Map.l4x.insert(Map.l4x.end(), Map.l2x.begin(), Map.l2x.end());
	Map.l4x.insert(Map.l4x.end(), Map.l3x.begin(), Map.l3x.end());

	Map.l4y.insert(Map.l4y.end(), Map.l2y.begin(), Map.l2y.end());
	Map.l4y.insert(Map.l4y.end(), Map.l3y.begin(), Map.l3y.end());

	Map.x = originx;
	Map.y = originy;
	Map.coordinatezero = 0;
	mapnumber++;
	SaveGenerateGPSMap(Map, mapnumber);
	//SaveRecordIBEOData(RecordIBEO);
}

void BuildmapGPS(mapdata& Map, string dir)
{
	static int mapnumber;
	PacketVELD m_PVeld;
	PacketRoad m_PRoad;
	PacketTraffic m_PTraffic;
	PacketStopLine m_PStopLine;
	PacketIbeo m_PIbeo;
	visual::VisualizationBasicApi vsb;
	vector <GPS> gpsdata;
	vector <float> GPSfliterheading;
	float t_raw_lane1y[LENGTHOFLANE] = { 0.0f };
	float t_raw_lane2y[LENGTHOFLANE] = { 0.0f };
	float t_raw_lane3y[LENGTHOFLANE] = { 0.0f };
	float t_raw_lane4y[LENGTHOFLANE] = { 0.0f };

	long double raw_lat = 0; long double raw_lon = 0; long double raw_head = 0;
	long double l_lat = 0, l_lon = 0, l_head = 0;
	vector<float> raw_ibeocontourx, raw_ibeocontoury;
	vector<float> ibctrx, ibctry, originx, originy, l1x, l1y, l2x, l2y, l3x, l3y, l4x, l4y;
	vector<float> raw_lane1x;	vector<float> raw_lane2x;	vector<float> raw_lane3x;	vector<float> raw_lane4x;
	vector<float> raw_lane1y;	vector<float> raw_lane2y;	vector<float> raw_lane3y;	vector<float> raw_lane4y;

	float distanceofall = 0;
	int KalmanInit = 0;
	const int stateNum = 4;    //状态值6×1向量(x,y,head,△x,△y,△head)  
	const int measureNum = 2;   //测量值3×1向量(x,y,head)    
	cv::KalmanFilter KF(stateNum, measureNum, 0);
	cv::Mat measurement = cv::Mat::zeros(measureNum, 1, CV_32F);  //初始测量值x'(0)，因为后面要更新这个值，所以必须先定义  
	float dT = 0.03f;

	for (int i = 0; i < 40; i++)
	{
		t_raw_lane1y[i] = (float)i + 2.f;
		t_raw_lane2y[i] = (float)i + 2.f;
		t_raw_lane3y[i] = (float)i + 2.f;
		t_raw_lane4y[i] = (float)i + 2.f;
	}

	string headingname = "GPSheading.txt";
	int gpsfilterflag = 0;
	std::string line;
	int flag = 0;
	float Dhead;
	double GPSx, GPSy;
	float  oldhead, headfilter, headnofilter, ww;
	vector<float> Savex, Savey, Savexkalman, Saveykalman;
	vector<float> headangle, headangle2, gyroscope;
	PacketGPS  RecordGPS;
	RecordIBEO RecordIBEO;
	int gpsi = 0;
	ifstream  in(dir.c_str());
	if (!in)
	{
		cout << "file does not exist." << endl;
		return;
	}
	while (in.peek() != EOF)
	{
		std::vector<std::string> lineSplit;
		std::getline(in, line);
		split(line, lineSplit, "\t");
		if (lineSplit.empty()) break;
		if (lineSplit[1] == "[GPS]")
		{
			raw_lat = stringToNum<long double>(lineSplit[3]);
			raw_lon = stringToNum<long double>(lineSplit[4]);
			raw_head = stringToNum<long double>(lineSplit[5]);
			RecordGPS.lat = raw_lat;
			RecordGPS.lon = raw_lon;
			RecordGPS.heading = raw_head;
			RecordGPS.speed = stringToNum<float>(lineSplit[6]);
			RecordGPS.vx = stringToNum<float>(lineSplit[7]);
			RecordGPS.vy = stringToNum<float>(lineSplit[8]);
			RecordGPS.wz = stringToNum<float>(lineSplit[9]);
			ww = RecordGPS.wz;
			raw_lat = raw_lat / (long double)180 * (long double)PI64;
			raw_lon = raw_lon / (long double)180 * (long double)PI64;
			raw_head = raw_head / (long double)180 * (long double)PI64;

			GPS rawgps;
			rawgps.lat = raw_lat;
			rawgps.lon = raw_lon;
			rawgps.head = raw_head;
			gpsdata.push_back(rawgps);

			if (flag == 0)
			{
				l_lat = raw_lat;
				l_lon = raw_lon;
				l_head = 0;// raw_head;
				headfilter = (float)raw_head;
				headnofilter = (float)raw_head;
				oldhead = (float)raw_head;
				flag++;
			}
			if (raw_head - oldhead < -PI64 / 2)
			{
				Dhead = (float)(raw_head - oldhead + 2 * PI64);
			}
			else if (raw_head - oldhead>PI64 / 2)
			{
				Dhead = (float)(raw_head - oldhead - 2 * PI64);
			}
			else
			{
				Dhead = (float)(raw_head - oldhead);
			}
			headnofilter = oldhead + Dhead;
			oldhead = headnofilter;


			gpsi++;
			if (!gpsfilterflag)
			{
				headfilter = headnofilter;
				Dhead = (float)(headfilter - l_head);
			}
			else
			{
				headfilter = GPSfliterheading[gpsi];//高斯过程拟合航向角
				Dhead = (float)(headfilter - l_head);
			}
			//CalculateCoordinate(GPSx, GPSy, rawgps);//计算轨迹全局坐标

			//double x, y;
			Global2_Local(GPSx, GPSy, raw_lat, raw_lon, l_lat, l_lon, l_head);
			if (Savex.size() > 2)
			{
				distanceofall += Computedistance(GPSx, GPSy, Savex.back(), Savey.back());
			}
			Savex.push_back((float)GPSx);
			Savey.push_back((float)GPSy);

			Savexkalman.push_back((float)GPSx);
			Saveykalman.push_back((float)GPSy);
			headangle.push_back(headfilter);
			headangle2.push_back(headnofilter);
			gyroscope.push_back(ww);

			originx.push_back((float)GPSx);
			originy.push_back((float)GPSy);

		}
		if (0&&lineSplit[1] == "[SLPS]")
		{
			float slamposx=0, slamposy=0, slamposz=0, slamposheading=0;
			slamposx = stringToNum<float>(lineSplit[3]);
			slamposy = stringToNum<float>(lineSplit[4]);
			slamposz = stringToNum<float>(lineSplit[5]);
			slamposheading = stringToNum<float>(lineSplit[6]);
			slamposheading = slamposheading/(long double)180 * (long double)PI64;;

			if (flag == 0)
			{
				flag++;
				oldhead = slamposheading;
			}
			//GPSx = slamposx;
			//GPSy = slamposy;
			headfilter = slamposheading;

			/*if (slamposheading - oldhead < -PI64 / 2)
			{
				Dhead = (float)(slamposheading - oldhead + 2 * PI64);
			}
			else if (slamposheading - oldhead>PI64 / 2)
			{
				Dhead = (float)(slamposheading - oldhead - 2 * PI64);
			}
			else*/
			{
				Dhead = (float)(slamposheading - oldhead);
			}
			//oldhead = slamposheading;
			GPS rawgps;
			rawgps.x = GPSx;
			rawgps.y = GPSy;
			gpsdata.push_back(rawgps);

			originx.push_back((float)GPSx);
			originy.push_back((float)GPSy);

			if (Savex.size() > 2)
			{
				distanceofall += Computedistance(GPSx, GPSy, Savex.back(), Savey.back());
			}
			Savex.push_back((float)GPSx);
			Savey.push_back((float)GPSy);

		}
		else if (lineSplit[1] == "[VELD]")
		{
			/*if (!flag) continue;
			if (lineSplit.size() < 10)
			{
				std::getline(in, line);
				split(line, lineSplit, "\t");
			}*/
			raw_lane1x.clear();
			raw_lane1y.clear();
			raw_lane2x.clear();
			raw_lane2y.clear();
			raw_lane3x.clear();
			raw_lane3y.clear();
			raw_lane4x.clear();
			raw_lane4y.clear();

			std::getline(in, line);
			split(line, lineSplit, "\t");
			for (int j = 2; j < lineSplit.size();j++)
				raw_lane1x.push_back(stringToNum<float>(lineSplit[j]));
			std::getline(in, line);
			split(line, lineSplit, "\t");
			for (int j = 2; j < lineSplit.size();  j++)
				raw_lane1y.push_back(stringToNum<float>(lineSplit[j]));

			std::getline(in, line);
			split(line, lineSplit, "\t");
			for (int j = 2; j < lineSplit.size(); j++)
				raw_lane2x.push_back(stringToNum<float>(lineSplit[j]));
			std::getline(in, line);
			split(line, lineSplit, "\t");
			for (int j = 2; j < lineSplit.size(); j++)
				raw_lane2y.push_back(stringToNum<float>(lineSplit[j]));

			std::getline(in, line);
			split(line, lineSplit, "\t");
			for (int j = 2; j < lineSplit.size(); j++)
				raw_lane3x.push_back(stringToNum<float>(lineSplit[j]));
			std::getline(in, line);
			split(line, lineSplit, "\t");
			for (int j = 2; j < lineSplit.size(); j++)
				raw_lane3y.push_back(stringToNum<float>(lineSplit[j]));

			std::getline(in, line);
			split(line, lineSplit, "\t");
			for (int j = 2; j < lineSplit.size(); j++)
				raw_lane4x.push_back(stringToNum<float>(lineSplit[j]));
			std::getline(in, line);
			split(line, lineSplit, "\t");
			for (int j = 2; j < lineSplit.size(); j++)
				raw_lane4y.push_back(stringToNum<float>(lineSplit[j]));

			for (int i = 0; i < raw_lane3y.size(); i++)
			{
				if (raw_lane3y[i]>10)
				{
					raw_lane3y.erase(raw_lane3y.begin() + i);
					raw_lane3x.erase(raw_lane3x.begin() + i);
					i--;
				}
			}

			for (int i = 0; i < raw_lane2y.size(); i++)
			{
				if (raw_lane2y[i]>10)
				{
					raw_lane2y.erase(raw_lane2y.begin() + i);
					raw_lane2x.erase(raw_lane2x.begin() + i);
					i--;
				}
			}
		/*	for (int i = 0, j = 2; j < lineSplit.size(); i++, j++)
				m_PVeld.Lane1x[i] = stringToNum<float>(lineSplit[j]);
			std::getline(in, line);
			split(line, lineSplit, "\t");
			for (int i = 0, j = 2; j < lineSplit.size(); i++, j++)
				m_PVeld.Lane2x[i] = stringToNum<float>(lineSplit[j]);
			std::getline(in, line);
			split(line, lineSplit, "\t");
			for (int i = 0, j = 2; j < lineSplit.size(); i++, j++)
				m_PVeld.Lane3x[i] = stringToNum<float>(lineSplit[j]);
			std::getline(in, line);
			split(line, lineSplit, "\t");
			for (int i = 0, j = 2; j < lineSplit.size(); i++, j++)
				m_PVeld.Lane4x[i] = stringToNum<float>(lineSplit[j]);
			for (int i = 0; i < LENGTHOFLANE; i++)
			{
				if (i>15)
				{
					m_PVeld.Lane1x[i] = 999;
					m_PVeld.Lane2x[i] = 999;
					m_PVeld.Lane3x[i] = 999;
					m_PVeld.Lane4x[i] = 999;
				}
			}*/
		/*	visual::cvtData2VectorForVeld(m_PVeld.Lane1x, t_raw_lane1y, LENGTHOFLANE, raw_lane1x, raw_lane1y);
			visual::cvtData2VectorForVeld(m_PVeld.Lane2x, t_raw_lane2y, LENGTHOFLANE, raw_lane2x, raw_lane2y);
			visual::cvtData2VectorForVeld(m_PVeld.Lane3x, t_raw_lane3y, LENGTHOFLANE, raw_lane3x, raw_lane3y);
			visual::cvtData2VectorForVeld(m_PVeld.Lane4x, t_raw_lane4y, LENGTHOFLANE, raw_lane4x, raw_lane4y);*/

			float x0 = 0.f, y0 = 0.f, angle = 0;
			//CalculateSensorData(raw_ibeocontourx, raw_ibeocontoury, x0, y0, angle);
			//x0 = 0.f, y0 = 0.0f, angle = (float)(parameter/100.f/ 180.f*PI64);

			CalculateSensorData(raw_lane1x, raw_lane1y, 0, 0, -Dhead);//旋转
			CalculateSensorData(raw_lane1x, raw_lane1y, -GPSx, -GPSy, 0);    //平移
			CalculateSensorData(raw_lane2x, raw_lane2y, 0, 0, -Dhead);//旋转
			CalculateSensorData(raw_lane2x, raw_lane2y, -GPSx, -GPSy, 0);    //平移
			CalculateSensorData(raw_lane3x, raw_lane3y, 0, 0, -Dhead);//旋转
			CalculateSensorData(raw_lane3x, raw_lane3y, -GPSx, -GPSy, 0);    //平移
			CalculateSensorData(raw_lane4x, raw_lane4y, 0, 0, -Dhead);//旋转
			CalculateSensorData(raw_lane4x, raw_lane4y, -GPSx, -GPSy, 0);    //平移

			l1x.insert(l1x.end(), raw_lane1x.begin(), raw_lane1x.end());
			l1y.insert(l1y.end(), raw_lane1y.begin(), raw_lane1y.end());
			l2x.insert(l2x.end(), raw_lane2x.begin(), raw_lane2x.end());
			l2y.insert(l2y.end(), raw_lane2y.begin(), raw_lane2y.end());
			l3x.insert(l3x.end(), raw_lane3x.begin(), raw_lane3x.end());
			l3y.insert(l3y.end(), raw_lane3y.begin(), raw_lane3y.end());
			l4x.insert(l4x.end(), raw_lane4x.begin(), raw_lane4x.end());
			l4y.insert(l4y.end(), raw_lane4y.begin(), raw_lane4y.end());
		}
		else if (lineSplit[1] == "[ROAD]")
		{
			for (int i = 0, j = 2; j < lineSplit.size(); i++, j++)
				m_PRoad.leftx[i] = stringToNum<int>(lineSplit[j]);
			std::getline(in, line);
			split(line, lineSplit, "\t");
			for (int i = 0, j = 2; j < lineSplit.size(); i++, j++)
				m_PRoad.lefty[i] = stringToNum<int>(lineSplit[j]);
			std::getline(in, line);
			split(line, lineSplit, "\t");
			for (int i = 0, j = 2; j < lineSplit.size(); i++, j++)
				m_PRoad.rightx[i] = stringToNum<int>(lineSplit[j]);
			std::getline(in, line);
			split(line, lineSplit, "\t");
			for (int i = 0, j = 2; j < lineSplit.size(); i++, j++)
				m_PRoad.righty[i] = stringToNum<int>(lineSplit[j]);
		}
		else if (lineSplit[1] == "[STOP]")
		{
			m_PStopLine.stopline = stringToNum<int>(lineSplit[2]);
		}
		else if (lineSplit[1] == "[TRFL]")
		{
			m_PTraffic.trafficlight = stringToNum<int>(lineSplit[2]);
		}
		else if (lineSplit[1] == "[IBEO]")
		{
			if (!flag) continue;
			int ibeostyle = 0;
			if (lineSplit.size() < 5)
			{
				ibeostyle = stringToNum<int>(lineSplit[2]);
				std::getline(in, line);
				split(line, lineSplit, "\t");
			}
			for (int i = 0, j = 2; j < lineSplit.size(); i++, j++)
				m_PIbeo.ibx[i] = stringToNum<float>(lineSplit[j]);
			std::getline(in, line);
			split(line, lineSplit, "\t");
			for (int i = 0, j = 2; j < lineSplit.size(); i++, j++)
				m_PIbeo.iby[i] = stringToNum<float>(lineSplit[j]);
			std::getline(in, line);
			split(line, lineSplit, "\t");
			for (int i = 0, j = 2; j < lineSplit.size(); i++, j++)
				m_PIbeo.ibvx[i] = stringToNum<float>(lineSplit[j]);
			std::getline(in, line);
			split(line, lineSplit, "\t");
			for (int i = 0, j = 2; j < lineSplit.size(); i++, j++)
				m_PIbeo.ibvy[i] = stringToNum<float>(lineSplit[j]);
			std::getline(in, line);
			split(line, lineSplit, "\t");
			for (int i = 0, j = 2; j < lineSplit.size(); i++, j++)
				m_PIbeo.ibw[i] = stringToNum<float>(lineSplit[j]);
			std::getline(in, line);
			split(line, lineSplit, "\t");
			for (int i = 0, j = 2; j < lineSplit.size(); i++, j++)
				m_PIbeo.ibl[i] = stringToNum<float>(lineSplit[j]);
			std::getline(in, line);
			split(line, lineSplit, "\t");
			for (int i = 0, j = 2; j < lineSplit.size(); i++, j++)
				m_PIbeo.ibvabx[i] = stringToNum<float>(lineSplit[j]);
			std::getline(in, line);
			split(line, lineSplit, "\t");
			for (int i = 0, j = 2; j < lineSplit.size(); i++, j++)
				m_PIbeo.ibvaby[i] = stringToNum<float>(lineSplit[j]);
			std::getline(in, line);
			split(line, lineSplit, "\t");
			for (int i = 0, j = 2; j < lineSplit.size(); i++, j++)
				m_PIbeo.ibcontournum[i] = stringToNum<int>(lineSplit[j]);

			//if (!(ibeostyle % 2))
			if ((ibeostyle % 2))
			//if ((ibeostyle == 3))
			//if ((ibeostyle==1) || (ibeostyle ==0))
			{
				std::getline(in, line);
				std::getline(in, line);
				continue;
			}

			raw_ibeocontourx.clear();
			raw_ibeocontoury.clear();
			std::getline(in, line);
			split(line, lineSplit, "\t");
			for (int i = 0, j = 2; j < lineSplit.size(); i++, j++)
				//m_PIbeo.ibcontourx[i] = stringToNum<float>(lineSplit[j]);
				raw_ibeocontourx.push_back(stringToNum<float>(lineSplit[j]));
			std::getline(in, line);
			split(line, lineSplit, "\t");
			for (int i = 0, j = 2; j < lineSplit.size(); i++, j++)
				//m_PIbeo.ibcontoury[i] = stringToNum<float>(lineSplit[j]);
				raw_ibeocontoury.push_back(stringToNum<float>(lineSplit[j]));


			//visual::cvtData2Vector(m_PIbeo.ibcontourx, m_PIbeo.ibcontoury, LENGTHOFCONTOUR, raw_ibeocontourx, raw_ibeocontoury, 1.f, 0, true);

			float x0 = 0.f, y0 = 2.85f, angle = 0;
			//CalculateSensorData(raw_ibeocontourx, raw_ibeocontoury, x0, y0, angle);
			////CalculateSensorData(raw_ibeocontourx, raw_ibeocontoury, x0, y0, angle);//旋转
			//x0 = 0.f, y0 = 0.0f, angle = (float)(1.3 / 180.0*(long double)PI64);
			//CalculateSensorData(raw_ibeocontourx, raw_ibeocontoury, x0, y0, angle);//旋转

			for (int i = 0; i < raw_ibeocontoury.size(); i++)
			{
				if (raw_ibeocontoury[i]>50.f)
				{
					raw_ibeocontoury.erase(raw_ibeocontoury.begin() + i);
					raw_ibeocontourx.erase(raw_ibeocontourx.begin() + i);
					i--;
				}
			}

			/*GPSIBEO IBEOdata;
			if (gpsdata.size())
			{
			IBEOdata.gps = RecordGPS;
			IBEOdata.x = raw_ibeocontourx;
			IBEOdata.y = raw_ibeocontoury;
			RecordIBEO.data.push_back(IBEOdata);
			}*/

			CalculateSensorData(raw_ibeocontourx, raw_ibeocontoury, 0, 0, -Dhead);//旋转
			CalculateSensorData(raw_ibeocontourx, raw_ibeocontoury, -GPSx, -GPSy, 0);    //平移
			ibctrx.insert(ibctrx.end(), raw_ibeocontourx.begin(), raw_ibeocontourx.end());
			ibctry.insert(ibctry.end(), raw_ibeocontoury.begin(), raw_ibeocontoury.end());
		}

	}
	cout << "dis=" << distanceofall << endl;
	/*string dirr;
	std::ofstream outFile;
	dirr = "GPSheadingnofilter.txt";
	outFile.open(dirr);
	for (int i = 0; i < headangle2.size(); i++)
	{
	outFile << headangle[i] << '\t' << headangle2[i] << '\t' << gyroscope[i] << endl;
	}
	outFile.close();*/
	/*string dirr2;
	std::ofstream outFile2;
	dirr2 = "GPSXY.txt";
	outFile2.open(dirr2);
	for (int i = 0; i < Savex.size(); i++)
	{
	outFile2 << Savex[i] << '\t' << Savey[i] << '\t' << Savexkalman[i] << '\t' << Saveykalman[i] << endl;
	}
	outFile2.close();*/
	l1x.clear();
	l1y.clear();
	l4x.clear();
	l4y.clear();
	Map.gps = gpsdata;
	Map.ibeox = ibctrx;
	Map.ibeoy = ibctry;
	Map.l1x = l1x; Map.l2x = l2x; Map.l3x = l3x;
	Map.l1y = l1y; Map.l2y = l2y; Map.l3y = l3y;

	Map.l4x.insert(Map.l4x.end(), Map.l2x.begin(), Map.l2x.end());
	Map.l4x.insert(Map.l4x.end(), Map.l3x.begin(), Map.l3x.end());

	Map.l4y.insert(Map.l4y.end(), Map.l2y.begin(), Map.l2y.end());
	Map.l4y.insert(Map.l4y.end(), Map.l3y.begin(), Map.l3y.end());

	Map.x = originx;
	Map.y = originy;
	Map.coordinatezero = 0;
	mapnumber++;
	SaveGenerateGPSMap(Map, mapnumber);
	//SaveRecordIBEOData(RecordIBEO);
}

void BuildmapSLAMPOS(mapdata& Map, string dir)
{
	static int mapnumber;
	PacketVELD m_PVeld;
	PacketRoad m_PRoad;
	PacketTraffic m_PTraffic;
	PacketStopLine m_PStopLine;
	PacketIbeo m_PIbeo;
	visual::VisualizationBasicApi vsb;
	vector <GPS> gpsdata;
	vector <float> GPSfliterheading;
	float t_raw_lane1y[LENGTHOFLANE] = { 0.0f };
	float t_raw_lane2y[LENGTHOFLANE] = { 0.0f };
	float t_raw_lane3y[LENGTHOFLANE] = { 0.0f };
	float t_raw_lane4y[LENGTHOFLANE] = { 0.0f };

	long double raw_lat = 0; long double raw_lon = 0; long double raw_head = 0;
	long double l_lat = 0, l_lon = 0, l_head = 0;
	vector<float> raw_ibeocontourx, raw_ibeocontoury;
	vector<float> ibctrx, ibctry, originx, originy, l1x, l1y, l2x, l2y, l3x, l3y, l4x, l4y;
	vector<float> raw_lane1x;	vector<float> raw_lane2x;	vector<float> raw_lane3x;	vector<float> raw_lane4x;
	vector<float> raw_lane1y;	vector<float> raw_lane2y;	vector<float> raw_lane3y;	vector<float> raw_lane4y;

	float distanceofall = 0;
	int KalmanInit = 0;
	const int stateNum = 4;    //状态值6×1向量(x,y,head,△x,△y,△head)  
	const int measureNum = 2;   //测量值3×1向量(x,y,head)    
	cv::KalmanFilter KF(stateNum, measureNum, 0);
	cv::Mat measurement = cv::Mat::zeros(measureNum, 1, CV_32F);  //初始测量值x'(0)，因为后面要更新这个值，所以必须先定义  
	float dT = 0.03f;

	for (int i = 0; i < 40; i++)
	{
		t_raw_lane1y[i] = (float)i + 2.f;
		t_raw_lane2y[i] = (float)i + 2.f;
		t_raw_lane3y[i] = (float)i + 2.f;
		t_raw_lane4y[i] = (float)i + 2.f;
	}

	string headingname = "GPSheading.txt";
	int gpsfilterflag = 0;
	std::string line;
	int flag = 0;
	float Dhead;
	double GPSx, GPSy;
	float  oldhead, headfilter, headnofilter, ww;
	vector<float> Savex, Savey, Savexkalman, Saveykalman;
	vector<float> headangle, headangle2, gyroscope;
	PacketGPS  RecordGPS;
	RecordIBEO RecordIBEO;
	int gpsi = 0;
	ifstream  in(dir.c_str());
	if (!in)
	{
		cout << "file does not exist." << endl;
		return;
	}
	while (in.peek() != EOF)
	{

		std::vector<std::string> lineSplit;
		std::getline(in, line);
		split(line, lineSplit, "\t");
		if (lineSplit.empty()) break;
		if (0&&lineSplit[1] == "[GPS]")
		{
			raw_lat = stringToNum<long double>(lineSplit[3]);
			raw_lon = stringToNum<long double>(lineSplit[4]);
			raw_head = stringToNum<long double>(lineSplit[5]);
			RecordGPS.lat = raw_lat;
			RecordGPS.lon = raw_lon;
			RecordGPS.heading = raw_head;
			RecordGPS.speed = stringToNum<float>(lineSplit[6]);
			RecordGPS.vx = stringToNum<float>(lineSplit[7]);
			RecordGPS.vy = stringToNum<float>(lineSplit[8]);
			RecordGPS.wz = stringToNum<float>(lineSplit[9]);
			ww = RecordGPS.wz;
			raw_lat = raw_lat / (long double)180 * (long double)PI64;
			raw_lon = raw_lon / (long double)180 * (long double)PI64;
			raw_head = raw_head / (long double)180 * (long double)PI64;

			GPS rawgps;
			rawgps.lat = raw_lat;
			rawgps.lon = raw_lon;
			rawgps.head = raw_head;
			gpsdata.push_back(rawgps);

			if (flag == 0)
			{
				l_lat = raw_lat;
				l_lon = raw_lon;
				l_head = raw_head;
				headfilter = (float)raw_head;
				headnofilter = (float)raw_head;
				oldhead = (float)raw_head;
				flag++;
			}
			if (raw_head - oldhead < -PI64 / 2)
			{
				Dhead = (float)(raw_head - oldhead + 2 * PI64);
			}
			else if (raw_head - oldhead>PI64 / 2)
			{
				Dhead = (float)(raw_head - oldhead - 2 * PI64);
			}
			else
			{
				Dhead = (float)(raw_head - oldhead);
			}
			headnofilter = oldhead + Dhead;
			oldhead = headnofilter;


			gpsi++;
			if (!gpsfilterflag)
			{
				headfilter = headnofilter;
				Dhead = (float)(headfilter - l_head);
			}
			else
			{
				headfilter = GPSfliterheading[gpsi];//高斯过程拟合航向角
				Dhead = (float)(headfilter - l_head);
			}
			//CalculateCoordinate(GPSx, GPSy, rawgps);//计算轨迹全局坐标

			//double x, y;
			Global2_Local(GPSx, GPSy, raw_lat, raw_lon, l_lat, l_lon, l_head);
			if (Savex.size() > 2)
			{
				distanceofall += Computedistance(GPSx, GPSy, Savex.back(), Savey.back());
			}
			Savex.push_back((float)GPSx);
			Savey.push_back((float)GPSy);

			Savexkalman.push_back((float)GPSx);
			Saveykalman.push_back((float)GPSy);
			headangle.push_back(headfilter);
			headangle2.push_back(headnofilter);
			gyroscope.push_back(ww);

			originx.push_back((float)GPSx);
			originy.push_back((float)GPSy);

		}
		else if (lineSplit[1] == "[SLPS]")
		{
			float slamposx = 0, slamposy = 0, slamposz = 0, slamposheading = 0;
			slamposx = stringToNum<float>(lineSplit[3]);
			slamposy = stringToNum<float>(lineSplit[4]);
			slamposz = stringToNum<float>(lineSplit[5]);
			slamposheading = stringToNum<float>(lineSplit[6]);
			slamposheading = slamposheading / (long double)180 * (long double)PI64;;

			if (flag == 0)
			{
				flag++;
				oldhead = slamposheading;
			}
			GPSx = slamposx;
			GPSy = slamposy;
			headfilter = slamposheading;

			/*if (slamposheading - oldhead < -PI64 / 2)
			{
			Dhead = (float)(slamposheading - oldhead + 2 * PI64);
			}
			else if (slamposheading - oldhead>PI64 / 2)
			{
			Dhead = (float)(slamposheading - oldhead - 2 * PI64);
			}
			else*/
			{
				Dhead = (float)(slamposheading - oldhead);
			}
			//oldhead = slamposheading;
			GPS rawgps;
			rawgps.x = GPSx;
			rawgps.y = GPSy;
			gpsdata.push_back(rawgps);

			originx.push_back((float)GPSx);
			originy.push_back((float)GPSy);

			if (Savex.size() > 2)
			{
				distanceofall += Computedistance(GPSx, GPSy, Savex.back(), Savey.back());
			}
			Savex.push_back((float)GPSx);
			Savey.push_back((float)GPSy);

		}
		else if (lineSplit[1] == "[VELD]")
		{
			if (!flag) continue;
			if (lineSplit.size() < 10)
			{
				std::getline(in, line);
				split(line, lineSplit, "\t");
			}

			for (int i = 0, j = 2; j < lineSplit.size(); i++, j++)
				m_PVeld.Lane1x[i] = stringToNum<float>(lineSplit[j]);
			std::getline(in, line);
			split(line, lineSplit, "\t");
			for (int i = 0, j = 2; j < lineSplit.size(); i++, j++)
				m_PVeld.Lane2x[i] = stringToNum<float>(lineSplit[j]);
			std::getline(in, line);
			split(line, lineSplit, "\t");
			for (int i = 0, j = 2; j < lineSplit.size(); i++, j++)
				m_PVeld.Lane3x[i] = stringToNum<float>(lineSplit[j]);
			std::getline(in, line);
			split(line, lineSplit, "\t");
			for (int i = 0, j = 2; j < lineSplit.size(); i++, j++)
				m_PVeld.Lane4x[i] = stringToNum<float>(lineSplit[j]);
			for (int i = 0; i < LENGTHOFLANE; i++)
			{
				if (i>15)
				{
					m_PVeld.Lane1x[i] = 999;
					m_PVeld.Lane2x[i] = 999;
					m_PVeld.Lane3x[i] = 999;
					m_PVeld.Lane4x[i] = 999;
				}
			}
			visual::cvtData2VectorForVeld(m_PVeld.Lane1x, t_raw_lane1y, LENGTHOFLANE, raw_lane1x, raw_lane1y);
			visual::cvtData2VectorForVeld(m_PVeld.Lane2x, t_raw_lane2y, LENGTHOFLANE, raw_lane2x, raw_lane2y);
			visual::cvtData2VectorForVeld(m_PVeld.Lane3x, t_raw_lane3y, LENGTHOFLANE, raw_lane3x, raw_lane3y);
			visual::cvtData2VectorForVeld(m_PVeld.Lane4x, t_raw_lane4y, LENGTHOFLANE, raw_lane4x, raw_lane4y);

			float x0 = 0.f, y0 = 0.f, angle = 0;
			//CalculateSensorData(raw_ibeocontourx, raw_ibeocontoury, x0, y0, angle);
			//x0 = 0.f, y0 = 0.0f, angle = (float)(parameter/100.f/ 180.f*PI64);

			CalculateSensorData(raw_lane1x, raw_lane1y, 0, 0, -Dhead);//旋转
			CalculateSensorData(raw_lane1x, raw_lane1y, -GPSx, -GPSy, 0);    //平移
			CalculateSensorData(raw_lane2x, raw_lane2y, 0, 0, -Dhead);//旋转
			CalculateSensorData(raw_lane2x, raw_lane2y, -GPSx, -GPSy, 0);    //平移
			CalculateSensorData(raw_lane3x, raw_lane3y, 0, 0, -Dhead);//旋转
			CalculateSensorData(raw_lane3x, raw_lane3y, -GPSx, -GPSy, 0);    //平移
			CalculateSensorData(raw_lane4x, raw_lane4y, 0, 0, -Dhead);//旋转
			CalculateSensorData(raw_lane4x, raw_lane4y, -GPSx, -GPSy, 0);    //平移

			l1x.insert(l1x.end(), raw_lane1x.begin(), raw_lane1x.end());
			l1y.insert(l1y.end(), raw_lane1y.begin(), raw_lane1y.end());
			l2x.insert(l2x.end(), raw_lane2x.begin(), raw_lane2x.end());
			l2y.insert(l2y.end(), raw_lane2y.begin(), raw_lane2y.end());
			l3x.insert(l3x.end(), raw_lane3x.begin(), raw_lane3x.end());
			l3y.insert(l3y.end(), raw_lane3y.begin(), raw_lane3y.end());
			l4x.insert(l4x.end(), raw_lane4x.begin(), raw_lane4x.end());
			l4y.insert(l4y.end(), raw_lane4y.begin(), raw_lane4y.end());
		}
		else if (lineSplit[1] == "[ROAD]")
		{
			for (int i = 0, j = 2; j < lineSplit.size(); i++, j++)
				m_PRoad.leftx[i] = stringToNum<int>(lineSplit[j]);
			std::getline(in, line);
			split(line, lineSplit, "\t");
			for (int i = 0, j = 2; j < lineSplit.size(); i++, j++)
				m_PRoad.lefty[i] = stringToNum<int>(lineSplit[j]);
			std::getline(in, line);
			split(line, lineSplit, "\t");
			for (int i = 0, j = 2; j < lineSplit.size(); i++, j++)
				m_PRoad.rightx[i] = stringToNum<int>(lineSplit[j]);
			std::getline(in, line);
			split(line, lineSplit, "\t");
			for (int i = 0, j = 2; j < lineSplit.size(); i++, j++)
				m_PRoad.righty[i] = stringToNum<int>(lineSplit[j]);
		}
		else if (lineSplit[1] == "[STOP]")
		{
			m_PStopLine.stopline = stringToNum<int>(lineSplit[2]);
		}
		else if (lineSplit[1] == "[TRFL]")
		{
			m_PTraffic.trafficlight = stringToNum<int>(lineSplit[2]);
		}
		else if (lineSplit[1] == "[IBEO]")
		{
			if (!flag) continue;
			int ibeostyle = 0;
			if (lineSplit.size() < 5)
			{
				ibeostyle = stringToNum<int>(lineSplit[2]);
				std::getline(in, line);
				split(line, lineSplit, "\t");
			}
			for (int i = 0, j = 2; j < lineSplit.size(); i++, j++)
				m_PIbeo.ibx[i] = stringToNum<float>(lineSplit[j]);
			std::getline(in, line);
			split(line, lineSplit, "\t");
			for (int i = 0, j = 2; j < lineSplit.size(); i++, j++)
				m_PIbeo.iby[i] = stringToNum<float>(lineSplit[j]);
			std::getline(in, line);
			split(line, lineSplit, "\t");
			for (int i = 0, j = 2; j < lineSplit.size(); i++, j++)
				m_PIbeo.ibvx[i] = stringToNum<float>(lineSplit[j]);
			std::getline(in, line);
			split(line, lineSplit, "\t");
			for (int i = 0, j = 2; j < lineSplit.size(); i++, j++)
				m_PIbeo.ibvy[i] = stringToNum<float>(lineSplit[j]);
			std::getline(in, line);
			split(line, lineSplit, "\t");
			for (int i = 0, j = 2; j < lineSplit.size(); i++, j++)
				m_PIbeo.ibw[i] = stringToNum<float>(lineSplit[j]);
			std::getline(in, line);
			split(line, lineSplit, "\t");
			for (int i = 0, j = 2; j < lineSplit.size(); i++, j++)
				m_PIbeo.ibl[i] = stringToNum<float>(lineSplit[j]);
			std::getline(in, line);
			split(line, lineSplit, "\t");
			for (int i = 0, j = 2; j < lineSplit.size(); i++, j++)
				m_PIbeo.ibvabx[i] = stringToNum<float>(lineSplit[j]);
			std::getline(in, line);
			split(line, lineSplit, "\t");
			for (int i = 0, j = 2; j < lineSplit.size(); i++, j++)
				m_PIbeo.ibvaby[i] = stringToNum<float>(lineSplit[j]);
			std::getline(in, line);
			split(line, lineSplit, "\t");
			for (int i = 0, j = 2; j < lineSplit.size(); i++, j++)
				m_PIbeo.ibcontournum[i] = stringToNum<int>(lineSplit[j]);

			//if (!(ibeostyle % 2))
			if ((ibeostyle % 2))
				//if ((ibeostyle == 3))
				//if ((ibeostyle==1) || (ibeostyle ==0))
			{
				std::getline(in, line);
				std::getline(in, line);
				continue;
			}

			raw_ibeocontourx.clear();
			raw_ibeocontoury.clear();
			std::getline(in, line);
			split(line, lineSplit, "\t");
			for (int i = 0, j = 2; j < lineSplit.size(); i++, j++)
				//m_PIbeo.ibcontourx[i] = stringToNum<float>(lineSplit[j]);
				raw_ibeocontourx.push_back(stringToNum<float>(lineSplit[j]));
			std::getline(in, line);
			split(line, lineSplit, "\t");
			for (int i = 0, j = 2; j < lineSplit.size(); i++, j++)
				//m_PIbeo.ibcontoury[i] = stringToNum<float>(lineSplit[j]);
				raw_ibeocontoury.push_back(stringToNum<float>(lineSplit[j]));


			//visual::cvtData2Vector(m_PIbeo.ibcontourx, m_PIbeo.ibcontoury, LENGTHOFCONTOUR, raw_ibeocontourx, raw_ibeocontoury, 1.f, 0, true);

			float x0 = 0.f, y0 = 2.85f, angle = 0;
			//CalculateSensorData(raw_ibeocontourx, raw_ibeocontoury, x0, y0, angle);
			////CalculateSensorData(raw_ibeocontourx, raw_ibeocontoury, x0, y0, angle);//旋转
			//x0 = 0.f, y0 = 0.0f, angle = (float)(1.3 / 180.0*(long double)PI64);
			//CalculateSensorData(raw_ibeocontourx, raw_ibeocontoury, x0, y0, angle);//旋转

			for (int i = 0; i < raw_ibeocontoury.size(); i++)
			{
				if (raw_ibeocontoury[i]>50.f)
				{
					raw_ibeocontoury.erase(raw_ibeocontoury.begin() + i);
					raw_ibeocontourx.erase(raw_ibeocontourx.begin() + i);
					i--;
				}
			}

			/*GPSIBEO IBEOdata;
			if (gpsdata.size())
			{
			IBEOdata.gps = RecordGPS;
			IBEOdata.x = raw_ibeocontourx;
			IBEOdata.y = raw_ibeocontoury;
			RecordIBEO.data.push_back(IBEOdata);
			}*/

			CalculateSensorData(raw_ibeocontourx, raw_ibeocontoury, 0, 0, -Dhead);//旋转
			CalculateSensorData(raw_ibeocontourx, raw_ibeocontoury, -GPSx, -GPSy, 0);    //平移
			ibctrx.insert(ibctrx.end(), raw_ibeocontourx.begin(), raw_ibeocontourx.end());
			ibctry.insert(ibctry.end(), raw_ibeocontoury.begin(), raw_ibeocontoury.end());
		}

	}
	cout << "dis=" << distanceofall << endl;
	/*string dirr;
	std::ofstream outFile;
	dirr = "GPSheadingnofilter.txt";
	outFile.open(dirr);
	for (int i = 0; i < headangle2.size(); i++)
	{
	outFile << headangle[i] << '\t' << headangle2[i] << '\t' << gyroscope[i] << endl;
	}
	outFile.close();*/
	/*string dirr2;
	std::ofstream outFile2;
	dirr2 = "GPSXY.txt";
	outFile2.open(dirr2);
	for (int i = 0; i < Savex.size(); i++)
	{
	outFile2 << Savex[i] << '\t' << Savey[i] << '\t' << Savexkalman[i] << '\t' << Saveykalman[i] << endl;
	}
	outFile2.close();*/
	l1x.clear();
	l1y.clear();
	l4x.clear();
	l4y.clear();
	Map.gps = gpsdata;
	Map.ibeox = ibctrx;
	Map.ibeoy = ibctry;
	Map.l1x = l1x; Map.l2x = l2x; Map.l3x = l3x;
	Map.l1y = l1y; Map.l2y = l2y; Map.l3y = l3y;

	Map.l4x.insert(Map.l4x.end(), Map.l2x.begin(), Map.l2x.end());
	Map.l4x.insert(Map.l4x.end(), Map.l3x.begin(), Map.l3x.end());

	Map.l4y.insert(Map.l4y.end(), Map.l2y.begin(), Map.l2y.end());
	Map.l4y.insert(Map.l4y.end(), Map.l3y.begin(), Map.l3y.end());

	Map.x = originx;
	Map.y = originy;
	Map.coordinatezero = 0;
	mapnumber++;
	SaveGenerateGPSMap(Map, mapnumber);
	//SaveRecordIBEOData(RecordIBEO);
}
//合成地图
void BuilGoalMapGPS(mapdata& GoalMap, mapdata&newmap)
{
	if (GoalMap.gps.size() < 1) return;
	double x0, y0;
	float dhead;
	long double lat, lon, head;
	long double lat_, lon_, head_;
	lat = GoalMap.gps[0].lat;
	lon = GoalMap.gps[0].lon;
	head =  GoalMap.gps[0].head;

	lat_ = newmap.gps[0].lat;
	lon_ = newmap.gps[0].lon;
	head_ =  newmap.gps[0].head;

	dhead = (float)(head_ - head);
	x0 = 0.f;
	y0 = 0.f;
	dhead = -dhead;
	CalculateSensorData(newmap.ibeox, newmap.ibeoy, x0, y0, dhead);
	CalculateSensorData(newmap.l4x, newmap.l4y, x0, y0, dhead);

	Global2_Local(x0, y0, lat_, lon_, lat, lon, head);
	dhead = 0;
	x0 = -x0;
	y0 = -y0;
	CalculateSensorData(newmap.ibeox, newmap.ibeoy, x0, y0, dhead);
	CalculateSensorData(newmap.l4x, newmap.l4y, x0, y0, dhead);

	GoalMap.ibeox.insert(GoalMap.ibeox.end(), newmap.ibeox.begin(), newmap.ibeox.end());
	GoalMap.ibeoy.insert(GoalMap.ibeoy.end(), newmap.ibeoy.begin(), newmap.ibeoy.end());
	GoalMap.l4x.insert(GoalMap.l4x.end(), newmap.l4x.begin(), newmap.l4x.end());
	GoalMap.l4y.insert(GoalMap.l4y.end(), newmap.l4y.begin(), newmap.l4y.end());
	GoalMap.coordinatezero = 0;
}

void BuilGoalMapUTM(mapdata& GoalMap, mapdata&newmap)
{
	if (GoalMap.gps.size() < 1) return;

	//const my_Ellipse* e = standard_ellipse(ELLIPSE_WGS84);
	//GridZone zone = GRID_AUTO;
	//Hemisphere hemi = HEMI_AUTO;

	/*double yUTM_init, xUTM_init;
	double x0, y0;
	float dhead;
	long double lat, lon, head;
	long double lat_, lon_, head_;
	lat = GoalMap.gps[0].lat;
	lon = GoalMap.gps[0].lon;
	head = GoalMap.gps[0].head;

	lat_ = newmap.gps[0].lat;
	lon_ = newmap.gps[0].lon;
	head_ = newmap.gps[0].head;
*/
	//geographic_to_grid(e->a, e->e2, l_lat, l_lon, &zone, &hemi, &yUTM_init, &xUTM_init);

	/*dhead = (float)(head_ - head);
	x0 = 0.f;
	y0 = 0.f;
	dhead = -dhead;
	CalculateSensorData(newmap.ibeox, newmap.ibeoy, x0, y0, dhead);
	CalculateSensorData(newmap.l4x, newmap.l4y, x0, y0, dhead);

	Global2_Local(x0, y0, lat_, lon_, lat, lon, head);
	dhead = 0;
	x0 = -x0;
	y0 = -y0;
	CalculateSensorData(newmap.ibeox, newmap.ibeoy, x0, y0, dhead);
	CalculateSensorData(newmap.l4x, newmap.l4y, x0, y0, dhead);*/

	GoalMap.ibeox.insert(GoalMap.ibeox.end(), newmap.ibeox.begin(), newmap.ibeox.end());
	GoalMap.ibeoy.insert(GoalMap.ibeoy.end(), newmap.ibeoy.begin(), newmap.ibeoy.end());
	GoalMap.l4x.insert(GoalMap.l4x.end(), newmap.l4x.begin(), newmap.l4x.end());
	GoalMap.l4y.insert(GoalMap.l4y.end(), newmap.l4y.begin(), newmap.l4y.end());
	GoalMap.coordinatezero = 0;
}

void SaveRecordIBEOData(RecordIBEO &DATA)
{
	//存储单帧点云
	string dirr;
	std::ofstream outFile;
	dirr = "RecordIBEO.txt";
	outFile.open(dirr);
	for (size_t i = 0; i < DATA.data.size(); i++)
	{
		saveGPSData(outFile, DATA.data[i].gps.lat, DATA.data[i].gps.lon, DATA.data[i].gps.heading,0,0);
		saveDataVELD2(outFile, DATA.data[i].x, DATA.data[i].y, 2, "IBEO");
	}
	outFile.close();
	cout << "RecordIBEO.txt is saved!" << endl;
}
void SaveGenerateGPSMap(mapdata map, int index)
{
	string dir;
	string tostring;
	std::ofstream outFile;
	tostring = toString(index);
	string mapname = tostring+"GPSMap.txt";
	outFile.open(mapname);
	for (int i = 0; i < map.gps.size(); i++)
	{
		long double lat, lon, head;
		lat = map.gps[i].lat *(long double)180 / (long double)PI64;
		lon = map.gps[i].lon *(long double)180 / (long double)PI64;
		head = map.gps[i].head *(long double)180 / (long double)PI64;
		saveGPSData(outFile, lat, lon, head, map.x[i], map.y[i]);
	}
	outFile.close();
	cout << "GPSMap.txt is saved!" << endl;

	 mapname = tostring + "SlamPosMap.txt";
	outFile.open(mapname);
	float x, y;
	for (int i = 0; i < map.gps.size(); i++)
	{
		long double lat, lon, head;
		x = map.gps[i].x;
		y = map.gps[i].y;
		head = (map.gps[i].head - map.gps[0].head) *(long double)180 / (long double)PI64;
		outFile << setiosflags(ios::fixed) << setprecision(3);
		outFile << 0 << "\t";
		outFile << "[SLAMPOS]\t" << 0 << "\t" << x << "\t" << y << "\t" << head << "\t" << setprecision(3);
		outFile << 0 << "\t" << 0 << "\t" << std::endl;

	}
	outFile.close();
	//cout << "GPSMap.txt is saved!" << endl;

}
void SaveMap(mapdata& map,string name)
{
	if (map.gps.size() < 1) return;
	std::ofstream outFile;
	long double lat, lon, head;
	outFile.open(name);

	lat = map.gps[0].lat *(long double)180 / (long double)PI64;
	lon = map.gps[0].lon *(long double)180 / (long double)PI64;
	head = 0;// map.gps[0].head *(long double)180 / (long double)PI64;
	saveGPSData(outFile, lat, lon, head, map.x[0], map.y[0]);
	saveData(outFile, map.l1x, map.l1y, 2, "VELD");
	saveData(outFile, map.l2x, map.l2y, 2, "VELD");
	saveData(outFile, map.l3x, map.l3y, 2, "VELD");
	saveData(outFile, map.l4x, map.l4y, 2, "VELD");
	saveData(outFile, map.ibeox, map.ibeoy, 2, "IBEO");

	outFile.close();
}
void SaveDealLaneMap(mapdata& map, vector<LaneCell>& lane, string name)
{

	std::ofstream outFile;
	long double lat, lon, head;
	outFile.open(name);
	//地图原点
	lat = map.gps[0].lat *(long double)180 / (long double)PI64;
	lon = map.gps[0].lon *(long double)180 / (long double)PI64;
	head = map.gps[0].head *(long double)180 / (long double)PI64;
	saveGPSData(outFile, lat, lon, head, map.x[0], map.y[0]);

	saveData(outFile, map.l1x, map.l1y, 3, "VELD");
	saveData(outFile, map.l1x, map.l1y, 3, "VELD");
	saveData(outFile, map.l1x, map.l1y, 3, "VELD");
	saveData(outFile, map.l4x, map.l4y, 3, "VELD");
	saveData(outFile, map.ibeox, map.ibeoy, 3, "IBEO");
	for (int i = 0; i < lane.size(); i++)
	{
		if (lane[i].LaneData.x.size()>3)
		saveData(outFile, lane[i].LaneData.x, lane[i].LaneData.y, 2, "VELDINDEX");
	}
	outFile.close();
}
void SaveGPSRoadMap(mapdata&map,string name)
{
	std::ofstream outFile;
	outFile.open(name);
	for (int i = 1; i < map.gps.size(); i++)
	{
		if (map.gps[i].flag)
		{
			saveGPSRoadData(outFile, map.gps[i].flag, map.gps[i].lat, map.gps[i].lon, map.gps[i].head);
		}
		
	}
	outFile.close();
}
void SaveEditGPSRoadMap(mapdata&map, string name)
{
	std::ofstream outFile;
	outFile.open(name);
	for (int i = 1; i < map.x.size(); i++)
	{
			saveGPSRoadData(outFile, map.gps[i].flag, map.gps[i].lat, map.gps[i].lon, map.gps[i].head, map.x[i], map.y[i]);
	}
	outFile.close();
}
void SaveEditMap(mapdata& map, string name)
{

	std::ofstream outFile;
	long double lat, lon, head;
	outFile.open(name);
		lat = map.gps[0].lat *(long double)180 / (long double)PI64;
		lon = map.gps[0].lon *(long double)180 / (long double)PI64;
		head = map.gps[0].head *(long double)180 / (long double)PI64;
		saveGPSData(outFile, lat, lon, head, map.x[0], map.y[0]);
	saveData(outFile, map.stopx, map.stopy, 3, "STOP");
	saveData(outFile, map.l1x, map.l1y, 3, "VELD");
	saveData(outFile, map.l2x, map.l2y, 3, "VELD");
	saveData(outFile, map.l3x, map.l3y, 3, "VELD");
	saveData(outFile, map.l4x, map.l4y, 3, "VELD");
	saveData(outFile, map.ibeox, map.ibeoy, 3, "IBEO");
	for (int i = 0; i < map.Lane.size(); i++)
	{
		if (map.Lane[i].x.size()>3)
			saveData(outFile, map.Lane[i].x, map.Lane[i].y, 2, "VELDINDEX");
	}

	outFile.close();
}
//激光点云
void SaveOccupyGridMap(mapdata& map, string name,float scale)
{
	std::ofstream outFile;
	long double lat, lon, head;
	float minx, miny, maxx, maxy;
	vector <float> gridx, gridy;
	vector<int> style(map.ibeox.size(),1);
	vector<int> styleveld(map.l4x.size(),2);
	scale = scale / 100.f;
	outFile.open(name);
	lat = map.gps[0].lat *(long double)180 / (long double)PI64;
	lon = map.gps[0].lon *(long double)180 / (long double)PI64;
	head = map.gps[0].head *(long double)180 / (long double)PI64;
	gridx = map.ibeox;
	gridy = map.ibeoy;
	//gridx.insert(gridx.end(), map.l4x.begin(), map.l4x.end());
	//gridy.insert(gridy.end(), map.l4y.begin(), map.l4y.end());
	//style.insert(style.end(), styleveld.begin(), styleveld.end());
	calcMapWidthandHeight(map.ibeox, map.ibeoy, maxx, maxy, minx, miny);

	outFile << "[MAPNUM]\t" << gridx.size() << endl;
	outFile << "[SCALE]\t" << scale << endl;
	outFile << "[WIDTH]\t" << (int)((maxx - minx) / scale)+1 << endl;
	outFile << "[HEIGHT]\t" << (int)((maxy - miny) / scale) + 1 << endl;
	outFile << "[XOFFSET]\t" << (int)(-minx / scale) << endl;
	outFile << "[YOFFSET]\t" << (int)(-miny / scale) << endl;
	saveGPSData(outFile, lat, lon, head, 0, 0);
	//if (minx<0)
	xOffset(map.ibeox, -minx);
	//if (miny<0)
	xOffset(map.ibeoy, -miny);
	//saveData(outFile, map.l4x, map.l4y, 3, "VELD");
	saveOccupyGridData(outFile, map.ibeox, map.ibeoy, style, scale);

	outFile.close();
}
//车道线+激光
void SaveOccupyGridMap2(mapdata& map, string name, float scale, int SubmapSize)
{
	std::ofstream outFile;
	long double lat, lon, head;
	float minx, miny, maxx, maxy;
	vector <float> gridx, gridy;
	vector <float> indexx, indexy;
	vector<int> style(map.ibeox.size(), 1);
	int lanecount = 100;
	vector<int> stylestop(map.stopx.size(), 3);

	double xUTM_init, yUTM_init;
	yUTM_init = 3463220.5963258;
	xUTM_init = 329241.61917934;

	scale = scale / 100.f;
	outFile.open(name);
	lat = map.gps[0].lat *(long double)180 / (long double)PI64;
	lon = map.gps[0].lon *(long double)180 / (long double)PI64;
	head = map.gps[0].head *(long double)180 / (long double)PI64;
	gridx = map.ibeox;
	gridy = map.ibeoy;
	gridx.insert(gridx.begin(), map.stopx.begin(), map.stopx.end());
	gridy.insert(gridy.begin(), map.stopy.begin(), map.stopy.end());
	style.insert(style.begin(), stylestop.begin(), stylestop.end());


	for (size_t i = 0; i < map.Lane.size(); i++)
	{
		
		vector<int> styleveld(map.Lane[i].x.size(), lanecount + i);
		style.insert(style.end(), styleveld.begin(), styleveld.end());
		gridx.insert(gridx.end(), map.Lane[i].x.begin(), map.Lane[i].x.end());
		gridy.insert(gridy.end(), map.Lane[i].y.begin(), map.Lane[i].y.end());
	}
	calcMapWidthandHeight(gridx, gridy, maxx, maxy, minx, miny);
	outFile << "[MAPNUM]\t" << gridx.size() << endl;
	outFile << "[SCALE]\t" << scale << endl;
	outFile << "[INDEXSCALE]\t" << SubmapSize << endl;
	outFile << "[WIDTH]\t" << (int)((maxx - minx) / scale) + 1 << endl;
	outFile << "[HEIGHT]\t" << (int)((maxy - miny) / scale) + 1 << endl;
	outFile << "[XOFFSET]\t" << (int)(-minx / scale) << endl;
	outFile << "[YOFFSET]\t" << (int)(-miny / scale) << endl;
	saveGPSData(outFile, lat, lon, head, 0, 0);
	//if (minx<0)
	xOffset(gridx, -minx);
	//if (miny<0)
	xOffset(gridy, -miny);
	indexx = gridx;
	indexy = gridy;
	rasterizeDataGrid(indexx, indexy, SubmapSize * 100, 1);
	saveOccupyGridData(outFile, gridx, gridy, style, scale);
	saveData(outFile,indexx,indexy,0,"INDEX");
	outFile.close();
}
void SaveMap(mapdata& map, vector<float>mousex, vector<float>mousey,string name)
{
	std::ofstream outFile;
	long double lat, lon, head;
	outFile.open(name);
	for (int i = 0; i < map.gps.size(); i++)
	{
		lat = map.gps[i].lat *(long double)180 / (long double)PI64;
		lon = map.gps[i].lon *(long double)180 / (long double)PI64;
		head = map.gps[i].head *(long double)180 / (long double)PI64;
		saveGPSData(outFile, lat, lon, head, map.x[i], map.y[i]);
	}

	saveData(outFile, map.l1x, map.l1y, 3, "VELD");
	saveData(outFile, map.l2x, map.l2y, 3, "VELD");
	saveData(outFile, map.l3x, map.l3y, 3, "VELD");
	saveData(outFile, map.l4x, map.l4y, 3, "VELD");
	saveData(outFile, map.ibeox, map.ibeoy, 3, "IBEO");
	saveData(outFile, mousex, mousey, 3, "MOUSE");
	outFile.close();
}
//加载GPS地图
void LoadGPSMap(mapdata&gGPSMap)
{
	std::string fileName;
	OpenFile(fileName);
	int flag = 0;
	GPS gpszero;
	vector<float> gpsx, gpsy;
	ifstream in(fileName);
	if (!in)
	{
		int ret = MessageBox(nullptr, "Click OK to exit", "File Error", MB_OK | MB_ICONERROR);
		if (ret == MB_OK) exit(-1);
	}
	int i = 0;
	std::string line;
	while (in.peek() != EOF)
	{
		std::vector<std::string> lineSplit;
		std::getline(in, line);
		split(line, lineSplit, "\t");
		if (lineSplit[1] == "[GPS]")
		{
			GPS gps;
			double x, y;
			gps.flag = stringToNum<int>(lineSplit[0]);
			gps.lat = stringToNum<long double>(lineSplit[3]);
			gps.lon = stringToNum<long double>(lineSplit[4]);
			gps.head = stringToNum<long double>(lineSplit[5]);
			gps.lat = gps.lat / (long double)180 * (long double)PI64;
			gps.lon = gps.lon / (long double)180 * (long double)PI64;
			gps.head = gps.head / (long double)180 * (long double)PI64;
			if (!flag)
			{
				gpszero.lat = gps.lat;
				gpszero.lon = gps.lon;
				gpszero.head = gps.head;
				flag++;
			}
			Global2_Local(x, y, gps.lat, gps.lon, gpszero.lat, gpszero.lon, gpszero.head);
			gps.x = (float)x;
			gps.y = (float)y;
			gpsx.push_back((float)x);
			gpsy.push_back((float)y);
			gGPSMap.gps.push_back(gps);

		}
	}
	gGPSMap.coordinatezero = 0;
	gGPSMap.x = gpsx;
	gGPSMap.y = gpsy;
}
void LoadSlamPOSMap(mapdata&gSlamPOSMap,string dir)
{
	vector<float> gpsx, gpsy;
	ifstream in(dir);
	if (!in)
	{
		int ret = MessageBox(nullptr, "Click OK to exit", "File Error", MB_OK | MB_ICONERROR);
		if (ret == MB_OK) exit(-1);
	}
	int i = 0;
	std::string line;
	while (in.peek() != EOF)
	{
		std::vector<std::string> lineSplit;
		std::getline(in, line);
		split(line, lineSplit, "\t");
		if (lineSplit[1] == "[SLAMPOS]")
		{
			GPS gps;
			float x, y;
			gps.flag = stringToNum<int>(lineSplit[0]);
			x = stringToNum< float>(lineSplit[3]);
			y = stringToNum< float>(lineSplit[4]);
			gps.head = stringToNum< double>(lineSplit[5]);
			gps.head = gps.head / (long double)180 * (long double)PI64;		
			gps.x = x;
			gps.y = y;
			gpsx.push_back(x);
			gpsy.push_back(y);
			gSlamPOSMap.gps.push_back(gps);

		}
	}
	gSlamPOSMap.x = gpsx;
	gSlamPOSMap.y = gpsy;
}
void LoadGPSMap(mapdata&gGPSMap,GPS gpszero)
{

	std::string fileName;
	OpenFile(fileName);
	int flag = 0;
	ifstream in(fileName);
	if (!in)
	{
		int ret = MessageBox(nullptr, "Click OK to exit", "File Error", MB_OK | MB_ICONERROR);
		if (ret == MB_OK) exit(-1);
	}
	int i = 0;
	std::string line;
	while (in.peek() != EOF)
	{
		std::vector<std::string> lineSplit;
		std::getline(in, line);
		split(line, lineSplit, "\t");
		if (lineSplit[1] == "[GPS]")
		{
			GPS gps;
			double x, y;
			gps.flag = stringToNum<int>(lineSplit[0]);
			gps.lat = stringToNum<long double>(lineSplit[3]);
			gps.lon = stringToNum<long double>(lineSplit[4]);
			gps.head = stringToNum<long double>(lineSplit[5]);
			gps.lat = gps.lat / (long double)180 * (long double)PI64;
			gps.lon = gps.lon / (long double)180 * (long double)PI64;
			gps.head = gps.head / (long double)180 * (long double)PI64;
			
			Global2_Local(x, y, gps.lat, gps.lon, gpszero.lat, gpszero.lon, gpszero.head);
			gps.x = (float)x;
			gps.y = (float)y;
			gGPSMap.x.push_back((float)x);
			gGPSMap.y.push_back((float)y);
			gGPSMap.gps.push_back(gps);

		}
	}
	gGPSMap.coordinatezero = 0;
}
void LoadGPSMap(mapdata&gGPSMap, GPS gpszero,string dir)
{

	std::string fileName;
	//OpenFile(fileName);
	fileName = dir;
	int flag = 0;
	ifstream in(fileName);
	if (!in)
	{
		int ret = MessageBox(nullptr, "Click OK to exit", "File Error", MB_OK | MB_ICONERROR);
		if (ret == MB_OK) exit(-1);
	}
	int i = 0;
	std::string line;
	while (in.peek() != EOF)
	{
		std::vector<std::string> lineSplit;
		std::getline(in, line);
		split(line, lineSplit, "\t");
		if (lineSplit[1] == "[GPS]")
		{
			GPS gps;
			double x, y;
			gps.flag = stringToNum<int>(lineSplit[0]);
			gps.lat = stringToNum<long double>(lineSplit[3]);
			gps.lon = stringToNum<long double>(lineSplit[4]);
			gps.head = stringToNum<long double>(lineSplit[5]);
			gps.lat = gps.lat / (long double)180 * (long double)PI64;
			gps.lon = gps.lon / (long double)180 * (long double)PI64;
			gps.head = gps.head / (long double)180 * (long double)PI64;

			Global2_Local(x, y, gps.lat, gps.lon, gpszero.lat, gpszero.lon, gpszero.head);
			gps.x = (float)x;
			gps.y = (float)y;
			gGPSMap.x.push_back((float)x);
			gGPSMap.y.push_back((float)y);
			gGPSMap.gps.push_back(gps);
		}
		else if (lineSplit[1] == "[SLAMPOS]")
		{
			GPS gps;
			double x, y,head;
			gps.flag = stringToNum<int>(lineSplit[0]);
			gps.lat = 999;
			gps.lon = 999;

			x = stringToNum< double>(lineSplit[3]);
			y = stringToNum< double>(lineSplit[4]);
			head = stringToNum< double>(lineSplit[5]);
			
			gps.x = (float)x;
			gps.y = (float)y;
			gps.head = (float)head;
			gGPSMap.x.push_back((float)x);
			gGPSMap.y.push_back((float)y);
			gGPSMap.gps.push_back(gps);
		}
	}
	gGPSMap.coordinatezero = 0;

	ofstream outFile;
	outFile.open("C://D//MapData//HD//island//SLAMPOS2.txt");
	for (int i = 0; i < gGPSMap.x.size(); i++)
	{
		outFile << 0 << "\t" << "[SLAMPOS]\t" << 0 << "\t" << gGPSMap.x[i] << "\t" << gGPSMap.y[i] << "\t" << 0 << "\t" << endl;
	}
	outFile.close();

}
void LoadGPSMapUTM(mapdata&gGPSMap, string dir)
{
	const my_Ellipse* e = standard_ellipse(ELLIPSE_WGS84);
	GridZone zone = GRID_AUTO;
	Hemisphere hemi = HEMI_AUTO;
	double xUTM_init, yUTM_init;
	yUTM_init = 3463220.5963258;
	xUTM_init = 329241.61917934;
	


	std::string fileName;
	//OpenFile(fileName);
	fileName = dir;
	int flag = 0;
	ifstream in(fileName);
	if (!in)
	{
		int ret = MessageBox(nullptr, "Click OK to exit", "File Error", MB_OK | MB_ICONERROR);
		if (ret == MB_OK) exit(-1);
	}
	int i = 0;
	std::string line;
	while (in.peek() != EOF)
	{
		std::vector<std::string> lineSplit;
		std::getline(in, line);
		split(line, lineSplit, "\t");
		if (lineSplit[1] == "[GPS]")
		{
			GPS gps;
			double x, y;
			gps.flag = stringToNum<int>(lineSplit[0]);
			gps.lat = stringToNum<long double>(lineSplit[3]);
			gps.lon = stringToNum<long double>(lineSplit[4]);
			gps.head = stringToNum<long double>(lineSplit[5]);
			gps.lat = gps.lat / (long double)180 * (long double)PI64;
			gps.lon = gps.lon / (long double)180 * (long double)PI64;
			gps.head = gps.head / (long double)180 * (long double)PI64;

			//Global2_Local(x, y, gps.lat, gps.lon, gpszero.lat, gpszero.lon, gpszero.head);

			geographic_to_grid(e->a, e->e2, gps.lat, gps.lon, &zone, &hemi, &y, &x);
			x -= xUTM_init;
			y -= yUTM_init;

			gps.x = (float)x;
			gps.y = (float)y;
			gGPSMap.x.push_back((float)x);
			gGPSMap.y.push_back((float)y);
			gGPSMap.gps.push_back(gps);
		}
		else if (lineSplit[1] == "[SLAMPOS]")
		{
			GPS gps;
			double x, y, head;
			gps.flag = stringToNum<int>(lineSplit[0]);
			gps.lat = 999;
			gps.lon = 999;

			x = stringToNum< double>(lineSplit[3]);
			y = stringToNum< double>(lineSplit[4]);
			head = stringToNum< double>(lineSplit[5]);

			gps.x = (float)x;
			gps.y = (float)y;
			gps.head = (float)head;
			gGPSMap.x.push_back((float)x);
			gGPSMap.y.push_back((float)y);
			gGPSMap.gps.push_back(gps);
		}
	}
	gGPSMap.coordinatezero = 0;

	ofstream outFile;
	outFile.open("C://D//MapData//HD//island//SLAMPOS2.txt");
	for (int i = 0; i < gGPSMap.x.size(); i++)
	{
		outFile << 0 << "\t" << "[SLAMPOS]\t" << 0 << "\t" << gGPSMap.x[i] << "\t" << gGPSMap.y[i] << "\t" << 0 << "\t" << endl;
	}
	outFile.close();

}
void LoadGPSXYMap(mapdata&gGPSMap, GPS gpszero, string dir)
{

	std::string fileName;
	//OpenFile(fileName);
	fileName = dir;
	int flag = 0;
	ifstream in(fileName);
	if (!in)
	{
		int ret = MessageBox(nullptr, "Click OK to exit", "File Error", MB_OK | MB_ICONERROR);
		if (ret == MB_OK) exit(-1);
	}
	int i = 0;
	std::string line;
	while (in.peek() != EOF)
	{
		std::vector<std::string> lineSplit;
		std::getline(in, line);
		split(line, lineSplit, "\t");
		if (lineSplit[1] == "[GPS]")
		{
			GPS gps;
			double x, y;
			gps.flag = stringToNum<int>(lineSplit[0]);
			gps.lat = stringToNum<long double>(lineSplit[3]);
			gps.lon = stringToNum<long double>(lineSplit[4]);
			gps.head = stringToNum<long double>(lineSplit[5]);
			x = stringToNum< double>(lineSplit[6]);
			y = stringToNum< double>(lineSplit[7]);
			gps.lat = gps.lat / (long double)180 * (long double)PI64;
			gps.lon = gps.lon / (long double)180 * (long double)PI64;
			gps.head = gps.head / (long double)180 * (long double)PI64;
			//Global2_Local(x, y, gps.lat, gps.lon, gpszero.lat, gpszero.lon, gpszero.head);
			gps.x = (float)x;
			gps.y = (float)y;
			gGPSMap.x.push_back((float)x);
			gGPSMap.y.push_back((float)y);
			gGPSMap.gps.push_back(gps);
		}
	}
	gGPSMap.coordinatezero = 0;

	ofstream outFile;
	outFile.open("C://D//MapData//HD//island//SLAMPOS2.txt");
	for (int i = 0; i < gGPSMap.x.size(); i++)
	{
		outFile << 0 << "\t" << "[SLAMPOS]\t" << 0 << "\t" << gGPSMap.x[i] << "\t" << gGPSMap.y[i] << "\t" << 0 << "\t" << endl;
	}
	outFile.close();

}
//加载高精度地图
void LoadGoalMap(mapdata& Map, string dir)
{

	long double raw_lat = 0; long double raw_lon = 0; long double raw_head = 0;
	vector<float> raw_lane1x;	vector<float> raw_lane2x;	vector<float> raw_lane3x;	vector<float> raw_lane4x;
	vector<float> raw_lane1y;	vector<float> raw_lane2y;	vector<float> raw_lane3y;	vector<float> raw_lane4y;
	vector <float> gpsx, gpsy;
	vector <float> stopx, stopy;
	vector <float> ibeox, ibeoy;
	vector <GPS> gpsdata;
	int Deallanesflag = 0;
	Lanelines Lane;
	vector<Lanelines> Lanes;
	std::string line;
	ifstream  in2(dir.c_str());
	if (!in2)
	{
		cout << "file does not exist." << endl;
		return;
	}
	while (in2.peek() != EOF)
	{
		std::vector<std::string> lineSplit;
		std::getline(in2, line);
		split(line, lineSplit, "\t");
		if (lineSplit.empty()) break;
		if (lineSplit[1] == "[GPS]")
		{
			GPS rawgps;
			float x, y;

			raw_lat = stringToNum<long double>(lineSplit[3]);
			raw_lon = stringToNum<long double>(lineSplit[4]);
			raw_head = stringToNum<long double>(lineSplit[5]);
			x = stringToNum<float>(lineSplit[6]);
			y = stringToNum<float>(lineSplit[7]);

			raw_lat = raw_lat / (long double)180 * (long double)PI64;
			raw_lon = raw_lon / (long double)180 * (long double)PI64;
			raw_head = raw_head / (long double)180 * PI64;

			rawgps.lat = raw_lat;
			rawgps.lon = raw_lon;
			rawgps.head = raw_head;
			gpsx.push_back(x);
			gpsy.push_back(y);
			gpsdata.push_back(rawgps);
		}
		else if (lineSplit[0] == "[GPS]")
		{
			GPS rawgps;
			float x, y;

			raw_lat = stringToNum<long double>(lineSplit[1]);
			raw_lon = stringToNum<long double>(lineSplit[2]);
			raw_head = stringToNum<long double>(lineSplit[3]);
			x = stringToNum<float>(lineSplit[4]);
			y = stringToNum<float>(lineSplit[5]);

			raw_lat = raw_lat / (long double)180 * (long double)PI64;
			raw_lon = raw_lon / (long double)180 * (long double)PI64;
			raw_head = raw_head / (long double)180 * PI64;

			rawgps.lat = raw_lat;
			rawgps.lon = raw_lon;
			rawgps.head = raw_head;
			gpsx.push_back(x);
			gpsy.push_back(y);
			gpsdata.push_back(rawgps);
		}
		else if (lineSplit[0] == "[VELD]")
		{
			for (int i = 0, j = 1; j < lineSplit.size(); i++, j++)
				raw_lane1x.push_back(stringToNum<float>(lineSplit[j]));
			std::getline(in2, line);
			split(line, lineSplit, "\t");
			for (int i = 0, j = 1; j < lineSplit.size(); i++, j++)
				raw_lane1y.push_back(stringToNum<float>(lineSplit[j]));

			std::getline(in2, line);
			split(line, lineSplit, "\t");
			for (int i = 0, j = 1; j < lineSplit.size(); i++, j++)
				raw_lane2x.push_back(stringToNum<float>(lineSplit[j]));
			std::getline(in2, line);
			split(line, lineSplit, "\t");
			for (int i = 0, j = 1; j < lineSplit.size(); i++, j++)
				raw_lane2y.push_back(stringToNum<float>(lineSplit[j]));

			std::getline(in2, line);
			split(line, lineSplit, "\t");
			for (int i = 0, j = 1; j < lineSplit.size(); i++, j++)
				raw_lane3x.push_back(stringToNum<float>(lineSplit[j]));
			std::getline(in2, line);
			split(line, lineSplit, "\t");
			for (int i = 0, j = 1; j < lineSplit.size(); i++, j++)
				raw_lane3y.push_back(stringToNum<float>(lineSplit[j]));

			std::getline(in2, line);
			split(line, lineSplit, "\t");
			for (int i = 0, j = 1; j < lineSplit.size(); i++, j++)
				raw_lane4x.push_back(stringToNum<float>(lineSplit[j]));
			std::getline(in2, line);
			split(line, lineSplit, "\t");
			for (int i = 0, j = 1; j < lineSplit.size(); i++, j++)
				raw_lane4y.push_back(stringToNum<float>(lineSplit[j]));

		}
		else if (lineSplit[0] == "[IBEO]")
		{
			for (int i = 0, j = 1; j < lineSplit.size(); i++, j++)
				ibeox.push_back(stringToNum<float>(lineSplit[j]));
			std::getline(in2, line);
			split(line, lineSplit, "\t");
			for (int i = 0, j = 1; j < lineSplit.size(); i++, j++)
				ibeoy.push_back(stringToNum<float>(lineSplit[j]));
		}
		else if (lineSplit[0] == "[STOP]")
		{
			for (int i = 0, j = 1; j < lineSplit.size(); i++, j++)
				stopx.push_back(stringToNum<float>(lineSplit[j]));
			std::getline(in2, line);
			split(line, lineSplit, "\t");
			for (int i = 0, j = 1; j < lineSplit.size(); i++, j++)
				stopy.push_back(stringToNum<float>(lineSplit[j]));
		}
		else if (lineSplit[0] == "[VELDINDEX]")
		{
			if (!Deallanesflag)
			{
				Deallanesflag = 1;
				raw_lane4x.clear();
				raw_lane4y.clear();
			}
			Lane.x.clear();
			Lane.y.clear();

			for (int i = 0, j = 1; j < lineSplit.size(); i++, j++)
				Lane.x.push_back(stringToNum<float>(lineSplit[j]));
			std::getline(in2, line);
			split(line, lineSplit, "\t");
			for (int i = 0, j = 1; j < lineSplit.size(); i++, j++)
				Lane.y.push_back(stringToNum<float>(lineSplit[j]));
			raw_lane4x.insert(raw_lane4x.end(), Lane.x.begin(), Lane.x.end());
			raw_lane4y.insert(raw_lane4y.end(), Lane.y.begin(), Lane.y.end());
			Lanes.push_back(Lane);
		}

	}
	Map.gps = gpsdata;
	Map.ibeox = ibeox;
	Map.ibeoy = ibeoy;
	Map.l1x = raw_lane1x;
	Map.l2x = raw_lane2x;
	Map.l3x = raw_lane3x;
	Map.l4x = raw_lane4x;
	Map.l1y = raw_lane1y;
	Map.l2y = raw_lane2y;
	Map.l3y = raw_lane3y;
	Map.l4y = raw_lane4y;
	Map.x = gpsx;
	Map.y = gpsy;
	Map.stopx = stopx;
	Map.stopy = stopy;
	Map.Lane = Lanes;
	Map.coordinatezero = 0;
}
//加载路网文件
void LoadRoadMap(odometer&RoadMap, mapdata&map)
{
	vector <float> mapx;
	vector <float> mapy;
	vector <GPS>   roadgps;
	string dir;
	cout << "Load Road NetWork Map" << endl;

	OpenFile(dir);
	std::string line;
	ifstream  in(dir.c_str());

	while (in.peek() != EOF)
	{
		std::vector<std::string> lineSplit;
		std::getline(in, line);
		split(line, lineSplit, "\t");
		if (lineSplit.empty()) break;

		GPS gps;
		gps.lat = stringToNum<float>(lineSplit[2]);
		gps.lon = stringToNum<float>(lineSplit[1]);
		gps.head = 0;
		gps.lat = gps.lat / (float)180 * PI;
		gps.lon = gps.lon / (float)180 * PI;
		roadgps.push_back(gps);
	}
	for (int i = 0; i < map.gps.size(); i++)
	{
		double x, y;
		long double lat, lon;
		lat = map.gps[i].lat;
		lon = map.gps[i].lon;
		Global2_Local(x, y, lat, lon, map.gps.back().lat, map.gps.back().lon, map.gps.back().head);
		mapx.push_back((float)x);
		mapy.push_back((float)y);
	}
	for (int i = 0; i < roadgps.size(); i++)
	{
		double x, y;
		long double lat, lon;
		lat = roadgps[i].lat;
		lon = roadgps[i].lon;
		Global2_Local(x, y, lat, lon, map.gps.back().lat, map.gps.back().lon, map.gps.back().head);
		RoadMap.x.push_back((float)x);
		RoadMap.y.push_back((float)y);
		RoadMap.gps.push_back(roadgps[i]);
	}
	map.x = mapx;
	map.y = mapy;
}
bool LoadRecordIBEO(RecordIBEO& RectordData)
{
	RecordIBEO NewRectordData;
	std::string line;
	std::string FileName;
	cout << "load RecordIBEO.txt..." << endl;
	OpenFile(FileName);
	ifstream in(FileName);

	if (!in)
	{
		int ret = MessageBox(nullptr, "Click OK to exit", "File Error", MB_OK | MB_ICONERROR);
		if (ret == MB_OK) exit(-1);
	}

	while (in.peek() != EOF)
	{
		std::vector<std::string> lineSplit;
		std::getline(in, line);
		split(line, lineSplit, "\t");
		if (lineSplit.empty()) break;

		if (lineSplit[1] == "[GPS]")
		{
			GPSIBEO newdata;
			PacketGPS gps;
			gps.lat = stringToNum<long double>(lineSplit[3]);
			gps.lon = stringToNum<long double>(lineSplit[4]);
			gps.heading = stringToNum<long double>(lineSplit[5]);
			gps.lat = gps.lat / (long double)180 * (long double)PI64;
			gps.lon = gps.lon / (long double)180 * (long double)PI64;
			gps.heading = gps.heading / (long double)180 * (long double)PI64;
			newdata.gps = gps;

			std::getline(in, line);
			split(line, lineSplit, "\t");
			if (lineSplit[1] == "[IBEO]")
			{
				for (int i = 0, j = 2; j < lineSplit.size(); i++, j++)
					newdata.x.push_back(stringToNum<float>(lineSplit[j]));

				std::getline(in, line);
				split(line, lineSplit, "\t");
				for (int i = 0, j = 2; j < lineSplit.size(); i++, j++)
					newdata.y.push_back(stringToNum<float>(lineSplit[j]));

				NewRectordData.data.push_back(newdata);
			}
			else
			{
				cout << "datas are wrong!!!" << endl;
				return false;
			}
		}

	}

	RectordData = NewRectordData;
	cout << "     RecordIBEO.txt is loaded..." << endl;
	return true;
}
//复制地图至可视化数据
void CloneMap(vector <float> x, vector <float> y, vector <float> &showx, vector <float>&showy, int offsetx, int offsety)
{
	    showx.clear();
	    showy.clear();
	
		OffSET(x, showx, offsetx);
		OffSET(y, showy, offsety);
}
void CloneMap(mapdata& rawmap, mapdata& showmap, int offsetx, int offsety)
{

	vector<float> ibeox, ibeoy, xdata, ydata, mousex, mousey, roadx, roady;
	vector<float> l1x, l2x, l3x, l4x, l1y, l2y, l3y, l4y;
	OffSET(rawmap.ibeox, ibeox, offsetx);
	OffSET(rawmap.ibeoy, ibeoy, offsety);
	OffSET(rawmap.x, xdata, offsetx);
	OffSET(rawmap.y, ydata, offsety);
	
	OffSET(rawmap.l1x, l1x, offsetx);
	OffSET(rawmap.l2x, l2x, offsetx);
	OffSET(rawmap.l3x, l3x, offsetx);
	OffSET(rawmap.l4x, l4x, offsetx);
	OffSET(rawmap.l1y, l1y, offsety);
	OffSET(rawmap.l2y, l2y, offsety);
	OffSET(rawmap.l3y, l3y, offsety);
	OffSET(rawmap.l4y, l4y, offsety);


	showmap.ibeox = ibeox;
	showmap.ibeoy = ibeoy;
	showmap.l1x = l1x; showmap.l1y = l1y;
	showmap.l2x = l2x; showmap.l2y = l2y;
	showmap.l3x = l3x; showmap.l3y = l3y;
	showmap.l4x = l4x; showmap.l4y = l4y;
	showmap.x = xdata; showmap.y = ydata;

}
void CloneMap(mapdata& rawmap, mapdata& showmap, int offsetx, int offsety, odometer RoadMap, odometer& showRoadMap)
{

	vector<float> ibeox, ibeoy, xdata, ydata, mousex, mousey;
	vector<float>  roadx, roady;
	vector<float> l1x, l2x, l3x, l4x, l1y, l2y, l3y, l4y;
	OffSET(rawmap.ibeox, ibeox, offsetx);
	OffSET(rawmap.ibeoy, ibeoy, offsety);
	OffSET(rawmap.x, xdata, offsetx);
	OffSET(rawmap.y, ydata, offsety);
	OffSET(ReserveMousex, mousex, offsetx);
	OffSET(ReserveMousey, mousey, offsety);
	OffSET(RoadMap.x, roadx, offsetx);
	OffSET(RoadMap.y, roady, offsety);

	OffSET(rawmap.l1x, l1x, offsetx);
	OffSET(rawmap.l2x, l2x, offsetx);
	OffSET(rawmap.l3x, l3x, offsetx);
	OffSET(rawmap.l4x, l4x, offsetx);
	OffSET(rawmap.l1y, l1y, offsety);
	OffSET(rawmap.l2y, l2y, offsety);
	OffSET(rawmap.l3y, l3y, offsety);
	OffSET(rawmap.l4y, l4y, offsety);


	showmap.ibeox = ibeox;
	showmap.ibeoy = ibeoy;
	showmap.l1x = l1x; showmap.l1y = l1y;
	showmap.l2x = l2x; showmap.l2y = l2y;
	showmap.l3x = l3x; showmap.l3y = l3y;
	showmap.l4x = l4x; showmap.l4y = l4y;
	showmap.x = xdata; showmap.y = ydata;
	showRoadMap.x = roadx;
	showRoadMap.y = roady;
}
//插值函数
Point2D operator-(const Point2D&L, const Point2D&R)
{
	return Point2D(L.x - R.x, L.y - R.y);
}
Point2D operator-(const Point2D&R)
{
	return Point2D(-R.x, -R.y);
}
Point2D operator+(const Point2D&L, const Point2D&R)
{
	return Point2D(L.x + R.x, L.y + R.y);
}
void   operator+=(Point2D&L, const Point2D&R)
{
	L.x += R.x;
	L.y += R.y;
}
double operator*(const Point2D&L, const Point2D&R)
{
	return (L.x * R.x + L.y * R.y);
}
Point2D operator*(double k, const Point2D&R)
{
	return Point2D(k*R.x, k*R.y);
}
Point2D operator*(const Point2D&L, double k)
{
	return  Point2D(k*L.x, k*L.y);
}
Point2D operator/(const Point2D& src, double den)
{
	if (den)
	{
		return Point2D(src.x / den, src.y / den);
	}
	else
	{
		return Point2D(0, 0);
	}
}
Point2D catmullRom(Point2D P0, Point2D  P1, Point2D P2, Point2D P3, float t)
{
	float factor = 0.5;
	Point2D c0 = P1;
	Point2D a = (P2 - P0);
	Point2D b = (P2 - P1);
	Point2D c = (P3 - P1);

	Point2D c1 = a*factor;
	Point2D c2 = b*3.0 - c*factor - a*(2.0*factor);
	Point2D c3 = b*-2. + c*factor + a*factor;
	Point2D curvePoint = c3*t*t*t + c2*t*t + c1*t + c0;
	return curvePoint;
}
bool CatmullRomLine(const vector<float >&inputx, const vector<float >&inputy, int TotalNum, vector<float>&outx, vector<float>&outy)
{
	outx.clear();
	outy.clear();
	int inPutNum = inputx.size();
	if (inPutNum != inputy.size()){
		return false;
	}
	if (inPutNum == 1){
		return false;
	}
	if (TotalNum < inPutNum){
		return false;
	}
	int NumBetweenPoint = round((float)(TotalNum - inPutNum) / (float)(inPutNum - 1));
	int LastNumBwtweenPoint = NumBetweenPoint + ((TotalNum - inPutNum) - (NumBetweenPoint)*(inPutNum - 1));
	for (int i = 0; i < inPutNum - 1; ++i){
		outx.push_back(inputx[i]);
		outy.push_back(inputy[i]);
		Point2D P0;
		Point2D P1;
		Point2D P2;
		Point2D P3;
		if (i == 0){
			P1.x = inputx[i]; P1.y = inputy[i];
			P2.x = inputx[i + 1]; P2.y = inputy[i + 1];
			P3.x = inputx[i + 2]; P3.y = inputy[i + 2];
			/******************/
			P0 = 2.0 * P1 - P2;
		}
		else if (i < inPutNum - 2){
			P0.x = inputx[i - 1]; P0.y = inputy[i - 1];
			P1.x = inputx[i]; P1.y = inputy[i];
			P2.x = inputx[i + 1]; P2.y = inputy[i + 1];
			P3.x = inputx[i + 2]; P3.y = inputy[i + 2];
		}
		else {
			P0.x = inputx[i - 1]; P0.y = inputy[i - 1];
			P1.x = inputx[i]; P1.y = inputy[i];
			P2.x = inputx[i + 1]; P2.y = inputy[i + 1];
			/*********************/
			P3 = 2.0 * P2 - P1;
		}
		/**********************************/
		if (i < inPutNum - 2){
			int end = NumBetweenPoint + 1;
			for (int j = 1; j <end; ++j){
				float t = (float)j / (float)(end);
				Point2D P = catmullRom(P0, P1, P2, P3, t);
				outx.push_back(P.x);
				outy.push_back(P.y);
			}
		}
		else{
			int end = LastNumBwtweenPoint + 1;
			for (int j = 1; j < end; ++j){
				float t = (float)j / (float)(end);
				Point2D P = catmullRom(P0, P1, P2, P3, t);
				outx.push_back(P.x);
				outy.push_back(P.y);
			}
		}
	}
	outx.push_back(inputx[inPutNum - 1]);
	outy.push_back(inputy[inPutNum - 1]);
	return true;
}
