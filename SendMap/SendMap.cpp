#include "SendMap.h"
#include <iomanip>
#include "map.h"
#include <algorithm>
#include <sstream>

#define LENGTHOFLANE 40
#define LENGTHOFSLAMCONTOUR  3000
#define LENGTHOFGPSPATH 100
using namespace std;
extern MapZero MapZeroGps;
extern map_t *GoalGridMap;
extern int mapLoadFlag;
extern mapdata GPSMap;
vector<float> GPSPathx;
vector<float> GPSPathy;
float maxX, maxY, minX, minY;
vector<float> gpsx;//
vector<float> gpsy;//
vector<float> curve;//
vector<int  > MapFlag;//


void FitGoalXY(vector<float>&X, vector<float>&Y, float& fitx, float& fity);
void calcMapWidthandHeight(vector<float>& vecx, vector<float>& vecy, float& maxX, float& maxY, float& minX, float& minY);
void ClusteringLanes(vector<float>& Lanex, vector<float>&Laney, vector<LaneCell>& LaneMap, vector<float>& x, vector<float>&y);
float ComputeDistanceLane(float Lanex, float Laney, vector<float>& x, vector<float>&y);

size_t ComputeMinIndex(float GPSX, float GPSY);

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
float ComputeGPSDistance(long double lat1, long double lon1, long double lat2, long double lon2)
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
void  Global2_Local(double &goalPx, double &goalPy, /*Ŀ���GPS*/long double latGoal, long double lonGoal, /*��ǰGPS*/long double lat, long double lon, long double head)
{
	const double EPSINON = 0.000000000000000001;
	double angle;
	float dis, disla, dislo;
	dis = ComputeGPSDistance(latGoal, lonGoal, lat, lon);
	disla = ComputeGPSDistance(latGoal, lonGoal, lat, lonGoal);
	dislo = ComputeGPSDistance(lat, lonGoal, lat, lon);

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

void CalculateVehicleState(long double lat, long double l_lat, long double lon, long double l_lon, long double head, long double l_head, float& x0, float& y0, float& dhead)
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
void CalculateSensorData(float& vecx, float& vecy, float x0, float y0, float dhead)
{
	float _vecx = vecx;
	float _vecy = vecy;
	vecx = (_vecx - x0) * cos(dhead) - (_vecy - y0) * sin(dhead);
	vecy = (_vecx - x0) * sin(dhead) + (_vecy - y0) * cos(dhead);
}
void CalculateSensorData(vector<float>& vecx, vector<float>& vecy, float x0, float y0, float dhead)
{
	for (int i = 0; i < (int)vecx.size(); i++)
	{
		float _vecx = vecx[i];
		float _vecy = vecy[i];
		vecx[i] = (_vecx - x0) * cos(dhead) - (_vecy - y0) * sin(dhead);
		vecy[i] = (_vecx - x0) * sin(dhead) + (_vecy - y0) * cos(dhead);
	}
}
void CalculateSensorData2(vector<float>& vecx, vector<float>& vecy, float x0, float y0, float dhead)
{
	for (int i = 0; i < (int)vecx.size(); i++)
	{
		float _vecx = vecx[i];
		float _vecy = vecy[i];
		vecx[i] = (_vecx - x0) * cos(dhead) - (_vecy - y0) * sin(dhead);
		vecy[i] = (_vecx - x0) * sin(dhead) + (_vecy - y0) * cos(dhead);	
	}
}
template <class Type> inline Type stringToNum(const std::string& str)
{
	std::istringstream iss(str);
	Type num;
	iss >> num;
	return num;
}
int split(const std::string& str, std::vector<std::string>& ret_, std::string sep = ",")
{
	if (str.empty())
		return 0;

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

float ComputeDistanceLane(vector<float> Lanex, vector<float> Laney, vector<float>& x, vector<float>&y)
{
	if (x.size() != y.size())
	{
		return -100;
	}

	size_t GPSIndex = 0;
	size_t LanexIndex = 0;
	float distance;
	float MinDistance = 10000.f;
	for (size_t i = 0; i < Lanex.size(); i++)
	{
		for (size_t j = 0; j < x.size(); j++)
		{
			distance = Computedistance(x[j], y[j], Lanex[i], Laney[i]);
			if (distance < MinDistance)
			{
				MinDistance = distance;
				GPSIndex = j;
				LanexIndex = i;
			}
		}
	}


	if (MinDistance < 30.f)
	{
		if (Lanex[LanexIndex] < x[GPSIndex])
		{
			MinDistance = -MinDistance;
		}
		return MinDistance;
	}
	else
	{
		return -100;
	}
}

/*************�����ߴ�����********************/
int  LoadGPSMap(mapdata& gGPSMap,std::string dir)
{
	std::string fileName;
    fileName="./data/"+dir;
	cout << "load "<<fileName << endl;
	
	ifstream in(fileName);
	if (!in)
	{
        cout<<fileName<<" does not exist!!!"<<endl;
		return -1;
	}
	std::string line;
	while (in.peek() != EOF)
	{
		std::vector<std::string> lineSplit;
		std::getline(in, line);
		split(line, lineSplit, "\t");
		if (lineSplit[1] == "[GPS]")
		{
			GPS gps;
			gps.flag = stringToNum<int>(lineSplit[0]);
			gps.lat = stringToNum<long double>(lineSplit[3]);
			gps.lon = stringToNum<long double>(lineSplit[4]);
			gps.head = stringToNum<long double>(lineSplit[5]);
			gps.lat = gps.lat / (long double)180 * (long double)PI64;
			gps.lon = gps.lon / (long double)180 * (long double)PI64;
			gps.head = gps.head / (long double)180 * (long double)PI64;
            
			double x, y;
			Global2_Local(x, y, gps.lat, gps.lon, MapZeroGps.lat, MapZeroGps.lon, MapZeroGps.head);
			GPSPathx.push_back((float)x);
			GPSPathy.push_back((float)y);
			gGPSMap.gps.push_back(gps);
		}
	}
    return 1;
}
int  LoadSlamPOSMap(mapdata& gGPSMap,std::string dir)
{
    std::string fileName;
    fileName="./data/"+dir;
    cout << "load "<<fileName << endl;
    ifstream in(fileName);
    if (!in)
    {
        cout<<fileName<<" does not exist!!!"<<endl;
        return -1;
    }
    std::string line;
    while (in.peek() != EOF)
    {
        std::vector<std::string> lineSplit;
        std::getline(in, line);
        split(line, lineSplit, "\t");
        if (lineSplit[1] == "[SLAMPOS]")
        {
            GPS gps;
            double x, y;
            gps.flag = stringToNum<int>(lineSplit[0]);
            gps.lat =0;
            gps.lon =0;
            x= stringToNum<double>(lineSplit[3]);
            y= stringToNum<double>(lineSplit[4]);
            gps.head = stringToNum<double>(lineSplit[5]);
            gps.head = gps.head / (long double)180 * (long double)PI64;

            GPSPathx.push_back((float)x);
            GPSPathy.push_back((float)y);
            gGPSMap.gps.push_back(gps);
        }
    }
    return 1;
}
size_t ComputeMinIndex(float GPSX, float GPSY, float Curheading)
{
	static int FirstFlag = 0;
	static int LastIndex = 0;
	float distance, MinDistance = 0;
	static int MINIndex = 0;
    int StartIndex = 0;
    int EndIndex = 0;
	int  SuccessfulFlag = 0;
	float minangle = 0,dhead=0;
	if (!FirstFlag)
	{
		MinDistance = 1000;
		for (size_t i = 1; i < GPSPathx.size(); i++)
		{
			distance = Computedistance(GPSX, GPSY, GPSPathx[i], GPSPathy[i]);
            dhead=Curheading - GPSMap.gps[i].head;
            if(dhead>PI) dhead-=2*PI;
            if(dhead<-PI) dhead+=2*PI;
            if (distance < MinDistance && (fabs(Curheading - GPSMap.gps[i].head) < PI / 5 || fabs(Curheading - GPSMap.gps[i].head + 2 * PI)< PI / 5 || fabs(Curheading - GPSMap.gps[i].head - 2 * PI)< PI / 5))
               // if (distance < MinDistance)// && fabs(dhead) < PI / 5)
                {
				MinDistance = distance;
				minangle = (Curheading - GPSMap.gps[i].head) * 180 / PI;
				MINIndex = i;
			   }
		}
		if (MinDistance < 5)
		{
			std::cout << "Fist location success" << endl;
			FirstFlag = 1;
			SuccessfulFlag = 1;

		}
		else
		{
			cout << "Fist location fail, min distance= " << MinDistance << "  Angular deviation=" << minangle << endl;
		}
	}
	else
	{
		StartIndex = LastIndex - 20;

		if (StartIndex < 1)
		{
			StartIndex = 1;
			EndIndex = GPSPathx.size();
		}

        EndIndex = GPSPathx.size();

		MinDistance = 1000;
		for (size_t i = StartIndex; i < EndIndex; i++)
		{
			distance = Computedistance(GPSX, GPSY, GPSPathx[i], GPSPathy[i]);
            dhead=Curheading - GPSMap.gps[i].head;
            if(dhead>PI) dhead-=2*PI;
            if(dhead<-PI) dhead+=2*PI;

            if (distance < MinDistance && fabs(dhead) < PI / 5)
            {
				MinDistance = distance;
				MINIndex = i;
			}
            if(distance>20) break;
		}

		if (MinDistance < 2)
		{
			SuccessfulFlag = 1;
		}
		else
		{
			SuccessfulFlag = 0;
			cout << "Local location fail,min distance=" << MinDistance << endl;
			FirstFlag = 0;//重新定位初始化
		}
	}

	if (SuccessfulFlag)
	{
		LastIndex = MINIndex;
		cout << "Last Index=" << LastIndex <<" MinDistance="<<MinDistance<<" dheading="<<dhead<< endl;
		return MINIndex;
	}
	else
	{
		return 0;
	}
}

void Addlines(vector<LaneCell>& LaneMap )
{
	float mindis=3;
    float dis = 0;
    LaneCell LaneData;

    if (LaneMap.size()<1)
    {
        return;
    }

    for (size_t i_ = 0; i_ < LaneMap.size(); i_++)
    {
        LaneData = LaneMap[i_];
        for (size_t i = 0; i < LaneMap.size(); i++)
        {
            if (i_ == i) continue;
            dis = Computedistance(LaneData.LaneData.x[0], LaneData.LaneData.y[0], LaneMap[i].LaneData.x.back(), LaneMap[i].LaneData.y.back());
            if (dis < mindis)
            {
                LaneMap[i].LaneData.x.insert(LaneMap[i].LaneData.x.end(), LaneData.LaneData.x.begin(), LaneData.LaneData.x.end());
                LaneMap[i].LaneData.y.insert(LaneMap[i].LaneData.y.end(), LaneData.LaneData.y.begin(), LaneData.LaneData.y.end());
                LaneMap.erase(LaneMap.begin()+i_);
                i_--;
                break;
            }

            dis = Computedistance(LaneData.LaneData.x.back(), LaneData.LaneData.y.back(), LaneMap[i].LaneData.x[0], LaneMap[i].LaneData.y[0]);
            if (dis < mindis)
            {
                LaneMap[i].LaneData.x.insert(LaneMap[i].LaneData.x.begin(), LaneData.LaneData.x.begin(), LaneData.LaneData.x.end());
                LaneMap[i].LaneData.y.insert(LaneMap[i].LaneData.y.begin(), LaneData.LaneData.y.begin(), LaneData.LaneData.y.end());
                LaneMap.erase(LaneMap.begin() + i_);
                i_--;
                break;
            }
        }
    }
}
int ClusteringLanes(vector<float>& Lanex, vector<float>&Laney, vector<float>&Lanei, vector<LaneCell>& LaneMap, vector<float>& x, vector<float>&y)
{
    if (Lanex.size() < 10 || Laney.size() != Lanex.size()) return 0;
    LaneCell LaneData;
    size_t  LaneNumber = 0;
    float distance = 0;
    vector<float> LeftIndex, RightIndex;
    vector<float> Leftx, Rightx;
    float Lanedistance=0;
    int laneindex =0;
    LaneNumber = Lanex.size();
   int laneflag=0;
    while (LaneNumber > 1)
    {
        laneindex = Lanei[0];
        LaneData.LaneData.x.clear();
        LaneData.LaneData.y.clear();
        for (size_t i = 0; i < Lanex.size(); i++)
        {
            if (Lanei[i] == laneindex)
            {
                LaneData.LaneData.x.push_back(Lanex[i]);
                LaneData.LaneData.y.push_back(Laney[i]);
                Lanex.erase(Lanex.begin() + i);
                Laney.erase(Laney.begin() + i);
                Lanei.erase(Lanei.begin() + i);
                i--;
                LaneNumber--;
            }
        }
        pairsort(LaneData.LaneData.x, LaneData.LaneData.y);
        double angle = atan2(LaneData.LaneData.y.back() - LaneData.LaneData.y[0], LaneData.LaneData.x.back() - LaneData.LaneData.x[0]);
        //if (fabs(angle) < 10./180*PI||fabs(angle+PI) < 10./180*PI||fabs(angle-PI) < 10./180*PI) continue;//ÆœÐÐ³µµÀÏß

        if (angle >145./180*PI||angle < 45./180*PI) continue;//ÆœÐÐ³µµÀÏß
       // cout<<angle*180/PI<<endl;
        LaneMap.push_back(LaneData);

        if (LaneNumber == 0) break;
    }
    Addlines(LaneMap);

    //std::cout << "lane number=" << LaneMap.size() << endl;

    for (int i = 0; i < LaneMap.size(); i++)
    {
        Lanedistance = ComputeDistanceLane(LaneMap[i].LaneData.x, LaneMap[i].LaneData.y, x, y);
        if (fabs(Lanedistance) > 7.5f)
        {
            LaneMap.erase(LaneMap.begin()+i);
            i--;
            continue;
        }

        if (Lanedistance < 0 && Lanedistance>-99)
        {
            Leftx.push_back(-Lanedistance);
            LeftIndex.push_back(i);
        }
        else if (Lanedistance >= 0)
        {
            Rightx.push_back(Lanedistance);
            RightIndex.push_back(i);
        }

    }
    vector <float> LeftIndex_x, RightIndex_x;
    vector<LaneCell>LaneMap2(4);

    LeftIndex_x = LeftIndex;
    RightIndex_x = RightIndex;
    pairsort(LeftIndex_x, Leftx);
    pairsort(RightIndex_x, Rightx);

    for (size_t i = 0; i < LeftIndex_x.size(); i++)
    {
        laneflag++;
        if (LeftIndex_x.size() == 1)
        {
            std::cout << "lane 2  ";
            LaneMap2[1] = LaneMap[LeftIndex_x[0]];
        }
        else
        {
            if (i == 1)
            {
                std::cout << "lane 2  ";
                LaneMap2[1] = LaneMap[LeftIndex_x[0]];
            }
            if (i == 0)
            {
                std::cout << "lane 1  ";
                LaneMap2[0] = LaneMap[LeftIndex_x[1]];
            }
        }
    }
    for (size_t i = 0; i < RightIndex_x.size(); i++)
    {
        laneflag++;
        if (i == 0)
        {
            std::cout << "lane 3  ";
            LaneMap2[2] = LaneMap[RightIndex_x[i]];
        }
        else if (i == 1)
        {
            std::cout << "lane 4  ";
            LaneMap2[3] = LaneMap[RightIndex_x[i]];
        }

    }
   if(!laneflag) std::cout<<endl;

    if(LaneMap.size()>1) std::cout << "    Lanes number " << LaneMap.size() << endl;
    for (size_t i = 0; i < LaneMap2.size(); i++)
    {
        if (LaneMap2[i].LaneData.x.size())
        {
            vector<float> fitInx, fitIny;
          //  fitPoints(LaneMap2[i].LaneData.x, LaneMap2[i].LaneData.y, fitInx, fitIny, 10, 3);
            pairsort(LaneMap2[i].LaneData.x, LaneMap2[i].LaneData.y);
           // LaneMap2[i].LaneData.x = fitInx;
           // LaneMap2[i].LaneData.y = fitIny;
        }
    }

   if(LaneMap2.size()<5&&LaneMap2.size()>1) LaneMap = LaneMap2;

}
void ComputeStopLineDistance(vector<float>&x, vector<float>&y, int16_t &stopLineDis)
{
    if(x.size()<1)
    {
        stopLineDis=-1;
        return;
    }
    float distance=0;
    vector<float> disVector;
    vector<float> disVectori;
    for(int i=0;i<x.size();++i)
    {
        distance=Computedistance(0,0,x[i],y[i]);
        disVector.push_back(distance);
        disVectori.push_back((float)i);
    }

    pairsort(disVectori,disVector);
    float  disp12=0,disp1=0,disp2=0,disp=0,s=0;
    int indexp1=disVectori[0];
    int indexp2=disVectori[1];
    disp1=disVector[0];
    disp2=disVector[1];
    disp12=Computedistance(x[indexp1],y[indexp1],x[indexp2],y[indexp2]);
    disp=(disp1+disp2+disp12)/2;
    s=sqrt(disp*(disp-disp1)*(disp-disp2)*(disp-disp12));
    stopLineDis=100*2*s/disp12;//单位厘米

    if(y[indexp1]<0&&y[indexp2]<0)
    {
        stopLineDis=-stopLineDis;
    }
    return;

}
int GetMapdatasubmap(PacketGPS CurGPS, PacketHighResolutionMap& sendmap, int16_t &stopLineDis)
{
    static int loadsamplepointflg;
    vector<float> samplex, sampley;
    static vector<float> cursamplex, cursampley;
    //int sendgpsflag = 1;
    float Wide = 10.f;
    float Length = 60.f;
    float GPSMapLength = 80.f;
    static vector<float> Goalx;
    static vector<float> Goaly;
    CurGPS.lat = CurGPS.lat / (long double)180 * (long double)PI64;
    CurGPS.lon = CurGPS.lon / (long double)180 * (long double)PI64;
    CurGPS.heading = CurGPS.heading / (long double)180 * (long double)PI64;
    vector<float> Lanex, Laney;
    vector<float> IBEOx, IBEOy;
    vector<float> mappointx, mappointy;
    vector<int> mappointstyle;
    vector<float> SendLanex, SendLaney,SendLanei;
    float Distance=0;
    float x0 = 0, y0 = 0, dhead = 0;

    vector<LaneCell> LaneMap;
    double GPSX, GPSY;
    Global2_Local(GPSX, GPSY, CurGPS.lat, CurGPS.lon, MapZeroGps.lat, MapZeroGps.lon, MapZeroGps.head);
    int CoordsX, CoordsY;
    CoordsX = MAP_GXWX(GoalGridMap, GPSX);
    CoordsY = MAP_GYWY(GoalGridMap, GPSY);

    memset(sendmap.ibcontourx, 0, sizeof(sendmap.ibcontourx));
    memset(sendmap.ibcontoury, 0, sizeof(sendmap.ibcontoury));
    memset(sendmap.lane1x, 999, sizeof(sendmap.lane1x));
    memset(sendmap.lane2x, 999, sizeof(sendmap.lane1x));
    memset(sendmap.lane3x, 999, sizeof(sendmap.lane1x));
    memset(sendmap.lane4x, 999, sizeof(sendmap.lane1x));
    memset(sendmap.lane1y, 999, sizeof(sendmap.lane1x));
    memset(sendmap.lane2y, 999, sizeof(sendmap.lane1x));
    memset(sendmap.lane3y, 999, sizeof(sendmap.lane1x));
    memset(sendmap.lane4y, 999, sizeof(sendmap.lane1x));


	memset(sendmap.gpsx, 0, sizeof(sendmap.gpsx));
	memset(sendmap.gpsy, 0, sizeof(sendmap.gpsy));
	memset(sendmap.MapFlag, 0, sizeof(sendmap.MapFlag));
	memset(sendmap.curve, 0, sizeof(sendmap.curve));

    memset(sendmap.pointx, 0, sizeof(sendmap.pointx));
    memset(sendmap.pointy, 0, sizeof(sendmap.pointy));
    memset(sendmap.pointxstyle, 0, sizeof(sendmap.pointxstyle));

        size_t Index = 0;
        double  GPSX2, GPSY2;
        Global2_Local(GPSX2, GPSY2, CurGPS.lat, CurGPS.lon, MapZeroGps.lat, MapZeroGps.lon, MapZeroGps.head);
        Index = ComputeMinIndex(GPSX2, GPSY2,CurGPS.heading);
        if (Index)
        {
            gpsx.clear();
            gpsy.clear();
            MapFlag.clear();
            Distance=0;
            for (size_t i = Index; i < GPSPathx.size(); i++)
            {
                float distance;

                if(gpsx.size()>2)
                {
                    distance = Computedistance(gpsx.back(), gpsy.back(), GPSPathx[i], GPSPathy[i]);
                    Distance+=distance;
                    if (Distance>GPSMapLength)
                    {
                        break;
                    }

                }
                gpsx.push_back(GPSPathx[i]);
                gpsy.push_back(GPSPathy[i]);
                MapFlag.push_back(GPSMap.gps[i].flag);
            }
			cout << "flag  " << GPSMap.gps[Index].flag << "  " << endl;



            x0 = GPSX2;
            y0 = GPSY2;
            dhead = CurGPS.heading - MapZeroGps.head;
            CalculateSensorData(gpsx, gpsy, x0, y0, dhead);

			if (gpsx.size() < LENGTHOFGPSPATH)
			{
				for (size_t i = 0; i < LENGTHOFGPSPATH; i++)
				{
					if (i < gpsx.size())
					{
						sendmap.gpsx[i] = gpsx[i];
						sendmap.gpsy[i] = gpsy[i];
                        sendmap.MapFlag[i] = MapFlag[i];
						sendmap.curve[i] = 0;
					}
					else
					{
						sendmap.gpsx[i] = gpsx.back();
						sendmap.gpsy[i] = gpsy.back();
                        sendmap.MapFlag[i] = MapFlag.back();
						sendmap.curve[i] = 0;
					}
				}
			}
			else
			{
				float step = gpsx.size() / ((float)LENGTHOFGPSPATH);
				for (size_t i = 0; i < LENGTHOFGPSPATH; i++)
				{
					size_t j = (size_t)(i*step);
					sendmap.gpsx[i] = gpsx[j];
					sendmap.gpsy[i] = gpsy[j];
                    sendmap.MapFlag[i] = MapFlag[j];
		 			sendmap.curve[i] = 0;
				}

			}
        } else
        {
            cout << "no gps path" << endl;

            return 1;
        }

    if(mapLoadFlag<0)
    {
        cout<<"no map data"<<endl;
    }

    if (!MAP_VALID(GoalGridMap, CoordsX, CoordsY))
    {
        cout << "no in the map!!!" << endl;

        return 1;
    }

    //¹Ì¶šÇøÓò²ÉÑù
    //²ÉÑù²ßÂÔ£¬œ«¹Ì¶š²ÉÑùµã£¬Ðý×ªµœµØÍŒ¶ÔÓŠÇøÓò£¬Ö±œÓŒÆËã²ÉÑùµãindex²ÉÑù
    if (!loadsamplepointflg)
    {
        loadsamplepointflg++;
        for (int i = -Wide / MapZeroGps.scale; i < Wide / MapZeroGps.scale; i++)
            for (int j = -5 / MapZeroGps.scale; j < Length / MapZeroGps.scale; j++)
            {
                cursamplex.push_back(i*MapZeroGps.scale);
                cursampley.push_back(j*MapZeroGps.scale);
            }
    }
    /*****************µãÔÆ¡¢³µµÀÏßÌáÈ¡********************/
    x0 = 0, y0 = 0, dhead = 0;
    samplex = cursamplex;
    sampley = cursampley;
    dhead = (float)(MapZeroGps.head - CurGPS.heading);
    CalculateSensorData(samplex, sampley, x0, y0, dhead);
    x0 = (float)-GPSX;
    y0 = (float)-GPSY;
    dhead = 0;
    CalculateSensorData(samplex, sampley, x0, y0, dhead);

    int i_cloud = 0;
    for (size_t i = 0; i < samplex.size(); i++)
    {
        map_cell_t *cell;
        int cx, cy;
        int submapindex_ = 0;
        cx = MAP_GXWX(GoalGridMap, samplex[i]);
        cy = MAP_GYWY(GoalGridMap, sampley[i]);
        if (!MAP_VALID(GoalGridMap, cx, cy))
        {
            continue;
        }
        else
        {
            int subi_, subj_;
            int index_i, index_j;
            int *saveindex;
            map_cell_t *cell;
            index_i = int(cx / GoalGridMap->submapscale);
            index_j = int(cy / GoalGridMap->submapscale);

            subi_ = cx % GoalGridMap->submapscale;
            subj_ = cy % GoalGridMap->submapscale;
            if (!SUBMAPINDEX_VALID(GoalGridMap, index_i, index_j))
                continue;

            saveindex = GoalGridMap->submapindex + SUBMAPINDEX_INDEX(GoalGridMap, index_i, index_j);

            if (!SUBMAP_VALID(GoalGridMap, subi_, subj_))
                continue;

            cell = GoalGridMap->cells + *saveindex*GoalGridMap->submapsize + SUBMAP_INDEX(GoalGridMap, subi_, subj_);

            if (cell->occ_state == 1)//µãÔÆ±êÖŸÎ»
            {
                    IBEOx.push_back(cursamplex[i]);
                    IBEOy.push_back(cursampley[i]);
            }
            else if (cell->occ_state >1&&cell->occ_state<99) // µãÔÆ±êÖŸÎ»
            {
                mappointx.push_back(cursamplex[i]);
                mappointy.push_back(cursampley[i]);
                mappointstyle.push_back((int16_t)cell->occ_state);
               // cout<<"stop line"<<endl;
            }
            else if (cell->occ_state > 99) // µãÔÆ±êÖŸÎ»
            {
                SendLanex.push_back(cursamplex[i]);
                SendLaney.push_back(cursampley[i]);
                SendLanei.push_back((int)cell->occ_state);
            }
        }
    }
    /*****************µãÔÆ¡¢³µµÀÏßÌáÈ¡********************/
  //  cout << "lane number=" << SendLaney.size() << endl;

    for(int i=0;i<mappointx.size()&&i<50;i++)
    {

        sendmap.pointx[i]=mappointx[i];
        sendmap.pointy[i]=mappointy[i];
        sendmap.pointxstyle[i]=mappointstyle[i];
		sendmap.ibcontourx[i] = mappointx[i];
		sendmap.ibcontoury[i] = mappointy[i];
    }

    ComputeStopLineDistance(mappointx,mappointy,stopLineDis);

    // cout<<"mappoint number="<<mappointx.size()<<endl;
	int stoplinenum = mappointx.size();
    if(IBEOy.size()<LENGTHOFSLAMCONTOUR)
    {
		for (int i = stoplinenum; i<IBEOx.size(); i++)
        {
            sendmap.ibcontourx[i] = IBEOx[i];
            sendmap.ibcontoury[i] = IBEOy[i];
        }
    }
    else
    {
        float step=IBEOx.size()/((float)LENGTHOFSLAMCONTOUR);
		for (int i = stoplinenum; i<LENGTHOFSLAMCONTOUR; i++)
        {
            int index_cloud=int(i*step);
            if(index_cloud<IBEOx.size())
            {
                sendmap.ibcontourx[i] = IBEOx[index_cloud];
                sendmap.ibcontoury[i] = IBEOy[index_cloud];
            }
        }
    }

    ClusteringLanes(SendLanex, SendLaney, SendLanei, LaneMap, gpsx, gpsy);
    size_t SaveIndex = 0;
    for (size_t i = 0; i < LaneMap.size(); i++)
    {
        if (LaneMap[i].LaneData.x.size() < LENGTHOFLANE)
        {
            for (size_t j = 0; j < LaneMap[i].LaneData.x.size(); j++)
            {
                if (i == 0)
                {
                    sendmap.lane1x[j] = -LaneMap[i].LaneData.x[j];
                    sendmap.lane1y[j] = LaneMap[i].LaneData.y[j];
                }
                else if (i == 1)
                {
                    sendmap.lane2x[j] = -LaneMap[i].LaneData.x[j];
                    sendmap.lane2y[j] = LaneMap[i].LaneData.y[j];
                }
                else if (i == 2)
                {
                    sendmap.lane3x[j] = -LaneMap[i].LaneData.x[j];
                    sendmap.lane3y[j] = LaneMap[i].LaneData.y[j];
                }
                else if (i == 3)
                {
                    sendmap.lane4x[j] = -LaneMap[i].LaneData.x[j];
                    sendmap.lane4y[j] = LaneMap[i].LaneData.y[j];
                }
            }

            size_t j = LaneMap[i].LaneData.x.size() - 1;
            while (j<LENGTHOFLANE)
            {
                if (i == 0)
                {
                    sendmap.lane1x[j] = -LaneMap[i].LaneData.x.back();
                    sendmap.lane1y[j] = LaneMap[i].LaneData.y.back();
                }
                else if (i == 1)
                {
                    sendmap.lane2x[j] = -LaneMap[i].LaneData.x.back();
                    sendmap.lane2y[j] = LaneMap[i].LaneData.y.back();
                }
                else if (i == 2)
                {
                    sendmap.lane3x[j] = -LaneMap[i].LaneData.x.back();
                    sendmap.lane3y[j] = LaneMap[i].LaneData.y.back();
                }
                else if (i == 3)
                {
                    sendmap.lane4x[j] = -LaneMap[i].LaneData.x.back();
                    sendmap.lane4y[j] = LaneMap[i].LaneData.y.back();
                }
                j++;
            }
        }
        else
        {
            float step = (LaneMap[i].LaneData.x.size() / ((float)LENGTHOFLANE));
            SaveIndex = 0;
            for (size_t j = 0; j < LaneMap[i].LaneData.x.size(); j++)
            {
                if (i == 0)
                {
                    if (SaveIndex < LENGTHOFLANE)
                    {
                        sendmap.lane1x[SaveIndex] = -LaneMap[i].LaneData.x[j];
                        sendmap.lane1y[SaveIndex] = LaneMap[i].LaneData.y[j];
                    }
                }
                else if (i == 1)
                {
                    if (SaveIndex < LENGTHOFLANE)
                    {
                        sendmap.lane2x[SaveIndex] = -LaneMap[i].LaneData.x[j];
                        sendmap.lane2y[SaveIndex] = LaneMap[i].LaneData.y[j];
                    }
                }
                else if (i == 2)
                {
                    if (SaveIndex < LENGTHOFLANE)
                    {
                        sendmap.lane3x[SaveIndex] = -LaneMap[i].LaneData.x[j];
                        sendmap.lane3y[SaveIndex] = LaneMap[i].LaneData.y[j];
                    }
                }
                else if (i == 3)
                {
                    if (SaveIndex < LENGTHOFLANE)
                    {
                        sendmap.lane4x[SaveIndex] = -LaneMap[i].LaneData.x[j];
                        sendmap.lane4y[SaveIndex] = LaneMap[i].LaneData.y[j];
                    }
                }
                j = (size_t)(SaveIndex*step);
                SaveIndex++;
            }
        }

    }

    return 1;
}
int GetMapdatasubmap(PacketSlamPos CurPOS, PacketHighResolutionMap& sendmap, int16_t &stopLineDis)
{
    static int loadsamplepointflg;
    vector<float> samplex, sampley;
    static vector<float> cursamplex, cursampley;
    int sendgpsflag = 1;
    float Wide = 30.f;
    float Length = 60.f;
    float GPSMapLength = 80.f;
    static vector<float> Goalx;
    static vector<float> Goaly;

    vector<float> Lanex, Laney;
    vector<float> IBEOx, IBEOy;
    vector<float> mappointx, mappointy;
    vector<int> mappointstyle;
    vector<float> SendLanex, SendLaney, SendLanei;
    float Distance = 0;
    float x0 = 0, y0 = 0, dhead = 0;

    vector<LaneCell> LaneMap;

    double GPSX, GPSY;

    GPSX = CurPOS.x;
    GPSY = CurPOS.y;

    cout<<" "<<GPSX<<" "<<GPSY<<" "<<CurPOS.heading<<endl;

    CurPOS.heading = CurPOS.heading / 180 * (long double)PI64;


    int CoordsX, CoordsY;
    CoordsX = MAP_GXWX(GoalGridMap, GPSX);
    CoordsY = MAP_GYWY(GoalGridMap, GPSY);

    memset(sendmap.ibcontourx, 0, sizeof(sendmap.ibcontourx));
    memset(sendmap.ibcontoury, 0, sizeof(sendmap.ibcontoury));
    memset(sendmap.lane1x, 999, sizeof(sendmap.lane1x));
    memset(sendmap.lane2x, 999, sizeof(sendmap.lane1x));
    memset(sendmap.lane3x, 999, sizeof(sendmap.lane1x));
    memset(sendmap.lane4x, 999, sizeof(sendmap.lane1x));
    memset(sendmap.lane1y, 999, sizeof(sendmap.lane1x));
    memset(sendmap.lane2y, 999, sizeof(sendmap.lane1x));
    memset(sendmap.lane3y, 999, sizeof(sendmap.lane1x));
	memset(sendmap.lane4y, 999, sizeof(sendmap.lane1x));
	memset(sendmap.gpsx, 0, sizeof(sendmap.gpsx));
	memset(sendmap.gpsy, 0, sizeof(sendmap.gpsy));
	memset(sendmap.MapFlag, 0, sizeof(sendmap.MapFlag));
	memset(sendmap.curve, 0, sizeof(sendmap.curve));

    memset(sendmap.pointx, 0, sizeof(sendmap.pointx));
    memset(sendmap.pointy, 0, sizeof(sendmap.pointy));
    memset(sendmap.pointxstyle, 0, sizeof(sendmap.pointxstyle));

    if (sendgpsflag)
    {
        size_t Index = 0;

        Index = ComputeMinIndex(GPSX, GPSY, CurPOS.heading);
        if (Index)
        {
            gpsx.clear();
            gpsy.clear();
            MapFlag.clear();
            Distance = 0;
            for (size_t i = Index; i < GPSPathx.size(); i++)
            {
                float distance;

                if (gpsx.size()>2)
                {
                    distance = Computedistance(gpsx.back(), gpsy.back(), GPSPathx[i], GPSPathy[i]);
                    Distance += distance;
                    if (Distance>GPSMapLength)
                    {
                        break;
                    }

                }
                gpsx.push_back(GPSPathx[i]);
                gpsy.push_back(GPSPathy[i]);
                MapFlag.push_back(GPSMap.gps[i].flag);
            }
            cout << "flag  " << GPSMap.gps[Index].flag << "  ";


            x0 = (float)GPSX;
            y0 = (float)GPSY;
            dhead = (float)(CurPOS.heading);
            CalculateSensorData(gpsx, gpsy, x0, y0, dhead);



            if (gpsx.size() < LENGTHOFGPSPATH)
            {
                for (size_t i = 0; i < LENGTHOFGPSPATH; i++)
                {
                    if (i < gpsx.size())
                    {
                        sendmap.gpsx[i] = gpsx[i];
                        sendmap.gpsy[i] = gpsy[i];
                        sendmap.MapFlag[i] = MapFlag[i];
                        sendmap.curve[i] = 0;
                    }
                    else
                    {
                        sendmap.gpsx[i] = gpsx.back();
                        sendmap.gpsy[i] = gpsy.back();
                        sendmap.MapFlag[i] = MapFlag.back();;
                        sendmap.curve[i] = 0;
                    }
                }
            }
            else
            {
                float step = gpsx.size() / ((float)LENGTHOFGPSPATH);
                for (size_t i = 0; i < LENGTHOFGPSPATH; i++)
                {
                    size_t j = (size_t)(i*step);
                    sendmap.gpsx[i] = gpsx[j];
                    sendmap.gpsy[i] = gpsy[j];
                    sendmap.MapFlag[i] = MapFlag[j];
                    sendmap.curve[i] = 0;
                }

            }
        }
        else
        {
            cout << "no SlamPOS path" << endl;

            //return 1;
        }
    }


    if(mapLoadFlag<0)
    {
        cout<<"no map data"<<endl;
        return 1;
    }


    if (!MAP_VALID(GoalGridMap, CoordsX, CoordsY))
    {
        cout << "no in the map!!!" << endl;

        return 1;
    }

    //¹Ì¶šÇøÓò²ÉÑù
    //²ÉÑù²ßÂÔ£¬œ«¹Ì¶š²ÉÑùµã£¬Ðý×ªµœµØÍŒ¶ÔÓŠÇøÓò£¬Ö±œÓŒÆËã²ÉÑùµãindex²ÉÑù
    if (!loadsamplepointflg)
    {
        loadsamplepointflg++;
        for (int i = -Wide / MapZeroGps.scale; i < Wide / MapZeroGps.scale; i++)
            for (int j = -5 / MapZeroGps.scale; j < Length / MapZeroGps.scale; j++)
            {
                cursamplex.push_back(i*MapZeroGps.scale);
                cursampley.push_back(j*MapZeroGps.scale);
            }
    }
    /*****************µãÔÆ¡¢³µµÀÏßÌáÈ¡********************/
    x0 = 0, y0 = 0, dhead = 0;
    samplex = cursamplex;
    sampley = cursampley;
    dhead = (float)( - CurPOS.heading);
    CalculateSensorData(samplex, sampley, x0, y0, dhead);
    x0 = (float)-GPSX;
    y0 = (float)-GPSY;
    dhead = 0;
    CalculateSensorData(samplex, sampley, x0, y0, dhead);

    int i_cloud = 0;
    for (size_t i = 0; i < samplex.size(); i++)
    {
        map_cell_t *cell;
        int cx, cy;
        int submapindex_ = 0;
        cx = MAP_GXWX(GoalGridMap, samplex[i]);
        cy = MAP_GYWY(GoalGridMap, sampley[i]);
        if (!MAP_VALID(GoalGridMap, cx, cy))
        {
            continue;
        }
        else
        {
            int subi_, subj_;
            int index_i, index_j;
            int *saveindex;
            map_cell_t *cell;
            index_i = int(cx / GoalGridMap->submapscale);
            index_j = int(cy / GoalGridMap->submapscale);

            subi_ = cx % GoalGridMap->submapscale;
            subj_ = cy % GoalGridMap->submapscale;
            if (!SUBMAPINDEX_VALID(GoalGridMap, index_i, index_j))
                continue;

            saveindex = GoalGridMap->submapindex + SUBMAPINDEX_INDEX(GoalGridMap, index_i, index_j);

            if (!SUBMAP_VALID(GoalGridMap, subi_, subj_))
                continue;

            cell = GoalGridMap->cells + *saveindex*GoalGridMap->submapsize + SUBMAP_INDEX(GoalGridMap, subi_, subj_);

            if (cell->occ_state == 1)//µãÔÆ±êÖŸÎ»
            {
                IBEOx.push_back(cursamplex[i]);
                IBEOy.push_back(cursampley[i]);
            }
            else if (cell->occ_state >1 && cell->occ_state<99) // µãÔÆ±êÖŸÎ»
            {
                mappointx.push_back(cursamplex[i]);
                mappointy.push_back(cursampley[i]);
                mappointstyle.push_back((int16_t)cell->occ_state);
                //cout << "stop line" << endl;
            }
            else if (cell->occ_state > 99) // µãÔÆ±êÖŸÎ»
            {
                SendLanex.push_back(cursamplex[i]);
                SendLaney.push_back(cursampley[i]);
                SendLanei.push_back((int)cell->occ_state);
            }
        }
    }
    /*****************µãÔÆ¡¢³µµÀÏßÌáÈ¡********************/
    //  cout << "lane number=" << SendLaney.size() << endl;

    for (int i = 0; i<mappointx.size() && i<50; i++)
    {

        sendmap.pointx[i] = mappointx[i];
        sendmap.pointy[i] = mappointy[i];
        sendmap.pointxstyle[i] = mappointstyle[i];
    }

	ComputeStopLineDistance(mappointx, mappointy, stopLineDis);

    // cout<<"mappoint number="<<mappointx.size()<<endl;
    if (IBEOy.size()<LENGTHOFSLAMCONTOUR)
    {
        for (int i = 0; i<IBEOx.size(); i++)
        {
            sendmap.ibcontourx[i] = IBEOx[i];
            sendmap.ibcontoury[i] = IBEOy[i];
        }
      /*  int j=IBEOx.size();
        for (int i =0 ; i<gpsx.size(); i++)
        {
            sendmap.ibcontourx[i+j] = gpsx[i];
            sendmap.ibcontoury[i+j] = gpsy[i];
        }

        */
    }
    else
    {
        float step = IBEOx.size() / ((float)LENGTHOFSLAMCONTOUR);
        for (int i = 0; i<LENGTHOFSLAMCONTOUR; i++)
        {
            int index_cloud = int(i*step);
            if (index_cloud<IBEOx.size())
            {
                sendmap.ibcontourx[i] = IBEOx[index_cloud];
                sendmap.ibcontoury[i] = IBEOy[index_cloud];
            }
        }
    }

    ClusteringLanes(SendLanex, SendLaney, SendLanei, LaneMap, gpsx, gpsy);
    size_t SaveIndex = 0;
    for (size_t i = 0; i < LaneMap.size(); i++)
    {
        if (LaneMap[i].LaneData.x.size() < LENGTHOFLANE)
        {
            for (size_t j = 0; j < LaneMap[i].LaneData.x.size(); j++)
            {
                if (i == 0)
                {
                    sendmap.lane1x[j] = -LaneMap[i].LaneData.x[j];
                    sendmap.lane1y[j] = LaneMap[i].LaneData.y[j];
                }
                else if (i == 1)
                {
                    sendmap.lane2x[j] = -LaneMap[i].LaneData.x[j];
                    sendmap.lane2y[j] = LaneMap[i].LaneData.y[j];
                }
                else if (i == 2)
                {
                    sendmap.lane3x[j] = -LaneMap[i].LaneData.x[j];
                    sendmap.lane3y[j] = LaneMap[i].LaneData.y[j];
                }
                else if (i == 3)
                {
                    sendmap.lane4x[j] = -LaneMap[i].LaneData.x[j];
                    sendmap.lane4y[j] = LaneMap[i].LaneData.y[j];
                }
            }

            size_t j = LaneMap[i].LaneData.x.size() - 1;
            while (j<LENGTHOFLANE)
            {
                if (i == 0)
                {
                    sendmap.lane1x[j] = -LaneMap[i].LaneData.x.back();
                    sendmap.lane1y[j] = LaneMap[i].LaneData.y.back();
                }
                else if (i == 1)
                {
                    sendmap.lane2x[j] = -LaneMap[i].LaneData.x.back();
                    sendmap.lane2y[j] = LaneMap[i].LaneData.y.back();
                }
                else if (i == 2)
                {
                    sendmap.lane3x[j] = -LaneMap[i].LaneData.x.back();
                    sendmap.lane3y[j] = LaneMap[i].LaneData.y.back();
                }
                else if (i == 3)
                {
                    sendmap.lane4x[j] = -LaneMap[i].LaneData.x.back();
                    sendmap.lane4y[j] = LaneMap[i].LaneData.y.back();
                }
                j++;
            }
        }
        else
        {
            float step = (LaneMap[i].LaneData.x.size() / ((float)LENGTHOFLANE));
            SaveIndex = 0;
            for (size_t j = 0; j < LaneMap[i].LaneData.x.size(); j++)
            {
                if (i == 0)
                {
                    if (SaveIndex < LENGTHOFLANE)
                    {
                        sendmap.lane1x[SaveIndex] = -LaneMap[i].LaneData.x[j];
                        sendmap.lane1y[SaveIndex] = LaneMap[i].LaneData.y[j];
                    }
                }
                else if (i == 1)
                {
                    if (SaveIndex < LENGTHOFLANE)
                    {
                        sendmap.lane2x[SaveIndex] = -LaneMap[i].LaneData.x[j];
                        sendmap.lane2y[SaveIndex] = LaneMap[i].LaneData.y[j];
                    }
                }
                else if (i == 2)
                {
                    if (SaveIndex < LENGTHOFLANE)
                    {
                        sendmap.lane3x[SaveIndex] = -LaneMap[i].LaneData.x[j];
                        sendmap.lane3y[SaveIndex] = LaneMap[i].LaneData.y[j];
                    }
                }
                else if (i == 3)
                {
                    if (SaveIndex < LENGTHOFLANE)
                    {
                        sendmap.lane4x[SaveIndex] = -LaneMap[i].LaneData.x[j];
                        sendmap.lane4y[SaveIndex] = LaneMap[i].LaneData.y[j];
                    }
                }
                j = (size_t)(SaveIndex*step);
                SaveIndex++;
            }
		}
	}
	return 1;


}















