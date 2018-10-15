#ifndef _PACKETMAP_
#define _PACKETMAP_
#include <string>
#include <vector>
#define PI 3.14159265358979323846
using namespace std;

struct VisualData
{
	std::string Style;
	int size;
	int R;
	int G;
	int B;
	std::vector<float> x;
	std::vector<float> y;
};
struct ShowData
{
	std::string Name;
	std::vector <VisualData> Data;
};
struct VisualMouse
{
	float g_winScale;
	float g_offsetx;
	float g_offsety;
	float initScale;
	float gwinX;
	float gwinFwd;
	float gwinBkwd;
	float windowhigh;
	int Distanceflag;
	char labeldst[50];
	char labeldistance[100];
};

struct Point2D
{
	float x;
	float y;
	Point2D() {}
	Point2D(float x_, float y_) : x(x_), y(y_) {}
};
struct POS
{
	float x;
	float y;
	float theta;
};
struct VertexICP
{
	size_t id;
	POS pos;
};
struct EdgeICP
{
	int fix;
	size_t startid;
	size_t endid;
	POS pos;
};
struct GPS
{
	long double lat;
	long double lon;
	long double head;
};
struct POSGPS
{
	GPS gps;
	POS pos;
};
struct veld
{
	vector<float> l1x;
	vector<float> l2x;
	vector<float> l3x;
	vector<float> l4x;
};
struct ibeo
{
	vector<float> x;
	vector<float> y;
};
struct PacketMaps
{
	vector<POSGPS>  posgps;
	vector<veld> veld;
	vector<ibeo> ibeo;
};

struct ICPVisiualData
{
	vector<float> MapCloudx;
	vector<float> MapCloudy;

	vector<float> CurrentCloudx;
	vector<float> CurrentCloudy;

	vector<float> TransfomationCloudx;
	vector<float> TransfomationCloudy;
};

struct mapdata
{
	size_t coordinatezero;
	vector <GPS> gps;
	vector <float> x;
	vector <float> y;
	vector <float> ibeox;
	vector <float> ibeoy;
	vector <float> l1x;
	vector <float> l1y;
	vector <float> l2x;
	vector <float> l2y;
	vector <float> l3x;
	vector <float> l3y;
	vector <float> l4x;
	vector <float> l4y;
};
#endif


