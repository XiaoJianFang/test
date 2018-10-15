#pragma once
#include"Windows.h"
//#include "../include/client/Packet.h"
#include<fstream>
#include<vector>
#include<iostream>
#define PI				3.14159265358979323846f
#define PI64			3.14159265358979323846
using namespace std;
struct GPS
{
	long double lat;
	long double lon;
	long double head;
	float x;
	float y;
	int flag;
};
struct veld
{
	vector<float> l1x;
	vector<float> l2x;
	vector<float> l3x;
	vector<float> l4x;
	vector<float> l1y;
	vector<float> l2y;
	vector<float> l3y;
	vector<float> l4y;
};
struct ibeo
{
	vector<float> x;
	vector<float> y;
};
//struct PacketRef
//{
//	vector<PacketGPS> gps;
//	vector<float> x;
//	vector<float> y;
//};
struct POS
{
	float x;
	float y;
	float theta;
};
struct POSGPS
{
	GPS gps;
	POS pos;
};
//struct PacketMaps
//{
//	vector<GPS>  gps;
//	vector<veld> veld;
//	vector<ibeo> ibeo;
//	vector<PacketRef> Path;
//	vector<vector<int>> Flag;
//};
struct Parameters
{
	float LenForward;
	float LenBackward;
	int SendFrequence = 30;
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
struct Lanelines
{
	vector <float> x;
	vector <float> y;
};
struct LaneCell
{
	float minX;
	float minY;
	float maxX;
	float maxY;
	int RLFlag;
	float distance;
	Lanelines LaneData;
};
//void LoadMap(PacketMaps& gMap);
//void GetMapdata(PacketGPS Curgps, PacketMaps& gMap, PacketHighResolutionMap& sendmap);
//int GetMapdata2(PacketGPS Curgps, PacketMaps& gMap, PacketHighResolutionMap& sendmap);
//float Getdistance(float x1, float y1, float x2, float y2);
//void getGps_Distance(float &dis, long double lat1, long double lon1, long double lat2, long double lon2);
//void CalculateVehicleState(long double lat, long double l_lat, long double lon, long double l_lon, long double head, long double l_head, float& x0, float& y0, float& dhead);
//void CalculateSensorData(vector<float>& vecx, vector<float>& vecy, float x0, float y0, float dhead);
//void LoadGoalMapNoRasterize(mapdata& Map);
//int GetMapdata3(PacketGPS Curgps, PacketHighResolutionMap& sendmap);
//void LoadGPSMap(mapdata&gGPSMap);
