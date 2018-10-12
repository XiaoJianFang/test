#pragma once
#include<fstream>
#include<vector>
#include<iostream>
#include "PacketHighResolutionMap.hpp"
#include "PacketGPS.hpp"
#include "PacketSlamPos.hpp"

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
struct MapZero
{
	long double lat;
	long double lon;
	long double head;
	float xoffset;
	float yoffset;
	float scale;
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

int  GetMapdata(PacketGPS Curgps, PacketHighResolutionMap& sendmap);
int  LoadGPSMap(mapdata&gGPSMap,std::string dir);
int  LoadSlamPOSMap(mapdata& gGPSMap,std::string dir);
int GetMapdatasubmap(PacketGPS Curgps, PacketHighResolutionMap& sendmap,int16_t &stopLineDis);
int GetMapdatasubmap(PacketSlamPos CurPOS, PacketHighResolutionMap& sendmap,int16_t &stopLineDis);
