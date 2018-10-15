//#include "../include/PacketMap.h"
//#include "../include/Visual.h"
#include "math.h"
#include <vector>
#include <stdlib.h>
#include <iostream>
#define PI 3.14159265358979323846

using namespace std;
struct Point2D
{
	float x;
	float y;
	Point2D() {}
	Point2D(float x_, float y_) : x(x_), y(y_) {}
};
struct SpeedPoint
{
	float  x;
	float  y;
	float vx;
	float vy;
	float wide;
};
struct PacketVehicleInfo
{
	float speed;
	float wz;
	int steerAngle;
};

void CalculateSensorData(vector<Point2D>&path, float x0, float y0, float dhead)
{
	for (int i = 0; i < path.size(); i++)
	{
		float _vecx = path[i].x;
		float _vecy = path[i].y;
		path[i].x = (float)((_vecx - x0) * cos(dhead) - (_vecy - y0) * sin(dhead));
		path[i].y = (float)((_vecx - x0) * sin(dhead) + (_vecy - y0) * cos(dhead));
	}
}
void CalculateSensorData(Point2D&path, float x0, float y0, float dhead)
{
	float _vecx = path.x;
	float _vecy = path.y;
	path.x = (float)((_vecx - x0) * cos(dhead) - (_vecy - y0) * sin(dhead));
	path.y = (float)((_vecx - x0) * sin(dhead) + (_vecy - y0) * cos(dhead));
}
void  CalculateSensorData(float& vecx, float& vecy, float x0, float y0, float dhead)
{
	float _vecx = vecx;
	float _vecy = vecy;
	vecx = (_vecx - x0) * cos(dhead) - (_vecy - y0) * sin(dhead);
	vecy = (_vecx - x0) * sin(dhead) + (_vecy - y0) * cos(dhead);
}
float Computedistance(float x1, float y1, float x2, float y2)
{
	return sqrt((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2));
}
void NormalOffSet(vector<Point2D>&path, vector<Point2D>& pathoffset, float offset)
{
	int step = 4;
	pathoffset.clear();
	if (path.size() < step) return;
	float curx = offset, cury = 0;
	float dhead;
	vector<float> heading;
	float startheading = 0;
	float currentheading;
	for (size_t i = 1; i < step; i++)
	{
		startheading += atan2f(path[i].y - path[0].y, path[i].x - path[0].x);
	}
	startheading = startheading / (step - 1);
	//std::cout << "angle=" << startheading * 180 / PI << endl;;
	for (size_t i = 0; i < 2 * step; i++)
	{
		curx = offset, cury = 0;
		dhead = PI / 2 - startheading;
		CalculateSensorData(curx, cury, 0, 0, -dhead);
		Point2D point;
		point.x = path[i].x + curx;
		point.y = path[i].y + cury;
		pathoffset.push_back(point);
	}
	for (size_t i = 2 * step; i <path.size() - 2 * step; i++)
	{
		currentheading = 0;
		for (int j = -step; j <= step; j++)
		{
			float xx, yy;
			yy = path[i + j + step].y - path[i + j - step].y;
			xx = path[i + j + step].x - path[i + j - step].x;
			currentheading += atan2f(yy, xx);
		}

		currentheading = currentheading / (2 * step + 1);

		//std::cout << "angle=" << currentheading * 180 / PI << endl;;

		curx = offset, cury = 0;
		dhead = PI / 2 - currentheading;
		CalculateSensorData(curx, cury, 0, 0, -dhead);
		Point2D point;
		point.x = path[i].x + curx;
		point.y = path[i].y + cury;
		pathoffset.push_back(point);
	}

	for (size_t i = path.size() - 2 * step; i < path.size(); i++)
	{
		curx = offset, cury = 0;
		dhead = PI / 2 - currentheading;
		CalculateSensorData(curx, cury, 0, 0, -dhead);
		Point2D point;
		point.x = path[i].x + curx;
		point.y = path[i].y + cury;
		pathoffset.push_back(point);
	}


}
void NormalOffSet(vector<float>x, vector<float>y, vector<float>& xoffset, vector<float>& yoffset, float offset)
{
	int step = 5;
	if ((x.size() < step) || (x.size() != y.size())) return;
	vector<float> heading;
	float startheading = 0;
	float currentheading;
	for (size_t i = 1; i < step; i++)
	{
		startheading += atan2f(y[i] - y[0], x[i] - x[0]);
	}
	startheading = startheading / (step - 1);
	std::cout << "angle=" << startheading * 180 / PI;
	for (size_t i = 0; i < 2 * step; i++)
	{
		float curx = offset, cury = 0;
		float dhead = PI / 2 - startheading;
		CalculateSensorData(curx, cury, 0, 0, -dhead);
		xoffset.push_back(x[i] + curx);
		yoffset.push_back(y[i] + cury);
	}
	for (size_t i = 2 * step; i < x.size() - 2 * step; i++)
	{
		currentheading = 0;
		for (int j = -step; j <= step; j++)
		{
			currentheading += atan2f(y[i + j + step] - y[i + j - step], x[i + j + step] - x[i + j - step]);
		}
		currentheading = currentheading / (2 * step + 1);

		float curx = offset, cury = 0;
		float dhead = PI / 2 - currentheading;
		CalculateSensorData(curx, cury, 0, 0, -dhead);
		xoffset.push_back(x[i] + curx);
		yoffset.push_back(y[i] + cury);
	}

	for (size_t i = x.size() - 2 * step; i < x.size(); i++)
	{
		float curx = offset, cury = 0;
		float dhead = PI / 2 - currentheading;
		CalculateSensorData(curx, cury, 0, 0, -dhead);
		xoffset.push_back(x[i] + curx);
		yoffset.push_back(y[i] + cury);
	}


}
int FindDangerPoint(vector<Point2D>path, Point2D& point, float threshold, size_t& index)
{
	index = 0;
	for (size_t i = 1; i < path.size() - 1; i++)
	{
		if ((path[i].y - threshold)*(path[i + 1].y - threshold) <= 0)
		{
			point.x = path[i].x + (path[i + 1].x - path[i].x) * fabs(path[i].y / (path[i].y - path[i + 1].y));
			point.y = threshold;
			index = i;
			break;
		}
	}

	if (index == 0)
	{
		point.x = -1;
		point.y = threshold;
		return -1;
	}
	else
	{
		return 0;
	}
}

float ComputeCrashTime(const SpeedPoint& obs, const PacketVehicleInfo& vState, const vector<Point2D>&path, vector<Point2D>&view)
{
	float  VehicleChangeLanevx = 0.01f;//车辆变道横向速度阈值
	float   VehicleFrontDistance = 2.8f;//坐标原点距车头距离
	float   VehicleBehindDistance = 0.8f;//坐标原点距车尾距离
	float 	collisionthreshold = 4.0f;  //碰撞阈值
	float   AngleThreshold = 0.00001f / 180 * PI;//判断障碍物方向与汽车行进方向是否平行角度阈值
	float   Vehiclewide = 2;//车辆宽度
	float   ObsHeading;//障碍物速度方向平行车辆行进方向为零，偏右为正
	float   Obsspeed;//障碍物绝对速度
	float   Obstime, Obstime2;//障碍物进入碰撞区域时间，出碰撞区域时间
	float   MinSpeedStaticThreshold = 0.3;//当速度小于此速度，认为静止或者相对静止
	float   MinSpeedDynamicThreshold = 0.1;//当速度小于此速度，认为静止或者相对静止
	float   safedistance = 0;//安全距离
	int     CollisionIndex = 0;//碰撞点的Index（碰撞点在车辆规划轨迹序列的index）
	int     LeaveIndex = 0;//碰撞点的Index（碰撞点在车辆规划轨迹序列的index）
	float   Vehicletime;//车辆进入碰撞区域时间
	float   Vehicletime2;//离开碰撞区域时间
	float ObsticleDistance = 1000;//障碍物进入碰撞区的距离
	float ObsticleDistance2 = 0;//障碍物离开碰撞区的距离
	int     ObstaclesStaticFlag = 0;//障碍物静止
	int     VehicleStaticFlag = 0;//车辆静止
	vector<Point2D> temppath = path;//车辆规划轨迹
	vector<Point2D> DangerZoneLeft, DangerZoneRight;
	SpeedPoint ObsAbsolute = obs;//障碍物绝对速度信息
	float x0, y0, dhead;
	Point2D VehilceDanger;
	size_t DangerIndex;

	/*********************/
	//接收的障碍物速度信息为相对速度
	//车辆速度为绝对速度
	/*********************/
	NormalOffSet(temppath, DangerZoneLeft, -Vehiclewide);
	NormalOffSet(temppath, DangerZoneRight, Vehiclewide);

	if (obs.vy>0)//车速小于障碍物速度不会碰撞
	{
		std::cout << "车速小于障碍物速度不会碰撞" << endl;
		return -1;
	}
	ObsAbsolute.vy += vState.speed;//障碍物速度变换为绝对速度

	ObsHeading = PI / 2 - atan2f(ObsAbsolute.vy, obs.vx);
	if (ObsHeading>PI)
	{
		ObsHeading -= 2 * PI;
	}

	std::cout << "ObsHeading=" << ObsHeading * 180 / PI << endl;

	//将坐标转换为以障碍物原点
	dhead = PI / 2 - ObsHeading;
	x0 = obs.x;
	y0 = obs.y;
	x0 = obs.x;
	y0 = obs.y;
	CalculateSensorData(temppath, x0, y0, 0);
	CalculateSensorData(DangerZoneLeft, x0, y0, 0);
	CalculateSensorData(DangerZoneRight, x0, y0, 0);

	dhead = PI / 2 - ObsHeading;
	CalculateSensorData(temppath, 0, 0, -dhead);
	CalculateSensorData(DangerZoneLeft, 0, 0, -dhead);
	CalculateSensorData(DangerZoneRight, 0, 0, -dhead);

	if (vState.speed<MinSpeedStaticThreshold)
	{
		//车辆静止
		VehicleStaticFlag = 1;
		if (fabs(temppath[0].y)>Vehiclewide)
		{
			//车辆当前位置不位于碰撞区域
			std::cout << "车辆静止，不在碰撞区域" << endl;
			return -1;
		}
		else
		{
			//车辆当前位置位于碰撞区域
			//障碍物向着车辆运动
			if (fabs(ObsAbsolute.vy)>MinSpeedStaticThreshold)
			{
				std::cout << "车辆静止，在碰撞区域，危险！！！" << endl;
				return 0;
			}
			else
			{
				std::cout << "车辆静止，不在碰撞区域" << endl;
				return -1;
			}
		}
	}

	//车辆进入碰撞区域的坐标
	Point2D VehicleDangerFirst;
	Point2D VehicleDangerSecond;

	size_t DangerIndexFirst = 0, DangerIndexSecond = 0;
	if (obs.vx>0)
	{
		FindDangerPoint(DangerZoneLeft, VehicleDangerFirst, 0, DangerIndexFirst);
		FindDangerPoint(DangerZoneRight, VehicleDangerSecond, 0, DangerIndexSecond);
	}
	else if (obs.vx<0)
	{
		FindDangerPoint(DangerZoneRight, VehicleDangerFirst, 0, DangerIndexFirst);
		FindDangerPoint(DangerZoneLeft, VehicleDangerSecond, 0, DangerIndexSecond);
	}
	else if (obs.x>0)
	{
		FindDangerPoint(DangerZoneRight, VehicleDangerFirst, 0, DangerIndexFirst);
		FindDangerPoint(DangerZoneLeft, VehicleDangerSecond, 0, DangerIndexSecond);
	}
	else
	{
		FindDangerPoint(DangerZoneLeft, VehicleDangerFirst, 0, DangerIndexFirst);
		FindDangerPoint(DangerZoneRight, VehicleDangerSecond, 0, DangerIndexSecond);
	}

	dhead = PI / 2 - ObsHeading;
	x0 = obs.x;
	y0 = obs.y;
	Point2D point;

	if (DangerIndexFirst)
	{
		point = VehicleDangerFirst;
		CalculateSensorData(point, 0, 0, dhead);
		CalculateSensorData(point, -x0, -y0, 0);
		view.push_back(point);
	}

	if (DangerIndexSecond)
	{
		point = VehicleDangerSecond;
		CalculateSensorData(point, 0, 0, dhead);
		CalculateSensorData(point, -x0, -y0, 0);
		view.push_back(point);
	}

	if (ObsAbsolute.vy>0)
	{
		if (DangerIndexFirst == 0 && DangerIndexSecond == 0)
		{
			if (fabs(obs.x) > Vehiclewide)
			{
				std::cout << "不在同车道，轨迹不相交，不会碰撞" << endl;
				return -1;
			}
			else
			{
				for (int i = 0; i < temppath.size() - 1; i++)
				{
					safedistance += Computedistance(temppath[i].x, temppath[i].y, temppath[i + 1].x, temppath[i + 1].y);
				}
				Vehicletime2 = (safedistance + VehicleBehindDistance) / vState.speed;//车辆离开碰撞区域时间

				ObsticleDistance2 = temppath.back().x;
				Obsspeed = sqrtf(ObsAbsolute.vx*ObsAbsolute.vx + ObsAbsolute.vy*ObsAbsolute.vy);//计算障碍物绝对速度
				Obstime2 = ObsticleDistance2 / Obsspeed;//障碍物离开碰撞区域时间

				if (Obstime2 < Vehicletime2)//障碍物走过碰撞区域，车辆还未走过，车辆未追上，不会发生碰撞
				{
					std::cout << "障碍物走过碰撞区域，车辆还未追上，不会发生碰撞" << endl;
					return -1;
				}
				else                        //车辆和障碍物都在碰撞区域，一定会发生碰撞
				{
					std::cout << "在同车道，车辆会追上障碍物" << endl;
					std::cout << "一定会发生碰撞" << endl;
					safedistance = Computedistance(0, 0, obs.x, obs.y);
					std::cout << "safedistance=" << safedistance << endl;
					Vehicletime = (safedistance - VehicleFrontDistance) / (-obs.vy);//车辆进入碰撞区域时间
					std::cout << "time=" << Vehicletime << endl;
					return Vehicletime;
				}
			}
		}
		else if ((DangerIndexFirst&&VehicleDangerFirst.x<0) && (DangerIndexSecond&&VehicleDangerSecond.x<0))
		{
			std::cout << "轨迹不相交，不会碰撞" << endl;
			return -1;
		}
		else if ((DangerIndexFirst) && (DangerIndexSecond == 0))
		{
			if (VehicleDangerFirst.x>0)
			{
				for (int i = 0; i < DangerIndexFirst; i++)
				{
					safedistance += Computedistance(temppath[i].x, temppath[i].y, temppath[i + 1].x, temppath[i + 1].y);
				}
				Vehicletime = (safedistance - VehicleFrontDistance) / vState.speed;//车辆进入碰撞区域时间
				std::cout << "safedistance=" << safedistance << endl;
				ObsticleDistance = VehicleDangerFirst.x;
				if (ObsticleDistance<0)
				{
					std::cout << "error！！！" << endl;
					return -1;
				}
				Obsspeed = sqrtf(ObsAbsolute.vx*ObsAbsolute.vx + ObsAbsolute.vy*ObsAbsolute.vy);//计算障碍物绝对速度
				Obstime = ObsticleDistance / Obsspeed;//障碍物进入碰撞区域时间
				std::cout << "Obstime=" << Obstime << endl;
				std::cout << "Vehicletime=" << Vehicletime << endl;

				if (Obstime>Vehicletime)
				{
					std::cout << "车辆走过碰撞区域，障碍物还未进入，不会发生碰撞" << endl;
					return -1;
				}
				else
				{
					for (int i = DangerIndexFirst; i < temppath.size() - 1; i++)
					{
						safedistance += Computedistance(temppath[i].x, temppath[i].y, temppath[i + 1].x, temppath[i + 1].y);
					}
					Vehicletime2 = (safedistance + VehicleBehindDistance) / vState.speed;//车辆离开碰撞区域时间

					ObsticleDistance2 = temppath.back().x;
					Obsspeed = sqrtf(ObsAbsolute.vx*ObsAbsolute.vx + ObsAbsolute.vy*ObsAbsolute.vy);//计算障碍物绝对速度
					Obstime2 = ObsticleDistance2 / Obsspeed;//障碍物离开碰撞区域时间

					if (Obstime2 < Vehicletime2)//障碍物走过碰撞区域，车辆还未走过，车辆未追上，不会发生碰撞
					{
						std::cout << "障碍物走过碰撞区域，车辆还未进入，不会发生碰撞" << endl;
						return -1;
					}
					else                        //车辆和障碍物都在碰撞区域，一定会发生碰撞
					{
						safedistance = Computedistance(0, 0, obs.x, obs.y);
						std::cout << "safedistance=" << safedistance << endl;
						Vehicletime = (safedistance - VehicleFrontDistance) / (-obs.vy);//车辆进入碰撞区域时间
						std::cout << "知道相遇地点，障碍物在本车道会碰撞" << endl;
						std::cout << "一定会发生碰撞" << endl;
						std::cout << "time=" << Vehicletime << endl;
						return Vehicletime;
					}
				}
			}
			else //障碍物已在碰撞区
			{
				for (int i = 0; i < temppath.size() - 1; i++)
				{
					safedistance += Computedistance(temppath[i].x, temppath[i].y, temppath[i + 1].x, temppath[i + 1].y);
				}
				Vehicletime2 = (safedistance + VehicleBehindDistance) / vState.speed;//车辆离开碰撞区域时间

				ObsticleDistance2 = temppath.back().x;
				Obsspeed = sqrtf(ObsAbsolute.vx*ObsAbsolute.vx + ObsAbsolute.vy*ObsAbsolute.vy);//计算障碍物绝对速度
				Obstime2 = ObsticleDistance2 / Obsspeed;//障碍物离开碰撞区域时间
				std::cout << "Obstime=" << Obstime2 << endl;
				std::cout << "Vehicletime=" << Vehicletime2 << endl;

				if (Obstime2 < Vehicletime2)//障碍物走过碰撞区域，车辆还未走过，车辆未追上，不会发生碰撞
				{
					std::cout << "障碍物走过碰撞区域，车辆还未追上，不会发生碰撞" << endl;
					return -1;
				}
				else                        //车辆和障碍物都在碰撞区域，一定会发生碰撞
				{
					std::cout << "在同车道，车辆会追上障碍物" << endl;
					std::cout << "一定会发生碰撞" << endl;
					safedistance = Computedistance(0, 0, obs.x, obs.y);
					std::cout << "safedistance=" << safedistance << endl;
					Vehicletime = (safedistance - VehicleFrontDistance) / (-obs.vy);//车辆进入碰撞区域时间
					std::cout << "time=" << Vehicletime << endl;
					return Vehicletime;
				}
			}

		}
		else if ((DangerIndexFirst == 0 || (DangerIndexFirst&&VehicleDangerFirst.x<0)) && DangerIndexSecond)
		{
			if (VehicleDangerSecond.x<0)
			{
				std::cout << "障碍物远离车辆" << endl;
				return -1;

			}
			std::cout << "车辆已在危险区域！！！" << endl;
			for (int i = 0; i < DangerIndexSecond; i++)
			{
				safedistance += Computedistance(temppath[i].x, temppath[i].y, temppath[i + 1].x, temppath[i + 1].y);
			}
			Vehicletime = (safedistance - VehicleFrontDistance) / vState.speed;//车辆进入碰撞区域时间
			std::cout << "safedistance=" << safedistance << endl;
			ObsticleDistance = VehicleDangerSecond.x;

			Obsspeed = sqrtf(ObsAbsolute.vx*ObsAbsolute.vx + ObsAbsolute.vy*ObsAbsolute.vy);//计算障碍物绝对速度
			Obstime = ObsticleDistance / Obsspeed;//障碍物进入碰撞区域时间
			std::cout << "Obstime=" << Obstime << endl;
			std::cout << "Vehicletime=" << Vehicletime << endl;
			if (Obstime<Vehicletime)
			{
				std::cout << "障碍物先出碰撞区域，车辆还未追上，不会发生碰撞" << endl;
				return -1;
			}
			else
			{
				std::cout << "障碍物后出碰撞区" << endl;
				safedistance = Computedistance(0, 0, obs.x, obs.y);
				std::cout << "safedistance=" << safedistance << endl;
				Vehicletime = (safedistance - VehicleFrontDistance) / (-obs.vy);//车辆进入碰撞区域时间
				std::cout << "一定会发生碰撞" << endl;
				std::cout << "time=" << Vehicletime << endl;
				return Vehicletime;
			}
		}
		else if (DangerIndexFirst&&VehicleDangerFirst.x>0 && VehicleDangerSecond.x>0 && DangerIndexSecond)
		{

			safedistance = 0;
			for (int i = 0; i < DangerIndexFirst; i++)
			{
				safedistance += Computedistance(temppath[i].x, temppath[i].y, temppath[i + 1].x, temppath[i + 1].y);
			}
			Vehicletime = (safedistance - VehicleFrontDistance) / vState.speed;//车辆进入碰撞区域时间
			std::cout << "safedistance=" << safedistance << endl;

			for (int i = DangerIndexFirst; i < DangerIndexSecond; i++)
			{
				safedistance += Computedistance(temppath[i].x, temppath[i].y, temppath[i + 1].x, temppath[i + 1].y);
			}
			Vehicletime2 = (safedistance + VehicleBehindDistance) / vState.speed;//车辆离开碰撞区域时间

			ObsticleDistance = VehicleDangerFirst.x;
			ObsticleDistance2 = VehicleDangerSecond.x;
			Obsspeed = sqrtf(ObsAbsolute.vx*ObsAbsolute.vx + ObsAbsolute.vy*ObsAbsolute.vy);//计算障碍物绝对速度
			Obstime = ObsticleDistance / Obsspeed;//障碍物进入碰撞区域时间
			Obstime2 = ObsticleDistance2 / Obsspeed;//障碍物离开碰撞区域时间
			std::cout << "Obstime=" << Obstime << endl;
			std::cout << "Vehicletime=" << Vehicletime << endl;

			//前提:车速大于障碍物速度，障碍物模型为质点模型
			//不会碰撞的条件
			//1车辆先进入碰撞区
			//2障碍物先进碰撞区，障碍物先出碰撞区
			//碰撞条件
			//障碍物先进碰撞区，后出碰撞区
			if (Vehicletime < Obstime)//车辆走过碰撞区域，障碍物还未进入，不会发生碰撞
			{
				std::cout << "车辆先进入碰撞区域，障碍物还未进入，不会发生碰撞" << endl;
				return -1;
			}
			else if (Vehicletime>Obstime&&Vehicletime2>Obstime2)
			{
				std::cout << "障碍物先进入碰撞区域，障碍物先出碰撞区域，不会发生碰撞" << endl;
				return -1;
			}
			else  if (Obstime<Vehicletime&&Vehicletime2<Obstime2)
			{
				std::cout << "障碍物先进后出，一定会发生碰撞" << endl;
				safedistance = Computedistance(0, 0, obs.x, obs.y);
				std::cout << "safedistance=" << safedistance << endl;
				Vehicletime = (safedistance - VehicleFrontDistance) / (-obs.vy);
				std::cout << "time=" << Vehicletime << endl;
				return Vehicletime;
			}
			else
			{
				std::cout << "unknow" << endl;
				return -1;
			}
		}
	}
	else
	{
		if (DangerIndexFirst == 0 && DangerIndexSecond == 0)
		{
			if (fabs(obs.x) > Vehiclewide)
			{
				std::cout << "不在同车道，轨迹不相交，不会碰撞" << endl;
				return -1;
			}
			else
			{
				std::cout << "在同车道，相向行驶" << endl;
				std::cout << "一定会发生碰撞" << endl;
				safedistance = Computedistance(0, 0, obs.x, obs.y);
				std::cout << "safedistance=" << safedistance << endl;
				Vehicletime = (safedistance - VehicleFrontDistance) / (-obs.vy);//车辆进入碰撞区域时间
				std::cout << "time=" << Vehicletime << endl;
				return Vehicletime;
			}

		}
		else if ((DangerIndexFirst&&VehicleDangerFirst.x<0) && (DangerIndexSecond&&VehicleDangerSecond.x<0))
		{
			std::cout << "轨迹不相交，不会碰撞" << endl;
			return -1;
		}
		else if ((DangerIndexFirst) && (DangerIndexSecond == 0))
		{
			if (VehicleDangerFirst.x>0)
			{
				for (int i = 0; i < DangerIndexFirst; i++)
				{
					safedistance += Computedistance(temppath[i].x, temppath[i].y, temppath[i + 1].x, temppath[i + 1].y);
				}
				Vehicletime = (safedistance - VehicleFrontDistance) / vState.speed;//车辆进入碰撞区域时间
				std::cout << "safedistance=" << safedistance << endl;
				ObsticleDistance = VehicleDangerFirst.x;
				if (ObsticleDistance<0)
				{
					std::cout << "error！！！" << endl;
					return -1;
				}
				Obsspeed = sqrtf(ObsAbsolute.vx*ObsAbsolute.vx + ObsAbsolute.vy*ObsAbsolute.vy);//计算障碍物绝对速度
				Obstime = ObsticleDistance / Obsspeed;//障碍物进入碰撞区域时间
				std::cout << "Obstime=" << Obstime << endl;
				std::cout << "Vehicletime=" << Vehicletime << endl;

				if (Obstime>Vehicletime)
				{
					std::cout << "车辆走过碰撞区域，障碍物还未进入，不会发生碰撞" << endl;
					return -1;
				}
				else
				{
					//车辆和障碍物都在碰撞区域，一定会发生碰撞
					safedistance = Computedistance(0, 0, obs.x, obs.y);
					std::cout << "safedistance=" << safedistance << endl;
					Vehicletime = (safedistance - VehicleFrontDistance) / (-obs.vy);//车辆进入碰撞区域时间
					std::cout << "知道相遇地点，障碍物在本车道会碰撞" << endl;
					std::cout << "一定会发生碰撞" << endl;
					std::cout << "time=" << Vehicletime << endl;
					return Vehicletime;
				}
			}
			else
			{
				//车辆和障碍物都在碰撞区域，一定会发生碰撞
				safedistance = Computedistance(0, 0, obs.x, obs.y);
				std::cout << "safedistance=" << safedistance << endl;
				Vehicletime = (safedistance - VehicleFrontDistance) / (-obs.vy);//车辆进入碰撞区域时间
				std::cout << "知道相遇地点，障碍物在本车道会碰撞" << endl;
				std::cout << "一定会发生碰撞" << endl;
				std::cout << "time=" << Vehicletime << endl;
				return Vehicletime;
			}


		}
		else if ((DangerIndexFirst == 0 || (DangerIndexFirst&&VehicleDangerFirst.x<0)) && DangerIndexSecond)
		{
			if (VehicleDangerSecond.x<0)
			{
				std::cout << "障碍物远离车辆" << endl;
				return -1;

			}

			std::cout << "车辆已在危险区域！！！" << endl;
			for (int i = 0; i < DangerIndexSecond; i++)
			{
				safedistance += Computedistance(temppath[i].x, temppath[i].y, temppath[i + 1].x, temppath[i + 1].y);
			}
			Vehicletime = (safedistance - VehicleFrontDistance) / vState.speed;//车辆进入碰撞区域时间
			std::cout << "safedistance=" << safedistance << endl;
			ObsticleDistance = VehicleDangerSecond.x;

			Obsspeed = sqrtf(ObsAbsolute.vx*ObsAbsolute.vx + ObsAbsolute.vy*ObsAbsolute.vy);//计算障碍物绝对速度
			Obstime = ObsticleDistance / Obsspeed;//障碍物进入碰撞区域时间
			std::cout << "Obstime=" << Obstime << endl;
			std::cout << "Vehicletime=" << Vehicletime << endl;
			if (Obstime<Vehicletime)
			{
				std::cout << "障碍物先出碰撞区域，车辆还未追上，不会发生碰撞" << endl;
				return -1;
			}
			else
			{
				std::cout << "障碍物后出碰撞区" << endl;
				safedistance = Computedistance(0, 0, obs.x, obs.y);
				std::cout << "safedistance=" << safedistance << endl;
				Vehicletime = (safedistance - VehicleFrontDistance) / (-obs.vy);//车辆进入碰撞区域时间
				std::cout << "一定会发生碰撞" << endl;
				std::cout << "time=" << Vehicletime << endl;
				return Vehicletime;
			}
		}
		else if (DangerIndexFirst&&VehicleDangerFirst.x>0 && VehicleDangerSecond.x>0 && DangerIndexSecond)
		{

			safedistance = 0;
			for (int i = 0; i < DangerIndexFirst; i++)
			{
				safedistance += Computedistance(temppath[i].x, temppath[i].y, temppath[i + 1].x, temppath[i + 1].y);
			}
			Vehicletime = (safedistance - VehicleFrontDistance) / vState.speed;//车辆进入碰撞区域时间
			std::cout << "safedistance=" << safedistance << endl;

			for (int i = 0; i < DangerIndexSecond; i++)
			{
				safedistance += Computedistance(temppath[i].x, temppath[i].y, temppath[i + 1].x, temppath[i + 1].y);
			}
			Vehicletime2 = (safedistance + VehicleBehindDistance) / vState.speed;//车辆离开碰撞区域时间

			ObsticleDistance = VehicleDangerFirst.x;
			ObsticleDistance2 = VehicleDangerSecond.x;
			Obsspeed = sqrtf(ObsAbsolute.vx*ObsAbsolute.vx + ObsAbsolute.vy*ObsAbsolute.vy);//计算障碍物绝对速度
			Obstime = ObsticleDistance / Obsspeed;//障碍物进入碰撞区域时间
			Obstime2 = ObsticleDistance2 / Obsspeed;//障碍物离开碰撞区域时间
			std::cout << "Obstime=" << Obstime << endl;
			std::cout << "Vehicletime=" << Vehicletime << endl;

			//前提:相向行驶，障碍物模型为质点模型
			//不会碰撞的条件
			//1车辆未进入，障碍物已出
			//2障碍物未进入，车辆已出
			//碰撞条件
			//else
			if (Vehicletime < Obstime)//车辆走过碰撞区域，障碍物还未进入，不会发生碰撞
			{
				std::cout << "车辆先进入碰撞区域，障碍物还未进入，不会发生碰撞" << endl;
				return -1;
			}
			else if (Vehicletime2>Obstime2)
			{
				std::cout << "障碍物先出碰撞区域，车辆还未进入,不会发生碰撞" << endl;
				return -1;
			}
			else
			{
				std::cout << "障碍物车辆都在碰撞区         ，一定会发生碰撞" << endl;
				safedistance = Computedistance(0, 0, obs.x, obs.y);
				std::cout << "safedistance=" << safedistance << endl;
				Vehicletime = (safedistance - VehicleFrontDistance) / (-obs.vy);
				std::cout << "time=" << Vehicletime << endl;
				return Vehicletime;
			}
		}

	}



}
int main( )
{
	SpeedPoint Obstacle;
	PacketVehicleInfo  Vehicle;
	vector<Point2D> Path;
	vector<Point2D> viewdata;
	vector<Point2D> viewdata2;
	vector<float>x1, y1, x2, y2;


	Obstacle.x = 1.5;
	Obstacle.y = 50;
	Obstacle.vx = -0.15;
	Obstacle.vy = -5.00;

	Vehicle.speed = 5.0f;
	Vehicle.steerAngle = 0.f;
	Vehicle.wz = 0.f;
	Point2D point;
	for (int i = 0; i < 200; i++)
	{
		point.x = 0.1*sin(i / 80.0f*PI);
		point.y = i / 2.0;
		Path.push_back(point);
	}

	//生成障碍物轨迹
	for (int i = -100; i < 100; i++)
	{
		float ii;
		if (i>0)
		{
			ii = i / 2.0;
		}
		else
		{
			ii = i;
		}
		Point2D point;
		point.x = Obstacle.x + Obstacle.vx*ii;
		point.y = Obstacle.y + (Obstacle.vy + Vehicle.speed)*ii;
		viewdata2.push_back(point);
	}
	for (size_t i = 0; i < viewdata2.size(); i++)
	{
		x2.push_back(viewdata2[i].x);
		y2.push_back(viewdata2[i].y);
	}
	float time = 0;
	std::cout << "ObsAbsolute.vx=" << Obstacle.vx << endl;
	std::cout << "ObsAbsolute.vy=" << Obstacle.vy << endl;
	std::cout << "ObsAbsolute.x=" << Obstacle.x << endl;
	std::cout << "ObsAbsolute.y=" << Obstacle.y << endl;
	std::cout << "Vehicle.speed=" << Vehicle.speed << endl;
	time = ComputeCrashTime(Obstacle, Vehicle, Path, viewdata);

	for (size_t i = 0; i < Path.size(); i++)
	{
		x1.push_back(Path[i].x);
		y1.push_back(Path[i].y);
	}

	for (size_t i = 0; i < viewdata.size(); i++)
	{
		x1.push_back(viewdata[i].x);
		y1.push_back(viewdata[i].y);
	}

	point.x = Obstacle.x;
	point.y = Obstacle.y;
	x1.push_back(point.x);
	y1.push_back(point.y);

	NormalOffSet(Path, viewdata2, 2);

	for (size_t i = 0; i < viewdata2.size(); i++)
	{
		x2.push_back(viewdata2[i].x);
		y2.push_back(viewdata2[i].y);
	}

	NormalOffSet(Path, viewdata2, -2);

	for (size_t i = 0; i < viewdata2.size(); i++)
	{
		x2.push_back(viewdata2[i].x);
		y2.push_back(viewdata2[i].y);
	}
	while (1);
	//ShowDatas(x1, y1, x2, y2);
	return 0;
}