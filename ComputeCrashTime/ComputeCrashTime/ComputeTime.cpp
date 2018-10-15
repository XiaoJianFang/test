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
	float  VehicleChangeLanevx = 0.01f;//������������ٶ���ֵ
	float   VehicleFrontDistance = 2.8f;//����ԭ��೵ͷ����
	float   VehicleBehindDistance = 0.8f;//����ԭ��೵β����
	float 	collisionthreshold = 4.0f;  //��ײ��ֵ
	float   AngleThreshold = 0.00001f / 180 * PI;//�ж��ϰ��﷽���������н������Ƿ�ƽ�нǶ���ֵ
	float   Vehiclewide = 2;//�������
	float   ObsHeading;//�ϰ����ٶȷ���ƽ�г����н�����Ϊ�㣬ƫ��Ϊ��
	float   Obsspeed;//�ϰ�������ٶ�
	float   Obstime, Obstime2;//�ϰ��������ײ����ʱ�䣬����ײ����ʱ��
	float   MinSpeedStaticThreshold = 0.3;//���ٶ�С�ڴ��ٶȣ���Ϊ��ֹ������Ծ�ֹ
	float   MinSpeedDynamicThreshold = 0.1;//���ٶ�С�ڴ��ٶȣ���Ϊ��ֹ������Ծ�ֹ
	float   safedistance = 0;//��ȫ����
	int     CollisionIndex = 0;//��ײ���Index����ײ���ڳ����滮�켣���е�index��
	int     LeaveIndex = 0;//��ײ���Index����ײ���ڳ����滮�켣���е�index��
	float   Vehicletime;//����������ײ����ʱ��
	float   Vehicletime2;//�뿪��ײ����ʱ��
	float ObsticleDistance = 1000;//�ϰ��������ײ���ľ���
	float ObsticleDistance2 = 0;//�ϰ����뿪��ײ���ľ���
	int     ObstaclesStaticFlag = 0;//�ϰ��ﾲֹ
	int     VehicleStaticFlag = 0;//������ֹ
	vector<Point2D> temppath = path;//�����滮�켣
	vector<Point2D> DangerZoneLeft, DangerZoneRight;
	SpeedPoint ObsAbsolute = obs;//�ϰ�������ٶ���Ϣ
	float x0, y0, dhead;
	Point2D VehilceDanger;
	size_t DangerIndex;

	/*********************/
	//���յ��ϰ����ٶ���ϢΪ����ٶ�
	//�����ٶ�Ϊ�����ٶ�
	/*********************/
	NormalOffSet(temppath, DangerZoneLeft, -Vehiclewide);
	NormalOffSet(temppath, DangerZoneRight, Vehiclewide);

	if (obs.vy>0)//����С���ϰ����ٶȲ�����ײ
	{
		std::cout << "����С���ϰ����ٶȲ�����ײ" << endl;
		return -1;
	}
	ObsAbsolute.vy += vState.speed;//�ϰ����ٶȱ任Ϊ�����ٶ�

	ObsHeading = PI / 2 - atan2f(ObsAbsolute.vy, obs.vx);
	if (ObsHeading>PI)
	{
		ObsHeading -= 2 * PI;
	}

	std::cout << "ObsHeading=" << ObsHeading * 180 / PI << endl;

	//������ת��Ϊ���ϰ���ԭ��
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
		//������ֹ
		VehicleStaticFlag = 1;
		if (fabs(temppath[0].y)>Vehiclewide)
		{
			//������ǰλ�ò�λ����ײ����
			std::cout << "������ֹ��������ײ����" << endl;
			return -1;
		}
		else
		{
			//������ǰλ��λ����ײ����
			//�ϰ������ų����˶�
			if (fabs(ObsAbsolute.vy)>MinSpeedStaticThreshold)
			{
				std::cout << "������ֹ������ײ����Σ�գ�����" << endl;
				return 0;
			}
			else
			{
				std::cout << "������ֹ��������ײ����" << endl;
				return -1;
			}
		}
	}

	//����������ײ���������
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
				std::cout << "����ͬ�������켣���ཻ��������ײ" << endl;
				return -1;
			}
			else
			{
				for (int i = 0; i < temppath.size() - 1; i++)
				{
					safedistance += Computedistance(temppath[i].x, temppath[i].y, temppath[i + 1].x, temppath[i + 1].y);
				}
				Vehicletime2 = (safedistance + VehicleBehindDistance) / vState.speed;//�����뿪��ײ����ʱ��

				ObsticleDistance2 = temppath.back().x;
				Obsspeed = sqrtf(ObsAbsolute.vx*ObsAbsolute.vx + ObsAbsolute.vy*ObsAbsolute.vy);//�����ϰ�������ٶ�
				Obstime2 = ObsticleDistance2 / Obsspeed;//�ϰ����뿪��ײ����ʱ��

				if (Obstime2 < Vehicletime2)//�ϰ����߹���ײ���򣬳�����δ�߹�������δ׷�ϣ����ᷢ����ײ
				{
					std::cout << "�ϰ����߹���ײ���򣬳�����δ׷�ϣ����ᷢ����ײ" << endl;
					return -1;
				}
				else                        //�������ϰ��ﶼ����ײ����һ���ᷢ����ײ
				{
					std::cout << "��ͬ������������׷���ϰ���" << endl;
					std::cout << "һ���ᷢ����ײ" << endl;
					safedistance = Computedistance(0, 0, obs.x, obs.y);
					std::cout << "safedistance=" << safedistance << endl;
					Vehicletime = (safedistance - VehicleFrontDistance) / (-obs.vy);//����������ײ����ʱ��
					std::cout << "time=" << Vehicletime << endl;
					return Vehicletime;
				}
			}
		}
		else if ((DangerIndexFirst&&VehicleDangerFirst.x<0) && (DangerIndexSecond&&VehicleDangerSecond.x<0))
		{
			std::cout << "�켣���ཻ��������ײ" << endl;
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
				Vehicletime = (safedistance - VehicleFrontDistance) / vState.speed;//����������ײ����ʱ��
				std::cout << "safedistance=" << safedistance << endl;
				ObsticleDistance = VehicleDangerFirst.x;
				if (ObsticleDistance<0)
				{
					std::cout << "error������" << endl;
					return -1;
				}
				Obsspeed = sqrtf(ObsAbsolute.vx*ObsAbsolute.vx + ObsAbsolute.vy*ObsAbsolute.vy);//�����ϰ�������ٶ�
				Obstime = ObsticleDistance / Obsspeed;//�ϰ��������ײ����ʱ��
				std::cout << "Obstime=" << Obstime << endl;
				std::cout << "Vehicletime=" << Vehicletime << endl;

				if (Obstime>Vehicletime)
				{
					std::cout << "�����߹���ײ�����ϰ��ﻹδ���룬���ᷢ����ײ" << endl;
					return -1;
				}
				else
				{
					for (int i = DangerIndexFirst; i < temppath.size() - 1; i++)
					{
						safedistance += Computedistance(temppath[i].x, temppath[i].y, temppath[i + 1].x, temppath[i + 1].y);
					}
					Vehicletime2 = (safedistance + VehicleBehindDistance) / vState.speed;//�����뿪��ײ����ʱ��

					ObsticleDistance2 = temppath.back().x;
					Obsspeed = sqrtf(ObsAbsolute.vx*ObsAbsolute.vx + ObsAbsolute.vy*ObsAbsolute.vy);//�����ϰ�������ٶ�
					Obstime2 = ObsticleDistance2 / Obsspeed;//�ϰ����뿪��ײ����ʱ��

					if (Obstime2 < Vehicletime2)//�ϰ����߹���ײ���򣬳�����δ�߹�������δ׷�ϣ����ᷢ����ײ
					{
						std::cout << "�ϰ����߹���ײ���򣬳�����δ���룬���ᷢ����ײ" << endl;
						return -1;
					}
					else                        //�������ϰ��ﶼ����ײ����һ���ᷢ����ײ
					{
						safedistance = Computedistance(0, 0, obs.x, obs.y);
						std::cout << "safedistance=" << safedistance << endl;
						Vehicletime = (safedistance - VehicleFrontDistance) / (-obs.vy);//����������ײ����ʱ��
						std::cout << "֪�������ص㣬�ϰ����ڱ���������ײ" << endl;
						std::cout << "һ���ᷢ����ײ" << endl;
						std::cout << "time=" << Vehicletime << endl;
						return Vehicletime;
					}
				}
			}
			else //�ϰ���������ײ��
			{
				for (int i = 0; i < temppath.size() - 1; i++)
				{
					safedistance += Computedistance(temppath[i].x, temppath[i].y, temppath[i + 1].x, temppath[i + 1].y);
				}
				Vehicletime2 = (safedistance + VehicleBehindDistance) / vState.speed;//�����뿪��ײ����ʱ��

				ObsticleDistance2 = temppath.back().x;
				Obsspeed = sqrtf(ObsAbsolute.vx*ObsAbsolute.vx + ObsAbsolute.vy*ObsAbsolute.vy);//�����ϰ�������ٶ�
				Obstime2 = ObsticleDistance2 / Obsspeed;//�ϰ����뿪��ײ����ʱ��
				std::cout << "Obstime=" << Obstime2 << endl;
				std::cout << "Vehicletime=" << Vehicletime2 << endl;

				if (Obstime2 < Vehicletime2)//�ϰ����߹���ײ���򣬳�����δ�߹�������δ׷�ϣ����ᷢ����ײ
				{
					std::cout << "�ϰ����߹���ײ���򣬳�����δ׷�ϣ����ᷢ����ײ" << endl;
					return -1;
				}
				else                        //�������ϰ��ﶼ����ײ����һ���ᷢ����ײ
				{
					std::cout << "��ͬ������������׷���ϰ���" << endl;
					std::cout << "һ���ᷢ����ײ" << endl;
					safedistance = Computedistance(0, 0, obs.x, obs.y);
					std::cout << "safedistance=" << safedistance << endl;
					Vehicletime = (safedistance - VehicleFrontDistance) / (-obs.vy);//����������ײ����ʱ��
					std::cout << "time=" << Vehicletime << endl;
					return Vehicletime;
				}
			}

		}
		else if ((DangerIndexFirst == 0 || (DangerIndexFirst&&VehicleDangerFirst.x<0)) && DangerIndexSecond)
		{
			if (VehicleDangerSecond.x<0)
			{
				std::cout << "�ϰ���Զ�복��" << endl;
				return -1;

			}
			std::cout << "��������Σ�����򣡣���" << endl;
			for (int i = 0; i < DangerIndexSecond; i++)
			{
				safedistance += Computedistance(temppath[i].x, temppath[i].y, temppath[i + 1].x, temppath[i + 1].y);
			}
			Vehicletime = (safedistance - VehicleFrontDistance) / vState.speed;//����������ײ����ʱ��
			std::cout << "safedistance=" << safedistance << endl;
			ObsticleDistance = VehicleDangerSecond.x;

			Obsspeed = sqrtf(ObsAbsolute.vx*ObsAbsolute.vx + ObsAbsolute.vy*ObsAbsolute.vy);//�����ϰ�������ٶ�
			Obstime = ObsticleDistance / Obsspeed;//�ϰ��������ײ����ʱ��
			std::cout << "Obstime=" << Obstime << endl;
			std::cout << "Vehicletime=" << Vehicletime << endl;
			if (Obstime<Vehicletime)
			{
				std::cout << "�ϰ����ȳ���ײ���򣬳�����δ׷�ϣ����ᷢ����ײ" << endl;
				return -1;
			}
			else
			{
				std::cout << "�ϰ�������ײ��" << endl;
				safedistance = Computedistance(0, 0, obs.x, obs.y);
				std::cout << "safedistance=" << safedistance << endl;
				Vehicletime = (safedistance - VehicleFrontDistance) / (-obs.vy);//����������ײ����ʱ��
				std::cout << "һ���ᷢ����ײ" << endl;
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
			Vehicletime = (safedistance - VehicleFrontDistance) / vState.speed;//����������ײ����ʱ��
			std::cout << "safedistance=" << safedistance << endl;

			for (int i = DangerIndexFirst; i < DangerIndexSecond; i++)
			{
				safedistance += Computedistance(temppath[i].x, temppath[i].y, temppath[i + 1].x, temppath[i + 1].y);
			}
			Vehicletime2 = (safedistance + VehicleBehindDistance) / vState.speed;//�����뿪��ײ����ʱ��

			ObsticleDistance = VehicleDangerFirst.x;
			ObsticleDistance2 = VehicleDangerSecond.x;
			Obsspeed = sqrtf(ObsAbsolute.vx*ObsAbsolute.vx + ObsAbsolute.vy*ObsAbsolute.vy);//�����ϰ�������ٶ�
			Obstime = ObsticleDistance / Obsspeed;//�ϰ��������ײ����ʱ��
			Obstime2 = ObsticleDistance2 / Obsspeed;//�ϰ����뿪��ײ����ʱ��
			std::cout << "Obstime=" << Obstime << endl;
			std::cout << "Vehicletime=" << Vehicletime << endl;

			//ǰ��:���ٴ����ϰ����ٶȣ��ϰ���ģ��Ϊ�ʵ�ģ��
			//������ײ������
			//1�����Ƚ�����ײ��
			//2�ϰ����Ƚ���ײ�����ϰ����ȳ���ײ��
			//��ײ����
			//�ϰ����Ƚ���ײ���������ײ��
			if (Vehicletime < Obstime)//�����߹���ײ�����ϰ��ﻹδ���룬���ᷢ����ײ
			{
				std::cout << "�����Ƚ�����ײ�����ϰ��ﻹδ���룬���ᷢ����ײ" << endl;
				return -1;
			}
			else if (Vehicletime>Obstime&&Vehicletime2>Obstime2)
			{
				std::cout << "�ϰ����Ƚ�����ײ�����ϰ����ȳ���ײ���򣬲��ᷢ����ײ" << endl;
				return -1;
			}
			else  if (Obstime<Vehicletime&&Vehicletime2<Obstime2)
			{
				std::cout << "�ϰ����Ƚ������һ���ᷢ����ײ" << endl;
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
				std::cout << "����ͬ�������켣���ཻ��������ײ" << endl;
				return -1;
			}
			else
			{
				std::cout << "��ͬ������������ʻ" << endl;
				std::cout << "һ���ᷢ����ײ" << endl;
				safedistance = Computedistance(0, 0, obs.x, obs.y);
				std::cout << "safedistance=" << safedistance << endl;
				Vehicletime = (safedistance - VehicleFrontDistance) / (-obs.vy);//����������ײ����ʱ��
				std::cout << "time=" << Vehicletime << endl;
				return Vehicletime;
			}

		}
		else if ((DangerIndexFirst&&VehicleDangerFirst.x<0) && (DangerIndexSecond&&VehicleDangerSecond.x<0))
		{
			std::cout << "�켣���ཻ��������ײ" << endl;
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
				Vehicletime = (safedistance - VehicleFrontDistance) / vState.speed;//����������ײ����ʱ��
				std::cout << "safedistance=" << safedistance << endl;
				ObsticleDistance = VehicleDangerFirst.x;
				if (ObsticleDistance<0)
				{
					std::cout << "error������" << endl;
					return -1;
				}
				Obsspeed = sqrtf(ObsAbsolute.vx*ObsAbsolute.vx + ObsAbsolute.vy*ObsAbsolute.vy);//�����ϰ�������ٶ�
				Obstime = ObsticleDistance / Obsspeed;//�ϰ��������ײ����ʱ��
				std::cout << "Obstime=" << Obstime << endl;
				std::cout << "Vehicletime=" << Vehicletime << endl;

				if (Obstime>Vehicletime)
				{
					std::cout << "�����߹���ײ�����ϰ��ﻹδ���룬���ᷢ����ײ" << endl;
					return -1;
				}
				else
				{
					//�������ϰ��ﶼ����ײ����һ���ᷢ����ײ
					safedistance = Computedistance(0, 0, obs.x, obs.y);
					std::cout << "safedistance=" << safedistance << endl;
					Vehicletime = (safedistance - VehicleFrontDistance) / (-obs.vy);//����������ײ����ʱ��
					std::cout << "֪�������ص㣬�ϰ����ڱ���������ײ" << endl;
					std::cout << "һ���ᷢ����ײ" << endl;
					std::cout << "time=" << Vehicletime << endl;
					return Vehicletime;
				}
			}
			else
			{
				//�������ϰ��ﶼ����ײ����һ���ᷢ����ײ
				safedistance = Computedistance(0, 0, obs.x, obs.y);
				std::cout << "safedistance=" << safedistance << endl;
				Vehicletime = (safedistance - VehicleFrontDistance) / (-obs.vy);//����������ײ����ʱ��
				std::cout << "֪�������ص㣬�ϰ����ڱ���������ײ" << endl;
				std::cout << "һ���ᷢ����ײ" << endl;
				std::cout << "time=" << Vehicletime << endl;
				return Vehicletime;
			}


		}
		else if ((DangerIndexFirst == 0 || (DangerIndexFirst&&VehicleDangerFirst.x<0)) && DangerIndexSecond)
		{
			if (VehicleDangerSecond.x<0)
			{
				std::cout << "�ϰ���Զ�복��" << endl;
				return -1;

			}

			std::cout << "��������Σ�����򣡣���" << endl;
			for (int i = 0; i < DangerIndexSecond; i++)
			{
				safedistance += Computedistance(temppath[i].x, temppath[i].y, temppath[i + 1].x, temppath[i + 1].y);
			}
			Vehicletime = (safedistance - VehicleFrontDistance) / vState.speed;//����������ײ����ʱ��
			std::cout << "safedistance=" << safedistance << endl;
			ObsticleDistance = VehicleDangerSecond.x;

			Obsspeed = sqrtf(ObsAbsolute.vx*ObsAbsolute.vx + ObsAbsolute.vy*ObsAbsolute.vy);//�����ϰ�������ٶ�
			Obstime = ObsticleDistance / Obsspeed;//�ϰ��������ײ����ʱ��
			std::cout << "Obstime=" << Obstime << endl;
			std::cout << "Vehicletime=" << Vehicletime << endl;
			if (Obstime<Vehicletime)
			{
				std::cout << "�ϰ����ȳ���ײ���򣬳�����δ׷�ϣ����ᷢ����ײ" << endl;
				return -1;
			}
			else
			{
				std::cout << "�ϰ�������ײ��" << endl;
				safedistance = Computedistance(0, 0, obs.x, obs.y);
				std::cout << "safedistance=" << safedistance << endl;
				Vehicletime = (safedistance - VehicleFrontDistance) / (-obs.vy);//����������ײ����ʱ��
				std::cout << "һ���ᷢ����ײ" << endl;
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
			Vehicletime = (safedistance - VehicleFrontDistance) / vState.speed;//����������ײ����ʱ��
			std::cout << "safedistance=" << safedistance << endl;

			for (int i = 0; i < DangerIndexSecond; i++)
			{
				safedistance += Computedistance(temppath[i].x, temppath[i].y, temppath[i + 1].x, temppath[i + 1].y);
			}
			Vehicletime2 = (safedistance + VehicleBehindDistance) / vState.speed;//�����뿪��ײ����ʱ��

			ObsticleDistance = VehicleDangerFirst.x;
			ObsticleDistance2 = VehicleDangerSecond.x;
			Obsspeed = sqrtf(ObsAbsolute.vx*ObsAbsolute.vx + ObsAbsolute.vy*ObsAbsolute.vy);//�����ϰ�������ٶ�
			Obstime = ObsticleDistance / Obsspeed;//�ϰ��������ײ����ʱ��
			Obstime2 = ObsticleDistance2 / Obsspeed;//�ϰ����뿪��ײ����ʱ��
			std::cout << "Obstime=" << Obstime << endl;
			std::cout << "Vehicletime=" << Vehicletime << endl;

			//ǰ��:������ʻ���ϰ���ģ��Ϊ�ʵ�ģ��
			//������ײ������
			//1����δ���룬�ϰ����ѳ�
			//2�ϰ���δ���룬�����ѳ�
			//��ײ����
			//else
			if (Vehicletime < Obstime)//�����߹���ײ�����ϰ��ﻹδ���룬���ᷢ����ײ
			{
				std::cout << "�����Ƚ�����ײ�����ϰ��ﻹδ���룬���ᷢ����ײ" << endl;
				return -1;
			}
			else if (Vehicletime2>Obstime2)
			{
				std::cout << "�ϰ����ȳ���ײ���򣬳�����δ����,���ᷢ����ײ" << endl;
				return -1;
			}
			else
			{
				std::cout << "�ϰ��ﳵ��������ײ��         ��һ���ᷢ����ײ" << endl;
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

	//�����ϰ���켣
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