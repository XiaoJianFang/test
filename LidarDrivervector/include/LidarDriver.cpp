//======================================================================
/*! \file IbeoSdkLuxLiveDemo.cpp
*
* \copydoc Copyright
* \author Mario Brumm (mb)
* \date Jun 1, 2012
*
* Demo project for connecting to a LUX and process the received
* data blocks.
*///-------------------------------------------------------------------
#define LENGTHOFIBOBJ		50
#define LENGTHOFCONTOUR		800

#include <pthread.h>
#include <unistd.h>
#include <sys/time.h>
#include <iostream>
#include <cstdlib>
#include <lcm/lcm-cpp.hpp>
#include "PacketIbeo.hpp"
#include "PacketVELD.hpp"
//======================================================================
#include "velodyneHDL32E.h"
#include "LiDAR.h"
#include <ibeosdk/lux.hpp>
#include <ibeosdk/IpHelper.hpp>
#include <ibeosdk/datablocks/commands/CommandLuxReset.hpp>
#include <ibeosdk/datablocks/commands/CommandLuxGetStatus.hpp>
#include <ibeosdk/datablocks/commands/CommandLuxGetParameter.hpp>
#include <ibeosdk/datablocks/commands/CommandLuxSetParameter.hpp>
#include <ibeosdk/datablocks/commands/EmptyCommandReply.hpp>
#include <ibeosdk/datablocks/commands/ReplyLuxGetStatus.hpp>
#include <ibeosdk/datablocks/commands/ReplyLuxGetParameter.hpp>
#include <ibeosdk/datablocks/commands/CommandLuxSetNtpTimestampSync.hpp>
//======================================================================

//#define Xoff_set 0.f	 //向左为正
#define Yoff_set 2.85f //向后为正
#define PI 3.14159265358979323846

float LObject_x[LENGTHOFIBOBJ];//Obj_x
float LObject_y[LENGTHOFIBOBJ];//Obj_y
float LObject_Boxx[LENGTHOFIBOBJ];//Box_x
float LObject_Boxy[LENGTHOFIBOBJ];//ŽBox_y
float LObject_vx[LENGTHOFIBOBJ];
float LObject_vy[LENGTHOFIBOBJ];
float LObject_vabx[LENGTHOFIBOBJ];
float LObject_vaby[LENGTHOFIBOBJ];
int LObject_contournum[LENGTHOFIBOBJ];
float LObject_contourx[LENGTHOFCONTOUR];
float LObject_contoury[LENGTHOFCONTOUR];

float RObject_x[LENGTHOFIBOBJ];//Obj_x
float RObject_y[LENGTHOFIBOBJ];//Obj_y
float RObject_Boxx[LENGTHOFIBOBJ];//Box_x
float RObject_Boxy[LENGTHOFIBOBJ];//ŽBox_y
float RObject_vx[LENGTHOFIBOBJ];
float RObject_vy[LENGTHOFIBOBJ];
float RObject_vabx[LENGTHOFIBOBJ];
float RObject_vaby[LENGTHOFIBOBJ];
int RObject_contournum[LENGTHOFIBOBJ];
float RObject_contourx[LENGTHOFCONTOUR];
float RObject_contoury[LENGTHOFCONTOUR];

float FObject_x[LENGTHOFIBOBJ];//Obj_x
float FObject_y[LENGTHOFIBOBJ];//Obj_y
float FObject_Boxx[LENGTHOFIBOBJ];//Box_x
float FObject_Boxy[LENGTHOFIBOBJ];//ŽBox_y
float FObject_vx[LENGTHOFIBOBJ];
float FObject_vy[LENGTHOFIBOBJ];
float FObject_vabx[LENGTHOFIBOBJ];
float FObject_vaby[LENGTHOFIBOBJ];
int FObject_contournum[LENGTHOFIBOBJ];
float FObject_contourx[LENGTHOFCONTOUR];
float FObject_contoury[LENGTHOFCONTOUR];


int Objectnum = 0;
float zerosf[10000] = { 0.0f };
int zeros[10000] = { 0 };


bool  NewLeftibeoflag= false;
bool  NewRightibeoflag= false;
bool  NewFrontibeoflag= false;
using namespace ibeosdk;
using namespace std;

const ibeosdk::Version::MajorVersion majorVersion(4);
const ibeosdk::Version::MinorVersion minorVersion(3);
const ibeosdk::Version::Revision revision(1);
const ibeosdk::Version::PatchLevel patchLevel;
const ibeosdk::Version::Build build;
const std::string info = "Demo";
ibeosdk::Version appVersion(majorVersion, minorVersion, revision, patchLevel, build, info);
IbeoSDK ibeoSDK;
ibeosdk::TimeConversion tc;
pthread_mutex_t IbeoMutex ;
//======================================================================
void  CalculateSensorData(float vecx[], float vecy[], float x0, float y0, float dhead);

class LuxListenerONE :
	public ibeosdk::DataListener<ibeosdk::ObjectListLux>
{
public:
	//========================================
	void onData(const ibeosdk::ObjectListLux* const pObj)
	{
		std::vector< ibeosdk::ObjectLux > ::const_iterator idx = pObj->getObjects().begin();

        pthread_mutex_lock(&IbeoMutex);
		memcpy(LObject_x, zerosf, sizeof(LObject_x));
		memcpy(LObject_y, zerosf, sizeof(LObject_y));
		memcpy(LObject_Boxx, zerosf, sizeof(LObject_Boxx));
		memcpy(LObject_Boxy, zerosf, sizeof(LObject_Boxy));
		memcpy(LObject_vx, zerosf, sizeof(LObject_vx));
		memcpy(LObject_vy, zerosf, sizeof(LObject_vy));
		memcpy(LObject_vabx, zerosf, sizeof(LObject_vabx));
		memcpy(LObject_vaby, zerosf, sizeof(LObject_vaby));
		memcpy(LObject_contournum, zeros, sizeof(LObject_contournum));
		memcpy(LObject_contourx, zerosf, sizeof(LObject_contourx));
		memcpy(LObject_contoury, zerosf, sizeof(LObject_contoury));
		NewLeftibeoflag= false;
		pthread_mutex_unlock(&IbeoMutex);

        Objectnum = 0;
		int contournum = 0;
		for (; idx != pObj->getObjects().end(); ++idx)
		{
			std::vector<ibeosdk::Point2d>::const_iterator idcontour = idx->getContourPoints().begin();

			float Obj_x = 0.f, Obj_y = 0.f, Obj_vx = 0.f, Obj_vy = 0.f, Obj_vabx = 0.f, Obj_vaby = 0.f,  Box_x = 0.f, Box_y = 0.f;	//
			int Obj_contournum= 0;
			Obj_x = -(float)idx->getObjectBoxCenter().getY();
			Obj_y = (float)idx->getObjectBoxCenter().getX();
			Box_y = (float)idx->getObjectBoxSizeX();
			Box_x = (float)idx->getObjectBoxSizeY();
			Obj_vx = (float)idx->getRelativeVelocity().getY();
			Obj_vy = (float)idx->getRelativeVelocity().getX();
			Obj_vabx = (float)idx->getAbsoluteVelocity().getY();
			Obj_vaby = (float)idx->getAbsoluteVelocity().getX();
			Obj_contournum = (int)idx->getNumberOfContourPoints();

			if (Objectnum < LENGTHOFIBOBJ)
			{
                pthread_mutex_lock(&IbeoMutex);
				LObject_x[Objectnum] = Obj_x / 100;
				LObject_y[Objectnum] = Obj_y / 100 + Yoff_set;
				LObject_Boxx[Objectnum] = Box_x / 100;
               // Object_Boxy[Objectnum] = Box_y / 100;
                LObject_Boxy[Objectnum] = Box_y / 100+ Yoff_set;
				LObject_vx[Objectnum] = Obj_vx / 100;
				LObject_vy[Objectnum] = Obj_vy / 100;
				LObject_vabx[Objectnum] = Obj_vabx / 100;
				LObject_vaby[Objectnum] = Obj_vaby / 100;
				LObject_contournum[Objectnum] = Obj_contournum;
				pthread_mutex_unlock(&IbeoMutex);
				Objectnum++;
			}

			for (; idcontour != idx->getContourPoints().end(); ++idcontour)
			{
				float Obj_contourx, Obj_contoury;
				Obj_contourx = -(float)idcontour->getY();
				Obj_contoury = (float)idcontour->getX();
				if (contournum < LENGTHOFCONTOUR)
				{
					LObject_contourx[contournum] = Obj_contourx / 100 ;
					LObject_contoury[contournum] = Obj_contoury / 100 + Yoff_set;
					contournum++;
				}
			}
		}
		pthread_mutex_lock(&IbeoMutex);
		NewLeftibeoflag=true;
		pthread_mutex_unlock(&IbeoMutex);
	}
        
}; // LuxListener1

class LuxListenerTWO :
		public ibeosdk::DataListener<ibeosdk::ObjectListLux>
{
public:
	//========================================
	void onData(const ibeosdk::ObjectListLux* const pObj)
	{
		std::vector< ibeosdk::ObjectLux > ::const_iterator idx = pObj->getObjects().begin();

		pthread_mutex_lock(&IbeoMutex);
		memcpy(RObject_x, zerosf, sizeof(RObject_x));
		memcpy(RObject_y, zerosf, sizeof(RObject_y));
		memcpy(RObject_Boxx, zerosf, sizeof(RObject_Boxx));
		memcpy(RObject_Boxy, zerosf, sizeof(RObject_Boxy));
		memcpy(RObject_vx, zerosf, sizeof(RObject_vx));
		memcpy(RObject_vy, zerosf, sizeof(RObject_vy));
		memcpy(RObject_vabx, zerosf, sizeof(RObject_vabx));
		memcpy(RObject_vaby, zerosf, sizeof(RObject_vaby));
		memcpy(RObject_contournum, zeros, sizeof(RObject_contournum));
		memcpy(RObject_contourx, zerosf, sizeof(RObject_contourx));
		memcpy(RObject_contoury, zerosf, sizeof(RObject_contoury));
		NewRightibeoflag=false;
		pthread_mutex_unlock(&IbeoMutex);

		Objectnum = 0;
		int contournum = 0;
		for (; idx != pObj->getObjects().end(); ++idx)
		{
			std::vector<ibeosdk::Point2d>::const_iterator idcontour = idx->getContourPoints().begin();

			float Obj_x = 0.f, Obj_y = 0.f, Obj_vx = 0.f, Obj_vy = 0.f, Obj_vabx = 0.f, Obj_vaby = 0.f,  Box_x = 0.f, Box_y = 0.f;	//
			int Obj_contournum= 0;
			Obj_x = -(float)idx->getObjectBoxCenter().getY();
			Obj_y = (float)idx->getObjectBoxCenter().getX();
			Box_y = (float)idx->getObjectBoxSizeX();
			Box_x = (float)idx->getObjectBoxSizeY();
			Obj_vx = (float)idx->getRelativeVelocity().getY();
			Obj_vy = (float)idx->getRelativeVelocity().getX();
			Obj_vabx = (float)idx->getAbsoluteVelocity().getY();
			Obj_vaby = (float)idx->getAbsoluteVelocity().getX();
			Obj_contournum = (int)idx->getNumberOfContourPoints();

			if (Objectnum < LENGTHOFIBOBJ)
			{
				pthread_mutex_lock(&IbeoMutex);
				RObject_x[Objectnum] = Obj_x / 100;
				RObject_y[Objectnum] = Obj_y / 100 + Yoff_set;
				RObject_Boxx[Objectnum] = Box_x / 100;
				// Object_Boxy[Objectnum] = Box_y / 100;
				RObject_Boxy[Objectnum] = Box_y / 100+ Yoff_set;
				RObject_vx[Objectnum] = Obj_vx / 100;
				RObject_vy[Objectnum] = Obj_vy / 100;
				RObject_vabx[Objectnum] = Obj_vabx / 100;
				RObject_vaby[Objectnum] = Obj_vaby / 100;
				RObject_contournum[Objectnum] = Obj_contournum;
				pthread_mutex_unlock(&IbeoMutex);
				Objectnum++;
			}

			for (; idcontour != idx->getContourPoints().end(); ++idcontour)
			{
				float Obj_contourx, Obj_contoury;
				Obj_contourx = -(float)idcontour->getY();
				Obj_contoury = (float)idcontour->getX();
				if (contournum < LENGTHOFCONTOUR)
				{
					RObject_contourx[contournum] = Obj_contourx / 100 ;
					RObject_contoury[contournum] = Obj_contoury / 100 + Yoff_set;
					contournum++;
				}
			}
		}
		pthread_mutex_lock(&IbeoMutex);
		NewRightibeoflag=true;
        pthread_mutex_unlock(&IbeoMutex);
	}

}; // LuxListener2


int checkArguments(const int argc, const char** argv, bool& hasLogFile)
{
	const int minNbOfNeededArguments = 2;
	const int maxNbOfNeededArguments = 3;

	bool wrongNbOfArguments = false;
	if (argc < minNbOfNeededArguments) {
		std::cerr << "Missing argument" << std::endl;
		wrongNbOfArguments = true;
	}
	else if (argc > maxNbOfNeededArguments) {
		std::cerr << "Too many argument" << std::endl;
		wrongNbOfArguments = true;
	}

	if (wrongNbOfArguments) {
		std::cerr << argv[0] << " " << " IP [LOGFILE]" << std::endl;
		std::cerr << "\tIP is the ip address of the LUX sensor, e.g. 192.168.0.1." << std::endl;
		std::cerr << "\tLOGFILE name of the log file. If ommitted, the log output will be performed to stderr." << std::endl;
		return 1;
	}

	hasLogFile = (argc == maxNbOfNeededArguments);
	return 0;
}

//======================================================================

void *LuxListenerLeft(void* args)
{
	std::cerr << "  using IbeoSDK " << ibeoSDK.getVersion().toString() << std::endl;

	std::string ip("192.168.1.120");
	bool hasLogFile = 1;
	const off_t maxLogFileSize = 1000000;
	ibeosdk::LogFileManager logFileManager;
	ibeosdk::LogFile::setTargetFileSize(maxLogFileSize);
	const ibeosdk::LogLevel ll = ibeosdk::logLevelFromString("Debug");
	ibeosdk::LogFile::setLogLevel(ll);
	logFileManager.start();

	LuxListenerONE LuxListenerone;
	const uint16_t port = ibeosdk::getPort(ip, 12002);
	ibeosdk::IbeoLux lux(ip, port);
	lux.setLogFileManager(&logFileManager);
	lux.registerListener(&LuxListenerone);
	lux.getConnected();
	if (lux.isConnected())
		cout<<"IBEO Left getconnected!!!"<<endl;
    else
	{
		cout<<"IBEO Left connect fail!!!"<<endl;
		return NULL;
	}

	while (true)
	{
		usleep(1000);
		if (!lux.isConnected())
		{
			cout << "IBEO Left NO DATA!!!";
            return NULL;
        }
	}
}

void *LuxListenerRight(void* args)
{
	std::cerr << "  using IbeoSDK " << ibeoSDK.getVersion().toString() << std::endl;

	std::string ip("192.168.1.121");
	bool hasLogFile = 1;
	const off_t maxLogFileSize = 1000000;
	ibeosdk::LogFileManager logFileManager;
	ibeosdk::LogFile::setTargetFileSize(maxLogFileSize);
	const ibeosdk::LogLevel ll = ibeosdk::logLevelFromString("Debug");
	ibeosdk::LogFile::setLogLevel(ll);
	logFileManager.start();

	LuxListenerTWO LuxListenertwo;
	const uint16_t port = ibeosdk::getPort(ip, 12002);
	ibeosdk::IbeoLux lux(ip, port);
	lux.setLogFileManager(&logFileManager);
	lux.registerListener(&LuxListenertwo);
	lux.getConnected();

	if (lux.isConnected())
		cout<<"IBEO Right getconnected!!!"<<endl;
   else
	{
		cout<<"IBEO Right connect fail!!!"<<endl;
		return NULL;
	}
	while (true)
	{
		usleep(1000);
		if (!lux.isConnected())
		{
			cout << "IBEO Right  DATA!!!";
             return NULL;
		}
	}
}
void *VeloListenerFront(void* args)
{
	vector<pointType> velodata;

	velodyneHDL32E velodyneDevice(sensorDevice::modeType::online, sensorDevice::saveType::unsave);
    int count=0;
    while(true)
    {
        usleep(10000);
        velodyneDevice.getData(velodata,0);
        //std::cout<<"number"<<velodata.size()<< std::endl;

        pthread_mutex_lock(&IbeoMutex);
        memcpy(FObject_x, zerosf, sizeof(LObject_x));
        memcpy(FObject_y, zerosf, sizeof(LObject_y));
        memcpy(FObject_Boxx, zerosf, sizeof(LObject_Boxx));
        memcpy(FObject_Boxy, zerosf, sizeof(LObject_Boxy));
        memcpy(FObject_vx, zerosf, sizeof(LObject_vx));
        memcpy(FObject_vy, zerosf, sizeof(LObject_vy));
        memcpy(FObject_vabx, zerosf, sizeof(LObject_vabx));
        memcpy(FObject_vaby, zerosf, sizeof(LObject_vaby));
        memcpy(FObject_contournum, zeros, sizeof(LObject_contournum));
        memcpy(FObject_contourx, zerosf, sizeof(LObject_contourx));
        memcpy(FObject_contoury, zerosf, sizeof(LObject_contoury));
        NewFrontibeoflag= false;
        pthread_mutex_unlock(&IbeoMutex);
        int index=0;
        float step=0;
        if(velodata.size()<LENGTHOFCONTOUR)
        {
            step=1.f;
        } else
        {
            step=velodata.size()/((float)LENGTHOFCONTOUR);
        }
        for(int i=0;i<velodata.size();i++)
        {
             index=(int)(i/step+0.5);
            if(index<LENGTHOFCONTOUR)
            {
                RObject_contourx[index] = velodata[i].x ;
                RObject_contoury[index] = velodata[i].y;
            } else
                break;
        }
        FObject_x[0]=1;
        FObject_y[0]=1;
        FObject_Boxx[0]=1;
        FObject_Boxy[0]=1;
        FObject_vabx[0]=1;
        FObject_vaby[0]=1;
        FObject_vx[0]=1;
        FObject_vy[0]=1;
        if(index<LENGTHOFCONTOUR)
        {
            FObject_contournum[0]=index;
        }
        else
        {
            FObject_contournum[0]=index;
        }
        pthread_mutex_lock(&IbeoMutex);
        NewFrontibeoflag= true;
        pthread_mutex_unlock(&IbeoMutex);
    }
}
int main(const int argc, const char** argv)
{
	 lcm::LCM lcm;
	if(!lcm.good())   return 1;

	pthread_mutex_init(&IbeoMutex,NULL);
	struct timeval lastGPSMsg;
	struct timeval origLastMsg;

	int  ret=1;
	pthread_t LiDARid[3];
	ret=pthread_create(&LiDARid[0],NULL, LuxListenerLeft,NULL);
	if(ret!=0)
	{
		printf ("Create pthread Left error!/n");
		exit (1);
	}
	ret=pthread_create(&LiDARid[1],NULL, LuxListenerRight,NULL);
	if(ret!=0)
	{
		printf ("Create pthread  Right error!/n");
		exit (1);
	}
	ret=pthread_create(&LiDARid[2],NULL, VeloListenerFront,NULL);
	if(ret!=0)
	{
		printf ("Create pthread  Front error!/n");
		exit (1);
	}
    bool safeleftibeoflag=false;
    bool saferightibeoflag=false;
    bool safefrontveloflag=false;
    int count=0;
    gettimeofday(&lastGPSMsg,NULL);
    while (true)
	{
		usleep(1000);

		PacketIbeo PIbeo;

        pthread_mutex_lock(&IbeoMutex);
        safeleftibeoflag=NewLeftibeoflag;
        NewLeftibeoflag=false;
        pthread_mutex_unlock(&IbeoMutex);

        if(safeleftibeoflag)
        {
            safeleftibeoflag = false;
            pthread_mutex_lock(&IbeoMutex);
            memcpy(PIbeo.ibx, LObject_x, sizeof(PIbeo.ibx));
            memcpy(PIbeo.iby, LObject_y, sizeof(PIbeo.iby));
            memcpy(PIbeo.ibw, LObject_Boxx, sizeof(PIbeo.ibw));
            memcpy(PIbeo.ibl, LObject_Boxy, sizeof(PIbeo.ibl));
            memcpy(PIbeo.ibvx, LObject_vx, sizeof(PIbeo.ibvx));
            memcpy(PIbeo.ibvy, LObject_vy, sizeof(PIbeo.ibvy));
            memcpy(PIbeo.ibvabx, LObject_vabx, sizeof(PIbeo.ibvabx));
            memcpy(PIbeo.ibvaby, LObject_vaby, sizeof(PIbeo.ibvaby));
            memcpy(PIbeo.ibcontournum, LObject_contournum, sizeof(PIbeo.ibcontournum));
            memcpy(PIbeo.ibcontourx, LObject_contourx, sizeof(PIbeo.ibcontourx));
            memcpy(PIbeo.ibcontoury, LObject_contoury, sizeof(PIbeo.ibcontoury));
            pthread_mutex_unlock(&IbeoMutex);

            float x0 = 0, y0 = 0, dhead = 1.3f / 180.0 * PI;
            CalculateSensorData(PIbeo.ibcontourx, PIbeo.ibcontoury, x0, y0, dhead);

			gettimeofday(&origLastMsg,NULL);
			unsigned int timeDiff = 1000*(origLastMsg.tv_sec-lastGPSMsg.tv_sec)+(origLastMsg.tv_usec-lastGPSMsg.tv_usec)/1000;
			cout<<"time="<<timeDiff<<endl;
			lcm.publish("Ibeo", &PIbeo);
            gettimeofday(&lastGPSMsg,NULL);
		}

        pthread_mutex_lock(&IbeoMutex);
        saferightibeoflag=NewRightibeoflag;
        NewRightibeoflag=false;
        pthread_mutex_unlock(&IbeoMutex);

        if(saferightibeoflag)
        {
            saferightibeoflag = false;
            pthread_mutex_lock(&IbeoMutex);
            memcpy(PIbeo.ibx, RObject_x, sizeof(PIbeo.ibx));
            memcpy(PIbeo.iby,RObject_y, sizeof(PIbeo.iby));
            memcpy(PIbeo.ibw, RObject_Boxx, sizeof(PIbeo.ibw));
            memcpy(PIbeo.ibl, RObject_Boxy, sizeof(PIbeo.ibl));
            memcpy(PIbeo.ibvx, RObject_vx, sizeof(PIbeo.ibvx));
            memcpy(PIbeo.ibvy, RObject_vy, sizeof(PIbeo.ibvy));
            memcpy(PIbeo.ibvabx, RObject_vabx, sizeof(PIbeo.ibvabx));
            memcpy(PIbeo.ibvaby, RObject_vaby, sizeof(PIbeo.ibvaby));
            memcpy(PIbeo.ibcontournum, RObject_contournum, sizeof(PIbeo.ibcontournum));
            memcpy(PIbeo.ibcontourx, RObject_contourx, sizeof(PIbeo.ibcontourx));
            memcpy(PIbeo.ibcontoury, RObject_contoury, sizeof(PIbeo.ibcontoury));
            pthread_mutex_unlock(&IbeoMutex);

            float x0 = 0, y0 = 0, dhead = 1.3f / 180.0 * PI;
            CalculateSensorData(PIbeo.ibcontourx, PIbeo.ibcontoury, x0, y0, dhead);

            gettimeofday(&origLastMsg,NULL);
            unsigned int timeDiff = 1000*(origLastMsg.tv_sec-lastGPSMsg.tv_sec)+(origLastMsg.tv_usec-lastGPSMsg.tv_usec)/1000;
            cout<<"time="<<timeDiff<<endl;
            lcm.publish("Ibeo", &PIbeo);
            gettimeofday(&lastGPSMsg,NULL);
        }


        pthread_mutex_lock(&IbeoMutex);
        safefrontveloflag=NewFrontibeoflag;
        NewFrontibeoflag=false;
        pthread_mutex_unlock(&IbeoMutex);

        if(safefrontveloflag)
        {
            safefrontveloflag = false;
            pthread_mutex_lock(&IbeoMutex);
            memcpy(PIbeo.ibx, FObject_x, sizeof(PIbeo.ibx));
            memcpy(PIbeo.iby, FObject_y, sizeof(PIbeo.iby));
            memcpy(PIbeo.ibw, FObject_Boxx, sizeof(PIbeo.ibw));
            memcpy(PIbeo.ibl, FObject_Boxy, sizeof(PIbeo.ibl));
            memcpy(PIbeo.ibvx, FObject_vx, sizeof(PIbeo.ibvx));
            memcpy(PIbeo.ibvy, FObject_vy, sizeof(PIbeo.ibvy));
            memcpy(PIbeo.ibvabx, FObject_vabx, sizeof(PIbeo.ibvabx));
            memcpy(PIbeo.ibvaby, FObject_vaby, sizeof(PIbeo.ibvaby));
            memcpy(PIbeo.ibcontournum, FObject_contournum, sizeof(PIbeo.ibcontournum));
            memcpy(PIbeo.ibcontourx, FObject_contourx, sizeof(PIbeo.ibcontourx));
            memcpy(PIbeo.ibcontoury, FObject_contoury, sizeof(PIbeo.ibcontoury));
            pthread_mutex_unlock(&IbeoMutex);


            gettimeofday(&origLastMsg,NULL);
            unsigned int timeDiff = 1000*(origLastMsg.tv_sec-lastGPSMsg.tv_sec)+(origLastMsg.tv_usec-lastGPSMsg.tv_usec)/1000;
            cout<<"time="<<timeDiff<<endl;
            lcm.publish("Ibeo", &PIbeo);
            gettimeofday(&lastGPSMsg,NULL);
        }


	}
    pthread_mutex_destroy(&IbeoMutex);
	return 0;
}


void  inline CalculateSensorData(float vecx[], float vecy[], float x0, float y0, float dhead)
{
	for (int i = 0; i < LENGTHOFCONTOUR; i++)
	{
		float _vecx = vecx[i];
		float _vecy = vecy[i];
		vecx[i] = (_vecx - x0) * cos(dhead) - (_vecy - y0) * sin(dhead);
		vecy[i] = (_vecx - x0) * sin(dhead) + (_vecy - y0) * cos(dhead);
	}
}
