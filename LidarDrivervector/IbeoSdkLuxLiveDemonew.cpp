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
#define LENGTHOFCONTOUR		3000

#include <pthread.h>
#include <unistd.h>
#include <sys/time.h>
#include <iostream>
#include <cstdlib>
#include <lcm/lcm-cpp.hpp>
//#include "PacketIbeo.hpp"
#include "PacketIbeoVector.hpp"
#include "PacketVLP16RawVector.hpp"
#include "PointVelo.hpp"
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

#define PI 3.14159265358979323846

PacketIbeoVector unsafeLibeo;
PacketIbeoVector unsafeRibeo;
PacketIbeoVector unsafeFvelo;
PacketVLP16RawVector unsaferawvelo;

int veloindex[16]={0,8,1,9,2,10,3,11,4,12,5,13,6,14,7,15};
int Objectnum = 0;




static int   w,h,lowh;
static int  rawVlp16CollctFlag=1;
static int   period=80;
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

class LuxListenerONE :
		 public ibeosdk::DataListener<ibeosdk::ObjectListLux>
{
public:
	//========================================
	

	void onData(const ibeosdk::ObjectListLux* const pObj)
	{
		std::vector< ibeosdk::ObjectLux > ::const_iterator idx = pObj->getObjects().begin();

        pthread_mutex_lock(&IbeoMutex);
        unsafeLibeo.ibx.clear();
        unsafeLibeo.iby.clear();
        unsafeLibeo.ibw.clear();
        unsafeLibeo.ibl.clear();
        unsafeLibeo.ibvx.clear();
        unsafeLibeo.ibvy.clear();
        unsafeLibeo.ibvabx.clear();
        unsafeLibeo.ibvaby.clear();
        unsafeLibeo.ibcontournum.clear();
        unsafeLibeo.ibcontourx.clear();
        unsafeLibeo.ibcontoury.clear();
        pthread_mutex_unlock(&IbeoMutex);


        for (; idx != pObj->getObjects().end(); ++idx)
		{
			std::vector<ibeosdk::Point2d>::const_iterator idcontour = idx->getContourPoints().begin();

			float Obj_x = 0.f, Obj_y = 0.f, Obj_vx = 0.f, Obj_vy = 0.f, Obj_vabx = 0.f, Obj_vaby = 0.f,  Box_x = 0.f, Box_y = 0.f;	//
			short Obj_contournum= 0;
			Obj_x = -(float)idx->getObjectBoxCenter().getY()/100;
			Obj_y = (float)idx->getObjectBoxCenter().getX()/100;
			Box_y = (float)idx->getObjectBoxSizeX()/100;
			Box_x = (float)idx->getObjectBoxSizeY()/100;
			Obj_vx = (float)idx->getRelativeVelocity().getY()/100;
			Obj_vy = (float)idx->getRelativeVelocity().getX()/100;
			Obj_vabx = (float)idx->getAbsoluteVelocity().getY()/100;
			Obj_vaby = (float)idx->getAbsoluteVelocity().getX()/100;
			Obj_contournum = (short)idx->getNumberOfContourPoints();

            pthread_mutex_lock(&IbeoMutex);
            unsafeLibeo.ibx.push_back(Obj_x);
            unsafeLibeo.iby.push_back(Obj_y);
            unsafeLibeo.ibw.push_back(Box_x);
            unsafeLibeo.ibl.push_back(Box_y);
            unsafeLibeo.ibvx.push_back(Obj_vx);
            unsafeLibeo.ibvy.push_back(Obj_vy);
            unsafeLibeo.ibvabx.push_back(Obj_vabx);
            unsafeLibeo.ibvaby.push_back(Obj_vaby);
            unsafeLibeo.ibcontournum.push_back(Obj_contournum);

            for (; idcontour != idx->getContourPoints().end(); ++idcontour)
			{
				float Obj_contourx, Obj_contoury;
				Obj_contourx = -(float)idcontour->getY()/100;
				Obj_contoury = (float)idcontour->getX()/100;
                unsafeLibeo.ibcontourx.push_back(Obj_contourx);
                unsafeLibeo.ibcontoury.push_back(Obj_contoury);
			}
            pthread_mutex_unlock(&IbeoMutex);

        }
        pthread_mutex_lock(&IbeoMutex);
		NewLeftibeoflag=true;
        unsafeLibeo.lengthOfContour=(short)unsafeLibeo.ibcontourx.size();
        unsafeLibeo.lengthOfObject=(short)unsafeLibeo.ibvx.size();
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
        unsafeRibeo.ibx.clear();
        unsafeRibeo.iby.clear();
        unsafeRibeo.ibw.clear();
        unsafeRibeo.ibl.clear();
        unsafeRibeo.ibvx.clear();
        unsafeRibeo.ibvy.clear();
        unsafeRibeo.ibvabx.clear();
        unsafeRibeo.ibvaby.clear();
        unsafeRibeo.ibcontournum.clear();
        unsafeRibeo.ibcontourx.clear();
        unsafeRibeo.ibcontoury.clear();
        pthread_mutex_unlock(&IbeoMutex);

        for (; idx != pObj->getObjects().end(); ++idx)
        {
            std::vector<ibeosdk::Point2d>::const_iterator idcontour = idx->getContourPoints().begin();

            float Obj_x = 0.f, Obj_y = 0.f, Obj_vx = 0.f, Obj_vy = 0.f, Obj_vabx = 0.f, Obj_vaby = 0.f,  Box_x = 0.f, Box_y = 0.f;	//
            short Obj_contournum= 0;
            Obj_x = -(float)idx->getObjectBoxCenter().getY()/100;
            Obj_y = (float)idx->getObjectBoxCenter().getX()/100;
            Box_y = (float)idx->getObjectBoxSizeX()/100;
            Box_x = (float)idx->getObjectBoxSizeY()/100;
            Obj_vx = (float)idx->getRelativeVelocity().getY()/100;
            Obj_vy = (float)idx->getRelativeVelocity().getX()/100;
            Obj_vabx = (float)idx->getAbsoluteVelocity().getY()/100;
            Obj_vaby = (float)idx->getAbsoluteVelocity().getX()/100;
            Obj_contournum = (short)idx->getNumberOfContourPoints();

            pthread_mutex_lock(&IbeoMutex);
            unsafeRibeo.ibx.push_back(Obj_x);
            unsafeRibeo.iby.push_back(Obj_y);
            unsafeRibeo.ibw.push_back(Box_x);
            unsafeRibeo.ibl.push_back(Box_y);
            unsafeRibeo.ibvx.push_back(Obj_vx);
            unsafeRibeo.ibvy.push_back(Obj_vy);
            unsafeRibeo.ibvabx.push_back(Obj_vabx);
            unsafeRibeo.ibvaby.push_back(Obj_vaby);
            unsafeRibeo.ibcontournum.push_back(Obj_contournum);

            for (; idcontour != idx->getContourPoints().end(); ++idcontour)
            {
                float Obj_contourx, Obj_contoury;
                Obj_contourx = -(float)idcontour->getY()/100;
                Obj_contoury = (float)idcontour->getX()/100;
                unsafeRibeo.ibcontourx.push_back(Obj_contourx);
                unsafeRibeo.ibcontoury.push_back(Obj_contoury);
            }
            pthread_mutex_unlock(&IbeoMutex);

        }
        pthread_mutex_lock(&IbeoMutex);
        NewRightibeoflag=true;
        unsafeRibeo.lengthOfContour=(short)unsafeRibeo.ibcontourx.size();
        unsafeRibeo.lengthOfObject=(short)unsafeRibeo.ibvx.size();
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

	std::string ip("192.168.9.10");

	//bool hasLogFile = 1;
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

    unsafeLibeo.index=0;

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

	std::string ip("192.168.9.20");
	//bool hasLogFile = 1;   
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
    unsafeRibeo.index=2;

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

template <class Type> inline Type stringToNum(const std::string& str)
{
    std::istringstream iss(str);
    Type num;
    iss >> num;
    return num;
}
void *VeloListenerFront(void* args)
{
	vector<pointType> velodata;
     float height=2.5;
     float wide=7;
     float lowheight=0.2;
    height=h/100.f;
    wide=w/100.f;
    lowheight=lowh/100.f;
   
   velodyneHDL32E velodyneDevice(sensorDevice::modeType::online, sensorDevice::saveType::unsave);
   // int count=0;

    unsafeFvelo.index=1;
    unsafeFvelo.ibx.push_back(0);
    unsafeFvelo.iby.push_back(0);
    unsafeFvelo.ibw.push_back(0);
    unsafeFvelo.ibl.push_back(0);
    unsafeFvelo.ibvx.push_back(0);
    unsafeFvelo.ibvy.push_back(0);
    unsafeFvelo.ibvabx.push_back(0);
    unsafeFvelo.ibvaby.push_back(0);
    unsafeFvelo.lengthOfObject=1;

    vector<PointVelo> rawvelo;
    PointVelo p;

    while(true)
    {
        usleep(1000);
        velodyneDevice.getData(velodata,0);
        //std::cout<<"number"<<velodata.size()<< std::endl;
       if(velodata.size()<10) continue;
        vector<float> midx,midy;
       rawvelo.clear();
        for(int i=0;i<velodata.size();i++)
          {
            p.x=velodata[i].x;
            p.y=velodata[i].y;
            p.z=velodata[i].z;
            p.id=velodata[i].id;
            p.r=velodata[i].r;
            rawvelo.push_back(p);
          //  p=velodata[i];
            if((velodata[i].z<(height-0.85))&&(velodata[i].z>(lowheight-0.85))&&(fabs(velodata[i].x)<wide))
             {
                 midx.push_back(velodata[i].x);
                 midy.push_back(velodata[i].y);
             }
          }


        short num=(short)midx.size();

        pthread_mutex_lock(&IbeoMutex);
        unsafeFvelo.ibcontournum.clear();
        unsafeFvelo.ibcontourx=midx;
        unsafeFvelo.ibcontoury=midy;
        unsafeFvelo.ibcontournum.push_back(num);
        unsafeFvelo.lengthOfContour=num;
        unsaferawvelo.points=rawvelo;
        unsaferawvelo.lengthOfPoints=rawvelo.size();
        NewFrontibeoflag= true;
        pthread_mutex_unlock(&IbeoMutex);
    }
}

int LoadParameter(void)
{
    string dir="./data/lidarConfig.txt";
    ifstream file(dir.c_str());
    string line;

    cout<<"load "<<dir<<endl;

    if(!file)
    {
        cout<<" no this file"<<endl;
        return 0;
    }
    getline(file,line);
    getline(file,line);
    w=stringToNum<int>(line);
    getline(file,line);
    getline(file,line);
    h =stringToNum<int>(line);
    getline(file,line);
    getline(file,line);
    lowh =stringToNum<int>(line);
    getline(file,line);
    getline(file,line);
    period =stringToNum<int>(line);
    getline(file,line);
    getline(file,line);
    rawVlp16CollctFlag =stringToNum<int>(line);

    file.close();

    return 1;
}

int main(const int argc, const char** argv)
{
    
    if(!LoadParameter())
    {
        return 0;
    }
    lcm::LCM lcm;
	if(!lcm.good())   return 1;

	pthread_mutex_init(&IbeoMutex,NULL);
    struct timeval lastLMsg;
    struct timeval lastRMsg;
	struct timeval lastFMsg;
	struct timeval origCurMsg;


    int  ret=1;
	pthread_t LiDARid[3];
	ret=pthread_create(&LiDARid[0],NULL, LuxListenerLeft,NULL);
	if(ret!=0)
	{
		printf ("Create pthread Left error!/n");
		exit (1);
	}
    usleep(1);
	ret=pthread_create(&LiDARid[1],NULL, LuxListenerRight,NULL);
	if(ret!=0)
	{
		printf ("Create pthread  Right error!/n");
		exit (1);
	}
    usleep(1);

	ret=pthread_create(&LiDARid[2],NULL, VeloListenerFront,NULL);
	if(ret!=0)
	{
		printf ("Create pthread  Front error!/n");
		exit (1);
	}
    usleep(1);

    bool safeleftibeoflag=false;
    bool saferightibeoflag=false;
    bool safefrontveloflag=false;
    int count=0;
    gettimeofday(&lastLMsg,NULL);
    gettimeofday(&lastRMsg,NULL);
    gettimeofday(&lastFMsg,NULL);
    while (true)
	{
		usleep(1000);

		PacketIbeoVector PIbeo;

        pthread_mutex_lock(&IbeoMutex);
        safeleftibeoflag=NewLeftibeoflag;
        NewLeftibeoflag=false;
        pthread_mutex_unlock(&IbeoMutex);

        if(safeleftibeoflag)
        {
            gettimeofday(&origCurMsg,NULL);
	        unsigned int timeDiff = 1000*(origCurMsg.tv_sec-lastLMsg.tv_sec)+(origCurMsg.tv_usec-lastLMsg.tv_usec)/1000;

	        if(timeDiff>period)
            {
              safeleftibeoflag = false;
              pthread_mutex_lock(&IbeoMutex);
              PIbeo=unsafeLibeo;
              pthread_mutex_unlock(&IbeoMutex);
              cout<<" "<<timeDiff<<std::endl;

                int rawnum=0;
              for(int i=0;i<PIbeo.ibcontournum.size();i++)
              {
                  if(PIbeo.ibcontournum[i])
                  {
                      rawnum+=PIbeo.ibcontournum[i];
                  }
                  else break;
              }
              int   pointsnum=PIbeo.ibcontourx.size();

              if((rawnum-pointsnum))
              cout<<"r  raw="<<rawnum<<"   pointsnum="<<pointsnum<<"       "<<rawnum-pointsnum<<endl;

				  lcm.publish("IbeoVector", &PIbeo);

				  count++;
				  gettimeofday(&lastLMsg, NULL);

            }
                       
	    }

        pthread_mutex_lock(&IbeoMutex);
        saferightibeoflag=NewRightibeoflag;
        NewRightibeoflag=false;
        pthread_mutex_unlock(&IbeoMutex);

        if(saferightibeoflag)
        {
            gettimeofday(&origCurMsg,NULL);
	        unsigned int timeDiff = 1000*(origCurMsg.tv_sec-lastRMsg.tv_sec)+(origCurMsg.tv_usec-lastRMsg.tv_usec)/1000;
	      if(timeDiff>period)
          {
              saferightibeoflag = false;
              pthread_mutex_lock(&IbeoMutex);
              PIbeo=unsafeRibeo;
              pthread_mutex_unlock(&IbeoMutex);
              cout<<"            "<<timeDiff<<std::endl;
              int rawnum=0;
              for(int i=0;i<PIbeo.ibcontournum.size();i++)
              {
                  if(PIbeo.ibcontournum[i])
                  {
                      rawnum+=PIbeo.ibcontournum[i];
                  }
                  else break;
              }
               int   pointsnum=PIbeo.ibcontourx.size();
               if((rawnum-pointsnum))
                 cout<<"r  raw="<<rawnum<<"   pointsnum="<<pointsnum<<"       "<<rawnum-pointsnum<<endl;

				   lcm.publish("IbeoVector", &PIbeo);
				  count++;
				  gettimeofday(&lastRMsg, NULL);
            }
        }


        pthread_mutex_lock(&IbeoMutex);
        safefrontveloflag=NewFrontibeoflag;
        NewFrontibeoflag=false;
        pthread_mutex_unlock(&IbeoMutex);

        if(safefrontveloflag)
        {
            gettimeofday(&origCurMsg,NULL);
            unsigned int timeDiff = 1000*(origCurMsg.tv_sec-lastFMsg.tv_sec)+(origCurMsg.tv_usec-lastFMsg.tv_usec)/1000;

            if(timeDiff>period)
            {
                safefrontveloflag = false;
                pthread_mutex_lock(&IbeoMutex);
                lcm.publish("IbeoVector", &unsafeFvelo);
                if(rawVlp16CollctFlag)
                {
                  lcm.publish("VLP16RawVector", &unsaferawvelo);
                }
                pthread_mutex_unlock(&IbeoMutex);

                 cout<<"      "<<timeDiff<<std::endl;
                 count++;
                 gettimeofday(&lastFMsg,NULL);
            }
        }

        if(count>=3)
        {
            count=0;
            //cout<<endl;
        }
	}
    pthread_mutex_destroy(&IbeoMutex);
	return 0;
}

