#include <iostream>
#include <sstream>
#include <stdio.h>
#include "PacketHighResolutionMap.hpp"
#include "PacketGPS.hpp"
#include "PacketStopLine.hpp"
#include <lcm/lcm-cpp.hpp>
#include "SendMap.h"
#include "map.h"
#include <sys/time.h>
#include <pthread.h>
#include <unistd.h>
#include <string>

pthread_mutex_t mutex;


map_t *GoalGridMap;
int mapLoadFlag=-1;
mapdata GPSMap;
MapZero MapZeroGps;
PacketHighResolutionMap HDMap;
PacketGPS unsafe_GPS;
PacketGPS new_GPS;
PacketSlamPos unsafe_SlamPOS;
PacketSlamPos new_SlamPOS;
int unsafe_isNewGPS=0;
int unsafe_isNewSlamPOS=0;
lcm::LCM lcmHandler;

string gpsFile="GPSMap.txt";
string slamposFile="SlamPOSMap.txt";
string mapFile="20CMGridMap.txt";

int modeType=0;
int loadParam(void);

class Handler
{
public:
    ~Handler() {}


    void handleMessage(const lcm::ReceiveBuffer* rbuf,
                       const std::string& chan,
                       const PacketGPS* msg)
    {
        pthread_mutex_lock(&mutex);
        memcpy(&unsafe_GPS,(PacketGPS*)msg, sizeof(PacketGPS));
        unsafe_isNewGPS=1;
        pthread_mutex_unlock(&mutex);
    }

    void handleMessage2(const lcm::ReceiveBuffer* rbuf,
                       const std::string& chan,
                       const PacketSlamPos* msg)
    {
        pthread_mutex_lock(&mutex);
        memcpy(&unsafe_SlamPOS,(PacketSlamPos*)msg, sizeof(PacketSlamPos));
        unsafe_isNewSlamPOS=1;
        pthread_mutex_unlock(&mutex);
    }

};

void *LCMPthread(void* args)
{
    cout<<"Start receive data"<<endl;
    if(!lcmHandler.good())
    {
        cout<<"lcm init failling"<<endl;
        //return 1;
    }
    while(true)
    {
        lcmHandler.handle();
        usleep(1000);
    }

}

int main(int argc, char** argv)
{


    loadParam();
    //turn on listener
    pthread_t LCMlistener;
    int  ret=1;
    ret=pthread_create(&LCMlistener,NULL, LCMPthread,NULL);
    if(ret!=0)
    {
        printf ("Create pthread Left error!/n");
        exit (1);
    }


     //load map

    GoalGridMap = map_alloc();
    mapLoadFlag=map_load_gridsub(GoalGridMap,mapFile);
    if(!modeType)
    {
        std::cout<<"GPS Mode"<<std::endl;
        if(mapLoadFlag<0)
        {
            cout<<"no mapdata, can not initialize!!!"<<endl;
        }

        LoadGPSMap(GPSMap,gpsFile);
    }
    else if(modeType==1)
    {
       //mapLoadFlag=-1;//no garage map
        std::cout<<"SlamPOS Mode"<<std::endl;
        LoadSlamPOSMap(GPSMap,slamposFile);
    }

    Handler handlerObject;
    lcmHandler.subscribe("GPS", &Handler::handleMessage, &handlerObject);
    lcmHandler.subscribe("SlamPos", &Handler::handleMessage2, &handlerObject);
    pthread_mutex_init(&mutex, NULL);


    timeval lastmsg;
    timeval origmsg;
    PacketGPS vehicleGPS;
    PacketSlamPos vehicleSlamPOS;


    int cnt = 0;
    if(!modeType)
    {
        cout<<"Waiting for GPS data"<<endl;
        while (true)
        {
            usleep(1000);
            if (new_GPS.lat == 0)
            {
                pthread_mutex_lock(&mutex);
                new_GPS.lat = unsafe_GPS.lat;
                pthread_mutex_unlock(&mutex);
                cnt++;
                if (new_GPS.lat == 0 && cnt<3)
                    std::cout << "Not Receiving GPS data!!!" << endl;
            }
            else
            {
                pthread_mutex_lock(&mutex);
                vehicleGPS = unsafe_GPS;
                pthread_mutex_unlock(&mutex);
                break;
            
            }
        }
        std::cout << " Receiving GPS data!" << endl;
    }
    else if(modeType==1)
    {
        cout<<"Waiting for SlamPos data"<<endl;

        while (true)
        {
            usleep(1000);
            if (new_SlamPOS.x == 0)
            {
                pthread_mutex_lock(&mutex);
                new_SlamPOS.x = unsafe_SlamPOS.x;
                pthread_mutex_unlock(&mutex);
                cnt++;
                if (new_SlamPOS.x == 0 && cnt<3)
                    std::cout << "Not Receiving SlamPOS data!!!" << endl;
            }
            else
            {
                pthread_mutex_lock(&mutex);
                vehicleSlamPOS = unsafe_SlamPOS;
                pthread_mutex_unlock(&mutex);
                break;
            }
        }
        std::cout << "Receiving SlamPOS data!" << endl;
    }


    bool isNewGPS = false;
    int  NoGPSCount = 0;
    bool isNewSlamPOS = false;
    int  NoSlamPOSCount = 0;
    int TimeDiff=0;
    int16_t stopLineDis=0;

    gettimeofday(&origmsg,NULL);
    gettimeofday(&lastmsg,NULL);



    while(true)
    {
        usleep(1000);

       if(!modeType)
       {
           pthread_mutex_lock(&mutex);
           vehicleGPS = unsafe_GPS;
           isNewGPS = unsafe_isNewGPS;
           unsafe_isNewGPS = false;
           pthread_mutex_unlock(&mutex);

           if (isNewGPS)
           {
               gettimeofday(&origmsg,NULL);
              // TimeDiff =(origmsg.tv_sec-lastmsg.tv_sec)*1000+(origmsg.tv_usec-lastmsg.tv_usec)/1000;


               NoGPSCount = 0;

              // if (TimeDiff > 50)
               {
                   PacketHighResolutionMap mapdata;
                   int sendflag =GetMapdatasubmap(vehicleGPS, mapdata,stopLineDis);

                   if (sendflag == 1)
                   {
                       lcmHandler.publish("HAD",&mapdata);
                   }
                   if(stopLineDis>0)
                   {
                       PacketStopLine stopLine;
                       stopLine.imgIndex=0;
                       stopLine.stopline=stopLineDis;
                       lcmHandler.publish("StopLine",&stopLine);
                       stopLineDis=-1;
                   }
                   gettimeofday(&lastmsg,NULL);
                   TimeDiff =(origmsg.tv_sec-lastmsg.tv_sec)*1000+(origmsg.tv_usec-lastmsg.tv_usec)/1000;

                    cout << "TimeDiff=" << TimeDiff << endl;
               }
           }
           if (TimeDiff > 300)
           {
               NoGPSCount++;
               if (!(NoGPSCount%100))
                   std::cout << "Not Receiving GPS data, TimeDiff=" << TimeDiff<<std::endl;
           }
       }
        else if(modeType==1)
       {
           pthread_mutex_lock(&mutex);
           vehicleSlamPOS = unsafe_SlamPOS;
           isNewSlamPOS = unsafe_isNewSlamPOS;
           unsafe_isNewSlamPOS = false;
           pthread_mutex_unlock(&mutex);

           if (isNewSlamPOS)
           {
               gettimeofday(&origmsg,NULL);
               TimeDiff =(origmsg.tv_sec-lastmsg.tv_sec)*1000+(origmsg.tv_usec-lastmsg.tv_usec)/1000;
               NoSlamPOSCount = 0;
               //if (TimeDiff > 50)
               {
                   PacketHighResolutionMap mapdata;
                   int sendflag =GetMapdatasubmap(vehicleSlamPOS, mapdata,stopLineDis);

                   if (sendflag == 1)
                   {
                       lcmHandler.publish("HAD",&mapdata);
                   }
                   if(stopLineDis>0)
                   {
                       PacketStopLine stopLine;
                       stopLine.imgIndex=0;
                       stopLine.stopline=stopLineDis;
                       lcmHandler.publish("StopLine",&stopLine);
                       stopLineDis=-1;
                   }
                   gettimeofday(&lastmsg,NULL);
                   // cout << "TimeDiff=" << TimeDiff << endl;
               }
           }
           if (TimeDiff > 300)
           {
               NoSlamPOSCount++;
               if (!(NoSlamPOSCount%100))
                   std::cout << "Not Receiving SlamPOS data, TimeDiff=" << TimeDiff<<std::endl;
           }
       }




    }
    return 0;
}

int loadParam(void)
{
    cout<<"load ./data/sendmapConfig.txt "<<endl;

    ifstream file("./data/sendmapConfig.txt");
    string line;
    if(!file)
    {
        cout<<" ./data/sendmapConfig.txt does not exist !!!"<<endl;
        return -1;
    }
    getline(file,line);
    getline(file,line);
    std::istringstream iss(line);
    iss >> modeType;

    getline(file,line);
    getline(file,gpsFile);

    getline(file,line);
    getline(file,slamposFile);

    getline(file,line);
    getline(file,mapFile);

}