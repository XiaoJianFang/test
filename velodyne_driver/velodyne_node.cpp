#include <iostream>
#include <string>
#include <pthread.h>
#include <velodyneHDL32E.h>
#include <lcm/lcm-cpp.hpp>
#include "PacketIbeo.hpp"
#include "PacketVELD.hpp"
#define LENGTHOFIBOBJ		50
#define LENGTHOFCONTOUR		800
float Object_x[LENGTHOFIBOBJ];//Obj_x
float Object_y[LENGTHOFIBOBJ];//Obj_y
float Object_Boxx[LENGTHOFIBOBJ];//Box_x
float Object_Boxy[LENGTHOFIBOBJ];//ŽBox_y
float Object_vx[LENGTHOFIBOBJ];
float Object_vy[LENGTHOFIBOBJ];
float Object_vabx[LENGTHOFIBOBJ];
float Object_vaby[LENGTHOFIBOBJ];
int16_t Object_contournum[LENGTHOFIBOBJ];
float Object_contourx[LENGTHOFCONTOUR];
float Object_contoury[LENGTHOFCONTOUR];
int Objectnum = 0;
float zerosf[50] = { 0.0f };
int16_t zeros[800] = { 0 };
pthread_mutex_t IbeoMutex ;

int main(int argc, char *argv[])
{
     lcm::LCM lcm;
     if(!lcm.good())
        return 1;

       string  filePrefix="velo";
       string fileSuffix="txt";
       vector<pointType> velodata;

	velodyneHDL32E velodyneDevice(sensorDevice::modeType::online, sensorDevice::saveType::unsave);
       // velodyneDevice.setFileName(filePrefix, fileSuffix);

      PacketIbeo IBEO;
      PacketVELD VELD;
      while(1)
      {
          usleep(10000);

          velodyneDevice.getData(velodata,0);
          std::cout<<"number"<<velodata.size()<< std::endl;

          memcpy(Object_x, zerosf, sizeof(Object_x));
          memcpy(Object_y, zerosf, sizeof(Object_y));
          memcpy(Object_Boxx, zerosf, sizeof(Object_Boxx));
          memcpy(Object_Boxy, zerosf, sizeof(Object_Boxy));
          memcpy(Object_vx, zerosf, sizeof(Object_vx));
          memcpy(Object_vy, zerosf, sizeof(Object_vy));
          memcpy(Object_vabx, zerosf, sizeof(Object_vabx));
          memcpy(Object_vaby, zerosf, sizeof(Object_vaby));
          memcpy(Object_contournum, zeros, sizeof(Object_contournum));
          memcpy(Object_contourx, zerosf, sizeof(Object_contourx));
          memcpy(Object_contoury, zerosf, sizeof(Object_contoury));

              Object_x[0]=1;
              Object_y[0]=1;
              Object_Boxx[0]=1;
              Object_Boxy[0]=1;
              Object_vx[0]=1;
              Object_vy[0]=1;
              Object_vabx[0]=1;
              Object_vaby[0]=1;



          memcpy(IBEO.ibx, Object_x, sizeof(IBEO.ibx));
          memcpy(IBEO.iby, Object_y, sizeof(IBEO.iby));
          memcpy(IBEO.ibw, Object_Boxx, sizeof(IBEO.ibw));
          memcpy(IBEO.ibl, Object_Boxy, sizeof(IBEO.ibl));
          memcpy(IBEO.ibvx, Object_vx, sizeof(IBEO.ibvx));
          memcpy(IBEO.ibvy, Object_vy, sizeof(IBEO.ibvy));
          memcpy(IBEO.ibvabx, Object_vabx, sizeof(IBEO.ibvabx));
          memcpy(IBEO.ibvaby, Object_vaby, sizeof(IBEO.ibvaby));
          memcpy(IBEO.ibcontournum, Object_contournum, sizeof(IBEO.ibcontournum));
          memcpy(IBEO.ibcontourx, Object_contourx, sizeof(IBEO.ibcontourx));
          memcpy(IBEO.ibcontoury, Object_contoury, sizeof(IBEO.ibcontoury));

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
                  IBEO.ibcontourx[index] = velodata[i].x ;
                  IBEO.ibcontoury[index] = velodata[i].y;
              } else
                  break;
          }



          IBEO.ibcontournum[0]=index;

        lcm.publish("Ibeo", &IBEO);
      }
        return 0;
}
