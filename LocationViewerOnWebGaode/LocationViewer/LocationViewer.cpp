#include "SendMap.h"
#include <tchar.h>  
#include <assert.h>  
#include <iomanip>
#include <iostream>
#include <time.h>
#include <fstream>
#include <sstream>
#include <string>

//#include "pngtest.h"
#include "opencv/cv.h"
#include "opencv/highgui.h"
#include "opencv/cxcore.h"
#define MAXBLOCKSIZE 502400
#include <windows.h>
#include <wininet.h>
#include <stdio.h>
#pragma comment(lib, "wininet.lib")
#pragma comment(lib, "opencv_world310d.lib")

//using namespace cv;

#include "tinystr.h"  
#include "tinyxml.h"  

void LoadGPSMap(mapdata&gGPSMap);
string StaticMap = "http://restapi.amap.com/v3/staticmap?location=";
string Key = "&key=4c0fcb7b63d0f2487b4b88927cb12084";
string Marks = "&markers=mid,,C:";
string Lables = "&labels=CAR,1,1,16,0xFF0FFF,0x008000:";
string Size = "&size=300*200";//w*h
string Zoom = "&zoom=16";
string Scale = "&scale=1";
string Coordsys = "&coordsys=gps";
string Output = "&output=xml";
string testgps = "121.202685,31.289964";
TCHAR gps[21] = "";
TCHAR szOperation[] = _T("open");
string Togps = "http://restapi.amap.com/v3/assistant/coordinate/convert?locations=";
string mapaddress = "http://restapi.amap.com/v3/staticmap?location=121.202685,31.289964&zoom=16&size=750*700&&markers=mid,,A:121.202685,31.289964&key=4c0fcb7b63d0f2487b4b88927cb12084";
clock_t lasttime = -6000;
clock_t currenttime = 0;
std::string valuedata;
std::string data;
string togpsaddress;
void GetWebSrcCode(const char *Url, float& lon, float& lat);
void readXml( char * xmlFile);
void GetWebSrcCodepng(const char *Url);
void show(char *data);



int main()
{
	float correctlon, correctlat;
	mapdata Map;
	LoadGPSMap(Map);
	string address;
	for (size_t i = 1000; i < Map.gps.size();)
	{
		currenttime = clock();

		if (currenttime - lasttime > 5000)
		{

			sprintf_s(gps, "%f,%f", Map.gps[i].lon, Map.gps[i].lat);
			testgps = gps;
			togpsaddress = Togps + testgps + Coordsys + Output + Key;
			GetWebSrcCode(togpsaddress.c_str(), correctlon, correctlat);

			sprintf_s(gps, "%f,%f", correctlon, correctlat);
			testgps = gps;
			mapaddress = StaticMap + testgps + Zoom + Scale + Size + Marks + testgps + Key;
			GetWebSrcCodepng(mapaddress.c_str());
			
			lasttime = clock();
			//HINSTANCE hRslt = ShellExecute(NULL, szOperation, mapaddress.c_str(), NULL, NULL, SW_SHOWNORMAL);
			i = i + 50;
		}
		cvWaitKey(200);
	}
	//assert(hRslt > (HINSTANCE)HINSTANCE_ERROR);
	return 0;
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
void OpenFile(std::string& fileName)
{
	TCHAR szBuffer[MAX_PATH] = { 0 };
	OPENFILENAME ofn = { 0 };
	ofn.lStructSize = sizeof(ofn);
	ofn.hwndOwner = NULL;
	ofn.lpstrFilter = "txt(*.txt)\0*.txt\0";
	ofn.lpstrInitialDir = "E:\\Intelligent vehicle\\2015\\UGVFinal";
	ofn.lpstrFile = szBuffer;
	ofn.nMaxFile = sizeof(szBuffer) / sizeof(*szBuffer);
	ofn.nFilterIndex = 0;
	ofn.Flags = OFN_PATHMUSTEXIST | OFN_FILEMUSTEXIST | OFN_EXPLORER;
	BOOL bSel = GetOpenFileName(&ofn);
	fileName = std::string(szBuffer);
}
void LoadGPSMap(mapdata&gGPSMap)
{
	std::string fileName;
	cout << "load GPSMap.txt" << endl;
	OpenFile(fileName);
	int flag = 0;
	GPS gpszero;
	ifstream in(fileName);
	if (!in)
	{
		int ret = MessageBox(nullptr, "Click OK to exit", "File Error", MB_OK | MB_ICONERROR);
		if (ret == MB_OK) exit(-1);
	}
	int i = 0;
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
			//gps.lat = gps.lat / (long double)180 * (long double)PI64;
			//gps.lon = gps.lon / (long double)180 * (long double)PI64;
			//gps.head = gps.head / (long double)180 * (long double)PI64;
			if (!flag)
			{
				gpszero.lat = gps.lat;
				gpszero.lon = gps.lon;
				gpszero.head = gps.head;
				flag++;
			}
			double x, y;
			gGPSMap.gps.push_back(gps);
		}
	}
	gGPSMap.coordinatezero = 0;
}

void GetWebSrcCode(const char *Url, float& lon, float& lat)
{

	HINTERNET hSession = InternetOpen("zwt", INTERNET_OPEN_TYPE_PRECONFIG, NULL, NULL, 0);
	if (hSession != NULL)
	{
		HINTERNET hURL = InternetOpenUrl(hSession, Url, NULL, 0, INTERNET_FLAG_DONT_CACHE, 0);
		if (hURL != NULL)
		{
			char Temp[MAXBLOCKSIZE] = { 0 };
			ULONG Number = 1;

			while (Number > 0)
			{
				InternetReadFile(hURL, Temp, MAXBLOCKSIZE - 1, &Number);
			}
			data = Temp;
			int start, end;
			start = data.find("<locations>");//+11    +26

			//readXml(Temp);

			data = data.substr(start + 11);
			start = data.find(",");//+11    +26
			valuedata = data.substr(0, start);

			data = data.substr(start + 1);
			lon = stringToNum<float>(valuedata);
			valuedata.clear();

			start = data.find("</locations>");//+11    +26
			valuedata = data.substr(0, start);
			lat = stringToNum<float>(valuedata);
			valuedata.clear();

			InternetCloseHandle(hURL);
			hURL = NULL;
		}

		InternetCloseHandle(hSession);
		hSession = NULL;
	}
}
void InternetReadFileEx(HINTERNET request, std::string& data) {
	DWORD readed = 0;
	char buffer[1025];
	do {
		ZeroMemory(buffer, 1025);
		InternetReadFile(request, buffer, 1024, &readed);
		buffer[readed] = '\0';
		data += buffer;
	} while (readed);
}
void GetWebSrcCodepng(const char *Url)
{

	HINTERNET hSession = InternetOpen("zwt", INTERNET_OPEN_TYPE_PRECONFIG, NULL, NULL, 0);
	if (hSession != NULL)
	{
		HINTERNET hURL = InternetOpenUrl(hSession, Url, NULL, 0, INTERNET_FLAG_DONT_CACHE, 0);
		if (hURL != NULL)
		{
			char Temp[MAXBLOCKSIZE] = { 0 };
			ULONG Number = 1;

		//	while (Number > MAXBLOCKSIZE)
 			//{
 			InternetReadFile(hURL, Temp, MAXBLOCKSIZE - 1, &Number);
			//cvWaitKey(50);
 			//}
			std::string pngdata;
		//	InternetReadFileEx(hURL,pngdata);
			FILE *fp;
			if ((fp = fopen("staticmap.png", "wb")) == NULL) 
			{
				printf("\nopen file error");
				getchar();
				exit(1);
			}
			int flag = 0;
			for (size_t i = 0; i < 102400; i++)
			{
				if (!flag&&Temp[i] == 'I'&&Temp[i + 1] == 'E'&&Temp[i + 2] == 'N'&&Temp[i + 3] == 'D')
				{
					flag++;
				}
				if (flag) flag++;
					
				fprintf(fp, "%c", Temp[i]);

				if (flag > 9) break;
			}
			fclose(fp);
			show(Temp);
			InternetCloseHandle(hURL);
			hURL = NULL;
		}

		InternetCloseHandle(hSession);
		hSession = NULL;
	}
}


void readXml( char * xmlData)
{
	using namespace std;
	char Temp[MAXBLOCKSIZE] = { 0 };
	char * Temp2;

	Temp2 = xmlData;
	TiXmlDocument doc("test2.xml");
	
	doc.Parse(xmlData);

	if (doc.Error())
	{
		printf("Error in %s: %s\n", doc.Value(), doc.ErrorDesc());
		return;// exit(1);
	}

	doc.SaveFile();
	
	TiXmlElement* rootElement = doc.RootElement();  //School元素    
	TiXmlElement* classElement = rootElement->FirstChildElement();  // Class元素  
	TiXmlElement* studentElement = classElement->FirstChildElement();  //Students    
	for (; studentElement != NULL; studentElement = studentElement->NextSiblingElement()) {
		TiXmlAttribute* attributeOfStudent = studentElement->FirstAttribute();  //获得student的name属性    
		for (; attributeOfStudent != NULL; attributeOfStudent = attributeOfStudent->Next()) {
			std::cout << attributeOfStudent->Name() << " : " << attributeOfStudent->Value() << std::endl;
		}

		TiXmlElement* studentContactElement = studentElement->FirstChildElement();//获得student的第一个联系方式   
		for (; studentContactElement != NULL; studentContactElement = studentContactElement->NextSiblingElement()) {
			std::string contactType = studentContactElement->Value();
			std::string contactValue = studentContactElement->GetText();
			std::cout << contactType << " : " << contactValue << std::endl;
		}

	}
}

void show(char *data)
{
	std::string name = "staticmap.png";
	IplImage* VisualImage = cvLoadImage(name.c_str(), CV_LOAD_IMAGE_ANYCOLOR);
	if (VisualImage != NULL)
	{
		cvShowImage(name.c_str(), VisualImage);
		cvWaitKey(16);
	}
}