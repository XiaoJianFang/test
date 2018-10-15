#ifndef _VISUAL_APP_H_
#define _VISUAL_APP_H_

#include "VisualizationBasicApi.h"
#include "../include/client/Packet.h"

namespace visual
{

	class VisualizationApplicationApi : public VisualizationBasicApi
	{
	public:
		VisualizationApplicationApi() {};
		~VisualizationApplicationApi() {};

		void showVehicle(IplImage* src, IplImage* vehicle);
		void showMeter(IplImage* src, IplImage* meter, IplImage* indicator, float speed, int gear);

		void showSonicPos(IplImage* src, CvScalar color, int radius);
		void showSonic(IplImage* src, vector<float>& vecx, vector<float>& vecy, int radius, CvScalar color, int thickness, int inter);
		void interpSonicPts(vector<float>& raw_sonicx, vector<float>& raw_sonicy, int npt);
		void showParkingAreaBox(IplImage* src, vector<BoxPt>& bxpts, CvScalar parkFill, CvScalar parkBoundParallel, CvScalar parkBoundVertical);
		void showVisualParkingAreaBox(IplImage* src, PacketVisualParkingArea vPark, CvScalar parkFill, CvScalar parkBound);

		void showPridictVehiclePath(IplImage* src,
			vector<float>& vecx0, vector<float>& vecy0,
			vector<float>& vecx1, vector<float>& vecy1,
			vector<float>& vecx2, vector<float>& vecy2,
			vector<float>& vecx3, vector<float>& vecy3,
			CvScalar pointColor, CvScalar rectColor, int pointThickness, int rectThickness);

		void showGridMap(IplImage* src, vector<float>& vecx, vector<float>& vecy, float size, CvScalar color, int shape);
		void showRoad(IplImage* src, vector<float>& rblx, vector<float>& rbly, vector<float>& rbrx, vector<float>& rbry, CvScalar color, int thickness);


	private:

	};

}


// extern void showVehicle(IplImage* src, IplImage* vehicle);
// extern void showMeter(IplImage* src, IplImage* meter, IplImage* indicator, float speed, int gear);
// 
// extern void showSonicPos(IplImage* src, CvScalar color, int radius);
// extern void showSonic(IplImage* src, vector<float>& vecx, vector<float>& vecy, int radius, CvScalar color, int thickness, int inter);
// extern void interpSonicPts(vector<float>& raw_sonicx, vector<float>& raw_sonicy, int npt);
// extern void showParkingAreaBox(IplImage* src, vector<BoxPt>& bxpts, CvScalar parkFill, CvScalar parkBoundParallel, CvScalar parkBoundVertical);
// extern void showVisualParkingAreaBox(IplImage* src, PacketVisualParkingArea vPark, CvScalar parkFill, CvScalar parkBound);
// 
// extern void showPridictVehiclePath(IplImage* src, 
// 	vector<float>& vecx0, vector<float>& vecy0, 
// 	vector<float>& vecx1, vector<float>& vecy1, 
// 	vector<float>& vecx2, vector<float>& vecy2, 
// 	vector<float>& vecx3, vector<float>& vecy3, 
// 	CvScalar pointColor, CvScalar rectColor, int pointThickness, int rectThickness);
// 
// extern void showRoad(IplImage* src, vector<float>& rblx, vector<float>& rbly, vector<float>& rbrx, vector<float>& rbry, CvScalar color, int thickness);

#endif

