#ifndef __NIGHTLANE_DETECT__
#define __NIGHTLANE_DETECT__



#include <string>
#include <iostream>

#include <opencv2/opencv.hpp>

#include "distance/lab_color_detect.h"

#include "lanedetect/lane_c.h"

using namespace cv;
using namespace std;


void makeFromVid(string path);
void makeFromFolder(string path);


class NightLaneDetect
{
public:
    Mat currFrame; //stores the upcoming frame
    Mat temp;      //stores intermediate results
    Mat temp2;     //stores the final lane segments

    Mat for_draw;

    int diff, diffL, diffR;
    int laneWidth;
    int diffThreshTop;
    int diffThreshLow;
    int ROIrows;
    int vertical_left;
    int vertical_right;
    int vertical_top;
    int smallLaneArea;
    int longLane;
    int  vanishingPt;
    float maxLaneWidth;

    //to store various blob properties
    Mat binary_image; //used for blob removal
    int minSize;
    int ratio;
    float  contour_area;
    float blob_angle_deg;
    float bounding_width;
    float bounding_length;
    Size2f sz;
    vector< vector<Point> > contours;
    vector<Vec4i> hierarchy;
    RotatedRect rotated_rect;


	//--add for color judge--//
	colorDetect color_detect;
	Mat white_mask;
	Mat yellow_mask;
	Rect roi_rect;
    NightLaneDetect(Mat startFrame);


    void updateSensitivity();


    void getLane();


    void markLane();


    void blobRemoval();



    void nextFrame(Mat &nxt);
    Mat getResult();


    Rect GetRoiRect(){
	return roi_rect;
    }





};//end of class LaneDetect


#endif
