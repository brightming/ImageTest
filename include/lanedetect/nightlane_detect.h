#ifndef __NIGHTLANE_DETECT__
#define __NIGHTLANE_DETECT__



#include <string>
#include <iostream>

#include <opencv2/opencv.hpp>

#include "distance/lab_color_detect.h"

#include "lanedetect/lane_c.h"
#include "math/img_math.h"
#include "lanedetect/hough_color_lane_detect.h"

using namespace cv;
using namespace std;


void makeFromVid(string path);
void makeFromFolder(string path);


struct rect_aux_info{
	int min_x;
	int max_x;
	int min_y;
	int max_y;
	int index;

	rect_aux_info(int mn_x,int mx_x,int mn_y,int mx_y,int idx){
		this->min_x=mn_x;
		this->max_x=mx_x;
		this->min_y=mn_y;
		this->max_y=mx_y;
		this->index=idx;
	}
	rect_aux_info(){
		this->min_x=0;
		this->max_x=0;
		this->min_y=0;
		this->max_y=0;
		this->index=0;
	}
	bool contains_y(rect_aux_info other){
		if(min_y<=other.min_y  && this->max_y>= other.max_y){
			return true;
		}
		return false;
	}
};

class NightLaneDetect
{
public:
    Mat currFrame; //stores the upcoming frame
    Mat temp;      //stores intermediate results
    Mat temp2;     //stores the final lane segments
    Mat ori_pic;   //stores the original pics,can be resized to specified size ,but no more other action

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

    //----//
    vector<RotatedRect> left_rects;
	vector<rect_aux_info> left_rect_aux_infos;
    vector<RotatedRect> right_rects;
	vector<rect_aux_info> right_rect_aux_infos;
	vector<int> valid_contour_index;

    vector<SegMent> seg_ms;//roi分段的信息


    //--add for color judge--//
    colorDetect color_detect;
    Mat white_mask;
    Mat yellow_mask;
    Rect roi_rect;
    NightLaneDetect(Mat startFrame,float roi_hight_ratio=0.5);


    //---曲线拟合的函数
    //如果当前frame没有检测到左右的线，考虑用前frame的进行拟合
    LeastSquare left_last_lsq;
    LeastSquare right_last_lsq;


    //
    KalmanFilter KF;
    Mat measurement;


    void updateSensitivity();


    void getLane();


    void markLane();


    void blobRemoval();



    void nextFrame(Mat &nxt);
    Mat getResult();


    Rect GetRoiRect(){
	return roi_rect;
    }

    void FitLaneLine();

    bool isValidContour(vector<vector<Point> >& contours,int index);


	/**
	 * @brief OptimizeFilter
	 * 优化后的挑选有效blog的方法
	 */
	void OptimizeFilter();

	/**
	 * @brief OptimizePolyfit
	 * 曲线拟合
	 */
	void OptimizePolyfit();



};//end of class LaneDetect


#endif
