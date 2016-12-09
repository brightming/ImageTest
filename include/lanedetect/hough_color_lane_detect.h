#ifndef __HOUGH_COLOR_LANE_DETECT__
#define __HOUGH_COLOR_LANE_DETECT__

#include <vector>
#include <opencv2/opencv.hpp>

#include "lanedetect/lane_c.h"

extern "C"{

std::vector<cv::Vec4i> hough_color_detect_img(cv::Mat& src,int draw_lines=0);

}


typedef struct SegMent{
    int min_line_len;
    int max_gap_len;
    int min_left_slope;
    int max_left_slope;
    int min_right_slope;
    int max_right_slope;
    cv::Rect  range;
    int y_offset;
    int x_offset;

    cv::Scalar yellow_color;
    cv::Scalar white_color;

    int white_lab_min_dist;
    int yellow_lab_min_dist;

    cv::Mat white_mask;
    cv::Mat yellow_mask;

    cv::Mat pic;
    cv::Mat middle_pic;
    cv::Mat all_mask; //white_mask | yellow_mask


	float seg_area;//该段面积
	float max_allow_ratio;//对于一个contour，如果落在该段内，所得到的面积，占该段面积最大的比例
}SegMent;

void filter_colors(cv::Mat& output_mask,cv::Mat& output_image,std::vector<SegMent> seg_ms);
void get_segments(cv::Mat& src,std::vector<SegMent> &seg_ms,int valid_roi_width,int valid_roi_height,int start_x,int start_y,int seg_cnt=5,float height_ratio=1.15);

void initial_segments(std::vector<SegMent> &seg_ms,int valid_roi_width,int valid_roi_height,int start_x,int start_y,int seg_cnt=5,float height_ratio=1.15);

void fill_segment_mat(cv::Mat& src,std::vector<SegMent> &seg_ms);





#endif //__HOUGH_COLOR_LANE_DETECT__
