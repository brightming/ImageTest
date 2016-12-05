#ifndef __HOUGH_COLOR_LANE_DETECT__
#define __HOUGH_COLOR_LANE_DETECT__

#include <vector>
#include <opencv2/opencv.hpp>

extern "C"{

std::vector<cv::Vec4i> hough_color_detect_img(cv::Mat& src,int draw_lines=0);

}
#endif //__HOUGH_COLOR_LANE_DETECT__
