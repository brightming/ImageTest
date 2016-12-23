#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <iostream>
#include <vector>
#include <algorithm>
#include <math.h>
#include <time.h>

#include "lane/road_detector.h"

class WidthLane :public LaneDetector{
public:
    cv::Mat currFrame_;
    cv::Mat ROIFrame_;
    cv::Mat binary_image_;
    cv::Mat binary_image_pers_;
    cv::Mat templateFrame_;
    cv::Mat templateFrame_pers_;
    cv::Mat contoursFrame_;
    cv::Mat MaskFrame_;
    cv::Mat OutFrame_;

    int diff_;
    int diffL_;
    int diffR_;
    int laneWidth_;
    int diffThreshTop_;
    int diffThreshLow_;
    int ROIrows_;
    int smallLaneArea_;
    int longLane_;
    int  vanishingPt_;
    float maxLaneWidth_;
    float minLaneWidth_;
    float maxLaneWidth_pers_;
    float template_minwidth_;
    float template_maxwidth_;
    float template_rows_;
    float template_rows_pers_;

    int ind_xl_;
    int ind_xr_;
    int subROI_w;

    int minSize_;
    int ratio_;
    float contour_area_;
    float blob_angle_deg_;
    float bounding_width_;
    float bounding_length_;
    cv::Size2f sz_;
    std::vector< std::vector<cv::Point>> contours_;
    std::vector<cv::Vec4i> hierarchy_;
    cv::RotatedRect rotated_rect_;

public:
    WidthLane(cv::Mat startFrame);
    void SetVanishPt(int vp);
    void SetROI();           //ROI设定
    void ROIPreprocess();    //ROI预处理
    void GaussianFitler();   //高斯滤波
    void MarkLane(cv::Mat &src);   //二值化可能为车道线的像素
    void MarkLanePers(cv::Mat &src);//透视变换下，二值化可能为车道线的像素
    void KernalFilter();
    void KernalFilterPers();
    void TemplateFilter(cv::Mat &src); //template过滤
    void TemplateFilterPers(cv::Mat &src);  //透视变换下，template过滤
    void ContoursComputePers(cv::Mat &src, cv::Mat& dst,cv::Scalar sc); //透视变换下，Contours边缘化及填充
    void ContoursCompute(cv::Mat &src, cv::Mat& dst); //Contours边缘化及填充（待实现）

    void FindNonZeroFill(cv::Mat &src, uchar val);  //单通道非零值设为val
    void PerspectiveTrans(cv::Mat &src); //透视变换
    void InversePerspectiveTrans(cv::Mat &src); //反透视变换
    void AutoCanny(cv::Mat &src,float type);
    //待实现
    void Initialize(std::vector<std::string> params){}
    void Update(cv::Mat& pict){}
    std::vector<Lane> GetLanes(){}
    cv::Mat GetLaneMat(){}
};
