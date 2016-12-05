
#include <iostream>
#include <opencv2/opencv.hpp>
using namespace std;
//using namespace cv;



class colorDetect{
private:
    int minDist; //minium acceptable distance
    cv::Vec3b target;//target color;
    cv::Mat result; //the result
public:
    colorDetect(){};
    void SetMinDistance(int dist);
    void SetTargetColor(uchar blue,uchar green,uchar red);
    void SetTargetColor(cv::Vec3b color); //set the target color
    cv::Mat process(const cv::Mat& image); //main process
};


