// Usage: ./app input.jpg
#include "opencv2/opencv.hpp"
#include <string>

using namespace cv;
using namespace std;

int main(int argc,char* argv[])
{

    Mat srcImage=imread(argv[1]);

    if(!srcImage.data){
        cout<<"empty data!"<<endl;
        return -1;
    }

    resize(srcImage,srcImage,Size(srcImage.rows/4,srcImage.cols/4));


    Mat srcGray;
    cvtColor(srcImage,srcGray,CV_BGR2GRAY);

    Mat srcBinary;
    threshold(srcGray,srcBinary,160,255,cv::THRESH_BINARY);

    Mat dstImage;
    distanceTransform(srcBinary,dstImage,CV_DIST_L2,CV_DIST_MASK_PRECISE);

    normalize(dstImage,dstImage,0,1.,cv::NORM_MINMAX);
    imshow("srcBinary",srcBinary);
    imshow("dstImage",dstImage);

    cv::waitKey(0);
    return 0;



}
