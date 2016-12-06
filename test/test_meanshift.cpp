#include"highgui.h"  
#include"cv.h"  
  
#include <iostream>  
  
using namespace cv;  
using namespace std;  
  
  
IplImage* src;  //source image  
IplImage* dst;  //the dst image after meanshift  
int spatialRad=10,colorRad=20,maxPryLevel=1;  
  
  
void on_Meanshift(int )  //the callback function  
{  
  
    //cout<<"spatialRad="<<spatialRad<<endl;   //for test  
    //cout<<"   colorRad="<<colorRad<<endl;  
    //cout<<"        maxPryLevel="<<maxPryLevel<<endl;  
    cvPyrMeanShiftFiltering(src,dst,spatialRad,colorRad,maxPryLevel);  //segmentation use meanshift  
    cvShowImage("dst",dst);   //show the segmented image  
  
}  
int main()
{  
    src = cvLoadImage("/home/gumh/Videos/challenge/image-001.jpeg");   //load the picture
    CvSize size;  
    size.width = src->width;  
    size.height = src->height;  
    dst = cvCreateImage(size,src->depth,3);  //set the size of the dst image  
    cvNamedWindow("src",CV_WINDOW_AUTOSIZE);  
    cvNamedWindow("dst",CV_WINDOW_AUTOSIZE);  
    cvShowImage("src",src);  
    cvPyrMeanShiftFiltering(src,dst,spatialRad,colorRad,maxPryLevel);  
  
    //create the trackbar  
    cvCreateTrackbar("spatialRad","dst",&spatialRad,50,on_Meanshift);   
    cvCreateTrackbar("colorRad","dst",&colorRad,60,on_Meanshift);  
    cvCreateTrackbar("maxPryLevel","dst",&maxPryLevel,5,on_Meanshift);  
  
    cvShowImage("dst",dst);  
  
    cvWaitKey(0);  

    return 0;
}  
