
#include "lanedetect/nightlane_detect.h"

void test_bit(){
    Mat a=Mat::eye(2,2,CV_8UC1);
    a.at<uchar>(0,0)=64;


    Mat b=Mat::zeros(2,2,CV_8UC1);
    b.at<uchar>(0,0)=255;
    b.at<uchar>(0,1)=1;
    Mat mask=Mat::ones(2,2,CV_8UC1);
    mask.at<uchar>(0,0)=1;


    Mat c=Mat::zeros(2,2,CV_8UC1);

    bitwise_or(a,b,c,mask);

    std::cout<<"a="<<std::endl<<a<<std::endl;
    std::cout<<"b="<<std::endl<<b<<std::endl;
    std::cout<<"mask="<<std::endl<<mask<<std::endl;
    std::cout<<"bitwise_or(a,b,c)="<<std::endl<<c<<std::endl;


    c-=c;
    bitwise_and(a,b,c,mask);
    std::cout<<"bitwise_and(a,b,c)="<<std::endl<<c<<std::endl;
}

int main()
{
//    test_bit();
//    return -0;


    string file="";
//    makeFromVid("/home/gumh/qtcreator-workspace/lanedetectsrc/Vehicle-Lane-Detection/sample/wandaor.mov");
    // makeFromVid("/home/yash/opencv-2.4.10/programs/road.m4v");
//    makeFromVid("/home/gumh/Videos/climb-down-changeroad.mp4");
//    makeFromVid("/home/gumh/Videos/own-highway.mp4");
    makeFromFolder("/home/gumh/Videos/hw2/");


//    vec4i_c lines;
//    int draw_lines;
//    IplImage *img=cvLoadImage("/home/gumh/tmp/highway/382.jpg");
//    night_lane_detect_img_c(img,&lines,1);
//    Mat t(img);
//    imshow("t",t);



    waitKey(0);
    destroyAllWindows();
}
