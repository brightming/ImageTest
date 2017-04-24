
#include <iostream>

#include <opencv2/opencv.hpp>


using namespace std;
using namespace cv;


void test_dig(){
    string name="/home/gumh/TrainData/AirPlane.2017.3.30/limit_speed/50-pai-1.png";
    Mat frame=imread(name);

    Mat img;

    cvtColor(frame,img,CV_BGR2GRAY);


    vector<Vec3f> circles;
    GaussianBlur(img, img, Size(3, 3), 2, 2);
//    threshold(img, img, 0, 10, CV_THRESH_BINARY);
    int edgeThresh=10;
    Canny(img,img, edgeThresh, edgeThresh*3, 3);
    imshow("pict",img);
    cout<<"img.width="<<img.cols<<",height="<<img.rows<<endl;
    //霍夫圆
    HoughCircles(img, circles, CV_HOUGH_GRADIENT, 1.5, 10, 200, 20, 0, 0);
    for (size_t i = 0; i < circles.size(); i++)
    {
        cout<<"------"<<endl;
        Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
        int radius = cvRound(circles[i][2]);
        //绘制圆心
//        circle(frame, center, 3, Scalar(0, 255, 0), -1, 8, 0);
        //绘制圆轮廓
        circle(frame, center, radius, Scalar(155, 50, 255), 1, 8, 0);
    }


    imshow("【效果图】", frame);

    waitKey(0);
}


int main(int argc,char* argv[]){
    test_dig();
    return 0;
}
