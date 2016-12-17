#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;
int main(int argc,char** av){

    vector<Point2f>  original_pts,bird_pts;

    original_pts.push_back(Point2f(708,562));
    original_pts.push_back(Point2f(1025,562));
    original_pts.push_back(Point2f(560,712));
    original_pts.push_back(Point2f(1208,712));


    bird_pts.push_back(Point2f(560,562));
    bird_pts.push_back(Point2f(1208,562));
    bird_pts.push_back(Point2f(560,712));
    bird_pts.push_back(Point2f(1208,712));



    Mat H = findHomography(original_pts, bird_pts, RANSAC, 4);



    cout<<H<<endl;

    Mat image=imread("/home/gumh/tmp/highway/535.jpg");
    Mat dst;

    warpPerspective(image,dst,H,Size(image.cols,image.rows));

    namedWindow("ori",2);
    namedWindow("dst",2);

    imshow("ori",image);
    imshow("dst",dst);

    waitKey(0);

    return 0;
}
