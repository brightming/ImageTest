#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;
int test_homogoraphy(){

    vector<Point2f>  original_pts,another_pts;

    original_pts.push_back(Point2f(1142,400));  //book在图1中的坐标
    original_pts.push_back(Point2f(2832,764));
    original_pts.push_back(Point2f(2300,3252));
    original_pts.push_back(Point2f(320,2676));


    another_pts.push_back(Point2f(1833,281)); //在图2中的坐标
    another_pts.push_back(Point2f(2584,1303));
    another_pts.push_back(Point2f(924,2218));
    another_pts.push_back(Point2f(163,960));



    Mat H = findHomography(original_pts, another_pts, RANSAC, 4);
    cout<<H<<endl;

    Mat image=imread("/home/gumh/Pictures/book1.jpg");
    Mat dst1;
    warpPerspective(image,dst1,H,Size(image.cols,image.rows)); //变为图2的样子


    Mat image2=imread("/home/gumh/Pictures/book2.jpg");
    Mat dst2;
    warpPerspective(image2,dst2,H.inv(),Size(image2.cols,image2.rows));//用逆矩阵将目标图，期望变换为图1的样子



    Mat dst3;
    warpPerspective(image2,dst3,H,Size(image2.cols,image2.rows),WARP_INVERSE_MAP|INTER_LINEAR);//逆变换，期望输出与pict1一样的。



    namedWindow("ori1",2);
    namedWindow("dst1",2);
    namedWindow("ori2",2);
    namedWindow("dst2",2);
    namedWindow("dst3",2);

    imshow("ori1",image);
    imshow("dst1",dst1);


    imshow("ori2",image2);
    imshow("dst2",dst2);

    imshow("dst3",dst3);

    waitKey(0);

    return 0;
}



void test_getperspective(){
    vector<Point2f>  original_pts,another_pts;

//    original_pts.push_back(Point2f(0,0));  //book在图1中的坐标
//    original_pts.push_back(Point2f(2,0));
//    original_pts.push_back(Point2f(0,1));
//    original_pts.push_back(Point2f(2,1));


//    another_pts.push_back(Point2f(758,2826)); //在图2中的坐标
//    another_pts.push_back(Point2f(2706,2830));
//    another_pts.push_back(Point2f(514,3826));
//    another_pts.push_back(Point2f(3002,3830));

//     Mat perspec_pict=imread("/home/gumh/Pictures/floor2.jpg");


    original_pts.push_back(Point2f(0,0));
    original_pts.push_back(Point2f(2,0));
    original_pts.push_back(Point2f(0,1));
    original_pts.push_back(Point2f(2,1));


    another_pts.push_back(Point2f(820,775));
    another_pts.push_back(Point2f(1243,775));
    another_pts.push_back(Point2f(577,986));
    another_pts.push_back(Point2f(1785,986));

    Mat perspec_pict=imread("/home/gumh/Videos/hw2/image-443.jpg");


     for(int i=0;i<4;i++){
         circle(perspec_pict,another_pts[i],20,Scalar(255,0,0),2);
     }


    Mat H=getPerspectiveTransform(original_pts,another_pts);
    cout<<"H="<<H<<endl;
    H.at<double>(2,2)=10;


    Mat dst;
    warpPerspective(perspec_pict,dst,H,Size(perspec_pict.cols,perspec_pict.rows),WARP_INVERSE_MAP|INTER_LINEAR);









    namedWindow("ori1",2);
    namedWindow("dst1",2);

    imshow("ori1",perspec_pict);
    imshow("dst1",dst);

    double Z=H.at<double>(2,2);

    int key=waitKey(0);
    while(key != 27) {


        key = cvWaitKey();
        if(key == 'u') Z *= 1.05;
        if(key == 'd') Z /= 1.05;


        H.at<double>(2,2)=Z;

        cout<<"H="<<Mat(H)<<endl;
        warpPerspective(perspec_pict,dst,H,Size(perspec_pict.cols,perspec_pict.rows),WARP_INVERSE_MAP|INTER_LINEAR);


        imshow("dst1",dst);

        waitKey(0);

    }




}

int main(int argc,char** av){
    test_getperspective();
}
