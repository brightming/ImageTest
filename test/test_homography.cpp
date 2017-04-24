#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;
int test_homogoraphy(){

    vector<Point2f>  obj_pts,image_pts;

    obj_pts.push_back(Point2f(1142,400));  //book在图1中的坐标
    obj_pts.push_back(Point2f(2832,764));
    obj_pts.push_back(Point2f(2300,3252));
    obj_pts.push_back(Point2f(320,2676));


    image_pts.push_back(Point2f(1833,281)); //在图2中的坐标
    image_pts.push_back(Point2f(2584,1303));
    image_pts.push_back(Point2f(924,2218));
    image_pts.push_back(Point2f(163,960));



    Mat H = findHomography(obj_pts, image_pts, RANSAC, 4);
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
    vector<Point2f>  obj_pts,image_pts;

//    obj_pts.push_back(Point2f(0,0));  //book在图1中的坐标
//    obj_pts.push_back(Point2f(2,0));
//    obj_pts.push_back(Point2f(0,1));
//    obj_pts.push_back(Point2f(2,1));


//    image_pts.push_back(Point2f(758,2826)); //在图2中的坐标
//    image_pts.push_back(Point2f(2706,2830));
//    image_pts.push_back(Point2f(514,3826));
//    image_pts.push_back(Point2f(3002,3830));

//     Mat perspec_pict=imread("/home/gumh/Pictures/floor2.jpg");


    obj_pts.push_back(Point2f(0,0));
    obj_pts.push_back(Point2f(21,0));
    obj_pts.push_back(Point2f(21,29));
    obj_pts.push_back(Point2f(0,29));


    image_pts.push_back(Point2f(396,403));
    image_pts.push_back(Point2f(459,408));
    image_pts.push_back(Point2f(459,446));
    image_pts.push_back(Point2f(385,437));

    string filename="/home/gumh/Videos/chessboard/intrinsic.yml";
    Mat intrinsic_mat,distort_mat;
    FileStorage fs(filename,FileStorage::READ);
    fs["camera_matrix"] >> intrinsic_mat;
    fs["distortion_coefficients"] >> distort_mat;
    fs.release();
    cout<<intrinsic_mat<<endl;
    cout<<distort_mat<<endl;

    Mat perspec_pict=imread("/home/gumh/Videos/chessboard/test2.jpg");
//    cv::initUndistortRectifyMap(intrinsic_mat,distort_mat,perspec_pict,
    cv::Mat undist_mat;
    cv::undistort(perspec_pict,undist_mat,intrinsic_mat,distort_mat);
    imshow("1",perspec_pict);


//    imwrite("/home/gumh/Videos/chessboard/test2-undist.jpg",undist_mat);


     for(int i=0;i<4;i++){
         circle(undist_mat,image_pts[i],20,Scalar(255,0,0),2);
     }



    Mat H=getPerspectiveTransform(obj_pts,image_pts);
    cout<<"H="<<H<<endl;
    H.at<double>(2,2)=10;


    Mat dst;
    warpPerspective(perspec_pict,dst,H,Size(perspec_pict.cols,perspec_pict.rows),WARP_INVERSE_MAP|INTER_LINEAR);

    namedWindow("ori1",2);
    namedWindow("dst1",2);

    imshow("ori1",undist_mat);
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
