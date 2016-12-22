/*
 * Project:  Inverse Perspective Mapping
 *
 * File:     main.cpp
 *
 * Contents: Creation, initialisation and usage of IPM object
 *           for the generation of Inverse Perspective Mappings of images or videos
 *
 * Author:   Marcos Nieto <marcos.nieto.doncel@gmail.com>
 * Date:	 22/02/2014
 * Homepage: http://marcosnietoblog.wordpress.com/
 */

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <stdio.h>
#include <ctime>

#include "ipm/IPM.h"
#include "str_common.h"

using namespace cv;
using namespace std;

int old( int _argc, char** _argv )
{
    // Images
    Mat inputImg, inputImgGray;
    Mat outputImg;

    if( _argc != 2 )
    {
        cout << "Usage: ipm.exe <videofile>" << endl;
        return 1;
    }

    // Video
    string videoFileName = _argv[1];
    cv::VideoCapture video;
    if( !video.open(videoFileName) )
        return 1;

    // Show video information
    int width = 0, height = 0, fps = 0, fourcc = 0;
    width = static_cast<int>(video.get(CV_CAP_PROP_FRAME_WIDTH));
    height = static_cast<int>(video.get(CV_CAP_PROP_FRAME_HEIGHT));
    fps = static_cast<int>(video.get(CV_CAP_PROP_FPS));
    fourcc = static_cast<int>(video.get(CV_CAP_PROP_FOURCC));

    cout << "Input video: (" << width << "x" << height << ") at " << fps << ", fourcc = " << fourcc << endl;

    // The 4-points at the input image
    vector<Point2f> origPoints;
    origPoints.push_back( Point2f(0, height) );
    origPoints.push_back( Point2f(width, height) );
    origPoints.push_back( Point2f(width/2+30, 140) );
    origPoints.push_back( Point2f(width/2-50, 140) );

    // The 4-points correspondences in the destination image
    vector<Point2f> dstPoints;
    dstPoints.push_back( Point2f(0, height) );
    dstPoints.push_back( Point2f(width, height) );
    dstPoints.push_back( Point2f(width, 0) );
    dstPoints.push_back( Point2f(0, 0) );

    // IPM object
    IPM ipm( Size(width, height), Size(width, height), origPoints, dstPoints );

    // Main loop
    int frameNum = 0;
    for( ; ; )
    {
        printf("FRAME #%6d ", frameNum);
        fflush(stdout);
        frameNum++;

        // Get current image
        video >> inputImg;
        if( inputImg.empty() )
            break;

        // Color Conversion
        if(inputImg.channels() == 3)
            cvtColor(inputImg, inputImgGray, CV_BGR2GRAY);
        else
            inputImg.copyTo(inputImgGray);

        // Process
        clock_t begin = clock();
        ipm.applyHomography( inputImg, outputImg );
        clock_t end = clock();
        double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
        printf("%.2f (ms)\r", 1000*elapsed_secs);
        ipm.drawPoints(origPoints, inputImg );

        // View
        imshow("Input", inputImg);
        imshow("Output", outputImg);
        waitKey(1);
    }

    return 0;
}		



void test_template_match(Mat& src){
    float k[10] = { 0.0, 255.0, 255.0, 255.0,0.0,
                    0.0, 255.0, 255.0, 255.0, 0.0 };  //核
    Mat km = Mat( 2, 5, CV_32FC1, k );  //构造单通道浮点矩阵，将图像IplImage结构转换为图像数组


    int dst_cols=src.cols-km.cols+1;
    int dst_rows=src.rows-km.rows+1;

    cout<<"src.cols="<<src.cols<<",km.cols="<<km.cols<<",,dst_cols="<<dst_cols<<",dst_rows="<<dst_rows<<endl;
    Mat dst=Mat::zeros(dst_cols,dst_rows,CV_32FC1);
    Mat for_match=src.clone();
    for_match.convertTo(for_match,CV_32F);
    cvtColor(for_match,for_match,CV_BGR2GRAY);


    matchTemplate(for_match,km,dst,  CV_TM_CCORR_NORMED);

//    normalize( dst, dst, 0, 1, NORM_MINMAX, -1, Mat() );

    double minVal; double maxVal; Point minLoc; Point maxLoc;
    Point matchLoc;

    minMaxLoc( dst, &minVal, &maxVal, &minLoc, &maxLoc, Mat() );

    cout<<"minVal="<<minVal<<",maxVal="<<maxVal<<endl;

    matchLoc=maxLoc;

//    rectangle( img_display, matchLoc, Point( matchLoc.x + templ.cols , matchLoc.y + templ.rows ), Scalar::all(0), 2, 8, 0 );
    rectangle( dst, matchLoc, Point( matchLoc.x + km.cols , matchLoc.y + km.rows ), Scalar::all(0), 2, 8, 0 );





    Mat for_show=dst;
//    dst.convertTo(for_show,CV_8U);

    for(int r=0;r<for_show.rows;r++){
        for(int c=0;c<for_show.cols;c++){
            if(for_show.at<float>(r,c)>0.75){
                for_show.at<float>(r,c)=255;
            }else{
                for_show.at<float>(r,c)=0;
            }
        }
    }

    imshow("dst_image",for_show);

    imwrite("/home/gumh/tmp/match.png",for_show);

}


#include "math/img_math.h"
Mat test_ipm(Mat& inputImg){
    vector<Point2f>  original_pts,bird_pts;

//    original_pts.push_back(Point2f(250,1080));
//    original_pts.push_back(Point2f(544,776));
//    original_pts.push_back(Point2f(1296,776));
//    original_pts.push_back(Point2f(1652,1080));


//    bird_pts.push_back(Point2f(250,1080));
//    bird_pts.push_back(Point2f(250,776));
//    bird_pts.push_back(Point2f(1652,776));
//    bird_pts.push_back(Point2f(1652,1080));





    bird_pts.push_back(Point2f(0,0));
    bird_pts.push_back(Point2f(1,0));
    bird_pts.push_back(Point2f(1,1));
    bird_pts.push_back(Point2f(0,1));


    original_pts.push_back(Point2f(820,775));
    original_pts.push_back(Point2f(1223,775));
    original_pts.push_back(Point2f(1730,986));
    original_pts.push_back(Point2f(577,986));


//    vector<double> x;
//    vector<double> y;
//    x.push_back(250);
//    x.push_back(544);
//    y.push_back(1080);
//    y.push_back(776);
//    LeastSquare lsq_l(x,y);

//    double y2=0;
//    double x2=lsq_l.getX(0);
//    line(inputImg,Point(544,776),Point(x2,y2),Scalar(255,0,0),1);


//    vector<double> xr;
//    vector<double> yr;
//    xr.push_back(1296);
//    xr.push_back(1652);
//    yr.push_back(776);
//    yr.push_back(1080);
//    LeastSquare lsq_r(xr,yr);

//    double y2r=0;
//    double x2r=lsq_r.getX(y2r);

//    line(inputImg,Point(1296,776),Point(x2r,y2r),Scalar(255,0,0),1);


//    double y_h=1080-2*(1080-776);
//   line(inputImg,Point(0,y_h),Point(1920,y_h),Scalar(255,2,0),1);

//   double xl_h=lsq_l.getX(y_h);
//   double xr_h=lsq_r.getX(y_h);
//   cout<<"y_h="<<y_h<<",xl_h="<<xl_h<<",xr_h="<<xr_h<<endl;


    Mat outputImg;


    // IPM object
    int width=inputImg.cols;
    int height=inputImg.rows;


//    IPM ipm( Size(width, height), Size(width, height), original_pts, bird_pts );
//    ipm.applyHomography( inputImg, outputImg );

//    ipm.drawPoints(original_pts, inputImg );


    Mat H,H2;
//    H=findHomography(original_pts,bird_pts,CV_RANSAC);
    H2=getPerspectiveTransform(bird_pts,original_pts);
//    cout<<"H="<<H<<endl;
    cout<<"H2="<<H2<<endl;
    H2.at<double>(2,2)=10;
    warpPerspective(inputImg,outputImg,H2,Size(width,height),CV_INTER_LINEAR+CV_WARP_INVERSE_MAP+CV_WARP_FILL_OUTLIERS);

//    cout<<"outputImg.width="<<outputImg.cols<<",outputImg.height="<<outputImg.rows<<endl;

//    Mat inverse_pict;
////    warpPerspective(outputImg,inverse_pict,H2,Size(width,height),CV_INTER_LINEAR+CV_WARP_INVERSE_MAP+CV_WARP_FILL_OUTLIERS);



//    warpPerspective(outputImg,inverse_pict,H.inv(),Size(width,height));
//    std::vector<Point2f> ground_corners(4);
//    ground_corners.push_back(Point(0,0));
//    ground_corners.push_back(Point(1920,0));
//    ground_corners.push_back(Point(0,1080));
//    ground_corners.push_back(Point(1920,1080));


//    std::vector<Point2f> scene_corners(4);
//    perspectiveTransform(ground_corners,scene_corners,H);

//    for(int i=0;i<4;i++){
//        cout<<"scene_pt["<<i<<"]="<<scene_corners[i]<<endl;
//    }

//    circle(inputImg,scene_corners[0],10,Scalar(255,0,0),2);
//    circle(inputImg,scene_corners[1],10,Scalar(255,0,0),2);
//    circle(inputImg,scene_corners[2],10,Scalar(255,0,0),2);
//    circle(inputImg,scene_corners[3],10,Scalar(255,0,0),2);


    // View
    imshow("Input", inputImg);
    imshow("Output", outputImg);
//    imshow("invert",inverse_pict);

    return outputImg;

}






int main(){
    namedWindow("Input",2);
    namedWindow("Output",2);
    namedWindow("dst_image",2);
    namedWindow("gray",2);

    string path="/home/gumh/Videos/hw2";
    vector<string> all_pics=getAllFilesWithPathFromDir(path);
    std::sort(all_pics.begin(),all_pics.end(),less<string>());

    int idx=0;
    while(1)
    {
        if(idx>=all_pics.size()){
            idx=0;
        }


        Mat inputImg=imread(all_pics[idx]);
        Mat ipm_pict=test_ipm(inputImg);

//        test_template_match(ipm_pict);


        int key=waitKey(33);
        if(key==32){ //enter space to pause
            key=waitKey(0);
        }
        else if(key==27 || (char)key=='q'){
            break;
        }

        idx++;

    }



}


#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

struct userdata{
    Mat im;
    vector<Point2f> points;
};


void mouseHandler(int event, int x, int y, int flags, void* data_ptr)
{
    if  ( event == EVENT_LBUTTONDOWN )
    {
        userdata *data = ((userdata *) data_ptr);
        circle(data->im, Point(x,y),3,Scalar(0,0,255), 5, CV_AA);
        imshow("Image", data->im);
        if (data->points.size() < 4)
        {
            data->points.push_back(Point2f(x,y));
        }
    }

}



int test_homography_book( int argc, char** argv)
{

    // Read source image.
    Mat im_src = imread("/home/gumh/Videos/book1.jpg");

    // Destination image. The aspect ratio of the book is 3/4
    Size size(300,400);
    Mat im_dst = Mat::zeros(size,CV_8UC3);


    // Create a vector of destination points.
    vector<Point2f> pts_dst;

    pts_dst.push_back(Point2f(0,0));
    pts_dst.push_back(Point2f(size.width - 1, 0));
    pts_dst.push_back(Point2f(size.width - 1, size.height -1));
    pts_dst.push_back(Point2f(0, size.height - 1 ));

    // Set data for mouse event
    Mat im_temp = im_src.clone();
    userdata data;
    data.im = im_temp;

    cout << "Click on the four corners of the book -- top left first and" << endl
    << "bottom left last -- and then hit ENTER" << endl;

    // Show image and wait for 4 clicks.
    imshow("Image", im_temp);
    // Set the callback function for any mouse event
    setMouseCallback("Image", mouseHandler, &data);
    waitKey(0);

    // Calculate the homography
    Mat h = findHomography(data.points, pts_dst);

    // Warp source image to destination
    warpPerspective(im_src, im_dst, h, size);

    // Show image
    imshow("Image", im_dst);
    waitKey(0);

    return 0;
}



#include <highgui.h>
#include <cv.h>
#include <cxcore.h>
#include <math.h>
#include <vector>
#include <stdio.h>

#include <iostream>

using namespace cv;
using namespace std;

int main2(int argc, char* argv[]) {

//if(argc != 4) return -1;
    // INPUT PARAMETERS:
    //
    int board_w = 3;//atoi(argv[1]); //inner corners per row
    int board_h = 4;//toi(argv[2]); //inner corners per column
    int board_n = board_w * board_h;
    CvSize board_sz = cvSize( board_w, board_h );

    //Hard coded intrinsics for the camera
    Mat intrinsicMat = (Mat_<double>(3, 3) <<
        418.7490, 0., 236.8528,
        0.,558.6650,322.7346,
        0., 0., 1.);

    //Hard coded distortions for the camera
    CvMat* distortion = cvCreateMat(1, 4, CV_32F);
    cvmSet(distortion, 0, 0, -0.0019);
    cvmSet(distortion, 0, 1, 0.0161);
    cvmSet(distortion, 0, 2, 0.0011);
    cvmSet(distortion, 0, 3, -0.0016);

    IplImage* image = 0;
    IplImage* gray_image = 0;

    char* path="/home/gumh/Videos/chess.png";
    if( (image = cvLoadImage(path)) == 0 ) {
        printf("Error: Couldn’t load %s\n",path);
        return -1;
    }

    gray_image = cvCreateImage( cvGetSize(image), 8, 1 );
    cvCvtColor(image, gray_image, CV_BGR2GRAY );
    // UNDISTORT OUR IMAGE
    //
    IplImage* mapx = cvCreateImage( cvGetSize(image), IPL_DEPTH_32F, 1 );
    IplImage* mapy = cvCreateImage( cvGetSize(image), IPL_DEPTH_32F, 1 );

    CvMat intrinsic (intrinsicMat);

    //This initializes rectification matrices
    //
    cvInitUndistortMap(
        &intrinsic,
        distortion,
        mapx,
        mapy
    );


    IplImage *t = cvCloneImage(image);
    // Rectify our image
    //

    cvRemap( t, image, mapx, mapy );
    // GET THE CHESSBOARD ON THE PLANE
    //

    cvNamedWindow("Chessboard");
    CvPoint2D32f* corners = new CvPoint2D32f[ board_n ];
    int corner_count = 0;
    int found = cvFindChessboardCorners(
        image,
        board_sz,
        corners,
        &corner_count,
        CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS
    );
    if(!found){
        printf("Couldn’t aquire chessboard on %s, "
        "only found %d of %d corners\n",
        argv[3],corner_count,board_n
    );
    return -1;
    }
    //Get Subpixel accuracy on those corners:
    cvFindCornerSubPix(
        gray_image,
        corners,
        corner_count,
        cvSize(11,11),
        cvSize(-1,-1),
        cvTermCriteria( CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1 )
    );

    //GET THE IMAGE AND OBJECT POINTS:
    // We will choose chessboard object points as (r,c):
    // (0,0), (board_w-1,0), (0,board_h-1), (board_w-1,board_h-1).
    //




    CvPoint2D32f objPts[4], imgPts[4];
    imgPts[0] = corners[0];
    imgPts[1] = corners[board_w-1];
    imgPts[2] = corners[(board_h-1)*board_w];
    imgPts[3] = corners[(board_h-1)*board_w + board_w-1];

    objPts[0].x = 0; objPts[0].y = 0;
    objPts[1].x = board_w -1; objPts[1].y = 0;
    objPts[2].x = 0; objPts[2].y = board_h -1;
    objPts[3].x = board_w -1; objPts[3].y = board_h -1;


    // DRAW THE POINTS in order: B,G,R,YELLOW
    //
    cvCircle( image, cvPointFrom32f(imgPts[0]), 9, CV_RGB(0,0,255), 3); //blue
    cvCircle( image, cvPointFrom32f(imgPts[1]), 9, CV_RGB(0,255,0), 3); //green
    cvCircle( image, cvPointFrom32f(imgPts[2]), 9, CV_RGB(255,0,0), 3); //red
    cvCircle( image, cvPointFrom32f(imgPts[3]), 9, CV_RGB(255,255,0), 3); //yellow
    // DRAW THE FOUND CHESSBOARD
    //

    cvDrawChessboardCorners(
        image,
        board_sz,
        corners,
        corner_count,
        found
    );
    cvShowImage( "Chessboard", image );
    // FIND THE HOMOGRAPHY
    //
    CvMat *H = cvCreateMat( 3, 3, CV_32F);
    cvGetPerspectiveTransform( objPts, imgPts, H);
    Mat homography = H;
    cvSave("Homography.xml",H); //We can reuse H for the same camera mounting


    //-----------gmh test begin-----//

    vector<Point2f> pts_img;
    vector<Point2f> pts_obj;
    pts_img.push_back(Point2f(imgPts[0].x,imgPts[0].y));
    pts_img.push_back(Point2f(imgPts[1].x,imgPts[1].y));
    pts_img.push_back(Point2f(imgPts[2].x,imgPts[2].y));
    pts_img.push_back(Point2f(imgPts[3].x,imgPts[3].y));


    pts_obj.push_back(Point2f(objPts[0].x,objPts[0].y));
    pts_obj.push_back(Point2f(objPts[1].x,objPts[1].y));
    pts_obj.push_back(Point2f(objPts[2].x,objPts[2].y));
    pts_obj.push_back(Point2f(objPts[3].x,objPts[3].y));

    Mat H3=getPerspectiveTransform(pts_obj,pts_img);//从  物体平面 到 成像平面 的变换
    Mat H4=findHomography(pts_obj,pts_img);
//    Mat H4=getPerspectiveTransform(pts_img,pts_obj);//从 成像平面 到 物体平面 的变换


    cout<<"H3.inv="<<H3.inv()<<endl;
    H3.at<double>(2,2)=23;
//    H4.at<double>(2,2)=24;
    cout<<"H3 2="<<H3<<endl;

    cout<<"H4="<<H4<<endl;
    cout<<"h4.inv="<<H4.inv()<<endl;
    Mat ori_pict=imread(path);
    Mat hogo_pict1,hogo_pict2;
    Mat hogo_pict3,hogo_pict4;

    //将成像平面映射到物体平面,注意两行的参数：H3，和 CV_WARP_INVERSE_MAP
    warpPerspective(ori_pict,hogo_pict1,H3.inv(),Size(ori_pict.cols,ori_pict.rows),CV_INTER_LINEAR /*|CV_WARP_INVERSE_MAP*/| CV_WARP_FILL_OUTLIERS);
    warpPerspective(ori_pict,hogo_pict2,H3,Size(ori_pict.cols,ori_pict.rows),CV_INTER_LINEAR |CV_WARP_INVERSE_MAP| CV_WARP_FILL_OUTLIERS);
    for(int i=0;i<pts_img.size();i++){
        circle(ori_pict,pts_img[i],10*(i+1),Scalar(255,50*i,0),2);
    }

    //将成像平面映射到物体平面
    double v=H4.at<double>(2,2);
    H4.at<double>(2,2)=24;
    warpPerspective(ori_pict,hogo_pict3,H4.inv(),Size(ori_pict.cols,ori_pict.rows),CV_INTER_LINEAR /*|CV_WARP_INVERSE_MAP*/| CV_WARP_FILL_OUTLIERS);


    imshow("ori_pict",ori_pict);
    imshow("hogo1",hogo_pict1);
    imshow("hogo2",hogo_pict2);
    imshow("hogo3",hogo_pict3);

    //-----------gmh test end--------//



    /**********************GENERATING 3X4 MATRIX***************************/

    // LET THE USER ADJUST THE Z HEIGHT OF THE VIEW
    //
    float Z = 1;
    int key = 0;
    IplImage *birds_image = cvCloneImage(image);
    cvNamedWindow("Birds_Eye");
    // LOOP TO ALLOW USER TO PLAY WITH HEIGHT:
    //
    // escape key stops
    //
    while(key != 27) {
        // Set the height
        //
        CV_MAT_ELEM(*H,float,2,2) = Z;
        // COMPUTE THE FRONTAL PARALLEL OR BIRD’S-EYE VIEW:
        // USING HOMOGRAPHY TO REMAP THE VIEW
        //

        cout<<"H="<<Mat(H)<<endl;
        cvWarpPerspective(
                    image,
                    birds_image,
                    H,
                    CV_INTER_LINEAR | CV_WARP_INVERSE_MAP | CV_WARP_FILL_OUTLIERS
                    );
        cvShowImage( "Birds_Eye", birds_image );
        //    imwrite("/home/lee/bird.jpg", birds_image);

        key = cvWaitKey();
        if(key == 'u') Z += 0.5;
        if(key == 'd') Z -= 0.5;
    }
    return 0;
}
