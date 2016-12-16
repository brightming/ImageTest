#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <stdio.h>
using namespace cv;
using namespace std;

#include <iostream>

const int winHeight=600;
const int winWidth=800;


Point mousePosition= Point(winWidth>>1,winHeight>>1);

//mouse event callback
void mouse_track_event(int event, int x, int y, int flags, void *param )
{
    if (event==CV_EVENT_MOUSEMOVE) {
        mousePosition = Point(x,y);
    }
}

int test_mouse_track (void)
{
    RNG rng;
    //1.kalman filter setup
    const int stateNum=4;                                      //状态值4×1向量(x,y,△x,△y)
    const int measureNum=2;                                    //测量值2×1向量(x,y)
    KalmanFilter KF(stateNum, measureNum, 0);

    KF.transitionMatrix = *(Mat_<float>(4, 4) <<1,0,1,0,0,1,0,1,0,0,1,0,0,0,0,1);  //转移矩阵A
    setIdentity(KF.measurementMatrix);                                             //测量矩阵H
    setIdentity(KF.processNoiseCov, Scalar::all(1e-5));                            //系统噪声方差矩阵Q
    setIdentity(KF.measurementNoiseCov, Scalar::all(1e-1));                        //测量噪声方差矩阵R
    setIdentity(KF.errorCovPost, Scalar::all(1));                                  //后验错误估计协方差矩阵P
    rng.fill(KF.statePost,RNG::UNIFORM,0,winHeight>winWidth?winWidth:winHeight);   //初始状态值x(0)
    Mat measurement = Mat::zeros(measureNum, 1, CV_32F);                           //初始测量值x'(0)，因为后面要更新这个值，所以必须先定义

    namedWindow("kalman");
    setMouseCallback("kalman",mouse_track_event);

    Mat image(winHeight,winWidth,CV_8UC3,Scalar(0));

    while (1)
    {
        //2.kalman prediction
        Mat prediction = KF.predict();
        Point predict_pt = Point(prediction.at<float>(0),prediction.at<float>(1) );   //预测值(x',y')

        //3.update measurement
        measurement.at<float>(0) = (float)mousePosition.x;
        measurement.at<float>(1) = (float)mousePosition.y;

        //4.update
        KF.correct(measurement);

        //draw
        image.setTo(Scalar(255,255,255,0));
        circle(image,predict_pt,5,Scalar(0,255,0),3);    //predicted point with green
        circle(image,mousePosition,5,Scalar(255,0,0),3); //current position with red

        char buf[256];
        sprintf(buf,"predicted position:(%3d,%3d)",predict_pt.x,predict_pt.y);
        putText(image,buf,Point(10,30),CV_FONT_HERSHEY_SCRIPT_COMPLEX,1,Scalar(0,0,0),1,8);
        sprintf(buf,"current position :(%3d,%3d)",mousePosition.x,mousePosition.y);
        putText(image,buf,cvPoint(10,60),CV_FONT_HERSHEY_SCRIPT_COMPLEX,1,Scalar(0,0,0),1,8);

        imshow("kalman", image);
        int key=waitKey(3);
        if (key==27){//esc
            break;
        }
    }
}


/**
 * @brief test_slope_track
 * y=mx+b
 */
void test_slope_track(){
    RNG rng;
    //1.kalman filter setup
    const int stateNum=4;                                      //状态值4×1向量(m,b,△m,△b)
    const int measureNum=2;                                    //测量值2×1向量(m,b)
    KalmanFilter KF(stateNum, measureNum, 0);

    KF.transitionMatrix = *(Mat_<float>(4, 4) <<1,0,1,0,0,1,0,1,0,0,1,0,0,0,0,1);  //转移矩阵A
    setIdentity(KF.measurementMatrix);                                             //测量矩阵H
    setIdentity(KF.processNoiseCov, Scalar::all(1e-5));                            //系统噪声方差矩阵Q
    setIdentity(KF.measurementNoiseCov, Scalar::all(1e-1));                        //测量噪声方差矩阵R
    setIdentity(KF.errorCovPost, Scalar::all(1));                                  //后验错误估计协方差矩阵P
    rng.fill(KF.statePost,RNG::UNIFORM,0,winHeight>winWidth?winWidth:winHeight);   //初始状态值x(0)
    Mat measurement = Mat::zeros(measureNum, 1, CV_32F);                           //初始测量值x'(0)，因为后面要更新这个值，所以必须先定义


    float initial_m=1;
    float initial_b=50;
    float current_m=initial_m;
    float current_b=initial_b;

    float max_m=1.2;
    float min_m=0.9;
    float max_b=100;
    float min_b=10;

    int m_dir=0;//0:increase 1:decrease
    int b_dir=0;//0:increase 1:decrease

    KF.statePost.at<float>(0)=3;
    KF.statePost.at<float>(1)=20;
    KF.statePost.at<float>(2)=0;
    KF.statePost.at<float>(3)=0;


    measurement.at<float>(0,0)=3;
    measurement.at<float>(1,0)=20;


    namedWindow("kalman");

    Mat image(winHeight,winWidth,CV_8UC3,Scalar(0));

    while (1)
    {
        //2.kalman prediction
        Mat prediction = KF.predict();
        //        Point predict_pt = Point(prediction.at<float>(0),prediction.at<float>(1) );   //预测值(x',y')


        cout<<"prediction="<<prediction<<endl;
        //3.update measurement
        if(m_dir==0){
            current_m*=1.01;
            if(current_m>max_m){
                current_m=max_m;
                m_dir=1;
            }
        }else{
            current_m/=1.01;
            if(current_m<min_m){
                current_m=min_m;
                m_dir=0;
            }
        }
        if(b_dir==0){
            current_b*=1;
            if(current_b>max_b){
                current_b=max_b;
                b_dir=1;
            }
        }else{
            current_b/=1;
            if(current_b<min_b){
                current_b=min_b;
                b_dir=0;
            }
        }

        measurement.at<float>(0) =current_m;
        measurement.at<float>(1) = current_b;

        cout<<"measurement="<<measurement<<endl;

        //4.update
        KF.correct(measurement);

        //draw
        image.setTo(Scalar(255,255,255,0));

        //predict line
        float x1=0;
        float y1=cvRound(prediction.at<float>(0)*x1+prediction.at<float>(1)) ;
        float x2=image.cols/2;
        float y2=cvRound(prediction.at<float>(0)*x2+prediction.at<float>(1)) ;
        line(image,Point(x1,y1),Point(x2,y2),Scalar(0,255,0),8);

        float x1_cr=0;
        float y1_m=cvRound(current_m*x1_cr+current_b) ;
        float x2_cr=image.cols/2;
        float y2_m=cvRound(current_m*x2_cr+current_b) ;
        line(image,Point(x1_cr,y1_m),Point(x2_cr,y2_m),Scalar(255,0,0),2);



                circle(image,Point(x1,y1),5,Scalar(0,255,0),8);    //predicted point with green
                circle(image,Point(x2,y2),5,Scalar(0,255,0),8);    //predicted point with green


                circle(image,Point(x1_cr,y1_m),5,Scalar(255,0,0),3); //current position with red
                circle(image,Point(x2_cr,y2_m),5,Scalar(255,0,0),3); //current position with red


        //        char buf[256];
        //        sprintf(buf,"predicted position:(%3d,%3d)",predict_pt.x,predict_pt.y);
        //        putText(image,buf,Point(10,30),CV_FONT_HERSHEY_SCRIPT_COMPLEX,1,Scalar(0,0,0),1,8);
        //        sprintf(buf,"current position :(%3d,%3d)",mousePosition.x,mousePosition.y);
        //        putText(image,buf,cvPoint(10,60),CV_FONT_HERSHEY_SCRIPT_COMPLEX,1,Scalar(0,0,0),1,8);



        imshow("kalman", image);
        int key=waitKey(500);
        if (key==27){//esc
            break;
        }
    }
}


#include "math/img_math.h"
Mat ori_pict,draw_pict;
Point pts[2];
int click_cnt=0;
void on_slope_mouse(int event,int x,int y,int flags,void *ustc)//event鼠标事件代号，x,y鼠标坐标，flags拖拽和键盘操作的代号
{

    char buf[64];
    static Point pre_pt (-1,-1);//初始坐标
    static Point cur_pt(-1,-1);//实时坐标
    char temp[16];
    if(event == CV_EVENT_RBUTTONDOWN){
        draw_pict-=draw_pict;
    }
    if (event == CV_EVENT_LBUTTONDOWN)//左键按下，读取初始坐标，并在图像上该点处划圆
    {
        //       std::cout<<"CV_EVENT_LBUTTONDOWN,x="<<x<<",y="<<y<<std::endl;
        if(click_cnt==0 || click_cnt==1){
            pts[click_cnt++]=Point(x,y);
            circle(draw_pict,Point(x,y),3,  Scalar(255,0,0),1);

            sprintf(buf,"(%d,%d)",x,y);
            putText(draw_pict,buf,Point(x+10,y),CV_FONT_HERSHEY_SCRIPT_COMPLEX,1,Scalar(0,128,128),1,8);

        }


    }else if(event == CV_EVENT_LBUTTONUP){



    }else if(event == CV_EVENT_LBUTTONDBLCLK){
        if(click_cnt==2){
            int order=1;
            Mat right_dst=Mat::zeros(order+1,1,CV_32FC1);
            Mat right_input_x(2,1,CV_32FC1);
            Mat right_input_y(2,1,CV_32FC1);

            for(int i=0;i<2;i++){
                right_input_x.at<float>(i,0)=pts[i].x;
                right_input_y.at<float>(i,0)=pts[i].y;
            }

            polyfit(right_input_x,right_input_y,right_dst,order);

//            LeastSquare
            sprintf(buf,"m=%f,b=%f",right_dst.at<float>(1),right_dst.at<float>(0));
            putText(draw_pict,buf,Point((pts[0].x+pts[1].x)/2,(pts[0].y+pts[1].y)/2),CV_FONT_HERSHEY_SCRIPT_COMPLEX,0.5,Scalar(0,128,128),1,8);
            line(draw_pict,pts[0],pts[1],Scalar(0,255,0),1);

            click_cnt=0;
        }
    }

    imshow("slope",draw_pict);


}



void test_calc_slope(){
    draw_pict=Mat::zeros(500,500,CV_8UC3);

    namedWindow("slope");
    setMouseCallback("slope",on_slope_mouse);

    waitKey(0);
}

int main(){


//    test_calc_slope();
    test_slope_track();

    return 0;
}
