#include <iostream>
#include <opencv2/opencv.hpp>
#include <stdlib.h>
#include <stdio.h>

#include "str_common.h"

#include "distance/lab_color_detect.h"
using namespace cv;

cv::Mat org,label_pic,dst,img,tmp,fill_pic;

Mat mask,draw;
Mat gray;

//CV_EVENT_MOUSEMOVE      =0,
//CV_EVENT_LBUTTONDOWN    =1,
//CV_EVENT_RBUTTONDOWN    =2,
//CV_EVENT_MBUTTONDOWN    =3,
//CV_EVENT_LBUTTONUP      =4,
//CV_EVENT_RBUTTONUP      =5,
//CV_EVENT_MBUTTONUP      =6,
//CV_EVENT_LBUTTONDBLCLK  =7,
//CV_EVENT_RBUTTONDBLCLK  =8,
//CV_EVENT_MBUTTONDBLCLK  =9


bool is_btn_down=false;
void try_polyfit();


/**
 * @brief on_mouse
 *
 * 在img图片上画曲线，将会调用opencv的polyfit进行曲线拟合，然后在polyfit中展现
 *
 * @param event
 * @param x
 * @param y
 * @param flags
 * @param ustc
 */
void on_mouse(int event,int x,int y,int flags,void *ustc)//event鼠标事件代号，x,y鼠标坐标，flags拖拽和键盘操作的代号
{
   static Point pre_pt (-1,-1);//初始坐标
   static Point cur_pt(-1,-1);//实时坐标
   char temp[16];
   if(event == CV_EVENT_RBUTTONDOWN){
       mask-=mask;
       gray-=gray;
       draw-=draw;
   }
   if (event == CV_EVENT_LBUTTONDOWN)//左键按下，读取初始坐标，并在图像上该点处划圆
   {
//       std::cout<<"CV_EVENT_LBUTTONDOWN,x="<<x<<",y="<<y<<std::endl;
       is_btn_down=true;
       pre_pt.x=x;
       pre_pt.y=y;
       gray.at<uchar>(y,x)=255;
   }else if(event == CV_EVENT_LBUTTONUP){
       is_btn_down=false;


   }else if(event == CV_EVENT_MOUSEMOVE){
       if(is_btn_down){
//           std::cout<<"CV_EVENT_MOUSEMOVE,line from ("<<pre_pt.x<<","<<pre_pt.y<<")->("<<x<<","<<y<<")"<<std::endl;
           line(mask,pre_pt,Point(x,y),Scalar(255,0,0),2);
           pre_pt.x=x;
           pre_pt.y=y;

           gray.at<uchar>(y,x)=255;
       }
   }else if(event == CV_EVENT_LBUTTONDBLCLK){
       try_polyfit();
   }

   imshow("img",mask);
   imshow("gray",gray);
   imshow("polyfit",draw);
}


void try_polyfit(){

    //搜索两条线

    vector<double> left_x;
    vector<double> left_y;

    vector<double> right_x;
    vector<double> right_y;



//    draw=mask.clone();
//    cvtColor(mask,gray,CV_BGR2GRAY);

    imshow("gray",gray);
    int left_cc=0,right_cc=0;
    for(int i=0;i<mask.rows;i=i+10){
        line(draw,Point(0,i),Point(mask.cols-1,i),Scalar(255,0,0),1);
        int cnt=0;
        for(int j=0;j<mask.cols;j++){
            if(gray.at<uchar>(j,i)>0){
//                circle(draw,Point(i,j),2,Scalar(255,255,0),1);
                if(cnt==0){
                    left_x.push_back(j);
                    left_y.push_back(i);

                    circle(draw,Point(left_y[left_cc],left_x[left_cc]),2,Scalar(255,255,255),2);

                    left_cc++;
                    cnt++;
                    j=j+10;
                }else{
                    right_x.push_back(j);
                    right_y.push_back(i);

                    std::cout<<"right  side......."<<std::endl;

                    circle(draw,Point(right_y[right_cc],right_x[right_cc]),5,Scalar(0,0,255),2);

                    right_cc++;

                    cnt++;
                    break;
                }

            }
        }
        if(cnt==1){
            std::cout<<"line "<<i<<" find only 1 point!"<<std::endl;
            line(draw,Point(0,i),Point(mask.cols-1,i),Scalar(255,0,0),2);
        }else if(cnt==0){

        }
    }

    int order=3;
    Mat dst=Mat::zeros(order+1,1,CV_32FC1);

    Mat input_x(left_x.size(),1,CV_32FC1);
    Mat input_y(left_y.size(),1,CV_32FC1);
    Mat pred_y(left_x.size(),1,CV_32FC1);

    for(int i=0;i<left_x.size();i++){
        input_x.at<float>(i,0)=left_x[i];
        input_y.at<float>(i,0)=left_y[i];
    }

    polyfit(input_x,input_y,dst,order);
    std::cout<<"input_x="<<input_x<<std::endl;
    std::cout<<"input_y="<<input_y<<std::endl;

    for(int i=0;i<left_x.size();i++){
        float y=0;
        for(int j=0;j<=order;j++){
            y+=dst.at<float>(0,j)*pow(left_x[i],j);
        }
        std::cout<<"input_x="<<left_x[i]<<",predict y="<<y<<std::endl;

        circle(draw,Point(y,left_x[i]),3,Scalar(255,0,255),2);
    }


}


/**
 * @brief main
 * 点击原图，显示其label图相应位置的灰度值
 * @param argc
 * @param argv
 * @return
 */
int main(int argc,char* argv[]){

    mask=Mat::zeros(500,500,CV_8UC3);
    draw=mask.clone();
    gray=Mat::zeros(500,500,CV_8UC1);
    namedWindow("img");//定义一个img窗口
    imshow("img",mask);
    setMouseCallback("img",on_mouse,0);//调用回调函数
    cv::waitKey(0);

    imwrite("/home/gumh/Videos/curve.png",mask);

//    try_polyfit();
//    imshow("draw",draw);
//    cv::waitKey(0);
}
