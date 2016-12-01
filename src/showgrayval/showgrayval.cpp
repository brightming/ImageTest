#include <iostream>
#include <opencv2/opencv.hpp>
#include <stdlib.h>
#include <stdio.h>

#include "str_common.h"

#include "distance/lab_color_detect.h"
using namespace cv;

cv::Mat org,label_pic,dst,img,tmp,fill_pic;

Mat mask;


class MorphoFeatures{
    private:
        cv::Mat cross;
        cv::Mat diamond;
        cv::Mat square;
        cv::Mat x;

    public:
        cv::Mat getEdges(const cv::Mat &image){
            cv::Mat result;
            cv::morphologyEx(image,result,cv::MORPH_GRADIENT,cv::Mat());
            applyThreshold(result);
            return result;
        }
        void applyThreshold(cv::Mat & result){
            cv::threshold(result,result, 40,255,cv::THRESH_BINARY);
        }
};

MorphoFeatures morpho;

void on_mouse(int event,int x,int y,int flags,void *ustc)//event鼠标事件代号，x,y鼠标坐标，flags拖拽和键盘操作的代号
{
   static Point pre_pt (-1,-1);//初始坐标
   static Point cur_pt(-1,-1);//实时坐标
   char temp[16];
   if (event == CV_EVENT_LBUTTONDOWN)//左键按下，读取初始坐标，并在图像上该点处划圆
   {
       org.copyTo(img);
       org.copyTo(fill_pic);//将原始图片复制到img中
       sprintf(temp,"(%d,%d)",x,y);
       pre_pt = Point(x,y);

       //print color
       Vec3b col=img.at<Vec3b>(y,x);
       std::cout<<"(r,g,b)=("<<(int)col[2]<<","<<(int)col[1]<<","<<(int)col[0]<<")"<<std::endl;
//       putText(fill_pic,temp,pre_pt,FONT_HERSHEY_SIMPLEX,0.5,Scalar(0,0,0,255),1,8);//在窗口上显示坐标
       circle(img,pre_pt,2,Scalar(255,0,0,0),CV_FILLED,CV_AA,0);//划圆

       //flood fill
       mask.create(org.rows, org.cols, CV_8UC1);
       mask = morpho.getEdges(org);
       cvtColor(mask, mask, COLOR_BGR2GRAY);
       cv::resize(mask,mask,Size(org.cols+2,org.rows+2));


       int loDiff = 20, upDiff = 20;
       int newMaskVal = 255;
       Scalar newVal = Scalar(255, 0, 0);
       Rect ccomp;
       int lo = loDiff;
       int up = upDiff;
       int flags = 8 + (newMaskVal << 8) + CV_FLOODFILL_FIXED_RANGE;


       floodFill(fill_pic, mask, pre_pt, newVal, &ccomp, Scalar(lo, lo, lo), Scalar(up, up, up), flags);

       //显示同色渲染的情况
       imshow("fill",fill_pic);

       //显示点击的位置
       imshow("img",img);
       //查看label_pic的颜色
//       std::cout<<"(x,y)=("<<x<<","<<y<<"),gray val="<<(int)(label_pic.at<char>(y,x))<<std::endl;
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

    string pic_file="/home/gumh/TrainData/Camvid/Camseq01/0016E5_07959.png";
    string pic_label="/home/gumh/TrainData/Camvid/Camseq01/0016E5_07959_L.png";

    org = cv::imread(pic_file);
    label_pic=cv::imread(pic_label,CV_LOAD_IMAGE_GRAYSCALE);

//    cv::resize(org,org,Size(label_pic.cols,label_pic.rows));

    std::cout<<"label_pic.channels="<<label_pic.channels()<<std::endl;

    org.copyTo(img);
    org.copyTo(tmp);
    org.copyTo(fill_pic);




    mask.create(org.rows, org.cols, CV_8UC1);
    mask = morpho.getEdges(org);
    cvtColor(mask, mask, COLOR_BGR2GRAY);
    cv::resize(mask,mask,Size(org.cols+2,org.rows+2));
    imshow("edge",mask);
    namedWindow("img");//定义一个img窗口
//    namedWindow("label");//定义一个label窗口
    namedWindow("fill");//定义一个fill窗口
    setMouseCallback("img",on_mouse,0);//调用回调函数
    imshow("img",img);
//    imshow("label",label_pic);
    imshow("fill",fill_pic);
    cv::waitKey(0);
}
