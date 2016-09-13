
#include"opencv2/opencv.hpp"
#include <iostream>
#include <math.h>
using namespace cv;
using namespace std;


void te1(){
    Mat img=imread("/home/gumh/feirui/20160907/stereo/left-realdist-150.000000-realangle-10.000000-1842095262.jpeg");
    Point2f p1(451,958);
    Point2f p2( 472,1015);
    Point2f p3(1144,847);
    Point2f p4(1145,791);

    vector<Point2f> pts;
    pts.push_back(p1);
    pts.push_back(p2);
    pts.push_back(p3);
    pts.push_back(p4);


    Point2f tr[4], center;

    RotatedRect box = minAreaRect(Mat(pts));//点集的最小外接旋转矩形
    box.points(tr);
    //    for (int i = 0; i < 4; i++)
    //    {
    //        line(img, tr[i], tr[(i + 1) % 4], Scalar(0, 255, 0), 1, CV_AA);
    //    }


    //-----------------------------//
    RNG rng=theRNG();
    float pi=3.1415926;
    Mat draw=img.clone();
    int start_y=905;
    for(;start_y>700;start_y-=10){
        for(float theta=0;theta<20;theta+=2){
            int  count = rng.uniform(1, 101);//uniform()返回指定范围的随机数
            float cs=cos(theta*pi/180);
            float len=750.0/cs;


            RotatedRect rRect = RotatedRect(Point2f(814,start_y), Size2f((int)len,50), -theta);
            Point2f vertices[4];
            rRect.points(vertices);
            for (int i = 0; i < 4; i++)
                line(draw, vertices[i], vertices[(i+1)%4], Scalar(0,255,count));


            cout<<"theta="<<theta<<",cs="<<cs<<",len="<<len<<",rotatedrect.angle="<<rRect.angle<<endl;
            Mat M, rotated, cropped;
            float angle = rRect.angle;
            Size rect_size = rRect.size;


            // get the rotation matrix
            M = getRotationMatrix2D(rRect.center, angle, 1.0);
            cout<<M<<endl;
            // perform the affine transformation
            warpAffine(img, rotated, M, img.size(), INTER_CUBIC);
//            imshow("rotated",rotated);
            // crop the resulting image
            getRectSubPix(rotated, rect_size, rRect.center, cropped);
            imshow("r1",cropped);
            waitKey(0);


#if 1
//截取小部分进行旋转
            Rect out=rRect.boundingRect();
            Mat out_img=img(out);
            imshow("out",out_img);

            Point2f out_box_center(out.width/2,out.height/2);

            int in_large_width=out.width*1.2;
            int in_large_height=out.height*1.2;
            int in_large_x=out.x-(in_large_width-out.width)/2;
            int in_large_y=out.y-(in_large_height-out.height)/2;
            if(in_large_x<0){
                in_large_x=0;
            }
            if(in_large_y<0){
                in_large_y=0;
            }
            if(in_large_x+in_large_width>=img.cols){
                in_large_width=img.cols-in_large_x;
            }
            if(in_large_y+in_large_height>=img.rows){
                in_large_height=img.rows-in_large_y;
            }

            cout<<"out.x="<<out.x<<",out.y="<<out.y<<",out.width="<<out.width<<",out.height="<<out.height<<endl;
            cout<<"in_large_x="<<in_large_x<<",in_large_y="<<in_large_y<<",in_large_width="<<in_large_width<<",in_large_height="<<in_large_height<<endl;

            Rect in_large_rect(in_large_x,in_large_y,in_large_width,in_large_height);
            Mat in_large=img(in_large_rect);
            imshow("myinlarge",in_large);

            Point2f new_center(rRect.center.x-in_large_x,rRect.center.y-in_large_y);

            M = getRotationMatrix2D(new_center, angle, 1.0);
            warpAffine(in_large, rotated, M, in_large.size(), INTER_CUBIC);
            imshow("rotated",rotated);
            getRectSubPix(rotated, rect_size, new_center, cropped);

            imshow("r",cropped);
            waitKey(0);
#endif
        }
    }



    imshow("t1",img);
    waitKey(0);
}

int show_random_min_rect()
{
    Mat img(500, 500, CV_8UC3);
    RNG rng = theRNG();//随机数类

    for (;;)
    {
        int  count = rng.uniform(1, 101);//uniform()返回指定范围的随机数
        vector<Point>points;
        for (int i = 0; i < count; i++)
        {
            Point pt;
            pt.x = rng.uniform(img.cols / 4, img.cols * 3 / 4);
            pt.y = rng.uniform(img.rows / 4, img.rows * 3 / 4);
            points.push_back(pt);
        }
        RotatedRect box = minAreaRect(Mat(points));//点集的最小外接旋转矩形
        Point2f tr[4], center;
        float radius=0;
        box.points(tr);
        minEnclosingCircle(Mat(points), center, radius);//点集的最小外接圆
        img = Scalar::all(0);
        for (int i = 0; i < count; i++)
        {
            circle(img, points[i], 3, Scalar(0, 0, 255), CV_FILLED, CV_AA);
        }
        for (int i = 0; i < 4; i++)
        {
            line(img, tr[i], tr[(i + 1) % 4], Scalar(0, 255, 0), 1, CV_AA);
        }
        circle(img, center, cvRound(radius), Scalar(0, 255, 255), 1, CV_AA);
        imshow("j", img);

        char key = (char)waitKey();
        if (key == 27 || key == 'q' || key == 'Q')
            break;

    }
    return 0;
}

int main(int argc,char* argv[]){
    te1();
}
