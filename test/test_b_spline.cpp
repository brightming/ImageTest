#include <iostream>
#include "math/b_spline.h"
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;
int basic_test_b_spline(){

    //参考http://www.cs.mtu.edu/~shene/COURSES/cs3621/NOTES/spline/B-spline/bspline-ex-1.html
    float knots_arr[]={ 0, 0.25, 0.5, 0.75, 1};
    int knots_size=sizeof(knots_arr) / sizeof(knots_arr[0]);
    int max_knots_idx=knots_size-1;

    std::vector<float> knots(knots_arr,knots_arr+knots_size);
    int p=1;
    float u=0.6;

    for(int i=0;i<knots.size();i++){
        std::cout<<"i="<<i<<" knots="<<knots[i]<<std::endl;
    }


    int max_control_point_cnt_idx=max_knots_idx-p-1;
    vector<float> coeffis(max_control_point_cnt_idx+1);
    calc_b_spline_coefficient(max_control_point_cnt_idx,p,max_knots_idx,knots_arr,u,coeffis);

    std::cout<<"coefficients of u="<<u<<" p="<<p<<" are as follows:\n";
    for(int i=0;i<=max_control_point_cnt_idx;i++){
        std::cout<<coeffis[i]<<" ";
    }
    std::cout<<"\n";
}



//为了让得到的曲线，经过首尾控制点，对u0,和um进行p次重复，及u0和um共有p+1个。
float knots_arr[]={ 0,0,0,0.2, 0.25, 0.5, 0.6,0.75,0.8, 1,1,1};
int knots_size=sizeof(knots_arr) / sizeof(knots_arr[0]);
int max_knots_idx=knots_size-1;
int p=2;
int max_control_point_cnt_idx=max_knots_idx  - p -1;
vector<float> coeffis(max_control_point_cnt_idx+1);
vector<Point> control_pts;


Mat pict;

//mouse event callback
void mouse_track_event(int event, int x, int y, int flags, void *param )
{

    //左键点击选则控制点
    if (event==CV_EVENT_LBUTTONDOWN) {
        control_pts.push_back(Point(x,y));
        cout<<"click,need control point ="<<max_control_point_cnt_idx+1<<",now has:"<<control_pts.size()<<endl;

        //控制点画圆
        circle(pict,Point(x,y),3,Scalar(255),2);


    }else if (event==CV_EVENT_RBUTTONDOWN){
        //右键单击清空图片
        pict-=pict;
        //清空
        control_pts.resize(0);
    }else if (event==CV_EVENT_LBUTTONDBLCLK){
        //左键单击进行拟合曲线
        //插值，画图
        if(control_pts.size()>=knots_size-p-1){
            //可以进行了

            //控制点画圆
            pict-=pict;
            for(int i=0;i<=max_control_point_cnt_idx;i++){
                circle(pict,control_pts[i],5,Scalar(255),2);
            }
            for(int i=max_control_point_cnt_idx+1;i<control_pts.size();i++){
                circle(pict,control_pts[i],3,Scalar(0,255,0),2);
            }

            //
            for(float u=0;u<1;u=u+0.01){
                calc_b_spline_coefficient(max_control_point_cnt_idx,p,max_knots_idx,knots_arr,u,coeffis);
                //计算
                float u_x=0;
                float u_y=0;
                for(int i=0;i<=max_control_point_cnt_idx;i++){
                    u_x+=coeffis[i]*control_pts[i].x;
                    u_y+=coeffis[i]*control_pts[i].y;
                }
//                cout<<"u="<<u<<",u_x="<<u_x<<",u_y="<<u_y<<endl;
                circle(pict,Point((int)u_x,(int)u_y),1,Scalar(0,0,255));
            }

        }else{
            cout<<"not enough control point~!"<<endl;
        }
    }
    imshow("b-spline",pict);
}


void test_show_b_spline(){

    namedWindow("b-spline");

    pict=Mat::zeros(500,500,CV_8UC3);
    setMouseCallback("b-spline",mouse_track_event);

    waitKey(0);
}


int main(){
//    basic_test_b_spline();
    test_show_b_spline();
}
