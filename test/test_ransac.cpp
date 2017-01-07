#include <iostream>
#include <vector>

#include <opencv2/opencv.hpp>

#include "math/img_math.h"


#include "common/ransac.h"
using namespace std;
using namespace cv;


static void getCircle(cv::Point& p1, cv::Point& p2, cv::Point& p3, cv::Point2f& center, float& radius)
{
    float x1 = p1.x, y1 = p1.y;
    float x2 = p2.x, y2 = p2.y;
    float x3 = p3.x, y3 = p3.y;
    center.x = (x1*x1 + y1*y1)*(y2 - y3) + (x2*x2 + y2*y2)*(y3 - y1) + (x3*x3 + y3*y3)*(y1 - y2);
    center.x /= (2 * (x1*(y2 - y3) - y1*(x2 - x3) + x2*y3 - x3*y2));
    center.y = (x1*x1 + y1*y1)*(x3 - x2) + (x2*x2 + y2*y2)*(x1 - x3) + (x3*x3 + y3*y3)*(x2 - x1);
    center.y /= (2 * (x1*(y2 - y3) - y1*(x2 - x3) + x2*y3 - x3*y2));
    radius = sqrt((center.x - x1)*(center.x - x1) + (center.y - y1)*(center.y - y1));
}

static float verifyCircle(cv::Mat& dt, cv::Point2f center, float radius)
{
    // 總採樣點

    unsigned int counter = 0;
    // 落在輪廓上的採樣點計數

    unsigned int inlier = 0;
    // 該採樣點與輪廓的距離落在這個範圍內則認定為在輪廓上

    float InlierDist = 2.0f;
    // 用來判斷採樣點是否超出影像邊界

    Rect dt_rect(0, 0, dt.cols, dt.rows);

    //在假想圓上取64點作驗證，計算假想圓上與輪廓點重疊的比例，比例越高則圓越完整


    float angle_360 = 2 * CV_PI;
    float angle_step = angle_360 / 64;
    for (float t = 0; t < angle_360; t += angle_step)
    {
        counter++;
        cv::Point2f tmp_sampling_p(radius*cos(t) + center.x, radius*sin(t) + center.y);
        if (dt_rect.contains(tmp_sampling_p))
            if (dt.at<float>(tmp_sampling_p) < InlierDist)
                inlier++;
    }
    return float(inlier) / counter;
}

int test_get_circle()
{
    Mat image = cv::imread("/home/gumh/Pictures/circle.jpg");
    Mat edge, gray;
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);

    ///1.找出輪廓點

    cv::Canny(gray, edge, 120, 60);
    std::vector<cv::Point> edge_Points;
    cv::findNonZero(edge, edge_Points);

    ///2.計算distancetransform

    Mat1b mask = (255 - edge);
    Mat dt;
    cv::distanceTransform(mask, dt, CV_DIST_L1, 3);

    int MaxNOfIterations = edge_Points.size();
    cv::RNG rng(1234);
    float   bestCirclePercentage = 0;
    float bestCircleRadius = 0;
    cv::Point2f  bestCircleCenter;
    for (int idx = 0; idx < MaxNOfIterations; idx++)
    {

        ///3.隨機產生3點圓，且這三點不能重複

        unsigned int idx1 = rng.uniform(0, (int)edge_Points.size());
        unsigned int idx2 = rng.uniform(0, (int)edge_Points.size());
        unsigned int idx3 = rng.uniform(0, (int)edge_Points.size());
        if (idx1 == idx2) continue;
        if (idx1 == idx3) continue;
        if (idx3 == idx2) continue;
        cv::Point2f center; float radius;
        getCircle(edge_Points[idx1], edge_Points[idx2], edge_Points[idx3], center, radius);

        ///4.驗證圓

        float circle_perc = verifyCircle(dt, center, radius);

        ///5.判斷是否為最佳圓

        if (circle_perc >= bestCirclePercentage)
        {
            bestCirclePercentage = circle_perc;
            bestCircleRadius = radius;
            bestCircleCenter = center;
        }
    }

    ///6.繪圖

    cv::circle(image, bestCircleCenter, bestCircleRadius, Scalar(255, 255, 0), 2);
    cv::circle(image, bestCircleCenter, 2, Scalar(255, 0, 0), -1);

    ///存檔

    cv::imshow("E://Develop//TestData//RANSAC Circle//edge.bmp", edge);
    cv::imshow("E://Develop//TestData//RANSAC Circle//dst.bmp", image);

    waitKey(0);
    return 0;
}


int main(){

    float m, b, max_dist;

    std::vector<Point> all_pts;
    Mat pict=Mat::zeros(500,500,CV_8UC3);
    Mat consensus_mat=Mat::zeros(500,500,CV_8UC3);

    int max_x=pict.cols;
    int max_y=pict.rows;

    m=-0.5;
    b=500;
    max_dist=10;

    int max_iter=1000;
    float best_distance=max_x*max_y;
    float best_m,best_b;
    float get_dist;
    float get_m,get_b;
    float allow_dist=max_dist/2;
    vector<Point> consensus_pts;
    vector<Point> tmp_pts;
    vector<Point> seed_pts;

    vector<Point> in_set_pts;
    vector<int> in_set_idx;
    vector<Point> outlier_pts;



    namedWindow("pict",2);
    namedWindow("consensus",2);

    do{
        all_pts.resize(0);
        //----generate data------//
//        for(int x=0;x<max_x;x++){
//            for(int y=0;y<max_y;y++){
//                pict.at<Vec3b>(y,x)[0]=0;
//                pict.at<Vec3b>(y,x)[1]=0;
//                pict.at<Vec3b>(y,x)[2]=0;
//            }
//        }
        pict-=pict;
        consensus_mat-=consensus_mat;

        float random_p_ratio=2;
        generate_strait_line_point(m,b,max_dist,max_x,max_y,random_p_ratio,all_pts);
        for(Point p:all_pts){
            pict.at<Vec3b>(p.y,p.x)[0]=255;
            pict.at<Vec3b>(p.y,p.x)[1]=255;
            pict.at<Vec3b>(p.y,p.x)[2]=255;
        }
        cout<<"all_pts.size="<<all_pts.size()<<endl;

        //verify line
        max_iter=100;
        best_distance=max_x*max_y;
        allow_dist=max_dist;
        consensus_pts.resize(0);
        tmp_pts.resize(0);
        seed_pts.resize(0);
        int min_cnt=50;
        for(int k=0;k<max_iter;k++){
//            cout<<"iter -- "<<k<<endl;
            //--get random data---//
            in_set_pts.resize(0);
            in_set_idx.resize(0);
            outlier_pts.resize(0);
            int num=3;
//            cout<<"--random pts--"<<endl;
            get_random_line_pts(all_pts,num,in_set_pts,in_set_idx,outlier_pts);
            min_cnt=all_pts.size()/10;
            //        for(int i=0;i<in_set_pts.size();i++){
            //            circle(pict,in_set_pts[i],5,Scalar(255),1);
            //        }
            if(in_set_pts.size()<num){
                continue;
            }


//            cout<<"--very line--"<<endl;
            tmp_pts.resize(0);
            bool agree=verify_line(in_set_pts,outlier_pts,allow_dist,min_cnt,get_m,get_b,get_dist,tmp_pts);
            if(agree && get_dist<best_distance){
                best_distance=get_dist;
                best_m=get_m;
                best_b=get_b;
                consensus_pts.resize(0);
                for(int i=0;i<tmp_pts.size();i++){
                    consensus_pts.push_back(tmp_pts[i]);
                }
                seed_pts.resize(0);
                for(int i=0;i<in_set_pts.size();i++){
                    seed_pts.push_back(in_set_pts[i]);
                }
            }
        }

        cout<<"min_cnt="<<min_cnt<<",allow_dist="<<allow_dist<<",best_m="<<best_m<<",best_b="<<best_b<<",consensus_pts.size="<<consensus_pts.size()<<endl;


        char name[64];

        //draw points
        for(Point p:consensus_pts){
            consensus_mat.at<Vec3b>(p.y,p.x)[0]=255;
            consensus_mat.at<Vec3b>(p.y,p.x)[1]=255;
            consensus_mat.at<Vec3b>(p.y,p.x)[2]=255;
        }


        //draw line
        LeastSquare lsq;
        lsq.setM(best_m);
        lsq.setB(best_b);
        Point p1(0,(int)lsq.getY(0));
        Point p2((int)lsq.getX(0),0);
        line(pict,p1,p2,Scalar(255),2);
        line(consensus_mat,p1,p2,Scalar(255,0,0),1); //蓝色表示预测图
        {
            vector<double> x;
            vector<double> y;
            for(int i=0;i<seed_pts.size();i++){
                //初始选择的点，得到的直线
                x.push_back(seed_pts[i].x);
                y.push_back(seed_pts[i].y);

                std::cout<<"seed_pt-"<<i<<"=("<<seed_pts[i].x<<","<<seed_pts[i].y<<")"<<endl;
                circle(consensus_mat,seed_pts[i],20,Scalar(0,0,255),1);

            }
            LeastSquare seed_lsq(x,y);
            Point seed_p1(0,(int)seed_lsq.getY(0));
            Point seed_p2((int)seed_lsq.getX(0),0);
            line(consensus_mat,seed_p1,seed_p2,Scalar(0,0,255),1);

            for(int i=0;i<consensus_pts.size();i++){
                float dist=seed_lsq.get_dist(consensus_pts[i].x,consensus_pts[i].y);
                if(dist>allow_dist){
                    sprintf(name,"%f",dist);
                    cv::putText(consensus_mat,name,consensus_pts[i],CV_FONT_HERSHEY_SIMPLEX,0.5,Scalar(0,255,0));
                }
            }

        }

        //--real line---//
        LeastSquare lsq2;
        lsq2.setM(m);
        lsq2.setB(b);
        Point realp1(0,(int)lsq2.getY(0));
        Point realp2((int)lsq2.getX(0),0);
        line(pict,realp1,realp2,Scalar(0,255,0),2);




        imshow("pict",pict);
        imshow("consensus",consensus_mat);
        int key=waitKey(0);

        if(key==27){
            break;
        }


    }while(true);


}

