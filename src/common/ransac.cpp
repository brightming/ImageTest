



#include <iostream>
#include <vector>

#include <opencv2/opencv.hpp>

#include "math/img_math.h"
#include "common/ransac.h"
using namespace std;
using namespace cv;


/**
 * @brief generate_strait_line_point
 * 产生用于拟合直线的点
 * @param m
 * 目标直线的斜率
 * @param b
 * 目标直线的截距
 * @param max_dist
 * 与直线距离在此误差范围内的点都认为是内点
 * @param max_x
 * 最大允许产生的x
 * @param max_y
 * 最大允许产生的y
 * @param random_p_ratio
 * 随机点数量与内点数量的比例
 * @param pts
 * 返回的点
 */
void generate_strait_line_point(float m,float b,float max_dist,int max_x,int max_y,float random_p_ratio,std::vector<Point>& pts){

    cv::RNG rng(getTickCount());

    //先产生内点
    for(int x=0;x<max_x;x++){
        float y=m*x+b;
        bool duplicated=false;
        int cnt=0;
        do{
            int real_y=rng.uniform(y-max_dist<0?0:y-max_dist,y+max_dist>max_y?max_y:y+max_dist);
            duplicated=false;
            for(int j=0;j<pts.size();j++){
                if(pts[j].x==x && pts[j].y==real_y){
                    duplicated=true;
                }
            }
            if(duplicated==false){
                pts.push_back(Point(x,real_y));
                break;
            }
            ++cnt;
        }while(duplicated==true&& cnt<50);

    }

    cout<<"inliers cnt="<<pts.size()<<endl;


    int inlier_cnt=pts.size();
    //随机产生若干个点
    for(int i=0;i<inlier_cnt*random_p_ratio;i++){
        bool duplicated=false;
        do{
            int x=rng.uniform(0,max_x);
            int y=rng.uniform(0,max_y);
            duplicated=false;
            for(int j=0;j<pts.size();j++){
                if(pts[j].x==x && pts[j].y==y){
                    duplicated=true;
                    break;
                }
            }
            if(duplicated==false){
                pts.push_back(Point(x,y));
                break;
            }
        }while(duplicated==true);
    }

}

/**
 * @brief get_random_line_pts
 * 随机选取若干个点用来拟合模型
 * @param all_pts
 * 所有的点
 * @param num
 * 需要选取的点数量
 * @param in_set_pts
 * 选择的点的坐标
 * @param in_set_idx
 * 选择的点的下标
 */
void get_random_line_pts(vector<Point>& all_pts,int num,vector<Point>& in_set_pts,vector<int>& in_set_idx,vector<Point>& outlier_pts){

    if (num>all_pts.size()){
        cout<<"num("<<num<<") large than all data size("<<all_pts.size()<<")!"<<endl;
        return;
    }
    cv::RNG rng(getTickCount());
    int total_cnt=all_pts.size();
    int cnt=0;
    int max_loop_cnt=10;
    for(int i=0;i<num;i++){
        bool duplicated=false;
        int idx;
        do{
            idx=rng.uniform(0,total_cnt-1);
            for(int j=0;j<in_set_idx.size();j++){
                if(j==in_set_idx[j]){
                    duplicated=true;
                    break;
                }
            }
            if(duplicated==false){
                in_set_idx.push_back(idx);
                in_set_pts.push_back(all_pts[idx]);
                break;
            }
            ++cnt;
        }while(duplicated && cnt<max_loop_cnt);
    }
    if(in_set_idx.size()<num){
        return;
    }
    for(int i=0;i<all_pts.size();i++){
        bool in=false;
        for(int j=0;j<in_set_idx.size();j++){
            if(i==in_set_idx[j]){
                in=true;
                break;
            }
        }
        if(!in){
            outlier_pts.push_back(all_pts[i]);
        }
    }


}

/**
 * @brief verify_line
 * 验证得到的直线有多好
 * @param all_pts
 * 所有的点
 * @param in_set_pts
 * 选择的点的坐标
 * @param in_set_idx
 * 选择的点的下标
 * @param get_m
 * 计算得到的斜率
 * @param get_b
 * 计算得到的截距
 */
bool verify_line(vector<Point>& in_set_pts,vector<Point>& outliers,float allow_dist,int min_cnt,float& get_m,float& get_b,float& total_dist,vector<Point>& consensus_pts){

    vector<double> x,y;
    for(int i=0;i<in_set_pts.size();i++){
        x.push_back(in_set_pts[i].x);
        y.push_back(in_set_pts[i].y);
    }
    LeastSquare lsq(x,y);
    //    vector<Point> maybe_inliers;
    total_dist=0;
    double every_dist;




    x.resize(0);
    y.reserve(0);
    consensus_pts.resize(0);
    for(int i=0;i<outliers.size();i++){
        every_dist=lsq.get_dist(outliers[i].x,outliers[i].y);
        if(every_dist<=allow_dist){
            consensus_pts.push_back(outliers[i]);
            x.push_back(outliers[i].x);
            y.push_back(outliers[i].y);
        }

    }

    //重新计算模型
    if(consensus_pts.size()>=min_cnt){
        for(int i=0;i<in_set_pts.size();i++){
            consensus_pts.push_back(in_set_pts[i]);
            x.push_back(in_set_pts[i].x);
            y.push_back(in_set_pts[i].y);
        }
        lsq.setData(x,y);
        get_m=lsq.getM();
        get_b=lsq.getB();

        for(int i=0;i<consensus_pts.size();i++){
            every_dist=lsq.get_dist(consensus_pts[i].x,consensus_pts[i].y);
            total_dist+=every_dist;
        }
        total_dist/=consensus_pts.size();//平均距离

        return true;
    }else{
        return false;
    }



}

