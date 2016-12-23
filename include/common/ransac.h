#include <iostream>
#include <vector>

#include <opencv2/opencv.hpp>

#include "math/img_math.h"
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
void generate_strait_line_point(float m,float b,float max_dist,int max_x,int max_y,float random_p_ratio,std::vector<Point>& pts);






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
void get_random_line_pts(vector<Point>& all_pts,int num,vector<Point>& in_set_pts,vector<int>& in_set_idx,vector<Point>& outlier_pts);

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
bool verify_line(vector<Point>& in_set_pts,vector<Point>& outliers,float allow_dist,int min_cnt,float& get_m,float& get_b,float& total_dist,vector<Point>& consensus_pts);


