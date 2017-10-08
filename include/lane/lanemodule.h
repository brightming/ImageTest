
#include <opencv2/core/core.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <vector>
#include <algorithm>
#include <math.h>
#include <time.h>

using namespace std;
using namespace cv;
clock_t start, stop;

namespace gumh{
#define PI 3.1415926

class RoadContext;

class LaneFilter{
public:
    Mat currFrame; //stores the upcoming frame
    Mat temp;      //stores intermediate results
    Mat temp2;     //stores the final lane segments

    int diff, diffL, diffR;
    int laneWidth;
    int diffThreshTop;
    int diffThreshLow;
    int ROIrows;
    int vertical_left;
    int vertical_right;
    int vertical_top;
    int smallLaneArea;
    int longLane;
    int  vanishingPt;
    float maxLaneWidth;

    //to store various blob properties
    Mat binary_image; //used for blob removal
    int minSize;
    int ratio;
    float  contour_area;
    float blob_angle_deg;
    float bounding_width;
    float bounding_length;
    Size2f sz;
    vector< vector<Point> > contours;
    vector< vector<Point> > keep_contours;//过滤后保留的contours
    vector<Vec4i> hierarchy;
    RotatedRect rotated_rect;

public:
    RoadContext* context=nullptr;

public:
    LaneFilter(Mat startFrame);

    void updateSensitivity();
    void getLane();
    void markLane();
    void blobRemoval();
    Mat nextFrame(Mat &nxt);
    Mat getResult();

};

class StraightLine{
public:
    float k=0;
    float b=0;
    float rho=0;
    float phi=0;//角度为单位
    bool is_vertical=false;
    bool is_horizontal=false;

    bool is_valid=false;

public:
    StraightLine(vector<cv::Point2f>& pts);
    StraightLine();
    bool SetData(vector<cv::Point2f>& pts);


    bool GetY(float x,float& y);
    bool GetX(float y,float& x);

    /**
     * @brief GetInterceptPoint
     * 求交点
     * @param another
     * @param pt
     * @return
     * 0：无交点
     * 1：两条线重合
     * 2：其他的正常线
     */
    int GetInterceptPoint(StraightLine& another,cv::Point2f& pt);


    /**
     * @brief GetDistanceToPoint
     * 计算到某个点的距离
     * @param pt
     * @return
     */
    float GetDistanceToPoint(cv::Point2f &pt);

    //----FOR DEBUG-----//
    void Draw(cv::Mat& pict,cv::Scalar color=cv::Scalar(255,0,0),int thickness=1);


    void Print();
};


class LaneMarkingSeg{
public:

    RoadContext *context;
    LaneMarkingSeg(RoadContext* ctx,vector<cv::Point>& pts);
    void ParseValue(vector<cv::Point> &pts);
    void FitMultiLines();
    vector<cv::RotatedRect> SplitMultiSubSegs(int cnt);

    long id;
    bool is_combined=false;//是否是被合并的线
    bool is_a_combine_segs=false;//是否是一个合并线
    vector<vector<cv::Point>> all_pts;//对于一个合并线，这个是记录所有的点

    vector<cv::Point> contour_pts;
    RotatedRect rotated_rect;
    cv::Point2f rect_points[4];
    StraightLine edge_lines[4]; //4条边线 依次为0-1,1-2,2-3,3-0
    cv::Point highest_pt,lowest_pt;//最高点y最小，最低点y最大
    cv::Point mid_p_low,mid_p_hight;//旋转矩形的朝向的两条边的中间点
    float blob_angle_deg;
    float bounding_width;
    float bounding_length;
    float dir_len=0;//朝向的长度
    float short_len=0;//另一个方向的长度
    Size2f sz;

    //与图片底边交点
    float x_btm=0;

    //直线方程
    StraightLine bounding_box_line_model;//外接矩形的中间点的直线方程

    //挑选过程中的参数
    bool in_group=false;
    vector<LaneMarkingSeg*> same_group_segs;//同组的
    long group_id=0;//所属组的id
    bool final_selected=false;//是否最后被挑选中
    vector<cv::Point2f> pt_with_others;//与其他线的交点
    int good_pt_cnt=0,bad_pt_cnt=0;//与其他线的交点，分析后认为好的交点的个数，与坏的交点的个数
    float x_var_pt=0,y_var_pt=0;//与其他线交点的x方差，y方差

    //将整个片段，分成几段,每段分别拟合直线
    vector<cv::RotatedRect> sub_rects;
    vector<vector<cv::Point2f>> select_pt_for_final_curve;//
    vector<StraightLine> inner_lines;

public:
    vector<LaneMarkingSeg*> wait_to_combine_segs;//判断为可以合并的，等待合并

    /**
     * @brief IsMidLineGoThroughAnother
     * 判断本bounding box的中间线是否穿过另一个bounding box
     * @param another
     * @return
     */
    bool IsMidLineGoThroughAnother(LaneMarkingSeg& another);

    /**
     * @brief CanCombine
     * 判断两个片段是否能合并
     * @param another
     * @return
     */
    bool CanCombine(LaneMarkingSeg& another);

    /**
     * @brief Combine
     * 合并另一片段到本片段
     * @param another
     */
    void Combine(LaneMarkingSeg& another);

    /**
     * @brief isContainPt
     * 判断是否包含了某个点
     * @param pt
     * @return
     */
    bool isContainPt(cv::Point2f &pt);

    /**
     * @brief IsReallyLengthSeg
     * 判断是否是一个真的够长的线
     * @return
     */
    bool IsReallyLengthSeg();

    void Print();

};


class VarianceValue{
public:
    float mean_delta_x=100000000.0;
    float variance_delta_x=100000000.0;
    float mean_delta_slope=100000000.0;
    float variance_delta_slope=100000000.0;
};

enum RoadPaintType{
    PaintType_RoadPaintMarking=0,
    PaintType_Arrow
};


class RoadPaintMarking{
public:
    RoadPaintMarking();
public:
    int color=0;//color=0 :white =1 yellow
    int is_curve=0; //0: straight 1:curve
    int is_oneof_double_lane=0;//是否是双线之一

    float btm_x=0;//底点x坐标
    float slope=0;//斜率

    int is_blur=0;//是否模糊线。如果一条线时而可见，时而不可见，则为模糊线


    int first_frame_seq_support=0;//找到本线的第一帧的编号
    int last_frame_seq_support=0;//最后一次找到本线支持的编号

    //history data
    //keep 50 frames' data
    vector<LaneMarkingSeg> support_history_segs;//支持的contours
//    vector<float> history_btm_xs;
//    vector<float> history_slopes;
//    vector<float> is_history_curves;



    /**
     * @brief candidate_segs
     * 满足x横向偏移、斜率偏移的seg，加入候选
     * 最后通过整体的优化，获取最优的选择
     */
    vector<LaneMarkingSeg*> candidate_segs;
    /**
     * @brief AddCandidateSupportSeg
     *
     * @param seg
     */
    void AddCandidateSupportSeg(LaneMarkingSeg& seg);
    void ClearCandidateSegs();

    void AddSupportSeg(LaneMarkingSeg& seg,long seq);



    RoadPaintMarking* left_lane;
    RoadPaintMarking* right_lane;


    float confidence=0;//可信度，目前简单的用dir_len叠加来表示

    //类型
    RoadPaintType road_paint_type;

    //for debug
    cv::Scalar draw_color;


};


class LaneMarking:public RoadPaintMarking{
public:
    LaneMarking():RoadPaintMarking(){
        road_paint_type=PaintType_RoadPaintMarking;
    }

};

class RoadContext{
public:
    vector<LaneMarkingSeg> segs;//当前帧记录的片段
    cv::Size pict_size=cv::Size(1280,720);
    long cur_seq=0;

    cv::Point2f varnish_pt=cv::Point2f(-1,-1);

    float allow_max_dist_from_varnish_pt=100;
    float varnish_p_y_value=250;//灭点的合理y值坐标
    float allow_max_y_offset=60;//

    float avg_lane_dist=-1;//平均两条车道线的x_btm间隔

    ///----for DEBUG----//
    RNG rng;//随机数产生器
    std::map<float,long> pos_accu_cnt;//每个x位置累计的seg的数量
};

class CandidateGroup{
public:
    vector<LaneMarkingSeg*> lane_marks;
    long id=clock();

    void Print();

    //
    float loss=0;

    bool IsExist(LaneMarkingSeg* one);

};

class Road{
public:

    Road();
    RoadContext* context=nullptr;
    LaneFilter* lane_filter;//lane过滤器

    vector<RoadPaintMarking*> lane_markings;//路上的标线
    int is_at_cross=0;//是否处于路口.0:no 1:yes
    int find_zarcross=0;//是否有斑马线
    int drive_dir_type=0;//行驶方向. 0: 直行或近似直行 1：左转  2：右转

    //
    cv::Mat cur_input_pict;//输入的原始图片，保持不变
    cv::Mat cur_draw_pict;


public:
    void FindRoadPaintMarkings(Mat& pict);

    /**
     * @brief GetCandidateLaneSegGroup
     * 获取可能是一组合理的搭配的车道线组合
     * @param groups
     * @param top
     * 获取得分最高的几组，如果-1,表示返回全部组
     */
    void GetCandidateLaneSegGroup(vector<CandidateGroup>& groups,int top=-1);

    /**
     * @brief CalculateGroupLoss
     * 计算指定group的损失值，越小越好
     * @param g
     * @return
     */
    float CalculateGroupLoss(CandidateGroup& g);

    /**
     * @brief InitialLanes
     * 初始化选择标线
     */
    void InitialLanes();


    /**
     * @brief UpdateLanes
     * 已有标线，对当前帧进行处理
     */
    void UpdateLanes();

    /**
     * @brief HandleMaybeNewerLanes
     * @param maybe_newer
     */
    void HandleMaybeNewerLanes(vector<RoadPaintMarking*> &maybe_newer);

    /**
     * @brief ChooseBestCandidateSegs
     * 加入已有lane的有不少候选者，需要进行全局优化，挑选最合适的
     */
    void ChooseBestCandidateSegs();

    /**
     * @brief IsAProperDistanceToNearby
     * @param candidate
     * 待判断的候选者
     * @param exists_one
     * 已存在的
     * @return
     */
    bool IsAProperDistanceToNearby(RoadPaintMarking* candidate,RoadPaintMarking* exists_one);

    /**
     * @brief BuildRelations
     * 对于初步过滤得到的contour，建立合并，相邻，前后等空间关系
     * @param binary_pict
     */
    void BuildRelations(Mat& binary_pict);


    /**
     * @brief findArrow
     * 找箭头
     */
    void FindArrow();


    /**
     * @brief RecheckLanes
     * 对初步确定的lane做全盘分析
     */
    void RecheckLanes();
    void AnalyzeByVarnishPoint();

    void RemoveBadPositionLines();

    void CombineSegs();


    static bool lanes_x_btm_cmp_less_ptr(RoadPaintMarking* a,RoadPaintMarking* b);
    static bool lanes_x_btm_cmp_less(LaneMarkingSeg& a,LaneMarkingSeg& b);
    static bool lanes_length_cmp_greator(LaneMarkingSeg& a,LaneMarkingSeg& b);

//-----------for DEBUG-------------//
    void DrawOneRectangle(Mat& pict,LaneMarkingSeg& one,cv::Scalar=cv::Scalar(255,0,0),int thickness=2,cv::Scalar endp_col=cv::Scalar(0,255,0));
    void DrawAllRectangle(Mat& pict,cv::Scalar=cv::Scalar(255,0,0),int thickness=2);

};



/**
 * @brief The SegInterceptPoint class
 * 记录两个RoadPaintMarking片段的交点
 */
class SegInterceptPoint{
public:
    bool is_bad=false;//交点是否有问题
    cv::Point2f pt;
    LaneMarkingSeg *one;
    LaneMarkingSeg *two;
};

}//end of namespace gumh
