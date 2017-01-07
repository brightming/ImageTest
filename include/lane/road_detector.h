#ifndef __ROAD_DETECTOR__

#define __ROAD_DETECTOR__

#include <string>
#include <vector>
#include <map>

#include <opencv2/core/core.hpp>



//线型：实线，虚线
typedef enum LineType{
    SOLID_LINE,
    BROKEN_LINE
}LineType;

typedef enum LinePosition{
    LINE_LEFT,
    LINE_RIGHT
}LinePosition;//这里有疑问，单双线，双实线，一单一实问题

typedef enum LineColor{
    YELLOW,
    WHITE
}LineColor;

struct LaneSegment{
    cv::Point start_p;
    cv::Point end_p;
    int width;
    int height;
    float slope;
    float intercept;
};

typedef struct Lane{
    std::string id;
    LineType line_type;
    LineColor line_color;
    LinePosition line_position;
    std::vector<LaneSegment> lane_segments;
    float car_line_angle;//车相对于车道线的角度
    float hor_distance;//线段相对于摄像头的水平距离；
}Lane;


class LaneDetector{
public:
    //    virtual ~LaneDetector() = 0;
    virtual void Initialize(std::vector<std::string> params)=0;
    virtual void Update(cv::Mat& pict)=0;
    virtual std::vector<Lane> GetLanes()=0;
    virtual cv::Mat GetLaneMat()=0;

public:

};


//--道路上物体的枚举---//
enum RoadObjectType{
    pedestrian,
    Bycycle,
    Car,
    Bus,
    MotoBike,
    Train
};

//---道路物体的信息----//
struct RoadObject{
    cv::Rect position;
    RoadObjectType type;
    float hor_distance;//object相对于摄像头的水平距离；正：右边，负：左边；作	为拓展使用
    float ver_distance;//object距离摄像头的垂直距离
    float obj_length;//object的长，作为拓展使用
    float obj_width;//object的宽，作为拓展使用
    float obj_high;//object的高，作为拓展使用
};

//图片中的所有Object与车道线结构体
typedef struct PicLanesObjects
 {
   cv::Mat left_pic_mat;//左摄像头原始图像mat矩阵 lsw modify 2016.12.16
   cv::Mat right_pic_mat;//右摄像头原始图像mat矩阵 lsw modify 2016.12.16
   std::vector<RoadObject> objects;//图片中所有object
   std::vector<Lane> lanes;//图片中所有车道线
 }PicLanesObjects;


class RoadDetector{
public:
    virtual ~RoadDetector(){};


    /**
     * @brief Initialize
     * @param params
     */
    virtual void Initialize(std::map<std::string,std::string> params)=0;


    /**
     * @brief NextFrame
     * @param pict
     */
    virtual void NextFrame(const cv::Mat& pict)=0;

    /**
     * @brief GetRoadObjectByType
     * 指定获取某类物体的位置信息
     * @param type
     * @return
     */
    virtual std::vector<RoadObject> GetRoadObjectByType(RoadObjectType type)=0;


    /**
     * @brief GetAllRoadObjects
     * 获取识别出来的所有除道路线外的物体
     * @return
     */
    virtual std::vector<RoadObject> GetAllRoadObjects()=0;


    /**
     * @brief GetRoadObjectMatByType
     * 以mat的形式获取指定的某类物体
     * @param type
     * @return
     */
    virtual cv::Mat GetRoadObjectMatByType(RoadObjectType type)=0;


    /**
     * @brief GetLaneMat
     * 获取车道线的mat
     * 除了车道线的像素值为255外，其他的为0
     * @return
     * 返回CV_8UC1的mat
     */
    virtual const cv::Mat GetLaneMat()=0;


};

#endif
