#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>

#include <math.h>
using namespace std;
using namespace cv;



//  图片是以左下角为原点的坐标系，命名为grid_coord,右手直角坐标系，设为坐标系1
//  其他是右手直角坐标系，为坐标系2。


float grid_step;
int rows=1000,cols=1000,grid_cnt=20;

/**
 * @brief get_dir_vector
 * 根据与水平方向的夹角，计算方向向量
 * @param x_axis_angle
 * @param dir_vector
 */
void get_dir_vector(float x_axis_angle,cv::Point2d& dir_vector);
void grid_to_pixel(cv::Point2f& grid_pt,cv::Point2f& pixel_pt,float grid_step);

/**
 * @brief draw_axis
 * @param canvas
 * @param coord_grid_orig
 * @param x_axis_angle
 * 坐标系的x轴与水平方向的夹角。逆时针为正。
 * 前提都是基于图像坐标系来画的。
 */
void draw_axis(cv::Mat& canvas,cv::Point2f& coord_grid_orig,float x_axis_angle,cv::Scalar line_color=cv::Scalar(0,255,255));

class Coord{
public:
    float x_axis_angle_;//与水平方向的夹角
    cv::Point2f original_pt_in_grid_;//以网格为单位的该坐标系的原点
    cv::Point2d dir_vector_;//方向向量
    cv::Scalar x_axis_color_=cv::Scalar(255,0,255);
    cv::Scalar y_axis_color_=cv::Scalar(0,255,255);
    void Draw(cv::Mat& canvas){
        float y_axis_angle=x_axis_angle_+90;
        cout<<"x_axis_angle_="<<x_axis_angle_<<",y_axis_angle="<<y_axis_angle<<endl;
        draw_axis(canvas,original_pt_in_grid_,x_axis_angle_,x_axis_color_);
        draw_axis(canvas,original_pt_in_grid_,y_axis_angle,y_axis_color_);

        cv::Point2f pixel_orig;
        grid_to_pixel(original_pt_in_grid_,pixel_orig,grid_step);
        cv::circle(canvas,pixel_orig,2,Scalar(0,255,0),2);

    }

    //绘画该坐标系下的某点
    void DrawPoint(cv::Mat& canvas,cv::Point2f pt_in_my_coord,int radius=3,cv::Scalar color=cv::Scalar(0,0,255),int thickness=2){
        cv::Point2f pixel_pt,pt_in_grid_coord;
        ConvertToGridCoord(pt_in_my_coord,pt_in_grid_coord);
        grid_to_pixel(pt_in_grid_coord,pixel_pt,grid_step);
        cv::circle(canvas,pixel_pt,radius,color,thickness);
    }



    /**
     * @brief ConvertToGridCoord
     * 转换为grid_coord坐标系
     * @param pt
     * @param pt_in_grid_coord
     * cos -sin
     * -sin -cos
     *
     * 变换矩阵的角度是逆时针的。
     * 参考《解析几何》p132
     */
    void ConvertToGridCoord(cv::Point2f& pt,cv::Point2f& pt_in_grid_coord){
          pt_in_grid_coord.x=std::cos(x_axis_angle_*3.1415926/180)*pt.x+(-1)*std::sin(x_axis_angle_*3.1415926/180)*pt.y+this->original_pt_in_grid_.x;
          pt_in_grid_coord.y=std::sin(x_axis_angle_*3.1415926/180)*pt.x+std::cos(x_axis_angle_*3.1415926/180)*pt.y+this->original_pt_in_grid_.y;

    }

    /**
     * @brief ConvertFromGridCoordPt
     * 已知grid_coord的坐标值，转换为本坐标系的表达
     * @param pt_in_grid_coord
     * @param pt
     */
    void ConvertFromGridCoordPt(cv::Point2f& pt_in_grid_coord,cv::Point2f& pt){
        Mat m=Mat::zeros(2,2,CV_32FC1);

        //右手系的grid_coord到此右手坐标系的旋转变换矩阵
        m.at<float>(0,0)=std::cos(x_axis_angle_*3.1415926/180);
        m.at<float>(0,1)=(-1)*std::sin(x_axis_angle_*3.1415926/180);
        m.at<float>(1,0)=std::sin(x_axis_angle_*3.1415926/180);
        m.at<float>(1,1)=std::cos(x_axis_angle_*3.1415926/180);


        Mat m2=m.inv();
        pt.x=m2.at<float>(0,0)*(pt_in_grid_coord.x-this->original_pt_in_grid_.x)
                +m2.at<float>(0,1)*(pt_in_grid_coord.y-this->original_pt_in_grid_.y);
        pt.y=m2.at<float>(1,0)*(pt_in_grid_coord.x-this->original_pt_in_grid_.x)
                +m2.at<float>(1,1)*(pt_in_grid_coord.y-this->original_pt_in_grid_.y);

        cout<<"pt_in_grid_coord("<<pt_in_grid_coord.x<<","<<pt_in_grid_coord.y<<")\n"
           <<" grid->mycoord matrix=\n"<<m<<endl
          <<" inv of matrix=\n"<<m2<<endl
         <<" original_pt_in_grid_=("<<original_pt_in_grid_.x<<","<<original_pt_in_grid_.y<<")"<<endl
         <<" my coord pt("<<pt.x<<","<<pt.y<<endl;


    }

    /**
     * @brief GetTranslateToOtherCoordMatrix
     * 以另一个坐标系为基准，获取转到到右手另一个坐标系的转换矩阵。
     * 用处是：当已知道本坐标系下的点的坐标情况时，可以用这个转换矩阵转为另一个坐标系下的坐标值。
     * 这里有点拗口，需要理解。
     * @param other
     * @param convert_matrix
     * 存放结果：0,1,2,3位置分别对应[0,0],[0,1],[1,0],[1,1]位置的数值(cos,-sin,cos,sin)
     * 4,5存放x，y的偏移值
     *
     */
    void GetTranslateToOtherCoordMatrix(Coord& other,vector<double>& convert_matrix){
        float angle=this->x_axis_angle_ - other.x_axis_angle_;
        if(this->x_axis_angle_ <other.x_axis_angle_){
            angle=360+angle;//相当于转了一圈
        }
        cout<<"angle="<<angle<<endl;
        convert_matrix.resize(0);
        convert_matrix.push_back(std::cos(angle*3.1415926/180));
        convert_matrix.push_back(-1*std::sin(angle*3.1415926/180));
        convert_matrix.push_back(std::cos(angle*3.1415926/180));
        convert_matrix.push_back(std::sin(angle*3.1415926/180));


        //这两个偏移值
        cv::Point2f relat_pt;
        other.ConvertFromGridCoordPt(this->original_pt_in_grid_,relat_pt);
        convert_matrix.push_back(relat_pt.x);
        convert_matrix.push_back(relat_pt.y);
        cout<<"original_pt_in_grid_=("<<original_pt_in_grid_.x<<","<<original_pt_in_grid_.y<<")"<<endl;
        cout<<"relat_pt=("<<relat_pt.x<<","<<relat_pt.y<<")"<<endl;
    }

    /**
     * @brief ConvertPointToOtherCoord
     * 将一个本坐标系表示的点转换为另一个坐标系的坐标
     * @param pt
     * @param other
     * @param dest_pt
     */
    void ConvertPointToOtherCoord(cv::Point2f& pt,Coord& other,cv::Point2f& dest_pt){
        vector<double> convert_matrix;

        GetTranslateToOtherCoordMatrix(other,convert_matrix);
        cout<<"convert matrix:\n"<<convert_matrix[0]<<" "<<convert_matrix[1]<<endl
           <<convert_matrix[2]<<" "<<convert_matrix[3]<<endl<<endl
                <<convert_matrix[4]<<" "<<convert_matrix[5]<<endl;


        dest_pt.x=convert_matrix[0]*pt.x+convert_matrix[1]*pt.y+convert_matrix[4];
        dest_pt.y=convert_matrix[2]*pt.x+convert_matrix[3]*pt.y+convert_matrix[5];
    }
};

/**
 * @brief get_dir_vector
 * 根据与水平方向的夹角，计算方向向量
 * 都是基于grid_coord来计算方向向量的
 * 左手坐标系
 * @param x_axis_angle
 * @param dir_vector
 */
void get_dir_vector(float x_axis_angle,cv::Point2d& dir_vector){
    if(x_axis_angle==0){
        dir_vector.x=1;
        dir_vector.y=0;
    }
    else if(0<x_axis_angle && x_axis_angle<90){
        dir_vector.x=1;
        dir_vector.y=1*std::tan(x_axis_angle*3.1415926/180);
    }
    else if(x_axis_angle==90){
        dir_vector.x=0;
        dir_vector.y=1;
    }else if(90<x_axis_angle && x_axis_angle<180){
        dir_vector.x=-1;
        dir_vector.y=1*std::tan((180-x_axis_angle)*3.1415926/180);
    }
    else if(x_axis_angle==180){
        dir_vector.x=-1;
        dir_vector.y=0;
    }else if(180<x_axis_angle && x_axis_angle<270){
        dir_vector.x=-1;
        dir_vector.y=-std::tan((x_axis_angle-180)*3.1415926/180);
    }
    else if(x_axis_angle==270){
        dir_vector.x=0;
        dir_vector.y=-1;
    }else if(270<x_axis_angle && x_axis_angle<360){
        dir_vector.x=1;
        dir_vector.y=-std::tan((360-x_axis_angle)*3.1415926/180);
    }
}


/**
 * @brief generate_grid
 * 产生一个带有网格的画布
 * @param canvas
 * @param grid_cnt
 * 分成几个格子
 * @param line_thickness
 * @param line_color
 */
void generate_grid(cv::Mat& canvas,int grid_cnt,int line_thickness=1,cv::Scalar line_color=cv::Scalar(255,0,0)){

    int r=0,r_line=0;
    grid_step=canvas.rows/grid_cnt;
    while(true){
        r_line=r*grid_step;
        if(r_line>canvas.rows){
            break;
        }
        cv::Point2i p1,p2;
        p1.x=0;
        p1.y=r_line;

        p2.x=canvas.cols-1;
        p2.y=r_line;
        cv::line(canvas,p1,p2,line_color,line_thickness);
        ++r;
    }
    int c=0,c_line=0;
    grid_step=canvas.cols/grid_cnt;
    while(true){
        c_line=c*grid_step;
        if(c_line>canvas.cols){
            break;
        }
        cv::Point2i p1,p2;
        p1.x=c_line;
        p1.y=0;

        p2.x=c_line;
        p2.y=canvas.rows-1;
        cv::line(canvas,p1,p2,line_color,line_thickness);
        ++c;
    }
}

void grid_to_pixel(cv::Point2f& grid_pt,cv::Point2f& pixel_pt,float grid_step){
    pixel_pt.x=grid_pt.x*grid_step;
    pixel_pt.y=1000-grid_pt.y*grid_step;
}


/**
 * @brief draw_axis
 * @param canvas
 * @param coord_grid_orig
 * @param x_axis_angle
 * 坐标系的x轴与水平方向的夹角。逆时针为正。
 * 前提都是基于图像坐标系来画的。
 */
void draw_axis(cv::Mat& canvas,cv::Point2f& coord_grid_orig,float axis_angle,cv::Scalar line_color){

    if(axis_angle>=360){
        axis_angle-=360;
    }else if(axis_angle<0){
        axis_angle+=360;
    }
    cv::Point2d dir_vector;
    get_dir_vector(axis_angle,dir_vector);

    cv::Point2f pt1,pt2;
    if(dir_vector.x==0){
        pt1.x=coord_grid_orig.x;
        pt1.y=0;
        pt2.x=coord_grid_orig.x;
        pt1.y=100;
    }else if(dir_vector.y==0){
        pt1.x=0;
        pt1.y=coord_grid_orig.y;
        pt2.x=100;
        pt2.y=coord_grid_orig.y;
    }else {
        pt1.x=0;
        pt1.y=(pt1.x-coord_grid_orig.x)/dir_vector.x*dir_vector.y+coord_grid_orig.y;

        pt2.x=100;
        pt2.y=(pt2.x-coord_grid_orig.x)/dir_vector.x*dir_vector.y+coord_grid_orig.y;
    }


    cv::Point2f pixel_et1,pixel_et2;
    grid_to_pixel(pt1,pixel_et1,grid_step);
    grid_to_pixel(pt2,pixel_et2,grid_step);
    cv::line(canvas,pixel_et1,pixel_et2,line_color,1);

}

void test_draw_axis(){
    cv::Mat canvas=cv::Mat::zeros(rows,cols,CV_8UC3);
    generate_grid(canvas,grid_cnt);
    //基准坐标
    Coord base_coord;
    cv::Point2f coord_grid_orig(5,3);
    base_coord.original_pt_in_grid_=coord_grid_orig;
    base_coord.x_axis_angle_=280;
    base_coord.Draw(canvas);

    imshow("canvas",canvas);
    waitKey(0);
}

void test_convert_to_gridcoord(){
    cv::Mat canvas=cv::Mat::zeros(rows,cols,CV_8UC3);
    generate_grid(canvas,grid_cnt);
    //基准坐标
    Coord base_coord;
    cv::Point2f coord_grid_orig(10,10);
    cv::Point2f pt_in_grid_coord,my_coord_pt(-2,3.5);
    base_coord.original_pt_in_grid_=coord_grid_orig;
    base_coord.x_axis_angle_=300;
    base_coord.Draw(canvas);
    base_coord.ConvertToGridCoord(my_coord_pt,pt_in_grid_coord);
    base_coord.DrawPoint(canvas,my_coord_pt,8);
    cout<<"pt_in_grid_coord="<<pt_in_grid_coord.x<<","<<pt_in_grid_coord.y<<endl;

    cv::Point2f pixel_pt;
    grid_to_pixel(pt_in_grid_coord,pixel_pt,grid_step);
    cv::circle(canvas,pixel_pt,3,Scalar(0,255,0),1);

    imshow("canvas",canvas);
    waitKey(0);
}



void test_convert_from_gridcoord(){
    cv::Mat canvas=cv::Mat::zeros(rows,cols,CV_8UC3);
    generate_grid(canvas,grid_cnt);
    //基准坐标
    Coord base_coord;
    cv::Point2f orig_in_grid(10,10);
    cv::Point2f test_grid_coord_pt(15,3),my_coord_pt;
    base_coord.original_pt_in_grid_=orig_in_grid;
    base_coord.x_axis_angle_=10;
    base_coord.Draw(canvas);
    base_coord.ConvertFromGridCoordPt(test_grid_coord_pt,my_coord_pt);
    base_coord.DrawPoint(canvas,my_coord_pt,5);

    cv::Point2f pixel_pt;
    grid_to_pixel(test_grid_coord_pt,pixel_pt,grid_step);
    cv::circle(canvas,pixel_pt,3,Scalar(0,255,0),1);

    cout<<"my_coord_pt="<<my_coord_pt.x<<","<<my_coord_pt.y<<endl;

    imshow("canvas",canvas);
    waitKey(0);
}


void test_two_coord_transfer(){

    cv::Mat canvas=cv::Mat::zeros(rows,cols,CV_8UC3);
    generate_grid(canvas,grid_cnt);

    //grid_coord是所有其他坐标系的母坐标系，一切的初始表示都是从grid_coord开始。

    //基准坐标
    Coord base_coord;
    cv::Point2f coord_grid_orig(10,10);
    base_coord.original_pt_in_grid_=coord_grid_orig;
    base_coord.x_axis_angle_=10;
    base_coord.Draw(canvas);


    //移动坐标系
    float x_off=-5,y_off=-6;
    cv::Point2f off_pt(x_off,y_off);
    cv::Point2f off_pt_grid;
    base_coord.ConvertToGridCoord(off_pt,off_pt_grid);//以base_coord为基准，偏移一定量,得到真正的在grid_coord中的表示
    base_coord.DrawPoint(canvas,off_pt);


    Coord car_coord;
    car_coord.original_pt_in_grid_.x=off_pt_grid.x;
    car_coord.original_pt_in_grid_.y=off_pt_grid.y;//都是基于grid_coord坐标系的
    car_coord.x_axis_angle_=10;
    car_coord.Draw(canvas);
    cv::Point2f pt1(2,2);
    car_coord.DrawPoint(canvas,pt1);

    //用base_coord来表示
    cv::Point2f dest_pt;
    car_coord.ConvertPointToOtherCoord(pt1,base_coord,dest_pt);
    base_coord.DrawPoint(canvas,dest_pt,5,cv::Scalar(255,255,255),1);

    imshow("canvas",canvas);
    waitKey(0);
}

int main(int argc,char* argv[]){

//    test_draw_axis();
//    test_convert_to_gridcoord();
//    test_convert_from_gridcoord();
    test_two_coord_transfer();
    return 1;

}
