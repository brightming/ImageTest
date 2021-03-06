

#include <stdio.h>
#include <dirent.h>
#include <sys/stat.h>
#include <cctype>
#include <algorithm>
#include <math.h>
#include <fstream>
#include <string>

#include <opencv2/opencv.hpp>

#include "str_common.h"
#include "distance/lab_color_detect.h"
#include "math/img_math.h"
#include "lanedetect/lane_c.h"
#include "lanedetect/hough_color_lane_detect.h"

using namespace cv;
using namespace std;

//----------------


//# Global parameters

//Gaussian smoothing
int kernel_size = 3;

// Canny Edge Detector
int low_threshold = 50;
int high_threshold = 150;

// Region-of-interest vertices
//We want a trapezoid shape, with bottom edge at the bottom of the image
float trap_bottom_width = 1 ;// # width of bottom edge of trapezoid, expressed as percentage of image width
float trap_top_width = 1;//0.07 ;// # ditto for top edge of trapezoid
float trap_height = 0.75 ;// # height of the trapezoid expressed as percentage of image height

//Hough Transform
int rho = 2;// # distance resolution in pixels of the Hough grid
const float PI=3.1415926;
float theta = 1 * PI/180 ;//# angular resolution in radians of the Hough grid
int hough_threshold = 25	;// # minimum number of votes (intersections in Hough grid cell)
int min_line_length = 20 ;//#minimum number of pixels making up a line
int max_line_gap = 10	;//# maximum gap in pixels between connectable line segments

//for valid line slope
float min_left_slope=15;
float max_left_slope=80;
float min_right_slope=15;
float max_right_slope=80;

Scalar default_white_color(200,229,236);
Scalar default_yellow_color(137,202,231);


#define DEBUG_SHOW
void test_imshow(string name,Mat& pic){
#ifdef DEBUG_SHOW
    imshow(name,pic);
#endif
}

/////----duhw----//
//struct line_dist{
//    double k;
//    double b;
//    cv::Vec2d upper_point;
//    cv::Vec2d lower_point;
//    cv::Vec2d center_point;
//};

///**
// * @author du.hw
// * @create 2016.11.25
// * sort line by center's point x value asc
// */

//bool sortLineCenterX(line_dist a1,line_dist a2){
//    return a1.center_point[0] < a2.center_point[0];//order by asc
//}

///**
// * @author du.hw
// * @create 2016.11.25
// * filter line and compute final line
// * input_lines:hough lines
// * output_lines:final lines of child img
// * child_width:child img's width
// * child_height:child img's height
// */
//void filterLine(cv::vector<cv::Vec4i> input_lines,cv::vector<line_dist> *output_lines,int child_width,int child_height){
//    if(input_lines.size() ==0 ){
//        std::cout << "have no hough lines!" << std::endl;
//        return;
//    }
//    //compute every hough line's k b and center point
//    cv::vector<line_dist> lines_temp;
//    for(int i=0;i<input_lines.size();i++){
//        line_dist line;
//        cv::Vec4i l = input_lines[i];
//        //line: y = kx + b;
//        double k = l[2] != l[0] ? (l[3] - l[1]) / (l[2] - l[0]) : 0;//line's k value
//        double b = l[1] - k * l[0];//line's b value
//        line.k = k;
//        line.b = b;

//        //left and upper of src image is coordinae origin,so min_y is upper and max_y is lower
//        double upper_y = 0;
//        double lower_y = child_height;
//        double center_y = (double)child_height / 2;
//        double upper_x = k != 0 ? (upper_y - b)/k : l[0];
//        double lower_x = k != 0 ? (lower_y - b)/k : l[0];
//        double center_x = (upper_x + lower_x)/2;


//        line.upper_point[0] = upper_x;
//        line.upper_point[1] = upper_y;
//        line.center_point[0] = center_x;
//        line.center_point[1] = center_y;
//        line.lower_point[0] = lower_x;
//        line.lower_point[1] = lower_y;

//        lines_temp.push_back(line);
//    }

//    //order by center point's x asc
//    std::sort(lines_temp.begin(),lines_temp.end(),sortLineCenterX);

//   //start cluster from center point of line
//    int distance = 30;//max distance of two adjacent points
//    double upper_x_total = 0,upper_y_total = 0;
//    double lower_x_total = 0,lower_y_total = 0;
//    int cluster_number = 0;
//    for(int i=0;i<lines_temp.size();i++){
//        line_dist line = lines_temp[i];
//        std::cout << "center:" << line.center_point[0] << "  " << line.center_point[1] << std::endl;
//        if( cluster_number == 0  ||
//            (i > 0 && (line.center_point[0] - lines_temp[i-1].center_point[0]) < distance) ){
//            //center point is adjacent
//            cluster_number++;
//            upper_x_total += line.upper_point[0];
//            upper_y_total += line.upper_point[1];
//            lower_x_total += line.lower_point[0];
//            lower_y_total += line.lower_point[1];
//        }else if((line.center_point[0] - lines_temp[i-1].center_point[0]) >= distance){
//            //adjacent distance must less max distance
//              line_dist final_line;
//              double upper_x_final = upper_x_total / cluster_number;
//              double upper_y_final = upper_y_total / cluster_number;
//              double lower_x_final = lower_x_total / cluster_number;
//              double lower_y_final = lower_y_total / cluster_number;

//              final_line.upper_point[0] = upper_x_final;
//              final_line.upper_point[1] = upper_y_final;
//              final_line.lower_point[0] = lower_x_final;
//              final_line.lower_point[1] = lower_y_final;

//              final_line.center_point[0] = (upper_x_final + lower_x_final)/2;
//              final_line.center_point[1] = (upper_y_final + lower_y_final)/2;
//              final_line.k = (upper_x_final != lower_x_final)?(upper_y_final - lower_y_final) / (upper_x_final - lower_x_final):0;
//              final_line.b = upper_y_final - final_line.k * upper_x_final;
//              output_lines->push_back(final_line);

//                //recompute next
//                cluster_number = 1;
//                upper_x_total = line.upper_point[0];
//                upper_y_total = line.upper_point[1];
//                lower_x_total = line.lower_point[0];
//                lower_y_total = line.lower_point[1];
//        }
//    }


//   //compute final lines
//    line_dist last_final_line;
//    double last_upper_x_final = upper_x_total / cluster_number;
//    double last_upper_y_final = upper_y_total / cluster_number;
//    double last_lower_x_final = lower_x_total / cluster_number;
//    double last_lower_y_final = lower_y_total / cluster_number;

//    last_final_line.upper_point[0] = last_upper_x_final;
//    last_final_line.upper_point[1] = last_upper_y_final;
//    last_final_line.lower_point[0] = last_lower_x_final;
//    last_final_line.lower_point[1] = last_lower_y_final;

//    last_final_line.center_point[0] = (last_upper_x_final + last_lower_x_final)/2;
//    last_final_line.center_point[1] = (last_upper_y_final + last_lower_y_final)/2;
//    last_final_line.k = last_upper_x_final != last_lower_x_final ?
//            (last_upper_y_final - last_lower_y_final) / (last_upper_x_final - last_lower_x_final):0;
//    last_final_line.b = last_upper_y_final - last_final_line.k * last_upper_x_final;


//    output_lines->push_back(last_final_line);



//}


Mat region_of_interest(Mat& img, vector<Point>& vertices){

    Mat mask=Mat::zeros(img.rows,img.cols,img.type());

    Scalar ignore_mask_color;
    int channel_count=img.channels();
    if(img.channels()>1){
        ignore_mask_color=Scalar(255,255,255);
    }else{

        ignore_mask_color=Scalar(255);
    }

    //    #filling pixels inside the polygon defined by "vertices" with the fill color
    Point root_points[1][vertices.size()];
    for(int i=0;i<vertices.size();i++){
        root_points[0][i]=vertices[i];

    }
    const Point* ppt[1] = {root_points[0]};
    int npt[1] ;
    npt[0]=vertices.size();

    fillPoly(mask, ppt, npt, 1, ignore_mask_color);
    test_imshow("mask",mask);

    //    returning the image only where mask pixels are nonzero
    Mat masked_image ;
    bitwise_and(img,img,masked_image,mask);

    test_imshow("masked_image",masked_image);
    return masked_image;


}

/**
 * @brief filter_colors
 * @param src
 * @return
 */
Mat filter_colors(SegMent& segment){

#define BY_LAB
    Vec3b target_color(12,168,255);
    int min_dist=segment.white_lab_min_dist;
    colorDetect cdect;

    int dilation_size = 3;
    int dilation_type=MORPH_ELLIPSE;//MORPH_RECT,MORPH_CROSS,MORPH_ELLIPSE


    //####################Filter white pixels#################//
//    Mat white_mask;
    Mat element;

#ifdef BY_LAB
    //----method 1 lab distance
    cdect.SetTargetColor(segment.white_color[0],segment.white_color[1],segment.white_color[2]);
    cdect.SetMinDistance(segment.white_lab_min_dist);
    Mat white_image;
    segment.white_mask=cdect.process(segment.pic,&segment.white_mask);
#else
    //----method 2
    int white_threshold=200;
    Scalar lower_white(white_threshold,white_threshold,white_threshold);
    Scalar upper_white(255, 255, 255);
    inRange(src,lower_white,upper_white,white_mask);

#endif
     //imshow("white_mask",white_mask);
    //膨胀
    element = getStructuringElement( dilation_type,
                                           Size( 2*dilation_size + 1, 2*dilation_size+1 ),
                                           Point( dilation_size, dilation_size ) );
      ///膨胀操作
//    dilate( segment.white_mask, segment.white_mask, element );
    erode( segment.white_mask, segment.white_mask, element );

    bitwise_and(segment.pic,segment.pic,white_image,segment.white_mask);
//    imshow("white_mask",segment.white_mask);



    //#################### Filter yellow pixels ##################//
//    Mat yellow_mask;
#ifdef BY_LAB
    //---method 1 use lab distance
    cdect.SetMinDistance(segment.yellow_lab_min_dist);
    cdect.SetTargetColor(segment.yellow_color[0],segment.yellow_color[1],segment.yellow_color[2]);
    segment.yellow_mask=cdect.process(segment.pic,&segment.yellow_mask);
#elif
    //--method 2 use hsv
    Mat hsv;
    cvtColor(src,hsv,COLOR_BGR2HSV);
    Scalar lower_yellow(90,100,100);
    Scalar upper_yellow(110,255,255);
    inRange(hsv,lower_yellow,upper_yellow,yellow_mask);

#endif
    //dilate
    dilation_size = 3;
    dilation_type=MORPH_ELLIPSE;//MORPH_RECT,MORPH_CROSS,MORPH_ELLIPSE
    element = getStructuringElement( dilation_type,
                                           Size( 2*dilation_size + 1, 2*dilation_size+1 ),
                                           Point( dilation_size, dilation_size ) );
      ///膨胀操作
//    dilate( segment.yellow_mask, segment.yellow_mask, element );
    erode(segment.yellow_mask, segment.yellow_mask, element);
//    imshow("yellow_mask",segment.yellow_mask);


    Mat yellow_image;
    bitwise_and(segment.pic,segment.pic,yellow_image,segment.yellow_mask);
//    imshow("yellow_image",yellow_image);



    //####################  road #########################//
//    Mat road_mask;
//    cdect.SetMinDistance(15);
//    cdect.SetTargetColor(78,87,94);
//    road_mask=cdect.process(src);
//    imshow("road_mask",road_mask);



    //    # Combine the two above images
//    Mat combined_img;
    if(segment.middle_pic.empty()){
        segment.middle_pic=Mat::zeros(segment.pic.rows,segment.pic.cols,segment.pic.type());
    }
    addWeighted(white_image,1.,yellow_image,1.,0,segment.middle_pic);
//    imshow("combined_img",segment.middle_pic);


    return segment.middle_pic;

}

void filter_colors(Mat& output_mask,Mat& output_image,vector<SegMent> seg_ms){

    //对于每一段都进行颜色过滤
    int seg_cnt=seg_ms.size();
    for(int scn=0;scn<seg_cnt;scn++){
        SegMent seg=seg_ms[scn];
//        std::cout<<"seg.pic.empty="<<seg.pic.empty()<<std::endl;
        filter_colors(seg);
//        imshow("seg.pic",seg.pic);
//        imshow("seg.white_mask",seg.white_mask);
//        imshow("seg.yellow_mask",seg.yellow_mask);
//        imshow("seg.middle_pic",seg.middle_pic);
        Mat part=output_image(seg.range);
        part+=seg.middle_pic;

        part=output_mask(seg.range);
        addWeighted(seg.white_mask,1.,seg.yellow_mask,1.,0,part);
//        waitKey(0);
    }
}

/**
 * @brief filter_lines_by_slope
 * exclude lines whose slopes are not in valid slope range.
 * for the left lines,the slope is large then 0;
 * for the right liens,the slope is less then 0;
 *
 * the max,min slope limit are all abs value.
 * @param input_lines
 * @param output_lines
 * @param left_min_slope
 * @param left_max_slope
 * @param right_min_slope
 * @param right_max_slope
 */
void filter_lines_by_slope(vector<Vec4i> &input_lines,vector<Vec4i>& output_lines,
                           int img_center_x,
                           double left_min_slope,double left_max_slope,
                           double right_min_slope,double right_max_slope){

    if(input_lines.size()==0){
        return;
    }

    float slope=0;
    for(Vec4i line : input_lines){
        int x1=line[0];
        int y1=line[1];
        int x2=line[2];
        int y2=line[3];

        //        # Calculate slope
        if (x2 - x1 == 0){//  # corner case, avoiding division by 0
            slope = 999.;  //# practically infinite slope
        }else{
            slope = atan((y2 - y1)*1.0 / (x2 - x1)) * 180.0/PI;
        }
//        cout<<"x1="<<x1<<",y1="<<y1<<",x2="<<x2<<",y2="<<y2<<",slope="<<slope<<endl;

        //left lines
        if(slope<0){
            if(abs(slope)>left_min_slope && abs(slope)<left_max_slope
                && x1<=img_center_x && x2<=img_center_x
                ){
            output_lines.push_back(line);
            }else{
                cout<<"drop left line:("<<x1<<","<<y1<<")->("<<x2<<","<<y2<<"),slope="<<slope<<endl;
            }
        }else{
            //right lines
            if(abs(slope)>right_min_slope && abs(slope)<right_max_slope
                 && x1>=img_center_x && x2>=img_center_x
                 ){
            output_lines.push_back(line);
            }else{
                cout<<"drop right line:("<<x1<<","<<y1<<")->("<<x2<<","<<y2<<"),slope="<<slope<<endl;
            }
        }

    }


}


void draw_lines(Mat& img, vector<Vec4i> lines, Scalar color=Scalar(255, 0, 0), int thickness=2){

    char name[64];
    for( size_t i = 0; i < lines.size(); i++ )
    {
        line( img, Point(lines[i][0], lines[i][1]),
                Point(lines[i][2], lines[i][3]), Scalar(0,255,0), 1, 8 );

        Vec4i line=lines[i];
        int x1=line[0];
        int y1=line[1];
        int x2=line[2];
        int y2=line[3];

        float slope = atan((y2 - y1)*1.0 / (x2 - x1)) * 180.0/PI;

//        if(i<-100){
//            sprintf(name,"(%d,%d)->(%d,%d):(%d)/(%d)=%f",x1,y1,x2,y2,(y2-y1),(x2-x1),slope);
//            putText( img, name, Point(x1,y1),CV_FONT_HERSHEY_COMPLEX, 0.5, Scalar(255, 0, 0) );
//        }
    }



    return;

    //	"""
    //	NOTE: this is the function you might want to use as a starting point once you want to
    //	average/extrapolate the line segments you detect to map out the full
    //	extent of the lane (going from the result shown in raw-lines-example.mp4
    //	to that shown in P1_example.mp4).

    //	Think about things like separating line segments by their
    //	slope ((y2-y1)/(x2-x1)) to decide which segments are part of the left
    //	line vs. the right line.  Then, you can average the position of each of
    //	the lines and extrapolate to the top and bottom of the lane.

    //	This function draws `lines` with `color` and `thickness`.
    //	Lines are drawn on the image inplace (mutates the image).
    //	If you want to make the lines semi-transparent, think about combining
    //	this function with the weighted_img() function below
    //	"""



    //	# In case of error, don't draw the line(s)
    if(lines.size()==0){
        return;
    }

    bool draw_right=true;
    bool draw_left=true;

    //	# Find slopes of all lines
    //	# But only care about lines where abs(slope) > slope_threshold
    float slope_threshold = 0.5;
    float slope;
    vector<float> slopes ;
    vector<Vec4i> new_lines;

    for(Vec4i line : lines){
        int x1=line[0];
        int y1=line[1];
        int x2=line[2];
        int y2=line[3];

        //        # Calculate slope
        if (x2 - x1 == 0){//  # corner case, avoiding division by 0
            slope = 999.;  //# practically infinite slope
        }else{
            slope = (y2 - y1) / (x2 - x1);
        }

        //# Filter lines based on slope
        if (abs(slope) > slope_threshold){
            slopes.push_back(slope);
            new_lines.push_back(line);
        }

    }

    lines=new_lines;



    //	# Split lines into right_lines and left_lines, representing the right and left lane lines
    //	# Right/left lane lines must have positive/negative slope, and be on the right/left half of the image
    vector<Vec4i> right_lines;
    vector<Vec4i> left_lines;

    for(int i=0;i<lines.size();i++){
        Vec4i line=lines[i];
        int x1=line[0];
        int y1=line[1];
        int x2=line[2];
        int y2=line[3];

        int img_x_center=img.cols/2;//x coordinate of center of image
        if(slopes[i]>0 && x1> img_x_center && x2 > img_x_center){
            right_lines.push_back(line);
        }else if(slopes[i]<0 && x1<img_x_center && x2< img_x_center){
            left_lines.push_back(line);
        }
    }


    //	# Run linear regression to find best fit line for right and left lane lines
    //	# Right lane lines
    vector<double> right_lines_x;
    vector<double> right_lines_y ;


    for(Vec4i line : right_lines){
        int x1=line[0];
        int y1=line[1];
        int x2=line[2];
        int y2=line[3];

        right_lines_x.push_back(x1);
        right_lines_x.push_back(x2);
        right_lines_y.push_back(y1);
        right_lines_y.push_back(y2);
    }


    float right_m,right_b;
    if(right_lines_x.size()>0){
        //right_m, right_b = np.polyfit(right_lines_x, right_lines_y, 1)  # y = m*x + b
        LeastSquare lsq(right_lines_x,right_lines_y);
        lsq.print();
        right_m=lsq.getM();
        right_b=lsq.getB();
    }else{
        right_m=1;
        right_b=1;
        draw_right=false;
    }



    //	# Left lane lines

    vector<double> left_lines_x;
    vector<double> left_lines_y ;


    for(Vec4i line : left_lines){
        int x1=line[0];
        int y1=line[1];
        int x2=line[2];
        int y2=line[3];

        left_lines_x.push_back(x1);
        left_lines_x.push_back(x2);
        left_lines_y.push_back(y1);
        left_lines_y.push_back(y2);
    }


    double left_m,left_b;
    if(left_lines_x.size()>0){
        //right_m, right_b = np.polyfit(right_lines_x, right_lines_y, 1)  # y = m*x + b
        LeastSquare lsq(left_lines_x,left_lines_y);
        lsq.print();
        left_m=lsq.getM();
        left_b=lsq.getB();
    }else{
        left_m=1;
        left_b=1;
        draw_left=false;
    }



    //	# Find 2 end points for right and left lines, used for drawing the line
    //	# y = m*x + b --> x = (y - b)/m
    int	y1 = img.rows;
    int y2 = img.rows * (1 - trap_height);

    int right_x1 = (y1 - right_b) / right_m;
    int right_x2 = (y2 - right_b) / right_m;

    int left_x1 = (y1 - left_b) / left_m;
    int left_x2 = (y2 - left_b) / left_m;

    //	# Convert calculated end points from float to int

    //	# Draw the right and left lines on image
    if (draw_right){
        //		cv2.line(img, (right_x1, y1), (right_x2, y2), color, thickness)
        line( img, Point(right_x1, y1),
              Point(right_x2, y2), color, thickness, 8 );
    }
    if(draw_left){
        //		cv2.line(img, (left_x1, y1), (left_x2, y2), color, thickness)
        line( img, Point(left_x1, y1),
              Point(left_x2, y2), color, thickness, 8 );
    }

}

Mat weighted_img(Mat &img,Mat& initial_img,float alpha=0.8, float belta=1., float landa=0.){
    //	"""
    //	`img` is the output of the hough_lines(), An image with lines drawn on it.
    //	Should be a blank image (all black) with lines drawn on it.

    //	`initial_img` should be the image before any processing.

    //	The result image is computed as follows:

    //	initial_img * α + img * β + λ
    //	NOTE: initial_img and img must be the same shape!
    //	"""
    //	return cv2.addWeighted(initial_img, α, img, β, λ)


    Mat result;

    addWeighted(initial_img,alpha,img,belta,landa,result);

    return result;
}


int hough_color_detect_img_c(IplImage *shrink,vec4i_c *lines,int draw_lines){
#ifndef OPENCV_VERSION3
    std::cout<<"----hough_color_detect_img_c begin--"<<std::endl;

    if(lines==NULL){
        return -1;
    }
//    IplImage* iplImg = cvLoadImage("/home/gumh/tmp/challenge/image-001.jpeg", 1);
    Mat tmp(shrink,false);
    std::cout<<"----hough_color_detect_img_c 2--"<<std::endl;
    vector<cv::Vec4i> pts=hough_color_detect_img(tmp,draw_lines);

    if(pts.size()>lines->len){
        if(lines->pts){
            std::cout<<"free old memory--------"<<std::endl;
            free(lines->pts);
        }
        std::cout<<"try to calloc new memory----"<<std::endl;
        lines->len=pts.size();
        lines->used=pts.size();
        lines->pts=(int*)calloc(sizeof(int),4*lines->len);
        std::cout<<"aft calloc new memory----"<<std::endl;
    }
    for(int i=0;i<pts.size();i++){
        lines->pts[4*i+0]=pts[i][0];
        lines->pts[4*i+1]=pts[i][1];
        lines->pts[4*i+2]=pts[i][2];
        lines->pts[4*i+3]=pts[i][3];

    }
    return lines->used;
#else
    return 0;
#endif
}



void initial_segments(std::vector<SegMent> &seg_ms,int valid_roi_width,int valid_roi_height,int start_x,int start_y,int seg_cnt,float height_ratio){
    int absolute_trap_height=valid_roi_height;
    int seg_height=absolute_trap_height/seg_cnt;

    int seg_width=valid_roi_width*trap_bottom_width;

    float base_height;
    if(height_ratio!=1){
        base_height=absolute_trap_height*1.*(1-height_ratio)/(1-pow(height_ratio,seg_cnt));
    }else{
        base_height=seg_height;
    }

//    std::cout<<"get_segments::src.height="<<src.rows<<",valid_roi_width="<<valid_roi_width<<",valid_roi_height="<<valid_roi_height<<
//               ",seg_cnt="<<seg_cnt<<",seg_height="<<seg_height<<",start_x="<<start_x<<",start_y="<<start_y<<std::endl;
    int total_h=0;
    //自图像底部向上分起，底部第一个为编号0
    float base_color_dist=15;
    for(int scn=0;scn<seg_cnt;scn++){
        SegMent seg;
        seg.max_gap_len=max_line_gap;
        seg.min_line_len=min_line_length*(1-scn*0.1);

        seg.x_offset=start_x;
        if(height_ratio==1){
            seg_height=base_height;
        }else{
            seg_height=base_height*pow(height_ratio,seg_cnt-1-scn);

        }
        total_h+=seg_height;
        if(scn==seg_cnt-1){
            seg_height+=(absolute_trap_height-total_h);//由于浮点运算关系，如果不够，则在最后一段补充足
//            cout<<"last segment ,h is modified to : "<<seg_height<<endl;
            total_h=absolute_trap_height;
        }

        seg.y_offset=start_y-total_h;

//        std::cout<<",scn="<<scn<<",height_ratio="<<height_ratio<<",absolute_trap_height="<<absolute_trap_height<<",total_h="<<total_h
//                <<",seg_height="<<seg_height
//               <<",seg.y_offset="<<seg.y_offset
//              <<",src.height="<<src.rows
//               <<std::endl;


        seg.range=Rect(seg.x_offset,seg.y_offset,seg_width,seg_height);


        seg.white_lab_min_dist=scn==0?base_color_dist:pow(1.05,scn)*base_color_dist;
        seg.yellow_lab_min_dist=scn==0?base_color_dist:pow(1.05,scn)*base_color_dist;
        seg.white_color=default_white_color;//Scalar(191,197,203);
        seg.yellow_color=default_yellow_color;//Scalar(93,218,248);

        seg.seg_area=seg_width*seg_height;

        seg_ms.push_back(seg);

    }
}

void fill_segment_mat(cv::Mat& src,std::vector<SegMent> &seg_ms){

    char name[64];
    for(int scn=0;scn<seg_ms.size();scn++){

        seg_ms[scn].pic=src(seg_ms[scn].range).clone();

        //draw separate line on img
         SegMent seg=seg_ms[scn];
        line(src,Point(0,seg.y_offset),Point(seg.range.width,seg.y_offset),Scalar(0,scn*10,255),1);
        sprintf(name,"seg:%d,y_offset:%d，seg_height:%d",scn,seg.y_offset,seg.range.height);
        putText(src,name,Point(seg.x_offset,seg.y_offset),CV_FONT_HERSHEY_COMPLEX,0.5,Scalar(0,0,255),1);
    }

}



void get_segments(Mat& src,vector<SegMent> &seg_ms,int valid_roi_width,int valid_roi_height,int start_x,int start_y,int seg_cnt,float height_ratio){

    initial_segments(seg_ms,valid_roi_width,valid_roi_height,start_x,start_y,seg_cnt,height_ratio);

    fill_segment_mat(src,seg_ms);

    return ;
}

std::vector<cv::Vec4i> hough_color_detect_img(cv::Mat& src,int draw_line){


    char name[64];
    //分段进行检测
    Mat combined_img=Mat::zeros(src.rows,src.cols,src.type());
    Mat combined_mask=Mat::zeros(src.rows,src.cols,CV_8UC1);
    int width=src.cols;
    int height=src.rows;
    vector<SegMent> seg_ms;
    int absolute_trap_height=height*trap_height;
    int seg_cnt=5;
    int start_y=height;
    int start_x=width*(1-trap_bottom_width)/2;

    get_segments(src,seg_ms,width,absolute_trap_height,start_x,start_y,seg_cnt);

//    //    """ Given an image Numpy array, return the annotated image as a Numpy array """
//    //        # Only keep white and yellow pixels in the image, all other pixels become black
//    //对于每一段都进行颜色过滤
//    for(int scn=0;scn<seg_cnt;scn++){
//        SegMent seg=seg_ms[scn];
//        filter_colors(seg);
////        imshow("seg.pic",seg.pic);
////        imshow("seg.white_mask",seg.white_mask);
////        imshow("seg.yellow_mask",seg.yellow_mask);
////        imshow("seg.middle_pic",seg.middle_pic);
//        Mat part=combined_img(seg.range);
//        part+=seg.middle_pic;
////        waitKey(0);
//    }

    filter_colors(combined_mask,combined_img,seg_ms);

    test_imshow("combined_img",combined_img);
    test_imshow("combined_msk",combined_mask);

    //        # Read in and grayscale the image
    Mat gray;
    cvtColor(combined_img,gray,COLOR_BGR2GRAY);

    //        Apply Gaussian smoothing
    Mat blur_gray;
    GaussianBlur(gray,blur_gray,Size(kernel_size,kernel_size),0,0);

    //        Apply Canny Edge Detector
    Mat edges;
    cv::Canny(blur_gray,edges,low_threshold,high_threshold);


    //        # Create masked edges using trapezoid-shaped region-of-interest
    vector<Point> vertices;
    int channel_count=src.channels();
    vertices.push_back(Point(width*(1 - trap_bottom_width) / 2,height));
    vertices.push_back(Point(width*(1 - trap_top_width) / 2,height-height*trap_height));
    vertices.push_back(Point(width - width*(1 - trap_top_width) / 2,height-height*trap_height));
    vertices.push_back(Point(width - width*(1 - trap_bottom_width) / 2,height));

    //draw valid area
    polylines(src,vertices,true,Scalar(0,0,255));

    Mat masked_edges = region_of_interest(edges, vertices);

    test_imshow("masked_edges",masked_edges);


    //# Run Hough on edge detected image
    //分段进行检测
    vector<Vec4i> all_lines;
    vector<Vec4i> lines;
    vector<Vec4i> valid_lines;
    int img_center_x=src.cols/2;//x coordinate of center of image
    for(int scn=0;scn<seg_cnt;scn++){
        SegMent seg=seg_ms[scn];
        lines.resize(0);
        HoughLinesP( masked_edges(seg.range), lines, rho, theta, hough_threshold, seg.min_line_len, seg.max_gap_len );

        //---filter out invalid lines by slope
        valid_lines.resize(0);
        filter_lines_by_slope(lines,valid_lines,img_center_x-seg.x_offset,min_left_slope,max_left_slope,min_right_slope,max_right_slope);
        std::cout<<"scn="<<scn<<",raw lines size="<<lines.size()<<",after fiter,line size="<<valid_lines.size()<<std::endl;
        for(Vec4i a:valid_lines){
            a[0]+=seg.x_offset;
            a[1]+=seg.y_offset;
            a[2]+=seg.x_offset;
            a[3]+=seg.y_offset;
            all_lines.push_back(a);
        }


    }
    //draw center point
//    circle(src,Point(width/2,height/2),10,Scalar(0,0,255),1);





    if(draw_line>0){
        draw_lines(src,all_lines);
    }

//    Mat line_image=Mat::zeros(src.rows,src.cols,src.type());
//    draw_lines(line_image,all_lines);
//    test_imshow("only lines",line_image);


    //        # Draw lane lines on the original image
//    Mat initial_image = src.clone();
//    Mat annotated_image = weighted_img(line_image, initial_image);

//    test_imshow("annotated_image",annotated_image);


    return all_lines;
    //        return annotated_image
}



