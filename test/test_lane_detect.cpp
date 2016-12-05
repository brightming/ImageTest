

//#include <stdio.h>
//#include <dirent.h>
//#include <sys/stat.h>
//#include <cctype>
//#include <algorithm>
//#include <math.h>
//#include <fstream>
//#include <string>

//#include "str_common.h"
//#include "distance/lab_color_detect.h"
//#include "math/img_math.h"

//#include <opencv2/opencv.hpp>



//using namespace cv;
//using namespace std;

////----------------


////# Global parameters

////Gaussian smoothing
//int kernel_size = 3;

//// Canny Edge Detector
//int low_threshold = 50;
//int high_threshold = 150;

//// Region-of-interest vertices
////We want a trapezoid shape, with bottom edge at the bottom of the image
//float trap_bottom_width = 0.8 ;// # width of bottom edge of trapezoid, expressed as percentage of image width
//float trap_top_width = 0.3;//0.07 ;// # ditto for top edge of trapezoid
//float trap_height = 0.4 ;// # height of the trapezoid expressed as percentage of image height

////Hough Transform
//int rho = 2;// # distance resolution in pixels of the Hough grid
//const float PI=3.1415926;
//float theta = 1 * PI/180 ;//# angular resolution in radians of the Hough grid
//int hough_threshold = 25	;// # minimum number of votes (intersections in Hough grid cell)
//int min_line_length = 30 ;//#minimum number of pixels making up a line
//int max_line_gap = 10	;//# maximum gap in pixels between connectable line segments

////for valid line slope
//float min_left_slope=30;
//float max_left_slope=46;
//float min_right_slope=15;
//float max_right_slope=50;



//extern "C" vector<Vec4i> hough_color_detect_img(Mat& src);

//struct SegMent{
//    int min_line_len;
//    int max_gap_len;
//    int min_left_slope;
//    int max_left_slope;
//    int min_right_slope;
//    int max_right_slope;
//    Rect  range;
//    int y_offset;
//    int x_offset;

//    Scalar yellow_color;
//    Scalar white_color;

//    int white_lab_min_dist;
//    int yellow_lab_min_dist;

//    Mat white_mask;
//    Mat yellow_mask;

//    Mat pic;
//    Mat middle_pic;
//};


//void test_imshow(string name,Mat& pic){
//#ifdef DEBUG_SHOW
//    imshow(name,pic);
//#endif
//}

//Mat region_of_interest(Mat& img, vector<Point>& vertices){

//    Mat mask=Mat::zeros(img.rows,img.cols,img.type());

//    Scalar ignore_mask_color;
//    int channel_count=img.channels();
//    if(img.channels()>1){
//        ignore_mask_color=Scalar(255,255,255);
//    }else{

//        ignore_mask_color=Scalar(255);
//    }

//    //    #filling pixels inside the polygon defined by "vertices" with the fill color
//    Point root_points[1][vertices.size()];
//    for(int i=0;i<vertices.size();i++){
//        root_points[0][i]=vertices[i];

//    }
//    const Point* ppt[1] = {root_points[0]};
//    int npt[1] ;
//    npt[0]=vertices.size();

//    fillPoly(mask, ppt, npt, 1, ignore_mask_color);
//    test_imshow("mask",mask);

//    //    returning the image only where mask pixels are nonzero
//    Mat masked_image ;
//    bitwise_and(img,img,masked_image,mask);

//    test_imshow("masked_image",masked_image);
//    return masked_image;


//}

///**
// * @brief filter_colors
// * @param src
// * @return
// */
//Mat filter_colors(SegMent& segment){

//#define BY_LAB
//    Vec3b target_color(12,168,255);
//    int min_dist=segment.white_lab_min_dist;
//    colorDetect cdect;

//    int dilation_size = 3;
//    int dilation_type=MORPH_ELLIPSE;//MORPH_RECT,MORPH_CROSS,MORPH_ELLIPSE


//    //####################Filter white pixels#################//
////    Mat white_mask;
//    Mat element;

//#ifdef BY_LAB
//    //----method 1 lab distance
//    cdect.SetTargetColor(segment.white_color[0],segment.white_color[1],segment.white_color[2]);
//    cdect.SetMinDistance(segment.white_lab_min_dist);
//    Mat white_image;
//    segment.white_mask=cdect.process(segment.pic,&segment.white_mask);
//#else
//    //----method 2
//    int white_threshold=200;
//    Scalar lower_white(white_threshold,white_threshold,white_threshold);
//    Scalar upper_white(255, 255, 255);
//    inRange(src,lower_white,upper_white,white_mask);

//#endif
//     //imshow("white_mask",white_mask);
//    //膨胀
//    element = getStructuringElement( dilation_type,
//                                           Size( 2*dilation_size + 1, 2*dilation_size+1 ),
//                                           Point( dilation_size, dilation_size ) );
//      ///膨胀操作
//    dilate( segment.white_mask, segment.white_mask, element );

//    bitwise_and(segment.pic,segment.pic,white_image,segment.white_mask);
////    imshow("white_mask",segment.white_mask);



//    //#################### Filter yellow pixels ##################//
////    Mat yellow_mask;
//#ifdef BY_LAB
//    //---method 1 use lab distance
//    cdect.SetMinDistance(segment.yellow_lab_min_dist);
//    cdect.SetTargetColor(segment.yellow_color[0],segment.yellow_color[1],segment.yellow_color[2]);
//    segment.yellow_mask=cdect.process(segment.pic,&segment.yellow_mask);
//#elif
//    //--method 2 use hsv
//    Mat hsv;
//    cvtColor(src,hsv,COLOR_BGR2HSV);
//    Scalar lower_yellow(90,100,100);
//    Scalar upper_yellow(110,255,255);
//    inRange(hsv,lower_yellow,upper_yellow,yellow_mask);

//#endif
//    //dilate
//    dilation_size = 3;
//    dilation_type=MORPH_ELLIPSE;//MORPH_RECT,MORPH_CROSS,MORPH_ELLIPSE
//    element = getStructuringElement( dilation_type,
//                                           Size( 2*dilation_size + 1, 2*dilation_size+1 ),
//                                           Point( dilation_size, dilation_size ) );
//      ///膨胀操作
//    dilate( segment.yellow_mask, segment.yellow_mask, element );
////    imshow("yellow_mask",segment.yellow_mask);


//    Mat yellow_image;
//    bitwise_and(segment.pic,segment.pic,yellow_image,segment.yellow_mask);
////    imshow("yellow_image",yellow_image);



//    //####################  road #########################//
////    Mat road_mask;
////    cdect.SetMinDistance(15);
////    cdect.SetTargetColor(78,87,94);
////    road_mask=cdect.process(src);
////    imshow("road_mask",road_mask);



//    //    # Combine the two above images
////    Mat combined_img;
//    if(segment.middle_pic.empty()){
//        segment.middle_pic=Mat::zeros(segment.pic.rows,segment.pic.cols,segment.pic.type());
//    }
//    addWeighted(white_image,1.,yellow_image,1.,0,segment.middle_pic);
////    imshow("combined_img",segment.middle_pic);


//    return segment.middle_pic;



//}

///**
// * @brief filter_lines_by_slope
// * exclude lines whose slopes are not in valid slope range.
// * for the left lines,the slope is large then 0;
// * for the right liens,the slope is less then 0;
// *
// * the max,min slope limit are all abs value.
// * @param input_lines
// * @param output_lines
// * @param left_min_slope
// * @param left_max_slope
// * @param right_min_slope
// * @param right_max_slope
// */
//void filter_lines_by_slope(vector<Vec4i> &input_lines,vector<Vec4i>& output_lines,
//                           int img_center_x,
//                           double left_min_slope,double left_max_slope,
//                           double right_min_slope,double right_max_slope){

//    if(input_lines.size()==0){
//        return;
//    }

//    float slope=0;
//    for(Vec4i line : input_lines){
//        int x1=line[0];
//        int y1=line[1];
//        int x2=line[2];
//        int y2=line[3];

//        //        # Calculate slope
//        if (x2 - x1 == 0){//  # corner case, avoiding division by 0
//            slope = 999.;  //# practically infinite slope
//        }else{
//            slope = atan((y2 - y1)*1.0 / (x2 - x1)) * 180.0/PI;
//        }
////        cout<<"x1="<<x1<<",y1="<<y1<<",x2="<<x2<<",y2="<<y2<<",slope="<<slope<<endl;

//        //left lines
//        if(slope<0){
//            if(abs(slope)>left_min_slope && abs(slope)<left_max_slope
//                && x1<=img_center_x && x2<=img_center_x
//                ){
//            output_lines.push_back(line);
//            }else{
//                cout<<"drop left line:("<<x1<<","<<y1<<")->("<<x2<<","<<y2<<"),slope="<<slope<<endl;
//            }
//        }else{
//            //right lines
//            if(abs(slope)>right_min_slope && abs(slope)<right_max_slope
//                 && x1>=img_center_x && x2>=img_center_x
//                 ){
//            output_lines.push_back(line);
//            }else{
//                cout<<"drop right line:("<<x1<<","<<y1<<")->("<<x2<<","<<y2<<"),slope="<<slope<<endl;
//            }
//        }

//    }


//}


//void draw_lines(Mat& img, vector<Vec4i> lines, Scalar color=Scalar(255, 0, 0), int thickness=2){

//    char name[64];
//    for( size_t i = 0; i < lines.size(); i++ )
//    {
//        line( img, Point(lines[i][0], lines[i][1]),
//                Point(lines[i][2], lines[i][3]), Scalar(0,255,0), 1, 8 );

//        Vec4i line=lines[i];
//        int x1=line[0];
//        int y1=line[1];
//        int x2=line[2];
//        int y2=line[3];

//        float slope = atan((y2 - y1)*1.0 / (x2 - x1)) * 180.0/PI;

//        if(i<-100){
//            sprintf(name,"(%d,%d)->(%d,%d):(%d)/(%d)=%f",x1,y1,x2,y2,(y2-y1),(x2-x1),slope);
//            putText( img, name, Point(x1,y1),CV_FONT_HERSHEY_COMPLEX, 0.5, Scalar(255, 0, 0) );
//        }
//    }



//    return;

//    //	"""
//    //	NOTE: this is the function you might want to use as a starting point once you want to
//    //	average/extrapolate the line segments you detect to map out the full
//    //	extent of the lane (going from the result shown in raw-lines-example.mp4
//    //	to that shown in P1_example.mp4).

//    //	Think about things like separating line segments by their
//    //	slope ((y2-y1)/(x2-x1)) to decide which segments are part of the left
//    //	line vs. the right line.  Then, you can average the position of each of
//    //	the lines and extrapolate to the top and bottom of the lane.

//    //	This function draws `lines` with `color` and `thickness`.
//    //	Lines are drawn on the image inplace (mutates the image).
//    //	If you want to make the lines semi-transparent, think about combining
//    //	this function with the weighted_img() function below
//    //	"""



//    //	# In case of error, don't draw the line(s)
//    if(lines.size()==0){
//        return;
//    }

//    bool draw_right=true;
//    bool draw_left=true;

//    //	# Find slopes of all lines
//    //	# But only care about lines where abs(slope) > slope_threshold
//    float slope_threshold = 0.5;
//    float slope;
//    vector<float> slopes ;
//    vector<Vec4i> new_lines;

//    for(Vec4i line : lines){
//        int x1=line[0];
//        int y1=line[1];
//        int x2=line[2];
//        int y2=line[3];

//        //        # Calculate slope
//        if (x2 - x1 == 0){//  # corner case, avoiding division by 0
//            slope = 999.;  //# practically infinite slope
//        }else{
//            slope = (y2 - y1) / (x2 - x1);
//        }

//        //# Filter lines based on slope
//        if (abs(slope) > slope_threshold){
//            slopes.push_back(slope);
//            new_lines.push_back(line);
//        }

//    }

//    lines=new_lines;



//    //	# Split lines into right_lines and left_lines, representing the right and left lane lines
//    //	# Right/left lane lines must have positive/negative slope, and be on the right/left half of the image
//    vector<Vec4i> right_lines;
//    vector<Vec4i> left_lines;

//    for(int i=0;i<lines.size();i++){
//        Vec4i line=lines[i];
//        int x1=line[0];
//        int y1=line[1];
//        int x2=line[2];
//        int y2=line[3];

//        int img_x_center=img.cols/2;//x coordinate of center of image
//        if(slopes[i]>0 && x1> img_x_center && x2 > img_x_center){
//            right_lines.push_back(line);
//        }else if(slopes[i]<0 && x1<img_x_center && x2< img_x_center){
//            left_lines.push_back(line);
//        }
//    }


//    //	# Run linear regression to find best fit line for right and left lane lines
//    //	# Right lane lines
//    vector<double> right_lines_x;
//    vector<double> right_lines_y ;


//    for(Vec4i line : right_lines){
//        int x1=line[0];
//        int y1=line[1];
//        int x2=line[2];
//        int y2=line[3];

//        right_lines_x.push_back(x1);
//        right_lines_x.push_back(x2);
//        right_lines_y.push_back(y1);
//        right_lines_y.push_back(y2);
//    }


//    float right_m,right_b;
//    if(right_lines_x.size()>0){
//        //right_m, right_b = np.polyfit(right_lines_x, right_lines_y, 1)  # y = m*x + b
//        LeastSquare lsq(right_lines_x,right_lines_y);
//        lsq.print();
//        right_m=lsq.getM();
//        right_b=lsq.getB();
//    }else{
//        right_m=1;
//        right_b=1;
//        draw_right=false;
//    }



//    //	# Left lane lines

//    vector<double> left_lines_x;
//    vector<double> left_lines_y ;


//    for(Vec4i line : left_lines){
//        int x1=line[0];
//        int y1=line[1];
//        int x2=line[2];
//        int y2=line[3];

//        left_lines_x.push_back(x1);
//        left_lines_x.push_back(x2);
//        left_lines_y.push_back(y1);
//        left_lines_y.push_back(y2);
//    }


//    double left_m,left_b;
//    if(left_lines_x.size()>0){
//        //right_m, right_b = np.polyfit(right_lines_x, right_lines_y, 1)  # y = m*x + b
//        LeastSquare lsq(left_lines_x,left_lines_y);
//        lsq.print();
//        left_m=lsq.getM();
//        left_b=lsq.getB();
//    }else{
//        left_m=1;
//        left_b=1;
//        draw_left=false;
//    }



//    //	# Find 2 end points for right and left lines, used for drawing the line
//    //	# y = m*x + b --> x = (y - b)/m
//    int	y1 = img.rows;
//    int y2 = img.rows * (1 - trap_height);

//    int right_x1 = (y1 - right_b) / right_m;
//    int right_x2 = (y2 - right_b) / right_m;

//    int left_x1 = (y1 - left_b) / left_m;
//    int left_x2 = (y2 - left_b) / left_m;

//    //	# Convert calculated end points from float to int

//    //	# Draw the right and left lines on image
//    if (draw_right){
//        //		cv2.line(img, (right_x1, y1), (right_x2, y2), color, thickness)
//        line( img, Point(right_x1, y1),
//              Point(right_x2, y2), color, thickness, 8 );
//    }
//    if(draw_left){
//        //		cv2.line(img, (left_x1, y1), (left_x2, y2), color, thickness)
//        line( img, Point(left_x1, y1),
//              Point(left_x2, y2), color, thickness, 8 );
//    }

//}

//Mat weighted_img(Mat &img,Mat& initial_img,float alpha=0.8, float belta=1., float landa=0.){
//    //	"""
//    //	`img` is the output of the hough_lines(), An image with lines drawn on it.
//    //	Should be a blank image (all black) with lines drawn on it.

//    //	`initial_img` should be the image before any processing.

//    //	The result image is computed as follows:

//    //	initial_img * α + img * β + λ
//    //	NOTE: initial_img and img must be the same shape!
//    //	"""
//    //	return cv2.addWeighted(initial_img, α, img, β, λ)


//    Mat result;

//    addWeighted(initial_img,alpha,img,belta,landa,result);

//    return result;
//}




//vector<Vec4i> hough_color_detect_img(Mat& src){


//    char name[64];
//    //分段进行检测
//    Mat combined_img=Mat::zeros(src.rows,src.cols,src.type());
//    int width=src.cols;
//    int height=src.rows;
//    vector<SegMent> seg_ms;
//    int absolute_trap_height=height*trap_height;
//    int seg_cnt=5;
//    int seg_height=absolute_trap_height/seg_cnt;
//    int start_y=height;
//    int start_x=width*(1-trap_bottom_width)/2;
//    int seg_width=width*trap_bottom_width;
//    for(int scn=0;scn<seg_cnt;scn++){
//        SegMent seg;
//        seg.max_gap_len=max_line_gap;
//        seg.min_line_len=min_line_length*(1-scn*0.1);
//        seg.y_offset=start_y-(scn+1)*seg_height;
//        seg.x_offset=start_x;
//        seg.range=Rect(seg.x_offset,seg.y_offset,seg_width,seg_height);
//        seg.pic=src(seg.range);

//        seg.white_lab_min_dist=scn==0?30:pow(1.05,scn)*30;
//        seg.yellow_lab_min_dist=scn==0?30:pow(1.05,scn)*30;
//        seg.white_color=Scalar(255,255,255);
//        seg.yellow_color=Scalar(93,218,248);


//        seg_ms.push_back(seg);


//        //draw separate line on img
//        line(src,Point(0,seg.y_offset),Point(width,seg.y_offset),Scalar(0,0,255),1);
//        sprintf(name,"seg:%d",scn);
//        putText(src,name,Point(seg.x_offset,seg.y_offset),CV_FONT_HERSHEY_COMPLEX,0.5,Scalar(0,0,255),1);

//    }
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

//    test_imshow("combined_img",combined_img);

//    //        # Read in and grayscale the image
//    Mat gray;
//    cvtColor(combined_img,gray,COLOR_BGR2GRAY);

//    //        Apply Gaussian smoothing
//    Mat blur_gray;
//    GaussianBlur(gray,blur_gray,Size(kernel_size,kernel_size),0,0);

//    //        Apply Canny Edge Detector
//    Mat edges;
//    cv::Canny(blur_gray,edges,low_threshold,high_threshold);


//    //        # Create masked edges using trapezoid-shaped region-of-interest
//    vector<Point> vertices;
//    int channel_count=src.channels();
//    vertices.push_back(Point(width*(1 - trap_bottom_width) / 2,height));
//    vertices.push_back(Point(width*(1 - trap_top_width) / 2,height-height*trap_height));
//    vertices.push_back(Point(width - width*(1 - trap_top_width) / 2,height-height*trap_height));
//    vertices.push_back(Point(width - width*(1 - trap_bottom_width) / 2,height));

//    Mat masked_edges = region_of_interest(edges, vertices);

//    test_imshow("masked_edges",masked_edges);


//    //# Run Hough on edge detected image
//    //分段进行检测
//    vector<Vec4i> all_lines;
//    vector<Vec4i> lines;
//    vector<Vec4i> valid_lines;
//    int img_center_x=src.cols/2;//x coordinate of center of image
//    for(int scn=0;scn<seg_cnt;scn++){
//        SegMent seg=seg_ms[scn];
//        lines.resize(0);
//        HoughLinesP( masked_edges(seg.range), lines, rho, theta, hough_threshold, seg.min_line_len, seg.max_gap_len );

//        //---filter out invalid lines by slope
//        valid_lines.resize(0);
//        filter_lines_by_slope(lines,valid_lines,img_center_x-seg.x_offset,min_left_slope,max_left_slope,min_right_slope,max_right_slope);
//        std::cout<<"scn="<<scn<<",raw lines size="<<lines.size()<<",after fiter,line size="<<valid_lines.size()<<std::endl;
//        for(Vec4i a:valid_lines){
//            a[0]+=seg.x_offset;
//            a[1]+=seg.y_offset;
//            a[2]+=seg.x_offset;
//            a[3]+=seg.y_offset;
//            all_lines.push_back(a);
//        }


//    }
//    //draw center point
//    circle(src,Point(width/2,height/2),10,Scalar(0,0,255),1);




//    Mat line_image=Mat::zeros(src.rows,src.cols,src.type());
////    std::cout<<"line_image.channel="<<line_image.channels()<<std::endl;
//    draw_lines(line_image,all_lines);
//    test_imshow("only lines",line_image);


//    //        # Draw lane lines on the original image
//    Mat initial_image = src.clone();
//    Mat annotated_image = weighted_img(line_image, initial_image);

//    test_imshow("annotated_image",annotated_image);


//    return all_lines;
//    //        return annotated_image
//}

#include <string>

#include "str_common.h"
#include "lanedetect/hough_color_lane_detect.h"

using namespace std;
using namespace cv;
int main(int argc,char* argv[]){

    string input_file="/home/gumh/tmp/challenge";
    string output_file="";



    vector<std::string> allfiles;
    if(IsDir(input_file)){
        allfiles=getAllFilesWithPathFromDir(input_file);
        std::sort(allfiles.begin(),allfiles.end(),std::less<string>());
    }else{
        allfiles.push_back(input_file);
    }


    for(string file:allfiles){
        Mat src=imread(file);
        Mat out;
        hough_color_detect_img(src);
        int key=waitKey(0);
        if(key==27){
            break;
        }
    }

    //    string video_file="/home/gumh/qtcreator-workspace/lanedetectsrc/road_lane_line_detection/challenge.mp4";
    //    VideoCapture vcap(video_file);
    //    if(!vcap.isOpened()){
    //        cerr<<"fail to open video file:"<<video_file<<endl;
    //        return -1;
    //    }

    //    cv::Mat frame;
    //    vcap >> frame;

    //    /** 若视频读取完毕，跳出循环 */
    //    while ( !frame.empty() )
    //    {

    //        vcap>>frame;
    //    }



}

