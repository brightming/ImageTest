/*TODO
 * improve edge linking
 * remove blobs whose axis direction doesnt point towards vanishing pt
 * Parallelisation
 * lane prediction
*/


#include <opencv2/core/core.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <vector>
#include <algorithm>
#include <math.h>
#include <time.h>

#include <stdio.h>
#include <dirent.h>
#include <sys/stat.h>
#include <cctype>

using namespace std;
using namespace cv;
clock_t start, stop;



#include "lanedetect/nightlane_detect.h"
#include "lanedetect/hough_color_lane_detect.h"
#include "str_common.h"
#include "math/img_math.h"

NightLaneDetect *night_lane_detector;

int night_lane_detect_img_c(IplImage *shrink,vec4i_c *lines,int draw_lines){
    Mat frame(shrink,false);
    if(night_lane_detector==NULL){
        night_lane_detector=new NightLaneDetect(frame);
    }

    night_lane_detector->nextFrame(frame);

    if(draw_lines>0){
        Mat mask=night_lane_detector->getResult();
        Rect roi=night_lane_detector->GetRoiRect();
        for(int y=roi.tl().y;y<mask.rows;y++){
            for(int x=0;x<mask.cols;x++){
                if(mask.at<uchar>(y,x)>0){
                    //                    printf("(%d,%d)=%d\n",y,x,mask.at<uchar>(y,x));
                    frame.at<Vec3b>(y,x)[0]=255;
                    frame.at<Vec3b>(y,x)[1]=0;
                    frame.at<Vec3b>(y,x)[2]=0;
                }
            }
        }
        //        imshow("mask-----",mask);
        //        imshow("frame111",frame);
    }

}


NightLaneDetect::NightLaneDetect(Mat startFrame,float roi_hight_ratio)
{
    currFrame = startFrame;                                    //if image has to be processed at original size

    currFrame = Mat(startFrame.rows,startFrame.cols,startFrame.type(),0.0);                        //initialised the image size to 320x480
    for_draw = Mat(startFrame.rows,startFrame.cols,startFrame.type(),0.0);
    resize(startFrame,ori_pic,currFrame.size());
    //        resize(startFrame, currFrame, currFrame.size());             // resize the input to required size

    temp      = Mat(currFrame.rows, currFrame.cols, CV_8UC1,0.0);//stores possible lane markings
    temp2     = Mat(currFrame.rows, currFrame.cols, CV_8UC1,0.0);//stores finally selected lane marks



    vanishingPt    = currFrame.rows*(1-roi_hight_ratio);//0.5;                           //for simplicity right now
    ROIrows        = currFrame.rows - vanishingPt;               //rows in region of interest
    minSize        = 0.00015 * (currFrame.cols*currFrame.rows);  //min size of any region to be selected as lane
    maxLaneWidth   = 0.025 * currFrame.cols;                     //approximate max lane width based on image size
    smallLaneArea  = 7 * minSize;
    longLane       = 0.3 * currFrame.rows;
    ratio          = 4;

    white_mask =Mat::zeros(currFrame.rows-vanishingPt,currFrame.cols,CV_8UC1);
    yellow_mask =Mat::zeros(currFrame.rows-vanishingPt,currFrame.cols,CV_8UC1);
    roi_rect=Rect(0,vanishingPt,currFrame.cols,currFrame.rows-vanishingPt);

    //these mark the possible ROI for vertical lane segments and to filter vehicle glare
    vertical_left  = 2*currFrame.cols/5;
    vertical_right = 3*currFrame.cols/5;
    vertical_top   = 2*currFrame.rows/3;



    //separate roi
    int valid_roi_width=roi_rect.width;
    int valid_roi_height=roi_rect.height;
    int start_x=roi_rect.x;
    int start_y=for_draw.rows;
    //    std::cout<<"valid_roi_height="<<valid_roi_height<<",start_y="<<start_y<<",vanishingPt.y="<<vanishingPt<<std::endl;
    int seg_cnt=5;
    initial_segments(seg_ms,valid_roi_width,valid_roi_height,start_x,start_y,seg_cnt);
    float bottom_left_min_slope=20;
    float bottom_left_max_slope=60;

    float bottom_right_min_slope=20;
    float bottom_right_max_slope=60;
    for(int i=0;i<seg_cnt;i++){
        seg_ms[i].min_left_slope=bottom_left_min_slope;
        seg_ms[i].max_left_slope=bottom_left_max_slope;

        seg_ms[i].min_right_slope=bottom_right_min_slope;
        seg_ms[i].max_right_slope=bottom_right_max_slope;

    }



    //1.kalman filter setup
//    const int stateNum=4;                                      //状态值4×1向量(m,b,△m,△b)
//    const int measureNum=2;                                    //测量值2×1向量(m,b)
//    KF.init(stateNum, measureNum, 0);

//    KF.transitionMatrix = *(Mat_<float>(4, 4) <<1,0,1,0,0,1,0,1,0,0,1,0,0,0,0,1);  //转移矩阵A
//    setIdentity(KF.measurementMatrix);                                             //测量矩阵H
//    setIdentity(KF.processNoiseCov, Scalar::all(1e-5));                            //系统噪声方差矩阵Q
//    setIdentity(KF.measurementNoiseCov, Scalar::all(1e-1));                        //测量噪声方差矩阵R
//    setIdentity(KF.errorCovPost, Scalar::all(1));                                  //后验错误估计协方差矩阵P
//    rng.fill(KF.statePost,RNG::UNIFORM,0,winHeight>winWidth?winWidth:winHeight);   //初始状态值x(0)

//    measurement = Mat::zeros(measureNum, 1, CV_32F);                           //初始测量值x'(0)，因为后面要更新这个值，所以必须先定义



    //    namedWindow("lane",2);
    //    namedWindow("midstep", 2);
    //    namedWindow("currframe", 2);
    //    namedWindow("laneBlobs",2);

    //    getLane();
}


void NightLaneDetect::updateSensitivity()
{
    int total=0, average =0;
    for(int i= vanishingPt; i<currFrame.rows; i++)
        for(int j= 0 ; j<currFrame.cols; j++)
            total += currFrame.at<uchar>(i,j);
    average = total/(ROIrows*currFrame.cols);
    cout<<"average : "<<average<<endl;
}

void NightLaneDetect::getLane()
{
    //medianBlur(currFrame, currFrame,5 );
    // updateSensitivity();
    //ROI = bottom half
    temp-=temp;
    temp2-=temp;
    for(int i=vanishingPt; i<currFrame.rows; i++)
        for(int j=0; j<currFrame.cols; j++)
        {
            temp.at<uchar>(i,j)    = 0;
            temp2.at<uchar>(i,j)   = 0;
        }

    //    imshow("currframe", currFrame);
    blobRemoval();  //use OptimizeFilter instead
    //    OptimizeFilter();
}

void NightLaneDetect::markLane()
{
    for(int i=vanishingPt; i<currFrame.rows; i++)
    {
        //IF COLOUR IMAGE IS GIVEN then additional check can be done
        // lane markings RGB values will be nearly same to each other(i.e without any hue)

        //min lane width is taken to be 5
        laneWidth =5+ maxLaneWidth*(i-vanishingPt)/ROIrows;
        for(int j=laneWidth; j<currFrame.cols- laneWidth; j++)
        {

            diffL = currFrame.at<uchar>(i,j) - currFrame.at<uchar>(i,j-laneWidth);
            diffR = currFrame.at<uchar>(i,j) - currFrame.at<uchar>(i,j+laneWidth);
            diff  =  diffL + diffR - abs(diffL-diffR);

            //1 right bit shifts to make it 0.5 times
            diffThreshLow = currFrame.at<uchar>(i,j)>>1;
            //diffThreshTop = 1.2*currFrame.at<uchar>(i,j);

            //both left and right differences can be made to contribute
            //at least by certain threshold (which is >0 right now)
            //total minimum Diff should be atleast more than 5 to avoid noise
            if (diffL>0 && diffR >0 && diff>1/*5*/)
                if(diff>=diffThreshLow /*&& diff<= diffThreshTop*/ )
                    temp.at<uchar>(i,j)=255;
        }
    }



    Mat combined_img=Mat::zeros(for_draw.rows,for_draw.cols,for_draw.type());
    Mat combined_mask=Mat::zeros(for_draw.rows,for_draw.cols,CV_8UC1);

    //    vector<SegMent> seg_ms;
    //    int valid_roi_width=roi_rect.width;
    //    int valid_roi_height=roi_rect.height;
    //    int start_x=roi_rect.x;
    //    int start_y=for_draw.rows;
    //    std::cout<<"valid_roi_height="<<valid_roi_height<<",start_y="<<start_y<<",vanishingPt.y="<<vanishingPt<<std::endl;
    //    int seg_cnt=5;
    fill_segment_mat(for_draw,seg_ms);

    filter_colors(combined_mask,combined_img,seg_ms);
    bitwise_or(temp,combined_mask,temp);
    //    imshow("combined_img",combined_img);
    //    imshow("combined_msk",combined_mask);

    //增加颜色的判断，将两者的选择做一个或操作
    //    Vec3b default_white_color(255,255,255);
    //    Vec3b default_yellow_color(26,147,202);
    //    color_detect.SetTargetColor(default_yellow_color);
    //    color_detect.SetMinDistance(60);

    //    std::cout<<"currFrame.channels="<<currFrame.channels()<<endl;

    //    Mat part_temp=temp(roi_rect);
    //    Mat part=for_draw(roi_rect);
    //    color_detect.process(part,&yellow_mask);
    //    Mat aft_or;
    //    bitwise_or(part_temp,yellow_mask,part_temp);
    //    addWeighted(part_temp,1.,yellow_mask,1.,0,part_temp);

    //    imshow("yellow_mask",yellow_mask);
    //    imshow("temp",temp);
    //    imshow("part_temp",part_temp);
    //    imshow("aft_or",aft_or);
    //    waitKey(0);


    //----边缘-----//


    //erode
    Mat element;
    int dilation_size = 1;
    int dilation_type=MORPH_ELLIPSE;//MORPH_RECT,MORPH_CROSS,MORPH_ELLIPSE
    element = getStructuringElement( dilation_type,
                                     Size( 2*dilation_size + 1, 2*dilation_size+1 ),
                                     Point( dilation_size, dilation_size ) );
    ///膨胀操作
    Mat before=temp.clone();
    //    imshow("bef",before);
    erode( temp, temp, element );


    dilation_size=2;
    element = getStructuringElement( dilation_type,
                                     Size( 2*dilation_size + 1, 2*dilation_size+1 ),
                                     Point( dilation_size, dilation_size ) );
    dilate( temp, temp, element );


    dilation_size=1;
    element = getStructuringElement( dilation_type,
                                     Size( 2*dilation_size + 1, 2*dilation_size+1 ),
                                     Point( dilation_size, dilation_size ) );
    erode( temp, temp, element );

}



void NightLaneDetect::OptimizePolyfit(){
    //    std::cout<<">>>>>>OptimizePolyfit"<<std::endl;
    char name[64];
    //简单策略
    //从x中心，向上搜素，每一行，都搜左边3个点，搜右边3个点


    vector<Point> left_ps;
    vector<Point> right_ps;
    vector<int> tmp_x;
    int tmp_t_x;
    int step_y=roi_rect.height/20;
    int idx=0;
    for(int r=temp2.rows-1;r>roi_rect.tl().y;r-=step_y){
        //搜左边
        tmp_x.resize(0);
        tmp_t_x=0;
        for(int c=temp2.cols/2;c>=0;c-=5){
            if(temp2.at<uchar>(r,c)==255){
                tmp_x.push_back(c);
                tmp_t_x+=c;
            }
        }

        if(tmp_x.size()>0){
            left_ps.push_back(Point(tmp_t_x/tmp_x.size(),r));
            circle(for_draw,Point(tmp_t_x/tmp_x.size(),r),3,Scalar(255,255,0),2);
            //            std::cout<<"left_p="<<Point(tmp_t_x/tmp_x.size(),r)<<std::endl;
            //            sprintf(name,"%d",idx++);
            //            putText(for_draw,name,Point(tmp_t_x/tmp_x.size()+100,r),CV_FONT_HERSHEY_COMPLEX,0.5,1);
        }


        //搜右边
        tmp_x.resize(0);
        tmp_t_x=0;
        for(int c=temp2.cols/2;c<temp2.cols;c+=5){
            if(temp2.at<uchar>(r,c)==255){
                tmp_x.push_back(c);
                tmp_t_x+=c;
            }
        }

        if(tmp_x.size()>0){
            right_ps.push_back(Point(tmp_t_x/tmp_x.size(),r));
            circle(for_draw,Point(tmp_t_x/tmp_x.size(),r),3,Scalar(255,255,0),2);

        }
    }

    //函数拟合

    int order=2;

    //----------left------------------//
    if(left_ps.size()<=2){
        order=1;
    }
    std::cout<<"left_ps.size=="<<left_ps.size()<<std::endl;
    std::cout<<"right_ps.size=="<<right_ps.size()<<std::endl;
    if(left_ps.size()>0){
        Mat left_dst=Mat::zeros(order+1,1,CV_32FC1);


        Mat left_input_x(left_ps.size(),1,CV_32FC1);
        Mat left_input_y(left_ps.size(),1,CV_32FC1);



        for(int i=0;i<left_ps.size();i++){
            left_input_x.at<float>(i,0)=left_ps[i].x;
            left_input_y.at<float>(i,0)=left_ps[i].y;
        }

        polyfit(left_input_x,left_input_y,left_dst,order);
        vector<Point> dst_left_ps;
        for(int r=0;r<for_draw.cols/2;r+=10){
            float y=0;
            for(int j=0;j<=order;j++){
                y+=left_dst.at<float>(0,j)*pow(r,j);
            }
            if(y>=roi_rect.y && y<for_draw.rows){
                dst_left_ps.push_back(Point(r,y));
                //                circle(for_draw,Point(r,y),3,Scalar(255,248,220),2);
                //                std::cout<<"get pt:"<<Point(r,y)<<endl;
            }

        }
        for(int i=1;i<dst_left_ps.size()-1;i++){
            line(for_draw,dst_left_ps[i],dst_left_ps[i+1],Scalar(255,144,30),2);
            //            std::cout<<"from pt:"<<dst_left_ps[i]<<" to pt : "<<dst_left_ps[i+1]<<std::endl;
        }
    }



    //    return;
    //-----------right------------//
    if(right_ps.size()>0){
        Mat right_dst=Mat::zeros(order+1,1,CV_32FC1);
        Mat right_input_x(right_ps.size(),1,CV_32FC1);
        Mat right_input_y(right_ps.size(),1,CV_32FC1);

        for(int i=0;i<right_ps.size();i++){
            right_input_x.at<float>(i,0)=right_ps[i].x;
            right_input_y.at<float>(i,0)=right_ps[i].y;
        }

        polyfit(right_input_x,right_input_y,right_dst,order);
        vector<Point> dst_right_ps;
        for(int r=for_draw.cols/2;r<for_draw.cols;r+=10){
            float y=0;
            for(int j=0;j<=order;j++){
                y+=right_dst.at<float>(0,j)*pow(r,j);
            }
            if(y>=roi_rect.y && y<for_draw.rows){
                dst_right_ps.push_back(Point(r,y));
            }

        }
        for(int i=1;i<dst_right_ps.size()-1;i++){
            line(for_draw,dst_right_ps[i],dst_right_ps[i+1],Scalar(60,179,113),2);
        }
    }

}

void NightLaneDetect::OptimizeFilter(){
    markLane();

    // find all contours in the binary image
    temp.copyTo(binary_image);
    findContours(binary_image, contours,
                 hierarchy, CV_RETR_CCOMP,
                 CV_CHAIN_APPROX_SIMPLE);

    left_rects.clear();
    right_rects.clear();



    if(contours.size()>0){
        for(int i=0;i<contours.size();i++){
            isValidContour(contours,i);
        }


        OptimizePolyfit();
    }

}


/**
 * @brief NightLaneDetect::isValidContour
 * 对于上一步提取出来的contour,进行过滤，保留可信度高的
 * 判断依据：
 * 1 按y轴方向（按距离远近），分段检测，每一段都近似为直线
 *  距离近的（靠近图片底部），斜率可以大一些，面积大一些，越远则越可以越小
 * 2 距离近的那一段对距离远的后一段，具有预测作用。
 * 3 距离最近的那一段的斜率应该基本保持不变，如果有变化，说明车在转弯，或者偏移原方向
 * 4 是否增加俯视变换？
 *
 *
 * @param contour
 * @return
 */
bool NightLaneDetect::isValidContour(vector<vector<Point> >& contours,int index){

    int valid_cont=0; //0 means invalid,奇数有效，偶数表示不符合条件的各情况
    char name[64];
    //====conditions for removing contours====//

    contour_area = contourArea(contours[index]) ;
    //    float area_ratio=contour_area*1.0/(for_draw.rows*for_draw.cols)*100;


    //    if(contour_area>30){

    //        drawContours(for_draw, contours,index, Scalar(255), CV_FILLED, 8);
    //        drawContours(temp2, contours,index, Scalar(255), CV_FILLED, 8);
    //        rotated_rect    = minAreaRect(contours[index]);
    //        Point2f rect_points[4];
    //        rotated_rect.points( rect_points );
    //        for( int j = 0; j < 4; j++ ){
    //            line( for_draw, rect_points[j], rect_points[(j+1)%4], Scalar(0,255,255), 1, 8 );
    //        }
    //        sprintf(name,"area:%f,angle=%f",contour_area,rotated_rect.angle);
    //        putText(for_draw,name,rect_points[0],CV_FONT_HERSHEY_COMPLEX,0.5,1);
    //    }


    if(contour_area > minSize){
        rotated_rect    = minAreaRect(contours[index]);
        sz              = rotated_rect.size;
        bounding_width  = sz.width;
        bounding_length = sz.height;

        //端点
        Point2f rect_points[4];
        rotated_rect.points( rect_points );





        //openCV selects length and width based on their orientation
        //so angle needs to be adjusted accordingly
        blob_angle_deg = rotated_rect.angle;
        int blob_plus_90=0;
        if (bounding_width < bounding_length){
            blob_angle_deg = 90 + blob_angle_deg;
            blob_plus_90=1;
        }



        //判断该contour在哪个区域
        set<int> area_seq;
        vector<int> area_seq_vec;
        float ref_seg_total_area;//横跨的segment的总面积
        float ref_seg_total_height;//
        int min_x=this->currFrame.cols,max_x=-1;
        int min_y=this->currFrame.rows,max_y=-1;
        for(int i=0;i<seg_ms.size();i++){
            for(int j=0;j<4;j++){//4 points
                if(seg_ms[i].range.contains(rect_points[j])){
                    if(area_seq.find(i)== area_seq.end()){
                        area_seq.insert(i);
                        area_seq_vec.push_back(i);

                        ref_seg_total_area+=seg_ms[i].seg_area;
                        ref_seg_total_height+=seg_ms[i].range.height;
                    }
                }
                if(rect_points[j].x>max_x){
                    max_x=rect_points[j].x;
                }
                if(rect_points[j].x<min_x){
                    min_x=rect_points[j].x;
                }
                if(rect_points[j].y>max_y){
                    max_y=rect_points[j].y;
                }
                if(rect_points[j].y<min_y){
                    min_y=rect_points[j].y;
                }
            }
            //            std::cout<<"i="<<i
            //                    <<",min_left_slope="<<seg_ms[i].min_left_slope
            //                   <<",max_left_slope="<<seg_ms[i].max_left_slope
            //                  <<",min_right_slope="<<seg_ms[i].min_right_slope
            //                 <<",max_right_slope="<<seg_ms[i].max_right_slope
            //                <<std::endl;

        }

        float area_ratio = contour_area/ref_seg_total_area;
        //        std::cout<<"ref_seg_total_area="<<ref_seg_total_area<<",area_ratio="<<area_ratio<<std::endl;


        //if such big line has been detected then it has to be a (curved or a normal)lane
        //        if(bounding_length>longLane || bounding_width >longLane ){
        //            //如果面积太大，也是不对的
        //            if(contour_area> maxLaneWidth*ref_seg_total_height){
        //                valid_cont=10;
        ////                std::cout<<"too large...."<<std::endl;

        //            }else{
        //                 valid_cont=1;
        //            }

        //        }else{
#if 1


        //---------看角度-------
        //越靠近中心位置的，斜率绝对值可以越大；如果已经确认了最近的右车道线，如果在该车道线的右边有检测到一个怀疑线，如果其斜率更大，说明这个更右边的就不是
        //越靠近底部的，可能会有90度；越靠近顶部的，不可能会有90度，且如果是弯道的话，角度应该越來越小，否则角度不变；
        //靠近底部地方，处于x中心线左边的，斜率不可能是大于0的（图像坐标的大的y值的端点的x值，比小的y值端点的x值大，就是斜向左）；同样处于x中心线右边的，斜率不可能是小于0的

        //---------看面积--------
        //靠近顶端的，面积可以不要这么大

        //看宽高比例 ，规定，横向为宽，竖向为高

        int pict_cent_x=this->currFrame.cols/2;
        bool seg_valid=false;
        if(blob_angle_deg==0){
            valid_cont=2;
        }else if(blob_angle_deg==90){
            valid_cont=10;
        }
        if(blob_angle_deg >0){
            //at right side-----maybe

            //前3个区域考虑
            if(area_seq_vec[0] <3 && (max_x<=pict_cent_x || min_x<=pict_cent_x)){
                valid_cont=20;
            }
            //之考虑第一个最靠近底边的区域
            else if(abs(blob_angle_deg)>= seg_ms[area_seq_vec[0]].min_left_slope
                    && abs(blob_angle_deg) <= seg_ms[area_seq_vec[0]].max_left_slope){


                seg_valid=true;
                valid_cont=3;
            }else{
                valid_cont=4;
            }

        }else{
            //at left side -- maybe
            ////前3个区域考虑,因为如果是后面的，由于弯道的存在，是会可以存在的
            if(area_seq_vec[0] <3 && (max_x>=pict_cent_x || min_x>=pict_cent_x)){
                valid_cont=40;
            }
            if(abs(blob_angle_deg)>= seg_ms[area_seq_vec[0]].min_right_slope
                    && abs(blob_angle_deg) <= seg_ms[area_seq_vec[0]].max_right_slope){
                seg_valid=true;
                valid_cont=5;
            }else{
                valid_cont=6;
            }
        }


        if(seg_valid % 2 ==1){
            if ((bounding_length/bounding_width)>=ratio ||
                    (bounding_width/bounding_length)>=ratio ||
                    (contour_area< smallLaneArea &&
                     ((contour_area/(bounding_width*bounding_length)) > .75) &&
                     ((bounding_length/bounding_width)>=2 || (bounding_width/bounding_length)>=2)
                     )){

            }else{
                valid_cont=8;
                std::cout<<"seg_valid ok! but not fill other conditions!"<<std::endl;
            }
        }

        //            if(seg_valid){
        //                if ((bounding_length/bounding_width)>=ratio ||
        //                        (bounding_width/bounding_length)>=ratio ||
        //                        (contour_area< smallLaneArea &&
        //                         ((contour_area/(bounding_width*bounding_length)) > .75) &&
        //                         ((bounding_length/bounding_width)>=2 || (bounding_width/bounding_length)>=2)
        //                         )
        //                        ){
        //                    valid_cont=7;
        //                }else{
        //                    valid_cont=8;
        //                    std::cout<<"seg_valid ok! but not fill other conditions!"<<std::endl;
        //                }
        //            }else{

        //            }

        //        }


        //         line(for_draw,Poi nt(for_draw.cols/2,0),Point(for_draw.cols/2,for_draw.rows-1),Scalar(0,255,0),1);
        //draw
        Scalar color;
        if(valid_cont==1){
            color=Scalar(0,255,0);
        }else if(valid_cont==2){
            color=Scalar(255,0,255);
        }
        else if(valid_cont==3 || valid_cont==5 ||  valid_cont==7){
            color=Scalar(255,0,0);
        }else if(valid_cont==8){
            color=Scalar(147,20,255);
        }else{
            color=Scalar(0,0,255);
        }


        if(valid_cont % 2 ==1){

            //只是简单的进行分类，不考虑转弯时末端的情况
            valid_contour_index.push_back(index);
            if(blob_angle_deg>0){
                right_rects.push_back(rotated_rect);
                right_rect_aux_infos.push_back(rect_aux_info(min_x,max_x,min_y,max_y,index));
            }else{
                left_rects.push_back(rotated_rect);
                left_rect_aux_infos.push_back(rect_aux_info(min_x,max_x,min_y,max_y,index));
                std::cout<<"add left_rect_aux_infos index="<<index<<std::endl;

            }

            drawContours(for_draw, contours,index, color, CV_FILLED, 8);
            drawContours(temp2, contours,index, Scalar(255), CV_FILLED, 8);
            //line
            Point2f rect_points[4];
            rotated_rect.points( rect_points );


            for( int j = 0; j < 4; j++ ){
                line( for_draw, rect_points[j], rect_points[(j+1)%4], color, 2, 8 );
                std::cout<<"rectPoint "<<j<<"="<<rect_points[j]<<std::endl;
            }
            Point ct((rect_points[0].x+rect_points[2].x)/2*2/3,(rect_points[0].y+rect_points[2].y)/2);


            //            sprintf(name,"0:(%f,%f)",rect_points[0].x,rect_points[0].y);
            //            putText(for_draw,name,rect_points[0],CV_FONT_HERSHEY_COMPLEX,0.5,1);
            //            sprintf(name,"1:(%f,%f)",rect_points[1].x,rect_points[1].y);
            //            putText(for_draw,name,rect_points[1],CV_FONT_HERSHEY_COMPLEX,0.5,1);
            //            sprintf(name,"2:(%f,%f)",rect_points[2].x,rect_points[2].y);
            //            putText(for_draw,name,rect_points[2],CV_FONT_HERSHEY_COMPLEX,0.5,1);
            //            sprintf(name,"3:(%f,%f)",rect_points[3].x,rect_points[3].y);
            //            putText(for_draw,name,rect_points[3],CV_FONT_HERSHEY_COMPLEX,0.5,1);

            vector<double> xs,ys;
            if(blob_plus_90){
                //右边,斜率大于0
                std::cout<<"\n-------------------------"
                           <<"\nright ----0-1("<<rect_points[0].x<<","<<rect_points[0].y<<")->("<<rect_points[1].x<<","<<rect_points[1].y<<"),slope="<<(rect_points[1].y-rect_points[0].y)/(rect_points[1].x-rect_points[0].x)
                        <<" 2-3("<<rect_points[0].x<<","<<rect_points[0].y<<")->("<<rect_points[1].x<<","<<rect_points[1].y<<"),slope="<<(rect_points[3].y-rect_points[2].y)/(rect_points[3].x-rect_points[2].x)
                        <<std::endl;
                xs.push_back(rect_points[1].x);
                xs.push_back(rect_points[0].x);

                ys.push_back(rect_points[1].y);
                ys.push_back(rect_points[0].y);

                LeastSquare lsq01(xs,ys);
                std::cout<<"lsq01,m="<<lsq01.getM()<<",b="<<lsq01.getB()<<std::endl;


                xs.resize(0);
                ys.resize(0);
                xs.push_back(rect_points[3].x);
                xs.push_back(rect_points[2].x);

                ys.push_back(rect_points[3].y);
                ys.push_back(rect_points[2].y);

                LeastSquare lsq23(xs,ys);
                std::cout<<"lsq23,m="<<lsq23.getM()<<",b="<<lsq23.getB()<<std::endl;


                xs.resize(0);
                ys.resize(0);
                xs.push_back((rect_points[0].x+rect_points[3].x)/2);
                xs.push_back((rect_points[1].x+rect_points[2].x)/2);

                ys.push_back((rect_points[0].y+rect_points[3].y)/2);
                ys.push_back((rect_points[1].y+rect_points[2].y)/2);

                LeastSquare lsqmid(xs,ys);
                std::cout<<"lsqmid,m="<<lsqmid.getM()<<",b="<<lsqmid.getB()<<std::endl;




            }else{

            }


            sprintf(name,"%d;%f;%f;%f;"
                    //%f,(%d,%d),(%d,%d),(%d,%d),(%d,%d)"/*,%d,%f,%f*/
                    ,blob_plus_90
                    ,bounding_width
                    ,bounding_length
                    ,blob_angle_deg
                    //                    contour_area*1.0/(for_draw.rows*for_draw.cols)*100,
                    //                    rect_points[0].x,rect_points[0].y,
                    //                    rect_points[1].x,rect_points[1].y,
                    //                    ,rect_points[2].x,rect_points[2].y
                    //                    ,rect_points[3].x,rect_points[3].y

                    /*,valid_cont,contour_area,rotated_rect.angle*/);
            putText(for_draw,name,ct,CV_FONT_HERSHEY_COMPLEX,0.5,1);



        }

#else
        //angle of orientation of blob should not be near horizontal or vertical
        //vertical blobs are allowed only near center-bottom region, where centre lane mark is present
        //length:width >= ratio for valid line segments
        //if area is very small then ratio limits are compensated
        if ((blob_angle_deg <-10 || blob_angle_deg >-10 ) &&
                ((blob_angle_deg > -70 && blob_angle_deg < 70 ) ||
                 (rotated_rect.center.y > vertical_top &&
                  rotated_rect.center.x > vertical_left &&
                  rotated_rect.center.x < vertical_right
                  )
                 )
                ){
            //                    std::cout<<"bounding_length="<<bounding_length<<",bounding_width="<<bounding_width
            //                            <<",ratio="<<ratio<<",contour_area="<<contour_area<<",smallLaneArea="<<smallLaneArea
            //                           <<",bounding_length/bounding_width="<<bounding_length/bounding_width;
            if ((bounding_length/bounding_width)>=ratio ||
                    (bounding_width/bounding_length)>=ratio ||
                    (contour_area< smallLaneArea &&
                     ((contour_area/(bounding_width*bounding_length)) > .75) &&
                     ((bounding_length/bounding_width)>=2 || (bounding_width/bounding_length)>=2)
                     )
                    ){
                valid_cont=2;
                //                        std::cout<<"----------take"<<std::endl;
                drawContours(for_draw, contours,index, Scalar(0,0,255), CV_FILLED, 8);
                drawContours(temp2, contours,index, Scalar(255), CV_FILLED, 8);
                //line
                Point2f rect_points[4];
                rotated_rect.points( rect_points );
                for( int j = 0; j < 4; j++ ){
                    line( for_draw, rect_points[j], rect_points[(j+1)%4], Scalar(0,0,255), 2, 8 );
                }

            }else{
                //                        std::cout<<"----------drop"<<std::endl;
            }
        }
#endif


    }


}


void NightLaneDetect::blobRemoval()
{



    markLane();

    // find all contours in the binary image
    temp.copyTo(binary_image);
    findContours(binary_image, contours,
                 hierarchy, CV_RETR_CCOMP,
                 CV_CHAIN_APPROX_SIMPLE);

    left_rects.resize(0);
    right_rects.resize(0);
    left_rect_aux_infos.resize(0);
    right_rect_aux_infos.resize(0);
    valid_contour_index.resize(0);

    std::cout<<"contours.size="<<contours.size()<<std::endl;

    char name[64];
    // for removing invalid blobs
    if (!contours.empty())
    {

        std::cout<<"----------------------------------\n"
                <<"------------------------------------\n";
        vector<vector<Point> > contours_poly( contours.size() );
        for (size_t i=0; i<contours.size(); ++i)
        {

            this->isValidContour(contours,i);

#if 0
            int valid_cont=0; //0 means invalid
            //====conditions for removing contours====//

            contour_area = contourArea(contours[i]) ;

            //            std::cout<<"contour_area="<<contour_area<<",minSize="<<minSize<<std::endl;
            //blob size should not be less than lower threshold

            //            if(contour_area>30){

            //                rotated_rect    = minAreaRect(contours[i]);
            //                Point2f rect_points[4];
            //                rotated_rect.points( rect_points );
            //                for( int j = 0; j < 4; j++ ){
            //                    line( for_draw, rect_points[j], rect_points[(j+1)%4], Scalar(0,255,255), 1, 8 );
            //                }
            //                sprintf(name,"area:%f,angle=%f",contour_area,rotated_rect.angle);
            //                putText(for_draw,name,rect_points[0],CV_FONT_HERSHEY_COMPLEX,0.5,1);
            //            }

            if(contour_area > minSize)
            {
                rotated_rect    = minAreaRect(contours[i]);
                sz              = rotated_rect.size;
                bounding_width  = sz.width;
                bounding_length = sz.height;


                //openCV selects length and width based on their orientation
                //so angle needs to be adjusted accordingly
                blob_angle_deg = rotated_rect.angle;
                if (bounding_width < bounding_length){
                    blob_angle_deg = 90 + blob_angle_deg;
                }




                //if such big line has been detected then it has to be a (curved or a normal)lane
                if(bounding_length>longLane || bounding_width >longLane)
                {

                    valid_cont=1;

                }

                //angle of orientation of blob should not be near horizontal or vertical
                //vertical blobs are allowed only near center-bottom region, where centre lane mark is present
                //length:width >= ratio for valid line segments
                //if area is very small then ratio limits are compensated
                else if ((blob_angle_deg <-10 || blob_angle_deg >-10 ) &&
                         ((blob_angle_deg > -70 && blob_angle_deg < 70 ) ||
                          (rotated_rect.center.y > vertical_top &&
                           rotated_rect.center.x > vertical_left && rotated_rect.center.x < vertical_right)))
                {


                    //                    std::cout<<"bounding_length="<<bounding_length<<",bounding_width="<<bounding_width
                    //                            <<",ratio="<<ratio<<",contour_area="<<contour_area<<",smallLaneArea="<<smallLaneArea
                    //                           <<",bounding_length/bounding_width="<<bounding_length/bounding_width;
                    if ((bounding_length/bounding_width)>=ratio || (bounding_width/bounding_length)>=ratio
                            ||(contour_area< smallLaneArea &&  ((contour_area/(bounding_width*bounding_length)) > .75) &&
                               ((bounding_length/bounding_width)>=2 || (bounding_width/bounding_length)>=2)))
                    {

                        valid_cont=2;
                        //                        std::cout<<"----------take"<<std::endl;

                    }else{
                        //                        std::cout<<"----------drop"<<std::endl;
                    }
                }
            }


            //valid one
            if(valid_cont>0 /*&& (abs(blob_angle_deg)>35 && abs(blob_angle_deg)<50)*/){
                //------------draw  begin -----//
                Point2f rect_points[4];
                rotated_rect.points( rect_points );
                Point ct((rect_points[0].x+rect_points[2].x)/2,(rect_points[0].y+rect_points[2].y)/2);

                //                //在原图以很小的阈值进行fillflood，填充
                //                //然后再次找contours，会合并一些
                //                int loDiff = 10, upDiff = 10;
                //                int newMaskVal = 255;
                //                Scalar newVal = Scalar(255,0,0);
                //                Rect ccomp;
                //                int lo = loDiff;
                //                int up = upDiff;
                //                int flags = 8 + (newMaskVal << 8) + CV_FLOODFILL_FIXED_RANGE;


                //                Mat mask;
                //                mask.create(temp3.rows, temp3.cols, CV_8UC1);
                ////                cvtColor(mask, mask, COLOR_BGR2GRAY);
                //                cv::resize(mask,mask,Size(temp3.cols+2,temp3.rows+2));

                //                mask-=mask;
                //                //mask 只包含了Rotated_rect,即是说只能在这个范围内fillflood，免得错的太离谱
                //                approxPolyDP( Mat(contours[i]), contours_poly[i], 3, true);
                //                for(int pi=0;pi<contours_poly[i].size();pi++){
                //                    Vec3i pi_v=ori_pic.at<Vec3i>(contours_poly[i][pi].y,contours_poly[i][pi].x);
                //                    //只填充那些接近白色和黄色的点
                //                    int dist_white=static_cast<int>(cv::norm<int,3>(Vec3i(pi_v[0]-white.at<Vec3i>(0,0)[0],
                //                                              pi_v[1]-white.at<Vec3i>(0,0)[1],pi_v[2]-white.at<Vec3i>(0,0)[2])));

                //                    int dist_yellow=static_cast<int>(cv::norm<int,3>(Vec3i(pi_v[0]-yellow.at<Vec3i>(0,0)[0],
                //                                              pi_v[1]-yellow.at<Vec3i>(0,0)[1],pi_v[2]-yellow.at<Vec3i>(0,0)[2])));

                //                    if(dist_white<10 || dist_yellow<10){

                //                    }else{
                //                        std::cout<<"don't fillflood------!!!!"<<std::endl;
                //                        continue;
                //                    }


                //                    //或者统计rotated_rect内的颜色的平均值
                //                    floodFill(temp3, mask, contours_poly[i][pi], newVal, &ccomp, Scalar(lo, lo, lo), Scalar(up, up, up), flags);
                //                }
                ////                imshow("mask",mask);



                drawContours(currFrame, contours,i, Scalar(255), CV_FILLED, 8);

                //                fillPoly(temp3,contours_poly[i],Scalar(0,128,255));
                //                drawContours( temp2, contours_poly, i, Scalar(255), 1, 8, vector<Vec4i>(), 0, Point() );
                drawContours(temp2, contours,i, Scalar(255), CV_FILLED, 8);
                //                drawContours(for_draw, contours,i, Scalar(255), CV_FILLED, 8);



                //line
                //                for( int j = 0; j < 4; j++ ){
                //                    line( for_draw, rect_points[j], rect_points[(j+1)%4], Scalar(0,0,255), 2, 8 );
                //                }

                sprintf(name,"area:%f,angle=%f",contour_area,blob_angle_deg);

                putText(for_draw,name,ct
                        ,CV_FONT_HERSHEY_COMPLEX,0.5,1);
                circle(for_draw,ct,1,Scalar(0,255,0),1);


                //------------draw  end -----//


                //right or left
                if(blob_angle_deg>0){
                    right_rects.push_back(rotated_rect);
                }else{
                    left_rects.push_back(rotated_rect);
                }

                //按照segments 将其分割为一段段的rotated_rect(本来是一个大的rotated_rect的）
                //TODO


            }
#endif
        }
    }


    static float global_avg_ms=0;
    static float global_avg_bs=0;

    //--看右边的平均斜率，平均截距产生的直线---//
    if(right_rects.size()>0){
        vector<double> xs,ys;
        double ms=0,bs=0;
        int valid_cnt=0;
        for(int i=0;i<right_rects.size();i++){
            Point2f rect_points[4];
            right_rects[i].points( rect_points );
            xs.resize(0);
            ys.resize(0);
            xs.push_back((rect_points[0].x+rect_points[3].x)/2);
            xs.push_back((rect_points[1].x+rect_points[2].x)/2);

            ys.push_back((rect_points[0].y+rect_points[3].y)/2);
            ys.push_back((rect_points[1].y+rect_points[2].y)/2);

            LeastSquare lsqmid(xs,ys);
//            std::cout<<"lsqmid,m="<<lsqmid.getM()<<",b="<<lsqmid.getB()<<std::endl;

            //标准
            if(lsqmid.getM()<0.6){
                cout<<"M="<<lsqmid.getM()<<",drop!\n";
                continue;
            }
            ms+=lsqmid.getM();
            bs+=lsqmid.getB();
            valid_cnt++;

        }
        if(valid_cnt>0){
            double avg_m=ms/valid_cnt;
            double avg_b=bs/valid_cnt;


            float y1=for_draw.rows-2;
            float x1=(y1-avg_b)/avg_m;

            float y2=0;
            float x2=(y2-avg_b)/avg_m;
            line(for_draw,Point(x1,y1),Point(x2,y2),Scalar(0,255,0),2);
            std::cout<<"use m="<<avg_m<<",use b="<<avg_b<<endl;

            global_avg_ms=avg_m;
            global_avg_bs=avg_b;
        }else{

            std::cout<<"use global m instead..."<<endl;
            if(global_avg_ms>0){
                float y1=for_draw.rows-2;
                float x1=(y1-global_avg_bs)/global_avg_ms;

                float y2=0;
                float x2=(y2-global_avg_bs)/global_avg_ms;
                line(for_draw,Point(x1,y1),Point(x2,y2),Scalar(0,255,255),2);
            }
        }


    }

    //        //去除同方向斜率，y长度较短、被另外一个差不多斜率的矩形范围的y包围的矩形框
    //        std::cout<<"left_rects.size="<<left_rects.size()<<std::endl;
    //        std::cout<<"right_rects.size="<<right_rects.size()<<std::endl;

    //        if(left_rects.size()>0 ){
    //            //left

    //            vector<int> left_flags;
    //            left_flags.resize(left_rects.size());
    //            for(int i=0;i<left_flags.size();i++){
    //                left_flags[i]=0;
    //            }

    //            for(int i=0;i<left_flags.size()-1;i++){
    //                for(int j=i+1;j<left_flags.size();j++){
    //                    std::cout<<"left----"<<std::endl;
    //                    if(left_flags[j]==1){
    //                        continue;
    //                    }
    //                    if(left_rect_aux_infos[i].contains_y(left_rect_aux_infos[j])){
    //                        left_flags[j]=1;
    //                    }else if(left_rect_aux_infos[j].contains_y(left_rect_aux_infos[i])){
    //                        left_flags[i]=1;
    //                        break;//这个i位置的已经被包含了，不需要继续了
    //                    }
    //                }
    //            }

    //            Scalar color;
    //            for(int i=0;i<left_rects.size();i++){
    //                std::cout<<"left_rect_aux_infos["<<i<<"].index="<<left_rect_aux_infos[i].index<<std::endl;
    //                if(left_flags[i]==1){
    //                    color=Scalar(0,0,255);
    //                }else{
    //                    color=Scalar(255,0,0);
    //                    std::cout<<"-111111111111111---left_rect_aux_infos[i].index="<<left_rect_aux_infos[i].index<<std::endl;
    //                    drawContours(temp2, contours,left_rect_aux_infos[i].index, Scalar(255), CV_FILLED, 8);
    //                     std::cout<<"-22222222222222---"<<std::endl;
    //                }
    //                drawContours(for_draw, contours,left_rect_aux_infos[i].index, color, CV_FILLED, 8);
    //                //line
    //                Point2f rect_points[4];
    //                left_rects[i].points( rect_points );
    //                for( int j = 0; j < 4; j++ ){
    //                    line( for_draw, rect_points[j], rect_points[(j+1)%4],color, 2, 8 );
    //                }

    //            }

    //        }

    //        if(right_rects.size()>0){
    //            //right
    //            vector<int> right_flags;
    //            right_flags.resize(right_rects.size());
    //            for(int i=0;i<right_flags.size();i++){
    //                right_flags[i]=0;
    //            }

    //            for(int i=0;i<right_flags.size()-1;i++){
    //                for(int j=i+1;j<right_flags.size();j++){
    //                    std::cout<<"right----"<<std::endl;
    //                    if(right_flags[j]==1){
    //                        continue;
    //                    }
    //                    if(right_rect_aux_infos[i].contains_y(right_rect_aux_infos[j])){
    //                        right_flags[j]=1;
    //                    }else if(right_rect_aux_infos[j].contains_y(right_rect_aux_infos[i])){
    //                        right_flags[i]=1;
    //                        break;//这个i位置的已经被包含了，不需要继续了
    //                    }
    //                }
    //            }


    //            //draw
    //            Scalar color;
    //            for(int i=0;i<right_rects.size();i++){
    //                std::cout<<"right_rect_aux_infos["<<i<<"].index="<<right_rect_aux_infos[i].index<<std::endl;
    //                if(right_flags[i]==1){
    //                    color=Scalar(0,0,255);
    //                }else{
    //                    color=Scalar(255,0,0);
    //                    std::cout<<"-right----11111---right_rect_aux_infos[i].index="<<right_rect_aux_infos[i].index<<std::endl;
    //                    drawContours(temp2, contours,right_rect_aux_infos[i].index, Scalar(255), CV_FILLED, 8);
    //                     std::cout<<"-right  22222222222222---"<<std::endl;
    //                }
    //                drawContours(for_draw, contours,right_rect_aux_infos[i].index, color, CV_FILLED, 8);
    //                //line
    //                Point2f rect_points[4];
    //                left_rects[i].points( rect_points );
    //                for( int j = 0; j < 4; j++ ){
    //                    line( for_draw, rect_points[j], rect_points[(j+1)%4],color, 2, 8 );
    //                }

    //            }
    //        }







    //        OptimizePolyfit();

    //    FitLaneLine();

    imshow("temp", temp);
    imshow("temp2", temp2);
    //    imshow("lane",currFrame);
    imshow("color-draw",for_draw);


    //    imshow("temp3",temp3);
    //            waitKey(0);

}






/**
 * @brief FitLaneLine
 *
 * @param valid_pixel_mat
 */

void NightLaneDetect::FitLaneLine(){

    //可能存在有多条线
    vector<vector<Point2i> > left_points;
    vector<vector<Point2i> > right_points;

    vector<Point2i> lf_ps;
    vector<Point2i> rt_ps;

    //left
    int poly_n=1;
    double left_p[poly_n+1];
    int left_n=left_rects.size()*2;
    double left_x[left_n];
    double left_y[left_n];

    vector<double> left_vec_x;
    vector<double> left_vec_y;

    //right
    double right_p[poly_n+1];
    int right_n=right_rects.size()*2;
    double right_x[right_n];
    double right_y[right_n];

    vector<double> right_vec_x;
    vector<double> right_vec_y;


    Point2f rps[4];
    Point2i rp1,rp2;
    int i=0;

    for(RotatedRect rr:left_rects){
        rr.points( rps );
        rp1.x=(rps[0].x+rps[1].x)/2;
        rp1.y=(rps[0].y+rps[1].y)/2;

        rp2.x=(rps[2].x+rps[3].x)/2;
        rp2.y=(rps[2].y+rps[3].y)/2;

        lf_ps.push_back(rp1);
        lf_ps.push_back(rp2);

        line(for_draw,rp1,rp2,Scalar(0,0,255),2);

        left_x[i]=rp1.x;
        left_x[i+1]=rp2.x;
        left_y[i]=rp1.y;
        left_y[i+1]=rp2.y;

        left_vec_x.push_back(rp1.x);
        left_vec_x.push_back(rp2.x);
        left_vec_y.push_back(rp1.y);
        left_vec_y.push_back(rp2.y);

        ++i;
    }

    i=0;
    for(RotatedRect rr:right_rects){
        rr.points( rps );
        rp1.x=(rps[1].x+rps[2].x)/2; //注意，左边和右边的取中间点不一样
        rp1.y=(rps[1].y+rps[2].y)/2;

        rp2.x=(rps[3].x+rps[0].x)/2;
        rp2.y=(rps[3].y+rps[0].y)/2;

        rt_ps.push_back(rp1);
        rt_ps.push_back(rp2);

        line(for_draw,rp1,rp2,Scalar(0,0,255),2);

        right_x[i]=rp1.x;
        right_x[i+1]=rp2.x;
        right_y[i]=rp1.y;
        right_y[i+1]=rp2.y;

        right_vec_x.push_back(rp1.x);
        right_vec_x.push_back(rp2.x);
        right_vec_y.push_back(rp1.y);
        right_vec_y.push_back(rp2.y);

        ++i;
    }


    //曲线拟合
    //    polyfit(left_n, left_x, left_y, poly_n, left_p);
    //    polyfit(right_n, right_x, right_y, poly_n, right_p);


    //    std::cout<<"left_p:"<<left_p[0]<<","<<left_p[1]/*<<","<<left_p[2]*/<<std::endl;
    //    for(i=0;i<left_n;i++){
    //         double y=left_p[0]+left_p[1]*pow(left_x[i],1);//+left_p[2]*pow(left_x[i],2);
    //         std::cout<<" x="<<left_x[i]<<",real_y="<<left_y[i]<<",fit_y="<<y<<std::endl;

    //         circle(for_draw,Point(left_x[i],left_y[i]),10,Scalar(255,0,0),1);
    //         circle(for_draw,Point(left_x[i],y),10,Scalar(0,0,255),1);
    //    }



    LeastSquare lsq_left;
    LeastSquare lsq_right;
    if(left_vec_x.size()>=2){
        lsq_left.setData(left_vec_x,left_vec_y);
        this->left_last_lsq.setM(lsq_left.getM());
        this->left_last_lsq.setB(lsq_left.getB());
    }else if(this->left_last_lsq.getB()!=0 && this->left_last_lsq.getM()!=0){
        std::cout<<"left_vec_x.size()<2 !! use previous lsq!"<<std::endl;
        lsq_left.setM(this->left_last_lsq.getM());
        lsq_left.setB(this->left_last_lsq.getB());
        waitKey(0);
    }else{
        std::cout<<"left_vec_x 以及previous left lsq both null!!!!"<<std::endl;
        lsq_left.setM(0);
        lsq_left.setB(0);
    }

    if(right_vec_x.size()>=2){
        lsq_right.setData(right_vec_x,right_vec_y);
        this->right_last_lsq.setM(lsq_right.getM());
        this->right_last_lsq.setB(lsq_right.getB());
    }else if(this->right_last_lsq.getB()!=0 && this->right_last_lsq.getM()!=0){
        std::cout<<"right_vec_x.size()<2 !! use previous lsq!"<<std::endl;
        lsq_right.setM(this->right_last_lsq.getM());
        lsq_right.setB(this->right_last_lsq.getB());
        waitKey(0);
    }else{
        std::cout<<"right_vec_x 以及previous right lsq both null!!!!"<<std::endl;
        lsq_right.setM(0);
        lsq_right.setB(0);
    }


    vector<Point> left_p_array;
    vector<Point> right_p_array;

    if(lsq_left.isValid()){
        for(i=0;i<roi_rect.width/2;i++){
            double y=lsq_left.getY(i);
            //        std::cout<<" x="<<left_vec_x[i]<<",real_y="<<left_vec_y[i]<<",fit_y="<<y<<std::endl;
            if(y>=ori_pic.rows){
                continue;
            }
            if(y<roi_rect.tl().y){
                break;
            }
            left_p_array.push_back(Point(i,y));
        }
        //划线---------------------//
        for(i=0;i<left_p_array.size()-1;i++){
            line(for_draw,left_p_array[i],left_p_array[i+1],Scalar(255,0,0),2);
        }
    }

    if(lsq_right.isValid()){
        for(i=ori_pic.cols-1;i>0;i--){
            double y=lsq_right.getY(i);


            if(y>=ori_pic.rows){
                continue;
            }
            if(y<roi_rect.tl().y){
                break;
            }
            right_p_array.push_back(Point(i,y));
        }


        for(i=0;i<right_p_array.size()-1;i++){
            line(for_draw,right_p_array[i],right_p_array[i+1],Scalar(255,0,0),2);
        }
    }



}



Mat do_clahe(Mat& bgr_image){
    // READ RGB color image and convert it to Lab
    cv::Mat lab_image;
    cv::cvtColor(bgr_image, lab_image, CV_BGR2Lab);

    // Extract the L channel
    std::vector<cv::Mat> lab_planes(3);
    cv::split(lab_image, lab_planes);  // now we have the L image in lab_planes[0]

    // apply the CLAHE algorithm to the L channel
    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
    clahe->setClipLimit(4);
    cv::Mat dst;
    clahe->apply(lab_planes[0], dst);

    // Merge the the color planes back into an Lab image
    dst.copyTo(lab_planes[0]);
    cv::merge(lab_planes, lab_image);

    // convert back to RGB
    cv::Mat image_clahe;
    cv::cvtColor(lab_image, image_clahe, CV_Lab2BGR);

    // display the results  (you might also want to see lab_planes[0] before and after).
    //       cv::imshow("image original", bgr_image);
    //    cv::imshow("resImg2", image_clahe);
    //           cv::waitKey();
    return image_clahe;
}

void NightLaneDetect::nextFrame(Mat &nxt)
{
    Mat tmp=do_clahe(nxt);
    if(nxt.channels()>1){
        cvtColor(tmp, tmp, CV_BGR2GRAY);
    }
    //		currFrame = nxt;                        //if processing is to be done at original size

    resize(tmp ,currFrame, currFrame.size()); //resizing the input image for faster processing
    resize(nxt ,for_draw, currFrame.size());
    resize(nxt,ori_pic,currFrame.size());
    getLane();
}

Mat NightLaneDetect::getResult()
{
    return temp2;
}

void makeFromFolder(string path){

    vector<string> all_pics=getAllFilesWithPathFromDir(path);
    std::sort(all_pics.begin(),all_pics.end(),less<string>());
    Mat frame=imread(all_pics[0]);
    NightLaneDetect detect(frame,0.4);

    namedWindow("temp", 2);
    namedWindow("temp2", 2);
    //    imshow("lane",currFrame);
    namedWindow("color-draw",2);


    int idx=0;
    while(1)
    {
        if(idx>=all_pics.size()){
            idx=0;
        }
        frame=imread(all_pics[idx++]);

        //        cvtColor(frame, frame, CV_BGR2GRAY);

        //        start = clock();
        detect.nextFrame(frame);
        //        stop =clock();
        //         cout<<"fps : "<<1.0/(((double)(stop-start))/ CLOCKS_PER_SEC)<<endl;

        int key=waitKey(33);
        if(key==32){ //enter space to pause
            key=waitKey(0);
        }
        else if(key==27 || (char)key=='q'){
            break;
        }
        //        if(waitKey(10) == 27) //wait for 'esc' key press for 10 ms. If 'esc' key is pressed, break loop
        //        {
        //            cout<<"video paused!, press q to quit, any other key to continue"<<endl;
        //            if(waitKey(0) == 'q')
        //            {
        //                cout << "terminated by user" << endl;
        //                break;
        //            }
        //        }
    }
}

void makeFromVid(string path)
{
    Mat frame;
    VideoCapture cap(path); // open the video file for reading



    if ( !cap.isOpened() )  // if not success, exit program
        cout << "Cannot open the video file" << endl;

    //cap.set(CV_CAP_PROP_POS_MSEC, 300); //start the video at 300ms

    double fps = cap.get(CV_CAP_PROP_FPS); //get the frames per seconds of the video
    cout << "Input video's Frame per seconds : " << fps << endl;

    cap.read(frame);
    resize(frame,frame,Size(frame.cols/2,frame.rows/2));

    NightLaneDetect detect(frame,0.6);

    while(1)
    {
        bool bSuccess = cap.read(frame); // read a new frame from video
        if (!bSuccess)                   //if not success, break loop
        {
            cout << "Cannot read the frame from video file" << endl;
            break;
        }

        //        cvtColor(frame, frame, CV_BGR2GRAY);


        //        start = clock();
        detect.nextFrame(frame);
        //        stop =clock();
        //        cout<<"fps : "<<1.0/(((double)(stop-start))/ CLOCKS_PER_SEC)<<endl;

        int key=waitKey(10);
        std::cout<<"key="<<key<<endl;
        if(key == 27) //wait for 'esc' key press for 10 ms. If 'esc' key is pressed, break loop
        {

            break;

        }else if(key==32){ //space key
            std::cout<<"pause=========="<<std::endl;
            key=waitKey(0);//pause
        }
    }
}

