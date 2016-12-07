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


NightLaneDetect::NightLaneDetect(Mat startFrame)
{
    currFrame = startFrame;                                    //if image has to be processed at original size

    currFrame = Mat(startFrame.rows,startFrame.cols,startFrame.type(),0.0);                        //initialised the image size to 320x480
    for_draw = Mat(startFrame.rows,startFrame.cols,startFrame.type(),0.0);
    resize(startFrame,ori_pic,currFrame.size());
    //        resize(startFrame, currFrame, currFrame.size());             // resize the input to required size

    temp      = Mat(currFrame.rows, currFrame.cols, CV_8UC1,0.0);//stores possible lane markings
    temp2     = Mat(currFrame.rows, currFrame.cols, CV_8UC1,0.0);//stores finally selected lane marks



    vanishingPt    = currFrame.rows*0.5;                           //for simplicity right now
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

    //    namedWindow("lane",2);
    //    namedWindow("midstep", 2);
    //    namedWindow("currframe", 2);
    //    namedWindow("laneBlobs",2);

    getLane();
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
    for(int i=vanishingPt; i<currFrame.rows; i++)
        for(int j=0; j<currFrame.cols; j++)
        {
            temp.at<uchar>(i,j)    = 0;
            temp2.at<uchar>(i,j)   = 0;
        }

    //    imshow("currframe", currFrame);
    blobRemoval();
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

    vector<SegMent> seg_ms;
    int valid_roi_width=roi_rect.width;
    int valid_roi_height=roi_rect.height;
    int start_x=roi_rect.x;
    int start_y=for_draw.rows;
//    std::cout<<"valid_roi_height="<<valid_roi_height<<",start_y="<<start_y<<",vanishingPt.y="<<vanishingPt<<std::endl;
    int seg_cnt=5;
    get_segments(for_draw,seg_ms,valid_roi_width,valid_roi_height,start_x,start_y,seg_cnt);

    filter_colors(combined_mask,combined_img,seg_ms);

    bitwise_or(temp,combined_mask,temp);
    imshow("combined_img",combined_img);
    imshow("combined_msk",combined_mask);

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


}


void NightLaneDetect::blobRemoval()
{
    markLane();

    //    Mat lab_img;
    //    cv::cvtColor(ori_pic,lab_img,CV_BGR2Lab);
    //    cv::Mat white(1,1,CV_8UC3);
    //    white.at<cv::Vec3b>(0,0)[0]=255;//创建了一张1*1的临时图像并用目标颜色填充
    //    white.at<cv::Vec3b>(0,0)[1]=255;
    //    white.at<cv::Vec3b>(0,0)[2]=255;
    //    cvtColor(white,white,CV_BGR2Lab);

    //    cv::Mat yellow(1,1,CV_8UC3);//93,218,248
    //    yellow.at<cv::Vec3b>(0,0)[0]=93;//创建了一张1*1的临时图像并用目标颜色填充
    //    yellow.at<cv::Vec3b>(0,0)[1]=218;
    //    yellow.at<cv::Vec3b>(0,0)[2]=248;
    //    cvtColor(yellow,yellow,CV_BGR2Lab);



    //    Mat temp3=ori_pic.clone();

    //    imshow("tmp",temp);
    // find all contours in the binary image
    temp.copyTo(binary_image);
    findContours(binary_image, contours,
                 hierarchy, CV_RETR_CCOMP,
                 CV_CHAIN_APPROX_SIMPLE);

    left_rects.clear();
    right_rects.clear();

    char name[64];
    // for removing invalid blobs
    if (!contours.empty())
    {

        vector<vector<Point> > contours_poly( contours.size() );
        for (size_t i=0; i<contours.size(); ++i)
        {

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
        }
    }



//    FitLaneLine();

    imshow("temp", temp);
    imshow("temp2", temp2);
    //    imshow("lane",currFrame);
    imshow("color-draw",for_draw);


    //    imshow("temp3",temp3);
    //    waitKey(0);

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

void NightLaneDetect::nextFrame(Mat &nxt)
{
    Mat tmp=nxt;
    if(nxt.channels()>1){
        cvtColor(nxt, tmp, CV_BGR2GRAY);
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
    NightLaneDetect detect(frame);

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
    NightLaneDetect detect(frame);

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
        if(key == 27) //wait for 'esc' key press for 10 ms. If 'esc' key is pressed, break loop
        {

            break;

        }else if(key==10){
            std::cout<<"pause=========="<<std::endl;
            key=waitKey(0);//pause
        }
    }
}

