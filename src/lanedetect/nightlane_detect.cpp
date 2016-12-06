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
//        resize(startFrame, currFrame, currFrame.size());             // resize the input to required size

    temp      = Mat(currFrame.rows, currFrame.cols, CV_8UC1,0.0);//stores possible lane markings
    temp2     = Mat(currFrame.rows, currFrame.cols, CV_8UC1,0.0);//stores finally selected lane marks



    vanishingPt    = currFrame.rows/3;                           //for simplicity right now
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
    std::cout<<"valid_roi_height="<<valid_roi_height<<",start_y="<<start_y<<",vanishingPt.y="<<vanishingPt<<std::endl;
    int seg_cnt=5;
    get_segments(for_draw,seg_ms,valid_roi_width,valid_roi_height,start_x,start_y,seg_cnt);

    filter_colors(combined_mask,combined_img,seg_ms);

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

//    imshow("tmp",temp);
    // find all contours in the binary image
    temp.copyTo(binary_image);
    findContours(binary_image, contours,
                 hierarchy, CV_RETR_CCOMP,
                 CV_CHAIN_APPROX_SIMPLE);

    char name[64];
    // for removing invalid blobs
    if (!contours.empty())
    {
        for (size_t i=0; i<contours.size(); ++i)
        {
            //====conditions for removing contours====//

            contour_area = contourArea(contours[i]) ;

//            std::cout<<"contour_area="<<contour_area<<",minSize="<<minSize<<std::endl;
            //blob size should not be less than lower threshold

//            if(contour_area>30){

//            rotated_rect    = minAreaRect(contours[i]);
//                Point2f rect_points[4];
//                rotated_rect.points( rect_points );
//                for( int j = 0; j < 4; j++ ){
//                    line( for_draw, rect_points[j], rect_points[(j+1)%4], Scalar(0,255,255), 1, 8 );
//                }
//                sprintf(name,"area:%f",contour_area);
//                putText(for_draw,name,rect_points[0],CV_FONT_HERSHEY_COMPLEX,0.5,1);
//            }

            if(contour_area > 50/*minSize*/)
            {
                rotated_rect    = minAreaRect(contours[i]);
                sz              = rotated_rect.size;
                bounding_width  = sz.width;
                bounding_length = sz.height;


                //openCV selects length and width based on their orientation
                //so angle needs to be adjusted accordingly
                blob_angle_deg = rotated_rect.angle;
                if (bounding_width < bounding_length)
                    blob_angle_deg = 90 + blob_angle_deg;




                //if such big line has been detected then it has to be a (curved or a normal)lane
                if(bounding_length>longLane || bounding_width >longLane)
                {

                    Point2f rect_points[4];
                    rotated_rect.points( rect_points );
                    for( int j = 0; j < 4; j++ ){
                        line( for_draw, rect_points[j], rect_points[(j+1)%4], Scalar(0,255,0), 2, 8 );
                    }

                    drawContours(currFrame, contours,i, Scalar(255), CV_FILLED, 8);
                    drawContours(temp2, contours,i, Scalar(255), CV_FILLED, 8);
                    drawContours(for_draw, contours,i, Scalar(255), CV_FILLED, 8);
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


                    std::cout<<"bounding_length="<<bounding_length<<",bounding_width="<<bounding_width
                            <<",ratio="<<ratio<<",contour_area="<<contour_area<<",smallLaneArea="<<smallLaneArea
                           <<",bounding_length/bounding_width="<<bounding_length/bounding_width;
                    if ((bounding_length/bounding_width)>=ratio || (bounding_width/bounding_length)>=ratio
                            ||(contour_area< smallLaneArea &&  ((contour_area/(bounding_width*bounding_length)) > .75) &&
                               ((bounding_length/bounding_width)>=2 || (bounding_width/bounding_length)>=2)))
                    {
                        std::cout<<"----------take"<<std::endl;

                        Point2f rect_points[4];
                        rotated_rect.points( rect_points );
                        for( int j = 0; j < 4; j++ ){
                            line( for_draw, rect_points[j], rect_points[(j+1)%4], Scalar(0,0,255), 2, 8 );
                        }

                        drawContours(currFrame, contours,i, Scalar(255), CV_FILLED, 8);
                        drawContours(temp2, contours,i, Scalar(255), CV_FILLED, 8);
                        drawContours(for_draw, contours,i, Scalar(255), CV_FILLED, 8);
                    }else{
                        std::cout<<"----------drop"<<std::endl;
                    }
                }
            }
        }
    }
    imshow("temp", temp);
    imshow("temp2", temp2);
//    imshow("lane",currFrame);
    imshow("color-draw",for_draw);

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

        //start = clock();
        detect.nextFrame(frame);
        //stop =clock();
        // cout<<"fps : "<<1.0/(((double)(stop-start))/ CLOCKS_PER_SEC)<<endl;

        if(waitKey(10) == 27) //wait for 'esc' key press for 10 ms. If 'esc' key is pressed, break loop
        {
            cout<<"video paused!, press q to quit, any other key to continue"<<endl;
            if(waitKey(0) == 'q')
            {
                cout << "terminated by user" << endl;
                break;
            }
        }
    }
}

