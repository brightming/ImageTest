
#include "lane/lanemodule.h"

using namespace gumh;

#define PI 3.1415926


/**
 * @brief mouseHandler
 * @param event
 * @param x
 * @param y
 * @param flags
 * @param data_ptr
 */
void mouseHandler(int event, int x, int y, int flags, void* data_ptr)
{
    Road *road=(Road*)data_ptr;
    Point2f pt(x,y);
    if  ( event == EVENT_LBUTTONDOWN ){
        bool find=false;
        cv::Mat tmpw=road->cur_draw_pict.clone();
        for(LaneMarkingSeg& one:road->context->segs){
//            one.contour_pts
            if(one.isContainPt(pt)){
                cout<<"找到一个seg包含了这个点。"<<endl;
                find=true;

                //扩大范围
                vector<RotatedRect> subs=one.SplitMultiSubSegs(4);
                if(one.blob_angle_deg<0){
                    RotatedRect r=subs[0];
                    cv::Point2f rpts[4];
                    r.points(rpts);
                    float x=(rpts[2].x+rpts[3].x)/2;
                    float y=(rpts[2].y+rpts[3].y)/2;

                    cv::Point2f cent(x,y);
                    cv::Size2f size(one.dir_len*2,one.short_len*4);
                    RotatedRect rect(cent,size,r.angle);

                    rect.points(rpts);

                    for( int j = 0; j < 4; j++ )
                    {
                        line( tmpw, rpts[j], rpts[(j+1)%4], cv::Scalar(255,0,0), 1, 8 );

                    }
                }

                break;
            }
        }
        if(find==true){
            circle(tmpw,pt,6,cv::Scalar(255,0,0),1);
        }else{
            circle(tmpw,pt,6,cv::Scalar(0,0,255),1);
        }
        road->cur_draw_pict=tmpw;
        cv::imshow("getlanemat",road->cur_draw_pict);
        cv::waitKey(0);

    }else if  (event == EVENT_RBUTTONDOWN ){

    }
}

void makeFromVid(string path)
{
    Mat frame;
    VideoCapture cap(path); // open the video file for reading

    if ( !cap.isOpened() )  // if not success, exit program
        cout << "Cannot open the video file" << endl;

    cap.set(CV_CAP_PROP_POS_MSEC, 3000); //start the video at 300ms

    double fps = cap.get(CV_CAP_PROP_FPS); //get the frames per seconds of the video
    cout << "Input video's Frame per seconds : " << fps << endl;

    cap.read(frame);
    LaneFilter detect(frame);

    gumh::Road road;
    road.lane_filter=&detect;

    namedWindow("getlanemat",2);
    setMouseCallback("getlanemat", mouseHandler,&road);

    while(1)
    {
        bool bSuccess = cap.read(frame); // read a new frame from video
        if (!bSuccess)                   //if not success, break loop
        {
            cout << "Cannot read the frame from video file" << endl;
            break;
        }

        //        cvtColor(frame, frame, CV_BGR2GRAY);

        //        detect.nextFrame(frame);
        road.FindRoadPaintMarkings(frame);
        char key;
//        if(road.context->cur_seq>20){
//            key=cv::waitKey(0);
//        }else
            key=cv::waitKey(10);
        if(key == 'q' || key==27)
        {
            cout << "terminated by user" << endl;
            break;
        }else if(key==32){
            key=waitKey(0);
        }else if(key==83){
            for(int kkk=0;kkk<30;kkk++){
                cap.read(frame);
                road.context->cur_seq++;
            }
        }

//        else if(key=='w'){
//            char name[64];
//            sprintf(name,"/home/gumh/Videos/lane_blob.jpeg");
//            cv::imwrite(name,road.lane_filter->temp2);
//        }

    }
}

int main()
{
//    test_cancombine();
//    return 0;
//    makeFromVid("/home/gumh/20170621/443893140145_src.avi");
     makeFromVid("/home/gumh/Videos/4239794161862_src.avi");
    destroyAllWindows();
}
