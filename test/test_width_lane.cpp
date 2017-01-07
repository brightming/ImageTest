#include "lane/width_lane.h"

#include "common/ransac.h"
using namespace cv;
using namespace std;
clock_t start, stop;


void ransac_fit(Mat& binary_pict){//,Mat& consensus_mat){


    vector<Point> all_pts;
    cv::findNonZero(binary_pict,all_pts);

//    cout<<"all_pts.size="<<all_pts.size()<<endl;

    int max_iter;
    float max_dist=10;
    float best_distance=binary_pict.cols*binary_pict.rows;
    int max_x=binary_pict.cols;
    int max_y=binary_pict.rows;
    float best_m,best_b;
    float get_dist;
    float get_m,get_b;
    float allow_dist=max_dist/2;
    vector<Point> consensus_pts;
    vector<Point> tmp_pts;
    vector<Point> seed_pts;

    vector<Point> in_set_pts;
    vector<int> in_set_idx;
    vector<Point> outlier_pts;

    vector<Vec3f> maybe_good_models;

    //verify line
    max_iter=100;
    allow_dist=max_dist;
    consensus_pts.resize(0);
    tmp_pts.resize(0);
    seed_pts.resize(0);
    int min_cnt=50;
    for(int k=0;k<max_iter;k++){
//            cout<<"iter -- "<<k<<endl;
        //--get random data---//
        in_set_pts.resize(0);
        in_set_idx.resize(0);
        outlier_pts.resize(0);
        int num=3;
//            cout<<"--random pts--"<<endl;
        get_random_line_pts(all_pts,num,in_set_pts,in_set_idx,outlier_pts);
        min_cnt=all_pts.size()/10;
        //        for(int i=0;i<in_set_pts.size();i++){
        //            circle(pict,in_set_pts[i],5,Scalar(255),1);
        //        }
        if(in_set_pts.size()<num){
            continue;
        }


//            cout<<"--very line--"<<endl;
        tmp_pts.resize(0);
        bool agree=verify_line(in_set_pts,outlier_pts,allow_dist,min_cnt,get_m,get_b,get_dist,tmp_pts);
        if(agree && get_dist<best_distance){
            best_distance=get_dist;
            best_m=get_m;
            best_b=get_b;
            consensus_pts.resize(0);
            for(int i=0;i<tmp_pts.size();i++){
                consensus_pts.push_back(tmp_pts[i]);
            }
            seed_pts.resize(0);
            for(int i=0;i<in_set_pts.size();i++){
                seed_pts.push_back(in_set_pts[i]);
            }
//            cout<<"get_dist="<<get_dist<<endl;
            if(get_dist<allow_dist/2){
                maybe_good_models.push_back(Vec3f(get_m,get_b,get_dist));
            }

            if(best_distance<allow_dist/3){
                cout<<"get very best point.break;"<<endl;
                break;
            }
        }
    }

    cout<<"best_distance="<<best_distance<<",min_cnt="<<min_cnt<<",allow_dist="<<allow_dist<<",best_m="<<best_m<<",best_b="<<best_b<<",consensus_pts.size="<<consensus_pts.size()<<endl;

//    cout<<"maybe_good_models.size="<<maybe_good_models.size()<<endl;
//    for(int i=0;i<maybe_good_models.size();i++){
//        cout<<"get-dist="<<maybe_good_models[i][2]
//           <<",get_m="<<maybe_good_models[i][0]
//          <<",get-b="<<maybe_good_models[i][1]
//         <<endl;
//    }

    char name[64];

    //draw points
//    for(Point p:consensus_pts){
//        consensus_mat.at<Vec3b>(p.y,p.x)[0]=255;
//        consensus_mat.at<Vec3b>(p.y,p.x)[1]=255;
//        consensus_mat.at<Vec3b>(p.y,p.x)[2]=255;
//    }


    //draw all line
//    for(int i=0;i<maybe_good_models.size();i++){
//        LeastSquare lsq;
//        lsq.setM(maybe_good_models[i][0]);
//        lsq.setB(maybe_good_models[i][1]);
//        Point p1(max_x/2,(int)lsq.getY(max_x/2));
//        Point p2((int)lsq.getX(max_y),max_y);

//        cout<<"p1=("<<p1.x<<","<<p1.y<<")"<<endl;
//        cout<<"p2=("<<p2.x<<","<<p2.y<<")"<<endl;

//        line(binary_pict,p1,p2,Scalar(255),2); //蓝色表示预测图
//        line(consensus_mat,p1,p2,Scalar(255,0,0),1); //蓝色表示预测图
//    }

    //draw best line
    LeastSquare lsq_best;
    lsq_best.setM(best_m);
    lsq_best.setB(best_b);
    Point p1(0,(int)lsq_best.getY(0));
    Point p2(max_x-1,(int)lsq_best.getY(max_x-1));

//    cout<<"p1=("<<p1.x<<","<<p1.y<<")"<<endl;
//    cout<<"p2=("<<p2.x<<","<<p2.y<<")"<<endl;

    line(binary_pict,p1,p2,Scalar(255),2); //蓝色表示预测图
//    line(consensus_mat,p1,p2,Scalar(255,0,0),1); //蓝色表示预测图

    {
        vector<double> x;
        vector<double> y;
//        for(int i=0;i<seed_pts.size();i++){
//            //初始选择的点，得到的直线
//            x.push_back(seed_pts[i].x);
//            y.push_back(seed_pts[i].y);

//            std::cout<<"seed_pt-"<<i<<"=("<<seed_pts[i].x<<","<<seed_pts[i].y<<")"<<endl;
//            circle(consensus_mat,seed_pts[i],20,Scalar(0,0,255),1);

//        }
        LeastSquare seed_lsq(x,y);
        Point seed_p1(0,(int)seed_lsq.getY(0));
        Point seed_p2((int)seed_lsq.getX(0),0);
//        line(consensus_mat,seed_p1,seed_p2,Scalar(0,0,255),1);

//        for(int i=0;i<consensus_pts.size();i++){
//            float dist=seed_lsq.get_dist(consensus_pts[i].x,consensus_pts[i].y);
//            if(dist>allow_dist){
//                sprintf(name,"%f",dist);
//                cv::putText(consensus_mat,name,consensus_pts[i],CV_FONT_HERSHEY_SIMPLEX,0.5,Scalar(0,255,0));
//            }
//        }

    }


//    imshow("consensus",consensus_mat);
//    imshow("binary_pict",binary_pict);
//    int key=waitKey(0);


}

void makeFromVid(string path)
{
    Mat frame;
    VideoCapture cap(path); // open the video file for reading

    if ( !cap.isOpened() )  // if not success, exit program
        cout << "Cannot open the video file" << endl;

//    cap.set(CV_CAP_PROP_POS_MSEC, 100000); //start the video at 300ms

    double fps = cap.get(CV_CAP_PROP_FPS); //get the frames per seconds of the video
    cout << "Input video's Frame per seconds : " << fps << endl;

    cap.read(frame);

    VideoWriter outputvideo("/home/gumh/Videos/testvideo.avi", CV_FOURCC('D', 'I', 'V', 'X'), 10, Size(frame.cols/2,frame.rows/2), true);
    if (!outputvideo.isOpened())	{
        cout << "initialize outputvideo fail!" << endl;
    }
    int loop=3;

    WidthLaneDetector lanedetect(frame);

    int width,height;

    while(1)
    {
        bool bSuccess = cap.read(frame); // read a new frame from video
        if (!bSuccess)                   //if not success, break loop
        {
            cout << "Cannot read the frame from video file" << endl;
            break;
        }
//        imwrite("/home/gumh/Videos/hubei.png",frame);
//        break;
        start = clock();

//        imshow("frame",frame);
//        waitKey(0);
        lanedetect.SetVanishPt(frame.rows*0.65);
        lanedetect.SetROI();
        lanedetect.ROIPreprocess();
        lanedetect.GaussianFitler();
        lanedetect.MarkLanePers(lanedetect.ROIFrame_);
        lanedetect.TemplateFilterPers(lanedetect.binary_image_pers_);
        lanedetect.ContoursComputePers(lanedetect.templateFrame_pers_,lanedetect.MaskFrame_, cv::Scalar(128,0,0));



        width=lanedetect.templateFrame_pers_.cols;
        height=lanedetect.templateFrame_pers_.rows;

//        line(lanedetect.templateFrame_pers_,Point(width/2,0),Point(width/2,height),Scalar(255),5);

        //left side
        Mat left_part=lanedetect.templateFrame_pers_(Rect(681,0,225,height));
        Mat right_part=lanedetect.templateFrame_pers_(Rect(1044,0,315,height));

//        ransac_fit(left_part);
//        ransac_fit(right_part);

        ransac_fit(lanedetect.templateFrame_pers_);

         imshow("ContoursPers",lanedetect.templateFrame_pers_);


//        imshow("MaskFrame",lanedetect.MaskFrame_);
//        lanedetect.InversePerspectiveTrans(lanedetect.templateFrame_pers_);
//        uchar val1= 255;
//        lanedetect.FindNonZeroFill(lanedetect.templateFrame_pers_,val1);
//        lanedetect.TemplateFilter(lanedetect.templateFrame_pers_);
        //        lanedetect.PerspectiveTrans(lanedetect.templateFrame_);
        //        imshow("TemplateFilter",lanedetect.templateFrame_);
        //        lanedetect.ContoursComputePers(lanedetect.templateFrame_pers_,lanedetect.MaskFrame_);
        //        lanedetect.KernalFilterPers();
        stop =clock();
        cout<<"fps : "<<1.0/(((double)(stop-start))/ CLOCKS_PER_SEC)<<endl;
        cout<<"seconds : "<<((double)(stop-start))/ CLOCKS_PER_SEC<<endl;

        //        imshow("ROIPreprocess",lanedetect.ROIFrame_);
//        imshow("MarkLane",lanedetect.binary_image_pers_);
//        imshow("TemplateFilterPers",lanedetect.templateFrame_pers_);
        imshow("currFrame",lanedetect.currFrame_);
        //        imwrite("/home/fung/test.png",frame);
        //        waitKey(0);

        lanedetect.InversePerspectiveTrans(lanedetect.MaskFrame_);
        for(int i = lanedetect.vanishingPt_; i<frame.rows;i++){
            for(int j=0; j<frame.cols;j++){
                if(lanedetect.MaskFrame_.at<Vec3b>(i-lanedetect.vanishingPt_,j)[0]==128){
                    lanedetect.OutFrame_.at<Vec3b>(i,j)[0]=lanedetect.MaskFrame_.at<Vec3b>(i-lanedetect.vanishingPt_,j)[0];
                    lanedetect.OutFrame_.at<Vec3b>(i,j)[1]=lanedetect.MaskFrame_.at<Vec3b>(i-lanedetect.vanishingPt_,j)[1];
                    lanedetect.OutFrame_.at<Vec3b>(i,j)[2]=lanedetect.MaskFrame_.at<Vec3b>(i-lanedetect.vanishingPt_,j)[2];
                }
            }
        }

        imshow("MaskFrame",lanedetect.MaskFrame_);
        loop--;
        if(loop==0){
            cv::resize(lanedetect.OutFrame_,lanedetect.OutFrame_,Size(frame.cols/2,frame.rows/2));
            imshow("outputvideo",lanedetect.OutFrame_);
//            outputvideo.write(lanedetect.OutFrame_);
            loop=3;
        }


        char forwardkey;
        forwardkey=waitKey(33);
//        cout<<"forwardkey="<<(int)forwardkey<<endl;
        if( forwardkey== 83)  {
            for(int cnt=0;cnt<30;cnt++){
                bool bSuccess = cap.read(frame); // read a new frame from video
                if (!bSuccess) {
                    cout << "Cannot read the frame from video file" << endl;
                    break;
                }
            }
        }else if(forwardkey==27){
            break;
        }else if(forwardkey==32){
            waitKey(0);
        }
    }
}


int main(){
        makeFromVid("/home/gumh/Videos/2016_12_22卡车驾驶室内拍路面视频/20161222_IMG_8571.MOV");

//    makeFromVid("/home/fung/Desktop/2016_12_22/20161222_IMG_8571.MOV");
//    Mat frame;
//    frame = imread("/home/fung/Desktop/mono-detect.png",1);
//        WidthLane lanedetect(frame);
//        lanedetect.SetVanishPt(1080*0.4);
//        lanedetect.SetROI();
//        lanedetect.ROIPreprocess();
//        imshow("currFrame",lanedetect.currFrame_);
//        waitKey(0);
    return 0;
}
