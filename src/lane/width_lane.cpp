#include "lane/width_lane.h"

WidthLane::WidthLane(cv::Mat startFrame){
    this->currFrame_ = startFrame;
    this->subROI_w = 0.2*this->currFrame_.cols;
    this->maxLaneWidth_  = 90;//70;//12;  //最大车道线宽度
    this->minLaneWidth_ =20;  //最小车道线宽度
    this->maxLaneWidth_pers_ = 12;//20;   //透视变换最大车道线宽度
    this->minSize_ = 50;//100; //min size of any region to be selected as lane
    this->smallLaneArea_  = 5 * this->minSize_;//10 * this->minSize_;//7 * minSize;
    this->longLane_      = 40;//200;  //长车道线阈值
    this->ratio_         = 4;//4    //车道线长宽比

    this->template_minwidth_ = 10;  //template过滤 最小车道线宽度
    this->template_maxwidth_ = 50;  //template过滤 最大车道线宽度
    this->template_rows_ = 10;      //template过滤 模板行数
    this->template_rows_pers_ = 20; //template过滤 透视变换模板行数
}

void WidthLane::SetVanishPt(int vp){
    this->vanishingPt_ = vp;
    this->ROIrows_ = this->currFrame_.rows - this->vanishingPt_;   //rows in region of interest
}

void WidthLane::SetROI(){
    this->OutFrame_ = this->currFrame_.clone();
    this->ROIFrame_ = this->currFrame_(cv::Rect(0,this->vanishingPt_,this->currFrame_.cols,
                                              this->currFrame_.rows-this->vanishingPt_));
    this->MaskFrame_.create(this->ROIFrame_.size(),CV_8UC3);
    this->MaskFrame_ = cv::Scalar::all(0);
    this->binary_image_.create(this->ROIFrame_.size(),CV_8UC1);
    this->binary_image_pers_.create(this->ROIFrame_.size(),CV_8UC1);
    this->binary_image_ = cv::Scalar::all(0);
    this->binary_image_pers_ = cv::Scalar::all(0);
    this->contoursFrame_.create(this->ROIFrame_.size(),CV_8UC1);
    this->contoursFrame_ = cv::Scalar::all(0);
}

void WidthLane::ROIPreprocess(){
    this->PerspectiveTrans(this->ROIFrame_);
    cv::cvtColor(this->ROIFrame_, this->ROIFrame_, CV_BGR2GRAY);
//    cv::equalizeHist(this->ROIFrame_, this->ROIFrame_);
}

void WidthLane::GaussianFitler(){
    cv::GaussianBlur(this->ROIFrame_, this->ROIFrame_, cv::Size(9, 9), 2, 2);
}

void WidthLane::MarkLane(cv::Mat &src){
    this->binary_image_ = cv::Scalar::all(0);
    for(int i=0; i<src.rows; i++)
    {
        this->laneWidth_ =this->minLaneWidth_+ this->maxLaneWidth_*i/this->ROIrows_;
        for(int j=this->laneWidth_+subROI_w; j<src.cols- this->laneWidth_-subROI_w; j++)
        {
            this->diffL_ = src.at<uchar>(i,j) - src.at<uchar>(i,j-this->laneWidth_);
            this->diffR_ = src.at<uchar>(i,j) - src.at<uchar>(i,j+this->laneWidth_);
            this->diff_  = this->diffL_ + this->diffR_ - abs(this->diffL_-this->diffR_);

            //1 right bit shifts to make it 0.5 times
            this->diffThreshLow_ = src.at<uchar>(i,j)>>1;

            if (this->diffL_>0 && this->diffR_ >0 && this->diff_>5)
                if(this->diff_>=this->diffThreshLow_ )
                    this->binary_image_.at<uchar>(i,j)=255;
        }
    }
    imshow("MarkLane",this->binary_image_);
}

void WidthLane::MarkLanePers(cv::Mat &src){
    this->binary_image_pers_ = cv::Scalar::all(0);
    for(int i=0; i<src.rows; i++)
    {
        this->laneWidth_ = maxLaneWidth_pers_;
        for(int j=this->laneWidth_+subROI_w; j<src.cols- this->laneWidth_-subROI_w; j++)
        {
            this->diffL_ = src.at<uchar>(i,j) - src.at<uchar>(i,j-this->laneWidth_);
            this->diffR_ = src.at<uchar>(i,j) - src.at<uchar>(i,j+this->laneWidth_);
            this->diff_  = this->diffL_ + this->diffR_ - abs(this->diffL_-this->diffR_);

            //1 right bit shifts to make it 0.5 times
            this->diffThreshLow_ = src.at<uchar>(i,j)*0.25;

            if (this->diffL_>0 && this->diffR_ >0 && this->diff_>5)
                if(this->diff_>=this->diffThreshLow_ )
                    this->binary_image_pers_.at<uchar>(i,j)=255;
        }
    }
    imshow("MarkLanePers",this->binary_image_pers_);
}

void WidthLane::KernalFilter(){
    this->binary_image_ = cv::Scalar::all(0);

    cv::Mat kmat;
    kmat.create(this->binary_image_.size(),CV_8UC1);
    kmat = cv::Scalar::all(0);

    this->laneWidth_ = this->maxLaneWidth_;
    int kernal_size = 3;
    int stride = (kernal_size-1)/2;
    int thres = 0*kernal_size;
    for(int i=kernal_size; i<this->ROIFrame_.rows; i=i+kernal_size)
    {

        for(int j=this->laneWidth_; j<this->ROIFrame_.cols-this->laneWidth_; j=j+kernal_size)
        {
            int kernal_val = 0;
            int difvalL = 0;
            int difvalR = 0;
            for(int m = i-stride; m <= i+stride; m++){
                for(int n = j-stride; n <= j+stride; n++){
                    difvalL += this->binary_image_.at<uchar>(m,n) - this->binary_image_.at<uchar>(m,n-this->laneWidth_);
                    difvalR += this->binary_image_.at<uchar>(m,n) - this->binary_image_.at<uchar>(m,n+this->laneWidth_);
                    kernal_val +=  abs(this->binary_image_.at<uchar>(m,n));
                }
            }
            this->diffL_ = difvalL;
            this->diffR_ = difvalR;
            this->diff_  = this->diffL_ + this->diffR_ - abs(this->diffL_-this->diffR_);
//            std::cout<<"this->diffL_="<<this->diffL_<<std::endl;
//            std::cout<<"this->diffR_="<<this->diffR_<<std::endl;
            std::cout<<"kernal_val_="<<kernal_val<<std::endl;
            //1 right bit shifts to make it 0.5 times
            this->diffThreshLow_ = kernal_val;
            //diffThreshTop = 1.2*currFrame.at<uchar>(i,j);

            if (this->diffL_>thres && this->diffR_ >thres && this->diff_>5)
                if(this->diff_>=this->diffThreshLow_)
                    for(int m = i-stride; m <= i+stride; m++){
                        for(int n = j-stride; n <= j+stride; n++){
                             kmat.at<uchar>(m,n)=255;
                        }
                    }
        }
    }
//    imshow("kmat",kmat);
}

void WidthLane::KernalFilterPers(){
    this->binary_image_pers_ = cv::Scalar::all(0);

    this->laneWidth_ = this->maxLaneWidth_pers_;
    int kernal_size = 1;
    int stride = (kernal_size-1)/2;
    int thres = 0*kernal_size;
    for(int i=kernal_size; i<this->ROIFrame_.rows; i=i+kernal_size)
    {

        for(int j=this->laneWidth_; j<this->ROIFrame_.cols-this->laneWidth_; j=j+kernal_size)
        {
            int kernal_val = 0;
            int difvalL = 0;
            int difvalR = 0;
            for(int m = i-stride; m <= i+stride; m++){
                for(int n = j-stride; n <= j+stride; n++){
                    difvalL += this->ROIFrame_.at<uchar>(m,n) - this->ROIFrame_.at<uchar>(m,n-this->laneWidth_);
                    difvalR += this->ROIFrame_.at<uchar>(m,n) - this->ROIFrame_.at<uchar>(m,n+this->laneWidth_);
                    kernal_val += this->ROIFrame_.at<uchar>(m,n);
                }
            }
            this->diffL_ = difvalL;
            this->diffR_ = difvalR;
            this->diff_  = this->diffL_ + this->diffR_ - abs(this->diffL_-this->diffR_);
            //            std::cout<<"this->diffL_="<<this->diffL_<<std::endl;
            //            std::cout<<"this->diffR_="<<this->diffR_<<std::endl;

            int kernal_val_mean = kernal_val/(kernal_size*kernal_size);
            this->diffThreshLow_ = kernal_val>>1;

            if (this->diffL_>thres && this->diffR_ >thres && this->diff_>5)
                if(this->diff_>=this->diffThreshLow_)
                    for(int m = i-stride; m <= i+stride; m++){
                        for(int n = j-stride; n <= j+stride; n++){
//                            if(this->ROIFrame_.at<uchar>(m,n)>=kernal_val_mean)
                            this->binary_image_pers_.at<uchar>(m,n)=255;
                        }
                    }
        }
    }
//    imshow("KernalFilterPers",this->binary_image_pers_);
}

void WidthLane::TemplateFilter(cv::Mat &src){
    int len_black,len_white,thres,score;
    cv::Mat templ_filter;

    cv::Mat templ_out = cv::Mat::zeros(src.rows, src.cols,CV_8UC1);
    //    std::cout<<"templ_img.size="<<templ_img.size()<<std::endl;
    for(int tran_num = 0; tran_num < 2; tran_num++){
        for(int i = 0; i<src.rows-template_rows_;i++){
            len_white = template_minwidth_+ template_maxwidth_*i/this->ROIrows_;
            len_black = 0.15*len_white;
            thres = 0.5*(len_black*2+len_white)*template_rows_;

            templ_filter = cv::Mat::zeros(template_rows_, len_black*2+len_white,CV_8UC1);
            for(int i1=0; i1<template_rows_; i1++){
                for(int j1=len_black ; j1<len_black+len_white; j1++){
                    templ_filter.at<uchar>(i1,j1)=255;
                }
            }

            for(int j=len_black+len_white+subROI_w;j<src.cols-(len_black+len_white)-subROI_w;j++){

                if(src.at<uchar>(i,j)==255&&src.at<uchar>(i,j-1)==0)
                {
                    score = 0;
                    for(int m=0; m<template_rows_; m++){
                        for(int n=-len_black; n<len_black+len_white; n++){
                            if(src.at<uchar>(i+m,j+n)==templ_filter.at<uchar>(m,n+len_black))
                                score++;
                        }
                    }
//                                    std::cout<<"score="<<score<<std::endl;
//                                    std::cout<<"thres="<<thres<<std::endl;
                    if(score>=thres){
                        for(int m=0; m<template_rows_; m++){
                            for(int n=0 ; n<len_white; n++){
                                if(src.at<uchar>(i+m,j+n)==255)
                                   templ_out.at<uchar>(i+m,j+n)=255;
                            }
                        }
                    }
                }
            }
        }
        flip(src,src,1);
        flip(templ_out,templ_out,1);
//        imshow("TemplateFilter",templ_out);
//        cv::waitKey(0);
    }
    imshow("TemplateFilter",templ_out);
    templ_out.copyTo(templateFrame_);
}

void WidthLane::TemplateFilterPers(cv::Mat &src){
    int len_black,len_white,thres,score;
    len_white = this->maxLaneWidth_pers_*0.9;//10;
    len_black = 0.25*len_white;//4;
    thres = 0.75*(len_black*2+len_white)*template_rows_pers_;
    cv::Mat templ = cv::Mat::zeros(template_rows_pers_, len_black*2+len_white,CV_8UC1);
    for(int i=0; i<template_rows_pers_; i++){
        for(int j=len_black ; j<len_black+len_white; j++){
            templ.at<uchar>(i,j)=255;
        }
    }
    //    std::cout<<templ<<std::endl;
    cv::Mat templ_img = cv::Mat::zeros(src.rows, src.cols,CV_8UC1);
    //    std::cout<<"templ_img.size="<<templ_img.size()<<std::endl;
    for(int tran_num = 0; tran_num < 2; tran_num++){
        for(int i = 0; i<src.rows-template_rows_pers_;i++){
            for(int j=len_black+len_white+subROI_w;j<src.cols-(len_black+len_white)-subROI_w;j++){

                if(src.at<uchar>(i,j)==255&&src.at<uchar>(i,j-1)==0/*&&this->binary_image_.at<uchar>(i+1,j)==255&&this->binary_image_.at<uchar>(i+1,j-1)==0*/)
                {
                    score = 0;
                    for(int m=0; m<template_rows_pers_; m++){
                        for(int n=-len_black; n<len_black+len_white; n++){
                            if(src.at<uchar>(i+m,j+n)==templ.at<uchar>(m,n+len_black))
                                score++;
                        }
                    }
                    //                std::cout<<"score="<<score<<std::endl;
                    //                std::cout<<"thres="<<thres<<std::endl;
                    if(score>=thres){
                        for(int m=0; m<template_rows_pers_; m++){
                            for(int n=0 ; n<len_white; n++){
                                if(src.at<uchar>(i+m,j+n)==255)
                                    templ_img.at<uchar>(i+m,j+n)=255;
                            }
                        }
                    }
                }
            }
        }
        flip(src,src,1);
        flip(templ_img,templ_img,1);
    }
    imshow("TemplateFilterPers",templ_img);
    templ_img.copyTo(templateFrame_pers_);
}

void WidthLane::ContoursComputePers(cv::Mat &src,cv::Mat& dst, cv::Scalar sc){
    cv::findContours(src, this->contours_,this->hierarchy_,
                 CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);

    if (!contours_.empty())
    {
//        std::cerr<<"this->contours_.size()="<<this->contours_.size()<<std::endl;
        for (size_t i=0; i<this->contours_.size(); ++i)
        {
            this->contour_area_ = cv::contourArea(contours_[i]);

            //blob size should not be less than lower threshold
            if(this->contour_area_ > this->minSize_)
            {
                this->rotated_rect_    = cv::minAreaRect(contours_[i]);
                this->sz_              = this->rotated_rect_.size;
                this->bounding_width_  = this->sz_.width;
                this->bounding_length_ = this->sz_.height;

                //openCV selects length and width based on their orientation
                //so angle needs to be adjusted accordingly
                this->blob_angle_deg_ = this->rotated_rect_.angle;
                if (this->bounding_width_ < this->bounding_length_)
                    this->blob_angle_deg_ = 90 + this->blob_angle_deg_;
                if(abs(this->blob_angle_deg_)<80)
                    continue;

//                std::cerr<<"this->rotated_rect_.center="<<this->rotated_rect_.center<<std::endl;
//                std::cerr<<"this->rotated_rect_.points="<<this->rotated_rect_.boundingRect()<<std::endl;

                if(this->bounding_length_ > this->longLane_ || this->bounding_width_ > this->longLane_)
                {
                    if(dst.channels()==3)
                        cv::drawContours(dst, this->contours_,i,sc,CV_FILLED,8);
                    else
                        cv::drawContours(dst, this->contours_,i,cv::Scalar(0),CV_FILLED,8);
                }
                else{
                    if ((this->bounding_length_/this->bounding_width_)>=this->ratio_ || (this->bounding_width_/this->bounding_length_)>=this->ratio_
                            ||(this->contour_area_< this->smallLaneArea_ &&  ((this->contour_area_/(this->bounding_width_*this->bounding_length_)) > .75) &&
                               ((this->bounding_length_/this->bounding_width_)>=4 || (this->bounding_width_/this->bounding_length_)>=4)))
                    {
                        if(dst.channels()==3)
                            cv::drawContours(dst, this->contours_,i,sc,CV_FILLED,8);
                        else
                            cv::drawContours(dst, this->contours_,i,cv::Scalar(0),CV_FILLED,8);
                    }
                }
            }
        }
    }
}

void WidthLane::FindNonZeroFill(cv::Mat &src, uchar val){
    for(int i=0; i<src.rows;i++){
        for(int j=0; j<src.cols; j++){
            if(src.at<uchar>(i,j)>0)
                src.at<uchar>(i,j)=val;
        }
    }
}


void WidthLane::PerspectiveTrans(cv::Mat &src){
    std::vector<cv::Point> not_a_rect_shape;
    not_a_rect_shape.push_back(cv::Point(0,0));
    not_a_rect_shape.push_back(cv::Point(src.cols,0));
    not_a_rect_shape.push_back(cv::Point(src.cols,src.rows));
    not_a_rect_shape.push_back(cv::Point(0,src.rows));

    //  topLeft, topRight, bottomRight, bottomLeft
    cv::Point2f src_vertices[4];
    src_vertices[0] = not_a_rect_shape[0];
    src_vertices[1] = not_a_rect_shape[1];
    src_vertices[2] = not_a_rect_shape[2];
    src_vertices[3] = not_a_rect_shape[3];

    cv::Point2f dst_vertices[4];
    dst_vertices[0] = cv::Point(0,0);
    dst_vertices[1] = cv::Point(src.cols,0);
//    dst_vertices[2] = cv::Point(900,src.rows);
//    dst_vertices[3] = cv::Point(800,src.rows);
    dst_vertices[2] = cv::Point(1050,src.rows);
    dst_vertices[3] = cv::Point(675,src.rows);
//    dst_vertices[2] = cv::Point(1000,src.rows);
//    dst_vertices[3] = cv::Point(775,src.rows);
    cv::Mat warpMatrix = cv::getPerspectiveTransform(src_vertices, dst_vertices);
    cv::warpPerspective(src, src, warpMatrix, src.size(), cv::INTER_LINEAR, cv::BORDER_CONSTANT);
    //    imshow( "warp perspective",src);
}

void WidthLane::InversePerspectiveTrans(cv::Mat &src){
    std::vector<cv::Point> not_a_rect_shape;
    not_a_rect_shape.push_back(cv::Point(0,0));
    not_a_rect_shape.push_back(cv::Point(src.cols,0));
    not_a_rect_shape.push_back(cv::Point(1050,src.rows));
    not_a_rect_shape.push_back(cv::Point(675,src.rows));

    //  topLeft, topRight, bottomRight, bottomLeft
    cv::Point2f src_vertices[4];
    src_vertices[0] = not_a_rect_shape[0];
    src_vertices[1] = not_a_rect_shape[1];
    src_vertices[2] = not_a_rect_shape[2];
    src_vertices[3] = not_a_rect_shape[3];

    cv::Point2f dst_vertices[4];
    dst_vertices[0] = cv::Point(0,0);
    dst_vertices[1] = cv::Point(src.cols,0);
    dst_vertices[2] = cv::Point(src.cols,src.rows);
    dst_vertices[3] = cv::Point(0,src.rows);

    cv::Mat warpMatrix = cv::getPerspectiveTransform(src_vertices, dst_vertices);
    cv::warpPerspective(src, src, warpMatrix, src.size(), cv::INTER_LINEAR, cv::BORDER_CONSTANT);
    //    imshow( "warp inverse perspective",src);
}

void WidthLane::AutoCanny(cv::Mat &src, float type){
/*------------判定是否CV_8UC1------------*/
    if (src.type() != CV_8UC1)
        CV_Error(CV_StsUnsupportedFormat, "");

    int total_piexl = 0;
    int nr = src.rows;
    int nc = src.cols;
    // If the input and output mat is store continuous in memory, then loop
    // the Mat just in one rows will be much more quickly.
    if (src.isContinuous()) {
        nr = 1;
        nc = nc * src.rows;
    }

    // Means gray level in image.
    for (int i = 0; i < nr; i++) {
        const uchar *src_data = src.ptr<uchar>(i);
        for (int j = 0; j < nc; j++) {
            total_piexl += *src_data;
        }
    }

    total_piexl /= nr * nc; // means gray level
    // the following threshould value is calc from experience.
    int thres1 = int(type * total_piexl);
    int thres2 = int(type * 3 * total_piexl);
    std::cout<<"thres1  "<<thres1<<"  "<<"thres2  "<<thres2<<std::endl;
}


