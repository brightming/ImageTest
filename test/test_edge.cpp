
#include <opencv2/opencv.hpp>

#include <iostream>
#include <str_common.h>

using namespace std;
using namespace cv;


static void MergeSeg(Mat& img,const Scalar& colorDiff=Scalar::all(1)){

    CV_Assert(!img.empty());

    RNG rng=theRNG();

    //mask
    Mat mask(img.rows+2,img.cols+2,CV_8UC1,Scalar::all(0));
    for(int y=0;y<img.rows;y++){
        for(int x=0;x<img.cols;x++){
            if(mask.at<uchar>(y+1,x+1)==0){
                //color
                Scalar newVal(rng(256),rng(256),rng(256));
                //floodfill
                floodFill(img,mask,Point(x,y),newVal,0,colorDiff,colorDiff);
            }
        }
    }
}

/**
 * @brief testpyrMeanShiftFiltering
 * 均值漂移
 * @param img
 */
void testpyrMeanShiftFiltering(Mat& img){
    int spatialRad=20;
    int colorRad=20;
    int maxPyrLevel=6;
    Mat resImg;

    pyrMeanShiftFiltering(img,resImg,spatialRad,colorRad,maxPyrLevel);

    //channel split and merge
    MergeSeg(resImg,Scalar::all(2));
    imshow("resImg",resImg);

}

Mat test_laplacian(Mat& src){

    cvtColor(src, src, CV_RGB2GRAY);
    namedWindow("Origin", CV_WINDOW_AUTOSIZE);
    imshow("Origin", src);

    Mat dst,dst1,dst2;
    // CV_EXPORTS_W void Laplacian( InputArray src, OutputArray dst, int ddepth,
    //                          int ksize=1, double scale=1, double delta=0,
    //                          int borderType=BORDER_DEFAULT );
    Laplacian(src, dst1, src.depth(), 3, 1, 0, BORDER_DEFAULT);
    GaussianBlur(src, src, Size(3, 3), 1.5, 1.5);
    Laplacian(src, dst, src.depth(), 3, 1, 0, BORDER_DEFAULT);
    GaussianBlur(dst, dst2, Size(3, 3), 1.5, 1.5);
    namedWindow("Laplacian", CV_WINDOW_AUTOSIZE);
    imshow("GaussianLalacian", dst);
    imshow("Laplacian", dst1);
    imshow("LaplacianGaussian", dst2);
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

Mat test_equalize(Mat& src){
    if(src.channels() >= 3)
    {
        Mat ycrcb;
        cvtColor(src,ycrcb,CV_BGR2YCrCb);

        vector<Mat> channels;
        split(ycrcb,channels);

        equalizeHist(channels[0], channels[0]);

        Mat result;
        merge(channels,ycrcb);
        cvtColor(ycrcb,result,CV_YCrCb2BGR);

        imshow("resImg",result);

        //methos 2
        Mat res;
        cvtColor(src,res,CV_BGR2GRAY);
        equalizeHist(res,res);

        //        imshow("resImg2",res);

        //method 3
        Mat res2=do_clahe(src);
        imshow("resImg2",res2);

        return result;
    }
    return src.clone();



}

Mat test_edge(Mat& img){

    float top_width_ratio=1;
    float height_ratio=0.5;
    int start_x=(img.cols-top_width_ratio*img.cols)/2;
    int start_y=img.rows-height_ratio*img.rows;

    Rect roi(0,start_y,img.cols,img.rows*height_ratio);

    int kernel_size = 3;
    // Canny Edge Detector
    int low_threshold = 10;
    int high_threshold = 220;


    Mat roi_img=img;
    roi_img=img(roi);

    Mat gray;
    cvtColor(roi_img,gray,COLOR_BGR2GRAY);

    //        Apply Gaussian smoothing
    Mat blur_gray;
    GaussianBlur(gray,blur_gray,Size(kernel_size,kernel_size),0,0);
    imshow("blur_gray",blur_gray);

    //        Apply Canny Edge Detector
    Mat edges;
    //    upperthresh=OTSU(blur_gray)
    Mat img_bw;
    //    high_threshold=(int)threshold(blur_gray, img_bw, 0, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);
    //    low_threshold=0.5*high_threshold;
    std::cout<<"high_threshold="<<high_threshold<<",low_threshold="<<low_threshold<<std::endl;
    cv::Canny(blur_gray,edges,low_threshold,high_threshold);

    Mat element;
    int dilation_size = 1;
    int dilation_type=MORPH_ELLIPSE;//MORPH_RECT,MORPH_CROSS,MORPH_ELLIPSE
    element = getStructuringElement( dilation_type,
                                     Size( 2*dilation_size + 1, 2*dilation_size+1 ),
                                     Point( dilation_size, dilation_size ) );
    ///膨胀操作
    dilate( edges, edges, element );
    erode( edges, edges, element );

    //
    Size2f sz;
    vector< vector<Point> > contours;
    vector<Vec4i> hierarchy;
    RotatedRect rotated_rect;
    findContours(edges, contours,
                 hierarchy, CV_RETR_CCOMP,
                 CV_CHAIN_APPROX_SIMPLE);



    int minSize;
    int ratio;
    float  contour_area;
    float blob_angle_deg;
    float bounding_width;
    float bounding_length;

    int diff, diffL, diffR;
    int laneWidth;
    int diffThreshTop;
    int diffThreshLow;
    int ROIrows;
    int vertical_left;
    int vertical_right;
    int vertical_top;
    int smallLaneArea;
    int longLane;
    int  vanishingPt;
    float maxLaneWidth;




    minSize        = 0.00015 * (img.cols*img.rows);  //min size of any region to be selected as lane
    maxLaneWidth   = 0.025 * img.cols;                     //approximate max lane width based on image size
    smallLaneArea  = 7 * minSize;
    longLane       = 0.3 * img.rows;
    ratio          = 4;

    //these mark the possible ROI for vertical lane segments and to filter vehicle glare
    vertical_left  = 2*img.cols/5;
    vertical_right = 3*img.cols/5;
    vertical_top   = 2*img.rows/3;

    if (!contours.empty())
    {

        vector<vector<Point> > contours_poly( contours.size() );
        for (size_t i=0; i<contours.size(); ++i)
        {

            int valid_cont=0; //0 means invalid
            //====conditions for removing contours====//

            float contour_area = contourArea(contours[i]) ;
            if(contour_area > 150)
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

            if(valid_cont>0 /*&& (abs(blob_angle_deg)>35 && abs(blob_angle_deg)<50)*/){
                //------------draw  begin -----//
                Point2f rect_points[4];
                rotated_rect    = minAreaRect(contours[i]);
                rotated_rect.points( rect_points );
                Point ct((rect_points[0].x+rect_points[2].x)/2,(rect_points[0].y+rect_points[2].y)/2);

                drawContours(roi_img, contours,i, Scalar(255), CV_FILLED, 8);




                //line
                for( int j = 0; j < 4; j++ ){
                    line( roi_img, rect_points[j], rect_points[(j+1)%4], Scalar(0,0,255), 2, 8 );
                }
            }

        }
    }

    imshow("resImg",edges);
    imshow("roi_img",img);

    return edges;
}

int main(int argc,char* argv[]){

    string file="/home/gumh/Videos/challenge/";

    namedWindow("img",2);
    namedWindow("resImg",2);
    namedWindow("resImg2",2);

    vector<string> all_pics;
    if(IsDir(file)){
        all_pics=getAllFilesWithPathFromDir(file);
        std::sort(all_pics.begin(),all_pics.end(),less<string>());
    }else{
        all_pics.push_back(file);
    }

    int idx=0;
    Mat img;
    while(1)
    {
        if(idx>=all_pics.size()){
            idx=0;
        }
        img=imread(all_pics[idx++]);
        imshow("img",img);


        //        test_edge(img);
        test_equalize(img);

        int key=waitKey(30);
        if(key==27 || (char)key=='q'){
            break;
        }

    }



    waitKey(0);
}
