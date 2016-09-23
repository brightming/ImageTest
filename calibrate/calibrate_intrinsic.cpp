#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdio.h>
#include <iostream>
//#include "popt_pp.h"

using namespace std;
using namespace cv;

vector< vector< Point3f > > object_points;
vector< vector< Point2f > > image_points;
vector< Point2f > corners;
vector< vector< Point2f > > left_img_points;

Mat img, gray;
Size im_size;

void setup_calibration(int board_width, int board_height, int num_imgs, 
                       float square_size, char* imgs_directory, char* imgs_filename,
                       char* extension,vector<Mat>& images) {
  Size board_size = Size(board_width, board_height);
  int board_n = board_width * board_height;

  for (int k = 1; k <= num_imgs; k++) {
    char img_file[100];
    sprintf(img_file, "%s%d%s.%s", imgs_directory,k, imgs_filename,extension);
//    cout<<"file="<<img_file<<endl;
    img = imread(img_file, CV_LOAD_IMAGE_COLOR);
    if(img.empty()){
        continue;
    }
    cv::cvtColor(img, gray, CV_BGR2GRAY);

    bool found = false;
    found = cv::findChessboardCorners(img, board_size, corners,
                                      CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);
    cout<<"file:"<<img_file<<" found="<<found<<endl;
    if (found)
    {
      cornerSubPix(gray, corners, cv::Size(5, 5), cv::Size(-1, -1),
                   TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
      drawChessboardCorners(gray, board_size, corners, found);
    }
    
    vector< Point3f > obj;
    for (int i = 0; i < board_height; i++)
      for (int j = 0; j < board_width; j++)
        obj.push_back(Point3f((float)j * square_size, (float)i * square_size, 0));

    if (found) {
      cout << k << ". Found corners!" << endl;
      image_points.push_back(corners);
      object_points.push_back(obj);

      images.push_back(img);
    }
  }
}

//! finds intrinsic and extrinsic camera parameters from several fews of a known calibration pattern.
double myCalibrateCamera( InputArrayOfArrays objectPoints,
                                     InputArrayOfArrays imagePoints,
                                     Size imageSize,
                                     CV_OUT InputOutputArray cameraMatrix,
                                     CV_OUT InputOutputArray distCoeffs,
                                     OutputArrayOfArrays rvecs, OutputArrayOfArrays tvecs,
                                     int flags=0, TermCriteria criteria = TermCriteria(
                                         TermCriteria::COUNT+TermCriteria::EPS, 30, DBL_EPSILON) );



void drawDottedLine(Point p1,Point p2,Mat& workimg)
{
    double k=(p2.y-p1.y)/(p2.x-p1.x+0.000001);
    double DOT_STEP;
    Point pa=p1,pb=p1;

    line(workimg,p1,p2,CV_RGB(255,255,0),3, CV_AA, 0 );

    //dotted line , has bug        ╮(╯_╰)╭

    //while(pb.x>0&&pb.x<workimg->width&&pb.y<p2.y)
    //{
    //  pb.x+=DOT_STEP;
    //  pb.y+=DOT_STEP*k;
    //  cvLine(workimg,pa,pb,CV_RGB(255,255,0),3, CV_AA, 0 );

    //  pb.x+=DOT_STEP;
    //  pb.y+=DOT_STEP*k;
    //  pa=pb;
    //  pb.x+=DOT_STEP;
    //  pb.y+=DOT_STEP*k;
    //}
}

// find two endpoint of the faint line
void drawDotline(Point s, Point e,Mat& workimg)
{
    Point pa,pb;
    if(s.y>e.y)//swap
    {
        pa=s;
        s=e;
        e=pa;
    }
    double k=(e.y-s.y)/(e.x-s.x+0.000001);
    double h=workimg.rows,w=workimg.cols;

    pb=s;
    pa.y=0,pa.x=s.x-s.y/k;          //start point on the low line
    if(pa.x<0)       //start point on the left line
        pa.x=0,     pa.y=k*s.x;
    else if(pa.x>=w)
        pa.x=w, pa.y=s.y+k*(w-s.x);

    drawDottedLine(pb,pa,workimg);

    pa=pb=e;
    pb.y=h; pb.x+=(h-e.y)/k;
    if(pb.x>w)
        pb.y=e.y+k*(w-e.x), pb.x=w;
    else if(pb.x<0)
        pb.y=e.y-k*e.x,     pb.x=0;

    drawDottedLine(pa,pb,workimg);
}

int main(int argc, char const **argv)
{
  int board_width=11, board_height=8, num_imgs=16;
  float square_size=20.0;
  char* imgs_directory="/home/gumh/qtcreator-workspace/ImageTest/picture/";
  char* imgs_filename="L";
  char* out_file="intrinsic.yml";
  char* extension="jpeg";

//  static struct poptOption options[] = {
//    { "board_width",'w',POPT_ARG_INT,&board_width,0,"Checkerboard width","NUM" },
//    { "board_height",'h',POPT_ARG_INT,&board_height,0,"Checkerboard height","NUM" },
//    { "num_imgs",'n',POPT_ARG_INT,&num_imgs,0,"Number of checkerboard images","NUM" },
//    { "square_size",'s',POPT_ARG_FLOAT,&square_size,0,"Size of checkerboard square","NUM" },
//    { "imgs_directory",'d',POPT_ARG_STRING,&imgs_directory,0,"Directory containing images","STR" },
//    { "imgs_filename",'i',POPT_ARG_STRING,&imgs_filename,0,"Image filename","STR" },
//    { "extension",'e',POPT_ARG_STRING,&extension,0,"Image extension","STR" },
//    { "out_file",'o',POPT_ARG_STRING,&out_file,0,"Output calibration filename (YML)","STR" },
//    POPT_AUTOHELP
//    { NULL, 0, 0, NULL, 0, NULL, NULL }
//  };

//  POpt popt(NULL, argc, argv, options, 0);
//  int c;
//  while((c = popt.getNextOpt()) >= 0) {}

  vector<Mat> images;
  setup_calibration(board_width, board_height, num_imgs, square_size,
                   imgs_directory, imgs_filename, extension,images);

  printf("Starting Calibration\n");
  Mat K;
  Mat D;
  vector< Mat > rvecs, tvecs;
  int flag = 0;
  flag |= CV_CALIB_FIX_K4;
  flag |= CV_CALIB_FIX_K5;

  float reprojErr=myCalibrateCamera(object_points, image_points, img.size(), K, D, rvecs, tvecs, flag);

  FileStorage fs(out_file, FileStorage::WRITE);
  fs << "K" << K;
  fs << "D" << D;
  fs << "board_width" << board_width;
  fs << "board_height" << board_height;
  fs << "square_size" << square_size;
  printf("Done Calibration.reprojErr=%f\n",reprojErr);


  //-------draw-----------//
  //对于每张图片，绘制找到的角点，以及用内参，外参投影的矩阵，对比
  Size boardSize(board_width,board_height);
  Mat view, rview, map1, map2;
  Rect valid_roi;
  Mat newMatrix=getOptimalNewCameraMatrix(K, D, img.size(), 1, img.size(), 0,&valid_roi);
  cout<<"valid_roi="<<valid_roi<<endl;
  cout<<"K="<<K<<endl;
  cout<<"D="<<D<<endl;
  cout<<"newMatrix="<<newMatrix<<endl;
  initUndistortRectifyMap(K, D, Mat(),
                          newMatrix,
                          img.size(), CV_16SC2, map1, map2);

  int totalPoints = 0;
  double totalErr = 0, err;
  vector<float> perViewErrors;
  perViewErrors.resize(images.size());
  for(int i=0;i<images.size();i++){
      Mat view=images[i].clone();
      vector<Point2f> img_point=image_points[i];
      vector<Point3f> obj_point=object_points[i];
      vector<Point2f> imagePoints;
      projectPoints(obj_point,rvecs[i],tvecs[i],K,D,imagePoints); //重投影

      cout<<"tvecs["<<i<<"]="<<tvecs[i]<<endl;
      if(i==0){
//          for(int j=0;j<img_point.size();j++){
//              cout<<"img_p:("<<img_point[j].x<<","<<img_point[j].y<<") reprj_p:("<<imagePoints[j].x<<","<<imagePoints[j].y<<")"<<endl;
//          }
      }



      drawChessboardCorners( view, boardSize, Mat(img_point), 1 );
      drawChessboardCorners( view, boardSize, Mat(imagePoints), 1 );

      //each project error
      vector<Point2f> imagePoints2;
      projectPoints( Mat(obj_point), rvecs[i], tvecs[i], K,
                             D, imagePoints2);
      err = norm(Mat(img_point), Mat(imagePoints2), CV_L2);


      int n = (int)obj_point.size();
      perViewErrors[i] = (float) std::sqrt(err*err/n);
      cout<<"pict:"<<i<<",err="<<err<<",perViewErrors="<<perViewErrors[i]<<endl;
      totalErr        += err*err;
      totalPoints     += n;

      char name[128];
      sprintf(name,"img-point-%d.jpeg",i);
      imwrite(name,view);

      //矫正
      Mat g1,g2;

      Mat temp = images[i].clone();
      undistort(temp, view, K, D);
      cvtColor(view.clone(),g1,CV_BGR2GRAY);

      //计算矫正后的角点的新坐标，其实就是去除畸变的影响，正常的投影位置
      vector<Point2f> imagePoints_no_dist;
      projectPoints( Mat(obj_point), rvecs[i], tvecs[i], K,
                             Mat(), imagePoints_no_dist);

      //在矫正后的图上画线，看交点
      drawDotline(imagePoints_no_dist[0],imagePoints_no_dist[77],view);
      line(view,imagePoints_no_dist[0],imagePoints_no_dist[77],Scalar(255,0,0),2,CV_AA);

      drawDotline(imagePoints_no_dist[10],imagePoints_no_dist[87],view);
      line(view,imagePoints_no_dist[10],imagePoints_no_dist[87],Scalar(255,0,0),2,CV_AA);
//      cout<<"imagePoints[0]="<<imagePoints[0]<<",imagePoints[11]="<<imagePoints[11]<<endl;
      rectangle(view,Rect(100,100,20,20),Scalar(0,255,255),2);
      if(i==0)
      for(int m=0;m<imagePoints.size();m++){
          Point2f p=imagePoints[m];
          cout<<m<<":"<<p.x<<","<<p.y<<endl;
      }

      drawChessboardCorners( view, boardSize, Mat(imagePoints_no_dist), 1 );//画点，对比
      sprintf(name,"img-point-rectifyed-%d.jpeg",i);
      imwrite(name,view);

      //第二种矫正
      Mat temp2 = images[i].clone();
      remap(temp2, rview, map1, map2, INTER_LINEAR);
      cvtColor(rview.clone(),g2,CV_BGR2GRAY);
      drawChessboardCorners( rview, boardSize, Mat(imagePoints), 1 );//还是用用原来的图像坐标来画点，对比
      sprintf(name,"img-point-rectifyed-2-%d.jpeg",i);
      imwrite(name,rview);

       Mat temp3 = images[i].clone();
      drawChessboardCorners( temp3, boardSize, Mat(imagePoints), 1 );//用project得到的坐标画点，对比
      sprintf(name,"img-point-reproj-%d.jpeg",i);
      imwrite(name,temp3);

      //比较两种矫正
      if(i==0){
//          cout<<"1=="<<g1(Rect(0,0,10,10))<<endl;
//          cout<<"2=="<<g2(Rect(0,0,10,10))<<endl;
//          imshow("1",g1);
//          imshow("2",g2);
//          waitKey(0);
      }


  }
  cout<<"totalErr="<<totalErr<<",std::sqrt(totalErr/totalPoints)="<<std::sqrt(totalErr/totalPoints)<<endl;


  return 0;
}


static void collectCalibrationData( InputArrayOfArrays objectPoints,
                                    InputArrayOfArrays imagePoints1,
                                    InputArrayOfArrays imagePoints2,
                                    Mat& objPtMat, Mat& imgPtMat1, Mat* imgPtMat2,
                                    Mat& npoints )
{
    int nimages = (int)objectPoints.total();
    int i, j = 0, ni = 0, total = 0;
    CV_Assert(nimages > 0 && nimages == (int)imagePoints1.total() &&
        (!imgPtMat2 || nimages == (int)imagePoints2.total()));

    for( i = 0; i < nimages; i++ )
    {
        ni = objectPoints.getMat(i).checkVector(3, CV_32F);
        CV_Assert( ni >= 0 );
        total += ni;
    }

    npoints.create(1, (int)nimages, CV_32S);
    objPtMat.create(1, (int)total, CV_32FC3);
    imgPtMat1.create(1, (int)total, CV_32FC2);
    Point2f* imgPtData2 = 0;

    if( imgPtMat2 )
    {
        imgPtMat2->create(1, (int)total, CV_32FC2);
        imgPtData2 = imgPtMat2->ptr<Point2f>();
    }

    Point3f* objPtData = objPtMat.ptr<Point3f>();
    Point2f* imgPtData1 = imgPtMat1.ptr<Point2f>();

    for( i = 0; i < nimages; i++, j += ni )
    {
        Mat objpt = objectPoints.getMat(i);
        Mat imgpt1 = imagePoints1.getMat(i);
        ni = objpt.checkVector(3, CV_32F);
        int ni1 = imgpt1.checkVector(2, CV_32F);
        CV_Assert( ni > 0 && ni == ni1 );
        npoints.at<int>(i) = ni;
        memcpy( objPtData + j, objpt.data, ni*sizeof(objPtData[0]) );
        memcpy( imgPtData1 + j, imgpt1.data, ni*sizeof(imgPtData1[0]) );

        if( imgPtData2 )
        {
            Mat imgpt2 = imagePoints2.getMat(i);
            int ni2 = imgpt2.checkVector(2, CV_32F);
            CV_Assert( ni == ni2 );
            memcpy( imgPtData2 + j, imgpt2.data, ni*sizeof(imgPtData2[0]) );
        }
    }
}

static Mat prepareCameraMatrix(Mat& cameraMatrix0, int rtype)
{
    Mat cameraMatrix = Mat::eye(3, 3, rtype);
    if( cameraMatrix0.size() == cameraMatrix.size() )
        cameraMatrix0.convertTo(cameraMatrix, rtype);
    return cameraMatrix;
}

static Mat prepareDistCoeffs(Mat& distCoeffs0, int rtype)
{
    Mat distCoeffs = Mat::zeros(distCoeffs0.cols == 1 ? Size(1, 8) : Size(8, 1), rtype);
    if( distCoeffs0.size() == Size(1, 4) ||
       distCoeffs0.size() == Size(1, 5) ||
       distCoeffs0.size() == Size(1, 8) ||
       distCoeffs0.size() == Size(4, 1) ||
       distCoeffs0.size() == Size(5, 1) ||
       distCoeffs0.size() == Size(8, 1) )
    {
        Mat dstCoeffs(distCoeffs, Rect(0, 0, distCoeffs0.cols, distCoeffs0.rows));
        distCoeffs0.convertTo(dstCoeffs, rtype);
    }
    return distCoeffs;
}
double myCalibrateCamera( InputArrayOfArrays _objectPoints,
                            InputArrayOfArrays _imagePoints,
                            Size imageSize, InputOutputArray _cameraMatrix, InputOutputArray _distCoeffs,
                            OutputArrayOfArrays _rvecs, OutputArrayOfArrays _tvecs, int flags, TermCriteria criteria )
{
    int rtype = CV_64F;
    Mat cameraMatrix = _cameraMatrix.getMat();
    cameraMatrix = prepareCameraMatrix(cameraMatrix, rtype);
    Mat distCoeffs = _distCoeffs.getMat();
    distCoeffs = prepareDistCoeffs(distCoeffs, rtype);
    if( !(flags & CALIB_RATIONAL_MODEL) )
        distCoeffs = distCoeffs.rows == 1 ? distCoeffs.colRange(0, 5) : distCoeffs.rowRange(0, 5);

    int    i;
    size_t nimages = _objectPoints.total();
    CV_Assert( nimages > 0 );
    Mat objPt, imgPt, npoints, rvecM((int)nimages, 3, CV_64FC1), tvecM((int)nimages, 3, CV_64FC1);
    collectCalibrationData( _objectPoints, _imagePoints, noArray(),
                            objPt, imgPt, 0, npoints );
    CvMat c_objPt = objPt, c_imgPt = imgPt, c_npoints = npoints;
    CvMat c_cameraMatrix = cameraMatrix, c_distCoeffs = distCoeffs;
    CvMat c_rvecM = rvecM, c_tvecM = tvecM;

    double reprojErr = cvCalibrateCamera2(&c_objPt, &c_imgPt, &c_npoints, imageSize,
                                          &c_cameraMatrix, &c_distCoeffs, &c_rvecM,
                                          &c_tvecM, flags, criteria );

    bool rvecs_needed = _rvecs.needed(), tvecs_needed = _tvecs.needed();

    if( rvecs_needed )
        _rvecs.create((int)nimages, 1, CV_64FC3);
    if( tvecs_needed )
        _tvecs.create((int)nimages, 1, CV_64FC3);

    for( i = 0; i < (int)nimages; i++ )
    {
        if( rvecs_needed )
        {
            _rvecs.create(3, 1, CV_64F, i, true);
            Mat rv = _rvecs.getMat(i);
            memcpy(rv.data, rvecM.ptr<double>(i), 3*sizeof(double));
        }
        if( tvecs_needed )
        {
            _tvecs.create(3, 1, CV_64F, i, true);
            Mat tv = _tvecs.getMat(i);
            memcpy(tv.data, tvecM.ptr<double>(i), 3*sizeof(double));
        }
    }
    cameraMatrix.copyTo(_cameraMatrix);
    distCoeffs.copyTo(_distCoeffs);

    return reprojErr;
}
