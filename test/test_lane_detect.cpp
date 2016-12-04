

#include <stdio.h>
#include <dirent.h>
#include <sys/stat.h>
#include <cctype>
#include <algorithm>
#include <math.h>
#include <fstream>
#include <string>

#include <opencv2/opencv.hpp>



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
float trap_bottom_width = 0.85 ;// # width of bottom edge of trapezoid, expressed as percentage of image width
float trap_top_width = 0.07 ;// # ditto for top edge of trapezoid
float trap_height = 0.4 ;// # height of the trapezoid expressed as percentage of image height

//Hough Transform
int rho = 2;// # distance resolution in pixels of the Hough grid
const float PI=3.1415926;
float theta = 1 * PI/180 ;//# angular resolution in radians of the Hough grid
int hough_threshold = 15	;// # minimum number of votes (intersections in Hough grid cell)
int min_line_length = 10 ;//#minimum number of pixels making up a line
int max_line_gap = 20	;//# maximum gap in pixels between connectable line segments

//for valid line slope
float min_left_slope=10;
float max_left_slope=60;
float min_right_slope=45;
float max_right_slope=60;


bool IsDir(std::string path)
{
    struct stat sb;
    if (stat(path.c_str(), &sb) == -1) return false;
    return S_ISDIR(sb.st_mode);
}



/**
 * 获取指定路径下的文件名，不包含路径名
 */
vector<string> getAllFilesWithPathFromDir(std::string dirPath){

    cout<<"getAllFilesWithPathFromDir:dirpath="<<dirPath<<endl;

    struct dirent* ptr;
    vector<string> files;
    DIR *dir;
    dir=opendir(dirPath.c_str());
    if(dir==NULL){
        cout<<"fail to open path:"<<dirPath<<endl;
        return files;
    }



    char returnFile[1024];
    while((ptr=readdir(dir))!=NULL){
        if(ptr->d_name[0]=='.' ){//|| ptr->d_type!= DT_REG){//对于xfs文件系统，这个值总是返回0！需要另一个方法去判断
            //            cout<<ptr->d_name<<" is directory."<<",ptr->d_name[0]="<<ptr->d_name[0]<<",ptr->d_type="<<(int)ptr->d_type<<",DT_REG="<<DT_REG<<endl;
            continue;
        }

        sprintf(returnFile,"%s/%s",dirPath.c_str(),ptr->d_name);
        if(IsDir(returnFile)){
            cout<<ptr->d_name<<" is directory."<<endl;
            continue;
        }
        files.push_back(returnFile);
    }
    closedir(dir);

    return files;

}


class LeastSquare{
    double m, b;
public:
    LeastSquare(const vector<double>& x, const vector<double>& y)
    {
        double t1=0, t2=0, t3=0, t4=0;
        for(int i=0; i<x.size(); ++i)
        {
            t1 += x[i]*x[i];
            t2 += x[i];
            t3 += x[i]*y[i];
            t4 += y[i];
        }
        m = (t3*x.size() - t2*t4) / (t1*x.size() - t2*t2);
        //b = (t4 - a*t2) / x.size();
        b = (t1*t4 - t2*t3) / (t1*x.size() - t2*t2);
    }

    double getY(const double x) const
    {
        return m*x + b;
    }

    double getM() {
        return m;
    }

    double getB() {
        return b;
    }

    void print() const
    {
        cout<<"y = "<<m<<"x + "<<b<<"\n";
    }

};

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
//    imshow("mask",mask);

    //    returning the image only where mask pixels are nonzero
    Mat masked_image ;
    bitwise_and(img,img,masked_image,mask);

    imshow("masked_image",masked_image);
    return masked_image;


}

Mat filter_colors(Mat& src){


    //        # Filter white pixels
    int white_threshold=200;
    Scalar lower_white(white_threshold,white_threshold,white_threshold);
    Scalar upper_white(255, 255, 255);
    Mat white_mask;
    inRange(src,lower_white,upper_white,white_mask);
    //    imshow("white_mask",white_mask);
    Mat white_image;
    bitwise_and(src,src,white_image,white_mask);
    //    imshow("white_image",white_image);

    //        # Filter yellow pixels

    Mat hsv;
    cvtColor(src,hsv,COLOR_BGR2HSV);
    Scalar lower_yellow(90,100,100);
    Scalar upper_yellow(110,255,255);
    Mat yellow_mask;
    inRange(hsv,lower_yellow,upper_yellow,yellow_mask);
    Mat yellow_image;
    bitwise_and(src,src,yellow_image,yellow_mask);
    //    imshow("yellow_image",yellow_image);

    //    # Combine the two above images
    Mat combined_img;
    addWeighted(white_image,1.,yellow_image,1.,0,combined_img);
    //    imshow("combined_img",combined_img);


    return combined_img;



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
        cout<<"x1="<<x1<<",y1="<<y1<<",x2="<<x2<<",y2="<<y2<<",slope="<<slope<<endl;

        if(slope<0 && abs(slope)>left_min_slope && abs(slope)<left_max_slope
                && x1<=img_center_x && x2<=img_center_x
                ){
            output_lines.push_back(line);
        }else if(slope>0 && abs(slope)>right_min_slope && abs(slope)<right_max_slope
                 && x1>=img_center_x && x2>=img_center_x
                 ){
            output_lines.push_back(line);
        }

    }


}


void draw_lines(Mat& img, vector<Vec4i> lines, Scalar color=Scalar(255, 0, 0), int thickness=5){

    for( size_t i = 0; i < lines.size(); i++ )
    {
        line( img, Point(lines[i][0], lines[i][1]),
                Point(lines[i][2], lines[i][3]), color, thickness, 8 );
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

void detect_img(Mat& src){
    //    """ Given an image Numpy array, return the annotated image as a Numpy array """
    //        # Only keep white and yellow pixels in the image, all other pixels become black
    Mat image = filter_colors(src);
//    Mat image=src.clone();

    //        # Read in and grayscale the image
    Mat gray;
    cvtColor(image,gray,COLOR_BGR2GRAY);

    //        Apply Gaussian smoothing
    Mat blur_gray;
    GaussianBlur(gray,blur_gray,Size(kernel_size,kernel_size),0,0);

    //        Apply Canny Edge Detector
    Mat edges;
    cv::Canny(blur_gray,edges,low_threshold,high_threshold);


    //        # Create masked edges using trapezoid-shaped region-of-interest
    vector<Point> vertices;
    int width=src.cols;
    int height=src.rows;
    int channel_count=src.channels();
    vertices.push_back(Point(width*(1 - trap_bottom_width) / 2,height));
    vertices.push_back(Point(width*(1 - trap_top_width) / 2,height-height*trap_height));
    vertices.push_back(Point(width - width*(1 - trap_top_width) / 2,height-height*trap_height));
    vertices.push_back(Point(width - width*(1 - trap_bottom_width) / 2,height));

    Mat masked_edges = region_of_interest(edges, vertices);


    //        # Run Hough on edge detected image
    vector<Vec4i> lines;
    HoughLinesP( masked_edges, lines, rho, theta, hough_threshold, min_line_length, max_line_gap );
    std::cout<<"lines size="<<lines.size()<<std::endl;


    //---filter out invalid lines by slope
    vector<Vec4i> valid_lines;
    int img_center_x=src.cols/2;//x coordinate of center of image
    filter_lines_by_slope(lines,valid_lines,img_center_x,min_left_slope,max_left_slope,min_right_slope,max_right_slope);

    Mat line_image=Mat::zeros(src.rows,src.cols,src.type());
    std::cout<<"line_image.channel="<<line_image.channels()<<std::endl;
    draw_lines(line_image,valid_lines);
//    imshow("lines",line_image);


    //        # Draw lane lines on the original image
    Mat initial_image = src.clone();
    Mat annotated_image = weighted_img(line_image, initial_image);

    imshow("annotated_image",annotated_image);


    //        return annotated_image
}

int main(int argc,char* argv[]){

    string input_file="/home/gumh/TrainData/KITTI/21/image_0/";
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
//        imshow("src",src);

        Mat out;
        detect_img(src);
        int key=waitKey(0);
        if(key==27){
            break;
        }
    }

}

