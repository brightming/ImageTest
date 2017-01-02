

#include <string>

#include "str_common.h"
#include "lanedetect/hough_color_lane_detect.h"

using namespace std;
using namespace cv;

void detect_vid(string path)
{
    Mat frame;
    VideoCapture cap(path); // open the video file for reading



    if ( !cap.isOpened() )  // if not success, exit program
        cout << "Cannot open the video file" << endl;

    //cap.set(CV_CAP_PROP_POS_MSEC, 300); //start the video at 300ms

    double fps = cap.get(CV_CAP_PROP_FPS); //get the frames per seconds of the video
    cout << "Input video's Frame per seconds : " << fps << endl;

    cap.read(frame);


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
        hough_color_detect_img(frame,1);
        //        stop =clock();
        //        cout<<"fps : "<<1.0/(((double)(stop-start))/ CLOCKS_PER_SEC)<<endl;

        imshow("src",frame);
//        imwrite("/home/gumh/Videos/1.png",frame);
//        break;
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


void detect_folder(string& input_file){


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
        hough_color_detect_img(src,1);
        imshow("src",src);
        int key=waitKey(33);
        if(key==27){
            break;
        }
    }
}

int main(int argc,char* argv[]){

    string input_file="/home/gumh/Videos/2016_12_22卡车驾驶室内拍路面视频/20161222_IMG_8572.MOV";
    string output_file="";

    detect_vid(input_file);



}

