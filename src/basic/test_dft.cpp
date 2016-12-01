#include <iostream>
#include <vector>
#include <math.h>
#include <cv.h>
#include <highgui.h>
#include "opencv2/opencv.hpp"

using namespace std;
using namespace cv;

#define PI 3.14159265

Mat generate_simple_black(Mat& src){





    int row=src.rows;
    int col=src.cols;
   // src.zeros(row,col,CV_8UC1);

    src(Rect(10,10,20,20))+=Scalar::all(100);

}


typedef struct dft_point{
    float real_part;
    float image_part;
}dft_point;

vector<float> one_dimensions_dft(vector<dft_point>& datas){
    vector<float> results;
    int M=datas.size();
    for(int x=0;x<M;x++){
        float val=0;
        for(int u=0;u<M;u++){
            val+=datas[u].real_part*cos(2*PI*u*x/M) - datas[u].image_part*sin(2*PI*u*x/M);
        }
        results.push_back(val);
    }

    return results;

}

vector<dft_point> one_dimensions_dft(vector<float>& datas){
    vector<dft_point> results;
    int M=datas.size();
    for(int u=0;u<M;u++){
        float real_p=0;
        float image_p=0;
        for(int x=0;x<M;x++){
            real_p+=datas[x]*cos(2*PI*u*x/M);
            image_p+=datas[x]*sin(2*PI*u*x/M);
//            cout<<"real_p="<<real_p<<endl;
        }
        real_p=real_p/M;
        image_p=image_p/M*(-1);

        dft_point dp;
        dp.real_part=real_p;
        dp.image_part=image_p;
//        cout<<"push_back"<<endl;
        results.push_back(dp);
    }

    return results;
}



int test_show_image_dft(int argc, char *argv[])
{
    Mat image(400,400,CV_8UC1);
    if (argc < 2) {
//        cout<<"Usage:./fft2 [image name]"<<endl;
//        return -1;

        //generate_simple_black(image);
        int cnt=5;
        int start_y=0;
        int height=image.rows/cnt;
        while(start_y<image.rows){
            if(start_y+height>=image.rows){
                 height=image.rows-start_y;
            }
            std::cout<<"start_y="<<start_y<<",height="<<height<<endl;
            image(Rect(0,start_y,image.cols,height))+=Scalar::all((start_y/height +1)* 60);
            start_y+=2*height;

        }
        imshow("img",image);
        waitKey(0);
    }else{

        // Read as grayscale image
        image = imread(argv[1], CV_LOAD_IMAGE_GRAYSCALE);
        if (!image.data) {
            cout << "Read image error"<<endl;
            return -1;
        }
        resize(image,image,Size(image.rows/4,image.cols/4));
    }

    Mat padded;
    int m = getOptimalDFTSize(image.rows);  // Return size of 2^x that suite for FFT
    int n = getOptimalDFTSize(image.cols);
    // Padding 0, result is @padded
    copyMakeBorder(image, padded, 0, m-image.rows, 0, n-image.cols, BORDER_CONSTANT, Scalar::all(0));

    // Create planes to storage REAL part and IMAGE part, IMAGE part init are 0
    Mat planes[] = {Mat_<float>(padded), Mat::zeros(padded.size(), CV_32F) };
    Mat complexI;
    merge(planes, 2, complexI);

    dft(complexI, complexI);

    // compute the magnitude and switch to logarithmic scale
    split(complexI, planes);
    magnitude(planes[0], planes[0], planes[1]);
    Mat magI = planes[0];

    // => log(1+sqrt(Re(DFT(I))^2+Im(DFT(I))^2))
    magI += Scalar::all(1);
    log(magI, magI);

    // crop the spectrum
    magI = magI(Rect(0, 0, magI.cols & (-2), magI.rows & (-2)));
    Mat _magI = magI.clone();
    normalize(_magI, _magI, 0, 1, CV_MINMAX);

    // rearrange the quadrants of Fourier image so that the origin is at the image center
    int cx = magI.cols/2;
    int cy = magI.rows/2;



    Mat q0(magI, Rect(0,0,cx,cy));    // Top-Left
    Mat q1(magI, Rect(cx,0,cx,cy));   // Top-Right
    Mat q2(magI, Rect(0,cy,cx,cy));   // Bottom-Left
    Mat q3(magI, Rect(cx,cy,cx,cy));  // Bottom-Right

    // exchange Top-Left and Bottom-Right
    Mat tmp;
    q0.copyTo(tmp);
    q3.copyTo(q0);
    tmp.copyTo(q3);

    // exchange Top-Right and Bottom-Left
    q1.copyTo(tmp);
    q2.copyTo(q1);
    tmp.copyTo(q2);

    normalize(magI, magI, 0, 1, CV_MINMAX);

    imshow("Input image", image);
    imshow("B"/*"Spectrum magnitude before shift frequency"*/, _magI);
    imshow("A"/*"Spectrum magnitude after shift frequency"*/, magI);
    waitKey();

    return 0;
}


int main(int argc,char* argv[]){


    vector<float> datas;
    datas.push_back(1);
    datas.push_back(2);
    datas.push_back(3);
    datas.push_back(4);
    datas.push_back(1);
    datas.push_back(2);
    datas.push_back(3);
    datas.push_back(4);
    datas.push_back(1);
    datas.push_back(2);
    datas.push_back(3);
    datas.push_back(4);


    cout<<"original data:"<<endl;
    for(float d:datas){
        cout<<d<<" ";
    }
    cout<<endl;


    vector<dft_point> dps=one_dimensions_dft(datas);
    cout<<"\nfft result----"<<endl;
    cout<<"real part:"<<endl;
    for(dft_point dp:dps){
        cout<<dp.real_part<<" ";
    }
    cout<<endl;
    cout<<"image part:"<<endl;
    for(dft_point dp:dps){
        cout<<dp.image_part<<" ";
    }
    cout<<endl;

    cout<<"\ndata revert from dft result:"<<endl;
    vector<float> twos=one_dimensions_dft(dps);
    for(float v:twos){
        cout<<v<<" ";
    }
    cout<<endl;


    //-------------------------------------------------------
    //filter out little coefficients
    vector<dft_point> no_little_coes;
    for(dft_point dp:dps){
        dft_point tmp=dp;
        if(abs(tmp.real_part)<0.0001){
            tmp.real_part=0;
        }
        if(abs(tmp.image_part)<0.0001){
            tmp.image_part=0;
        }
        no_little_coes.push_back(tmp);
    }
    cout<<"\nafter filter out little coefficients----"<<endl;
    cout<<"real part:"<<endl;
    for(dft_point dp:no_little_coes){
        cout<<dp.real_part<<" ";
    }
    cout<<endl;
    cout<<"image part:"<<endl;
    for(dft_point dp:no_little_coes){
        cout<<dp.image_part<<" ";
    }
    cout<<endl;

    cout<<"\ndata revert from dft result without little coefficients:"<<endl;
    vector<float> three=one_dimensions_dft(no_little_coes);
    for(float v:three){
        cout<<v<<" ";
    }
    cout<<endl;
    //---------------------------------------------


    //filter out big coefficients
    vector<dft_point> no_big_coes;
    for(dft_point dp:dps){
        dft_point tmp=dp;
        if(abs(tmp.real_part)>0.0001){
            tmp.real_part=0;
        }
        if(abs(tmp.image_part)>0.0001){
            tmp.image_part=0;
        }
        no_big_coes.push_back(tmp);
    }
    cout<<"\nafter filter out big coefficients----"<<endl;
    cout<<"real part:"<<endl;
    for(dft_point dp:no_big_coes){
        cout<<dp.real_part<<" ";
    }
    cout<<endl;
    cout<<"image part:"<<endl;
    for(dft_point dp:no_little_coes){
        cout<<dp.image_part<<" ";
    }
    cout<<endl;

    cout<<"\ndata revert from dft result without big coefficients:"<<endl;
    vector<float> four=one_dimensions_dft(no_big_coes);
    for(float v:four){
        cout<<v<<" ";
    }
    cout<<endl;
    //---------------------------------------------


    return 0;
}
