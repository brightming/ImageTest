#include"bspline.h"

using namespace std;
using namespace cv;

int main(){
    float points[4][2]={
//        -1.0,  0.0,
//        -0.5,  0.5,
//        0.5, -0.5,
//        1.0,  0.0
        -100.0,  0.0,
        -50,  50,
        50, -50,
        100,  0.0
    };
//    float tvalues[11] = {0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0};

    int num=11;
    float tvalues[num];
    float val = 0.;
    float step= 1.0/(num-1);
    for(int i=0;i<num;i++){
        tvalues[i]=val;
        val=val+step;
    }

    for(int i=0;i<num;i++){
        cout<<tvalues[i]<<" ";
    }
    cout<<endl;

    vector<float> knots,weights;
//    knots = {0, 0, 0, 1, 2, 2, 2};
    int degree = 2;
    vector<float> res;
    float res_x[num];
    float res_y[num];
    for(int i=0;i<num;i++){
        res=bspline_interpolate(tvalues[i], degree, points, knots, weights);
//        cout<<res[0]<<" "<<res[1]<<endl;
        res_x[i]=res[0];
        res_y[i]=res[1];
    }

    for(int i=0;i<num;i++){
        cout<<res_x[i]<<" ";
    }
    cout<<endl;

    for(int i=0;i<num;i++){
        cout<<res_y[i]<<" ";
    }
    cout<<endl;
    return 0;
}
