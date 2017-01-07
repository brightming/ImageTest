#include "bspline3order.h"
#include <fstream>

int main()
{
    double x[22] = {
        926.500000,
        928.000000,
        929.500000,
        931.000000,
        932.500000,
        934.000000,
        935.500000,
        937.000000,
        938.500000,
        940.000000,
        941.500000,
        943.000000,
        944.500000,
        946.000000,
        977.500000,
        980.500000,
        982.000000,
        983.500000,
        985.000000,
        986.500000,
        988.000000,
        989.500000};

    double y[22] = {
        381.732239,
        380.670530,
        380.297786,
        379.853896,
        379.272647,
        378.368584,
        379.319757,
        379.256485,
        380.233150,
        378.183257,
        377.543639,
        376.948999,
        376.253935,
        198.896327,
        670.369434,
        374.273702,
        372.498821,
        373.149402,
        372.139661,
        372.510891,
        372.772791,
        371.360553};

    std::vector<double> input_x(22), input_y(22);
    for ( int i=0; i<22; i++)
    {
        input_x[i] = x[i];
        input_y[i] = y[i];
    }


    CubicSplineCoeffs *cubicCoeffs;
    CubicSplineInterpolation cubicSpline;
    cubicSpline.calCubicSplineCoeffs(input_x, input_y, cubicCoeffs, CUBIC_NATURAL, CUBIC_MEDIAN_FILTER);

   std::vector<double> output_x, output_y;
   cubicSpline.cubicSplineInterpolation( cubicCoeffs, input_x, output_x, output_y );

    double xx(946.0), yy(0.0);
    cubicSpline.cubicSplineInterpolation2(cubicCoeffs, input_x, xx, yy);
    std::cout<<yy<<std::endl;

    std::ofstream outfile("/home/fung/workspace/width_lanedetection/build/test1.txt", std::ios::out);
    if ( outfile )
    {
        for ( int i=0; i<output_y.size(); i++ )
        {
            outfile<<std::fixed<<setprecision(3)<<output_x[i]<<" "<<output_y[i]<<std::endl;
        }
    }
    outfile.close();
return 0;
}
