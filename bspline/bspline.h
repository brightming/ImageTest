#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <iostream>
#include <vector>
#include <algorithm>
#include <math.h>
#include <time.h>

std::vector<float> bspline_interpolate(float t, int degree, float points[][2], std::vector<float> knots,
                         std::vector<float> weights);
