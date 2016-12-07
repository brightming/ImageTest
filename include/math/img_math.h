#ifndef __IMG_MATH_H__

#define __IMG_MATH_H__

#include <iostream>
#include <vector>
#include <stdio.h>
#include <stdlib.h>

#include "math.h"

using namespace std;
class LeastSquare{
    double m, b;
public:

    LeastSquare(){
	this->m=0;
	this->b=0;
    };
    LeastSquare(const vector<double>& x, const vector<double>& y);


    void setData(const vector<double>& x, const vector<double>& y);

    double getY(const double x) const;

    double getM() ;

    double getB() ;

    void print() const;

    void setM(double m){
	this->m=m;
    }

    void setB(double b){
	this->b=b;
    }

    bool isValid(){
	return !((int)m==0 && (int)b==0);
    }
};





void polyfit(int n, double *x, double *y, int poly_n, double p[]);
void gauss_solve(int n,double A[],double x[],double b[]);




#endif //__IMG_MATH_H__
