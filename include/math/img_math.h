#ifndef __IMG_MATH_H__

#define __IMG_MATH_H__


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

#endif //__IMG_MATH_H__
