#include"bspline/bspline.h"

std::vector<float> bspline_interpolate(float t, int degree, float points[][2], std::vector<float> knots,
                         std::vector<float> weights){
    int i,j,s,l;              // function-scoped iteration variables
    int points_num = 4;//points.size();    // points count
    int dim = 2; //2-D   // point dimensionality

    if(weights.empty()) {
        // build weight vector of length [n]
        for(i=0; i<points_num; i++) {
            weights.push_back(1.);
        }
    }
//    std::cout<<"weights.size()="<<weights.size()<<std::endl;

    if(knots.empty()) {
        // build knot vector of length [n + degree + 1]
        bool open=true;
        if(open){
            float step = 1.0/(points_num+degree);
            float val = 0.;
            for(int k=0; k<points_num+degree+1; k++) {
                knots.push_back(val);
                val = val + step;
            }
            for(int k=0;k<points_num+degree+1;k++){
                std::cout<<knots[k]<<" ";
            }
        }
        else{
            float step = 1.0/(points_num-degree);
            float val = 0.;
            for(int k=0;k<degree;k++)
                knots.push_back(val);
            for(int k=degree; k<points_num; k++) {
                knots.push_back(val);
                val = val + step;
            }
            for(int k=points_num;k<points_num+degree+1;k++)
                knots.push_back(val);
            for(int k=0;k<points_num+degree+1;k++){
                std::cout<<knots[k]<<" ";
            }
        }
        std::cout<<std::endl;
    } else {
        if(knots.size() != points_num+degree+1)
            std::cerr<<"error!"<<std::endl;
    }
//    std::cout<<"knots.size()="<<knots.size()<<std::endl;

    float domain [2]={degree,knots.size()-1 - degree};
    // remap t to the domain where the spline is defined
    float low  = knots[domain[0]];
    float high = knots[domain[1]];
    t = t * (high - low) + low;

    std::cout<<"domain[0]="<<domain[0]<<" "<<"domain[1]="<<domain[1]<<std::endl;
    std::cout<<"low="<<low<<" "<<"high="<<high<<std::endl;
    std::cout<<"t="<<t<<std::endl;
    // find s (the spline segment) for the [t] value provided
    for(s=domain[0]; s<domain[1]; s++) {
        if(t >= knots[s] && t <= knots[s+1]) {
            break;
        }
    }
    std::cout<<"s="<<s<<std::endl;
    // convert points to homogeneous coordinates
    float v[points_num][dim+1];
    for(i=0; i<points_num; i++) {
        for(j=0; j<dim; j++) {
            v[i][j] = points[i][j] * weights[i];
        }
        v[i][dim] = weights[i];
    }
    for(i=0; i<points_num; i++) {
        for(j=0; j<dim+1; j++) {
            std::cout<<v[i][j]<<" ";
        }
        std::cout<<std::endl;
    }
    std::cout<<std::endl;
    // l (level) goes from 1 to the curve degree + 1
    float alpha;
    for(l=1; l<=degree+1; l++) {
        // build level l of the pyramid
        for(i=s; i>s-degree-1+l; i--) {
            alpha = (t - knots[i]) / (knots[i+degree+1-l] - knots[i]);

            // interpolate each component
            for(j=0; j<dim+1; j++) {
                v[i][j] = (1 - alpha) * v[i-1][j] + alpha * v[i][j];
            }
        }
    }
    for(i=0; i<points_num; i++) {
        for(j=0; j<dim+1; j++) {
            std::cout<<v[i][j]<<" ";
        }
        std::cout<<std::endl;
    }
    std::cout<<std::endl;
    // convert back to cartesian and return
    std::vector<float> result;
    for(i=0; i<dim; i++) {
        result.push_back( v[s][i] / v[s][dim]);
    }
    return result;
}
