#include "mpcc_control/ArcLengthSpline.hpp"
#include <iostream>

namespace mpcc {
ArcLengthSpline::ArcLengthSpline(){}

void ArcLengthSpline::fitSpline(const Eigen::VectorXd &X,const Eigen::VectorXd &Y) {
    Eigen::VectorXd s_approximation;
    double total_arc_length;
    PathData first_refined_path,second_refined_path;
    
    s_approximation = compArcLength(X,Y);
    total_arc_length = s_approximation(s_approximation.size()-1);

    CubicSpline first_spline_x,first_spline_y;
    CubicSpline second_spline_x,second_spline_y;

    // 1. spline fit
    first_spline_x.genSpline(s_approximation,X,false);
    first_spline_y.genSpline(s_approximation,Y,false);

    first_refined_path = resamplePath(first_spline_x,first_spline_y,total_arc_length);
    s_approximation = compArcLength(first_refined_path.X,first_refined_path.Y);
    total_arc_length = s_approximation(s_approximation.size()-1);
     ///////////////////////////////////////////////
     // 2. spline fit
    second_spline_x.genSpline(s_approximation,first_refined_path.X,false);
    second_spline_y.genSpline(s_approximation,first_refined_path.Y,false);
    // 2. re-sample
    second_refined_path = resamplePath(second_spline_x,second_spline_y,total_arc_length);
    setRegularData(second_refined_path.X,second_refined_path.Y,second_refined_path.s);
//    setData(second_refined_path.X,second_refined_path.Y);
    // Final spline fit with fixed Delta_s
    spline_x_.genSpline(path_data_.s,path_data_.X,true);
    spline_y_.genSpline(path_data_.s,path_data_.Y,true);

}

void ArcLengthSpline::setRegularData(const Eigen::VectorXd &X_in,const Eigen::VectorXd &Y_in,const Eigen::VectorXd &s_in) {
    // set final x-y data if x and y have same length
    // x-y points are space such that they are very close to arc length parametrized
    if(X_in.size() == Y_in.size()){
        path_data_.X = X_in;
        path_data_.Y = Y_in;
        path_data_.n_points = X_in.size();
        path_data_.s = s_in;
    }
    else{
        std::cout << "input data does not have the same length" << std::endl;
    }
}

Eigen::VectorXd ArcLengthSpline::compArcLength(const Eigen::VectorXd &X_in,const Eigen::VectorXd &Y_in) const {
    double dx, dy;
    double dist;
    int n_points = X_in.size();
    Eigen::VectorXd s;
    s.setZero(n_points);
    for(int i=0;i<n_points-1;i++)
    {
        dx = X_in(i+1)-X_in(i);
        dy = Y_in(i+1)-Y_in(i);
        dist = std::sqrt(dx*dx + dy*dy);    //dist is straight line distance between points
        s(i+1) = s(i)+dist;       //s is cumulative sum of dist
    }
    return s;
}

}