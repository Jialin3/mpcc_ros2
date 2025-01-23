#ifndef MPCC_CUBIC_SPLINE_H
#define MPCC_CUBIC_SPLINE_H


#include <Eigen/Dense>

namespace mpcc {

struct SplineParams{
    Eigen::VectorXd a;
    Eigen::VectorXd b;
    Eigen::VectorXd c;
    Eigen::VectorXd d;
};
// input data for spline
struct SplineData{
    Eigen::VectorXd x_data;     //x data
    Eigen::VectorXd y_data;     //y data
    int n_points;          //number of points
    bool is_regular;    //regular (1) or irregular (0) spaced points in x direction
    double delta_x;  //spacing of regular space points
};

class CubicSpline {
public:
    void genSpline(const Eigen::VectorXd &x_in,const Eigen::VectorXd &y_in,const bool is_regular);
   
    
private:
    void setRegularData(const Eigen::VectorXd &x_in,const Eigen::VectorXd &y_in,const double delta_x);
    void setData(const Eigen::VectorXd &x_in,const Eigen::VectorXd &y_in);
    bool compSplineParams();
    SplineParams spline_params_;
    SplineData spline_data_;
    bool data_set_;
};
}
#endif //MPCC_CUBIC_SPLINE_H

