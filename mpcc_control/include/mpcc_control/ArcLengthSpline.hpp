#ifndef MPCC_ARC_LENGTH_SPLINE_H
#define MPCC_ARC_LENGTH_SPLINE_H

#include <Eigen/Dense> 
#include "mpcc_control/CubicSpline.hpp"


namespace mpcc {

struct PathData{
    Eigen::VectorXd X;
    Eigen::VectorXd Y;
    Eigen::VectorXd s;
    int n_points;
};


class ArcLengthSpline {
public:

private:
    void fitSpline(const Eigen::VectorXd &X,const Eigen::VectorXd &Y);
    Eigen::VectorXd compArcLength(const Eigen::VectorXd &X_in,const Eigen::VectorXd &Y_in) const;
    PathData resamplePath(const CubicSpline &initial_spline_x,const CubicSpline &initial_spline_y,double total_arc_length) const;
    void setRegularData(const Eigen::VectorXd &X_in,const Eigen::VectorXd &Y_in,const Eigen::VectorXd &s_in);

    CubicSpline spline_x_;
    CubicSpline spline_y_;
    PathData path_data_; 
};

}
#endif //MPCC_ARC_LENGTH_SPLINE_H