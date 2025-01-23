#include "mpcc_control/CubicSpline.hpp"
#include <iostream>

namespace mpcc {

void CubicSpline::setRegularData(const Eigen::VectorXd &x_in,const Eigen::VectorXd &y_in,const double delta_x) {
    //if x and y have same length, stare given data in spline data struct
    if(x_in.size() == y_in.size())
    {
        spline_data_.x_data = x_in;
        spline_data_.y_data = y_in;
        spline_data_.n_points = x_in.size();
        spline_data_.is_regular = true;
        spline_data_.delta_x = delta_x;

        data_set_ = true;
    }
    else
    {
        std::cout << "input data does not have the same length" << std::endl;
    }
}

void CubicSpline::setData(const Eigen::VectorXd &x_in,const Eigen::VectorXd &y_in)
{
    //if x and y have same length, stare given data in spline data struct
    if(x_in.size() == y_in.size())
    {
        spline_data_.x_data = x_in;
        spline_data_.y_data = y_in;
        spline_data_.n_points = x_in.size();
        spline_data_.is_regular = false;
        spline_data_.delta_x = 0;

        data_set_ = true;
    }
    else
    {
        std::cout << "input data does not have the same length" << std::endl;
    }
}

bool CubicSpline::compSplineParams()
{
    // compute spline parameters parameters
    // code is a replica of the wiki code
    if(!data_set_)
    {
        return false;
    }
    // spline parameters from parameter struct initialized to zero
    spline_params_.a.setZero(spline_data_.n_points);
    spline_params_.b.setZero(spline_data_.n_points-1);
    spline_params_.c.setZero(spline_data_.n_points);
    spline_params_.d.setZero(spline_data_.n_points-1);

    // additional variables used to compute a,b,c,d
    Eigen::VectorXd mu, h, alpha, l, z;
    mu.setZero(spline_data_.n_points-1);
    h.setZero(spline_data_.n_points-1);
    alpha.setZero(spline_data_.n_points-1);
    l.setZero(spline_data_.n_points);
    z.setZero(spline_data_.n_points);

    // a is equal to y data
    spline_params_.a = spline_data_.y_data;
    // compute h as diff of x data
    for(int i = 0;i<spline_data_.n_points-1;i++)
    {
        h(i) = spline_data_.x_data(i+1) - spline_data_.x_data(i);
    }
    // compute alpha
    for(int i = 1;i<spline_data_.n_points-1;i++)
    {
        alpha(i) = 3.0/h(i)*(spline_params_.a(i+1) - spline_params_.a(i)) - 3.0/h(i-1)*(spline_params_.a(i) - spline_params_.a(i-1));
    }

    // compute l, mu, and z
    l(0) = 1.0;
    mu(0) = 0.0;
    z(0) = 0.0;
    for(int i = 1;i<spline_data_.n_points-1;i++)
    {
        l(i) = 2.0*(spline_data_.x_data(i+1) - spline_data_.x_data(i-1)) - h(i-1)*mu(i-1);
        mu(i) = h(i)/l(i);
        z(i) = (alpha(i) - h(i-1)*z(i-1))/l(i);
    }
    l(spline_data_.n_points-1) = 1.0;
    z(spline_data_.n_points-1) = 0.0;

    // compute b,c,d data given the previous work
    spline_params_.c(spline_data_.n_points-1) = 0.0;

    for(int i = spline_data_.n_points-2;i>=0;i--)
    {
        spline_params_.c(i) = z(i) - mu(i)*spline_params_.c(i+1);
        spline_params_.b(i) = (spline_params_.a(i+1) - spline_params_.a(i))/h(i) - (h(i)*(spline_params_.c(i+1) + 2.0*spline_params_.c(i)))/3.0;
        spline_params_.d(i) = (spline_params_.c(i+1) - spline_params_.c(i))/(3.0*h(i));
    }

    return true;
}

void CubicSpline::genSpline(const Eigen::VectorXd &x_in,const Eigen::VectorXd &y_in,const bool is_regular){
// given x and y data generate spline
// special case for regular or irregular spaced data points in x
// if regular the spacing in x is given by deltaX

// store data in data struct
    if(is_regular)
    {
        double delta_x = x_in(1) - x_in(0);
        setRegularData(x_in,y_in,delta_x);
    }
    else
    {
        setData(x_in,y_in);
    }
// given data compute spline parameters

    bool succes = compSplineParams();

// TODO if succes is flase call exeption
}
}