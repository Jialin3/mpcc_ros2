#ifndef MPCC_SOLVER_H
#define MPCC_SOLVER_H

#include "mpcc_control/state.hpp"
#include "mpcc_control/ArcLengthSpline.hpp"
#include <Eigen/Dense>
#include <chrono>

namespace mpcc {

struct OptVariables {
    State xk;
    Input uk;
};

struct MPCReturn {
    const Input u0;
    const double time_total;
};

class mpcc_solver {
public:
    void runMPC(const State &x0);
    void setTrack(const Eigen::VectorXd &X, const Eigen::VectorXd &Y);

private:
    ArcLengthSpline track_;
};

}

#endif //MPCC_SOLVER_H