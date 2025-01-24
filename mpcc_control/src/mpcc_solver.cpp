#include "mpcc_control/mpcc_solver.hpp"

namespace mpcc {
void mpcc_solver::runMPC(const State &x0) {
    auto t1 = std::chrono::high_resolution_clock::now();
    


}

void mpcc_solver::setTrack(const Eigen::VectorXd &X, const Eigen::VectorXd &Y) {
    track_.gen2DSpline(X, Y);
    int solver_status = -1;

}
}
