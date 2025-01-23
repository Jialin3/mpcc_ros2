#ifndef MPCC_TRACK_H
#define MPCC_TRACK_H

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <Eigen/Dense> 

namespace mpcc {

struct TrackPos {
    const Eigen::VectorXd X;
    const Eigen::VectorXd Y;
};

class Track {
public:
    Track(std::string file);
    TrackPos getTrack();

private:
    Eigen::VectorXd X;
    Eigen::VectorXd Y;
};
};

#endif //MPCC_TRACK_H
