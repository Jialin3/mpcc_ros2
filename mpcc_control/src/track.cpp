#include "mpcc_control/track.hpp"

namespace mpcc {

Track::Track(std::string file) 
{
    std::ifstream infile(file);
    if (!infile.is_open()) {
        throw std::runtime_error("Failed to open track file");
    }

    std::string line;
    std::getline(infile, line);
    std::vector<double> x, y;

    while(std::getline(infile, line)) {
        std::istringstream iss(line);
        double val_x, val_y;
        char comma;
        if (!(iss >> val_x >> comma >> val_y)) {
            throw std::runtime_error("Failed to parse track file");
        }
        x.push_back(val_x);
        y.push_back(val_y);
    }
    X = Eigen::Map<Eigen::VectorXd>(x.data(), x.size());
    Y = Eigen::Map<Eigen::VectorXd>(y.data(), y.size());
}

TrackPos Track::getTrack()
{
    return {Eigen::Map<Eigen::VectorXd>(X.data(), X.size()), Eigen::Map<Eigen::VectorXd>(Y.data(), Y.size())};
}
}