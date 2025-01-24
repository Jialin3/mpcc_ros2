#ifndef MPCC_STATE_H
#define MPCC_STATE_H

#include <cmath>

namespace mpcc {
struct  State {
    double x;
    double y;
    double phi;
    double vx;
    double vy;
    double r;
    double s;
    double D;
    double delta;
    double vs;

    void setZeros(){
        x = 0.0;
        y = 0.0;
        phi = 0.0;
        vx = 0.0;
        vy = 0.0;
        r = 0.0;
        s = 0.0;
        D = 0.0;
        delta = 0.0;
        vs = 0.0;
    }

    void unwrap(double track_length)
    {
        if (phi > M_PI)
            phi -= 2.0 * M_PI;
        if (phi < -M_PI)
            phi += 2.0 * M_PI;

        if (s > track_length)
            s -= track_length;
        if (s < 0)
            s += track_length;
    }

     void vxNonZero(double vx_zero)
    {
        if(vx < vx_zero){
            vx = vx_zero;
            vy = 0.0;
            r = 0.0;
            delta = 0.0;
        }
    }
};

struct Input{
    double dD;
    double dDelta;
    double dVs;

    void setZero()
    {
        dD = 0.0;
        dDelta = 0.0;
        dVs = 0.0;
    }
};

}
#endif //MPCC_STATE_H