#ifndef MkMath_h
#define MkMath_h

#include <arduino.h>

namespace MkMath {
    // map function for floats (Arduino only allows int).
    float mapf(float, float, float, float, float);

    // Convert x and y components into radian
    float xy2rad(float x, float y);
    // Convert Radian into Euler angle (Degrees)
    float rad2deg(float rad);
    // Convert Euler angle (Degrees) into Radian
    float deg2rad(float deg);

    // Get distance between two points
    float distance(float x1, float y1, float x2, float y2);

    // Collision resolve and velocity transfer between 2 circles R 1 means perfect reflection and velocity transfer
    void collision2D(float m1, float m2, float R, float& x1, float& y1, float& x2, float& y2, float& vx1, float& vy1, float& vx2, float& vy2); 
} // namespace MkMath

#endif