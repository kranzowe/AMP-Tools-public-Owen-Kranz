#ifndef PRIMATIVE_H
#define PRIMATIVE_H
#include "AMPCore.h"
class LinearPrimative {

    public:
        Eigen::Vector2d point_a = Eigen::Vector2d(0.0, 0.0); // a point on the line
        Eigen::Vector2d point_b = Eigen::Vector2d(0.0, 0.0); // a second point on the line

        double evaluatePoint(Eigen::Vector2d) const;
};

class MovingCircularPrimative {

    public:
        double radius = 0.0; // radius

        double evaluatePoint(Eigen::Vector2d q, Eigen::Vector2d center) const;
};



#endif