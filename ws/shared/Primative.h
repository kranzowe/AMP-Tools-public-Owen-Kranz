#ifndef PRIMATIVE_H
#define PRIMATIVE_H
#include <Eigen/Dense>
class LinearPrimative {
    public:
        Eigen::Vector2d point_a; // a point on the line
        Eigen::Vector2d point_b; // a second point on the line

        double evaluatePoint(Eigen::Vector2d);
};

#endif