#include "Primative.h"

double LinearPrimative::evaluatePoint(Eigen::Vector2d q) {
    // defines linear primative with the two points on the class and
    // returns ax + by - c <= 0

    // from two points define slope
    double m = (point_b[1] - point_a[1]) / (point_b[0] - point_a[0]);

    return m*(q[0] - point_b[0]) - (q[1] - point_b[1]); // a rearranged point slope form

}