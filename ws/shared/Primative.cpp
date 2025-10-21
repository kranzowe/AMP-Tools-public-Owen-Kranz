#include "Primative.h"
#include "AMPCore.h"
double LinearPrimative::evaluatePoint(Eigen::Vector2d q) const{
    // defines linear primative with the two points on the class and
    // returns ax + by + c <= 0
    // from two points 
    // a = (y2-y1)
    // b = -(x2-x1)
    // c = x2*y1 - x1*y2

    double a = point_b[1] - point_a[1];
    double b = -(point_b[0] - point_a[0]);
    double c = (point_b[0]*point_a[1]) - (point_a[0]*point_b[1]);
    
    // so by my convention, all points to the left of the vector 
    // point a to point b are negative. therefore I define polygons cCW using vertices
    return a*q[0] + b*q[1] + c;

}

double MovingCircularPrimative::evaluatePoint(Eigen::Vector2d q, Eigen::Vector2d center) const{

    double val = ((q[0] - center[0])*(q[0] - center[0])) + ((q[1] - center[1])*(q[1] - center[1])) - (radius*radius);
    
    // so by my convention, all points to the left of the vector 
    // point a to point b are negative. therefore I define polygons cCW using vertices
    return val

}