#include "Primative.h"
#include "AMPCore.h"
double LinearPrimative::evaluatePoint(Eigen::Vector2d q) {
    // defines linear primative with the two points on the class and
    // returns ax + by - c <= 0

    // tbh, I think this is easier with the 2D cross product.

    Eigen::Vector2d v_line = point_b - point_a;
    Eigen::Vector2d v_point = q - point_a;

    // this returns the 2D cross product. 
    // if 0, they are parallel (point is on the line)
    // if negative, point is to the left of the line
    // if positive, point is to the right of the line
    
    // so by my convention, I define polygons CCW using vertices
    return v_line[0]*v_point[1] - v_line[1]*v_point[0];// a rearranged point slope form

}