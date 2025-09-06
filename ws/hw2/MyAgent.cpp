#include "MyAgent.h"
#include "MyObstacle.h"
#include "AMPCore.h"

void PointAgent::move(double dt){
    // moves the robot with x += heading*dt

    x += heading*dt;
}

void PointAgent::rotate(double dtheta){
    // moves the robot's heading by some dtheta (rhrule about +z)

    const Eigen::Matrix2d rot_mat;

    rot_mat << cos(dtheta * M_PI/180.0), -sin(dtheta * M_PI/180.0),
                sin(dtheta * M_PI/180.0), cos(dtheta * M_PI/180.0);

    heading = rot_mat * heading;
    heading.normalize(); //gotta make sure to normalize
}

void rotateUntilFreeRH(std::vector<MyObstacles> obstacles, const double dtheta, const double epsilon){
    /* rotates withe the following schema.

    if heading*epsilon is in collision and -heading*epsilon is not:
        rotate dtheta
    if -heading*epsilon is in collision and heading*epsilon is not:
        rotate -dtheta
    if -heading*epsilon is not collision and heading*epsilon is also not not
        and right_hand * epsilon not in collision

        flip heading

    */



};

