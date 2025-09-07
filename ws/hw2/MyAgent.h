#ifndef MYAGENT_H
#define MYAGENT_H
#include "AMPCore.h"
#include "MyObstacle.h"
class PointAgent {

    public:
        Eigen::Vector2d x = Eigen::Vector2d(0.0, 0.0); // current position
        Eigen::Vector2d heading = Eigen::Vector2d(1.0, 0.0); // heading. Unit vector. Consider it velocity
        bool collided = false; //todo might be bad??
        bool move(const std::vector<MyObstacle> obstacles, const double dt, const double epsilon); //updates x. returns true if collided
        void rotate(const double dtheta); //updates heading
        bool rotateToCircumnavigateRH(const std::vector<MyObstacle> obstacles, const double dtheta, const double epsilon); //rotates heading until heading and -heading is free and rh is not
        bool rotateToCircumnavigateRHInteriorCorner(const std::vector<MyObstacle> obstacles, const double dtheta, const double epsilon); //special case attempted if first effort didnt work

        //void follow_boundary(const double dt); //moves and rotates around a boundary
        void pointAtGoal(const Eigen::Vector2d q_goal); // points to goal, moves
};

#endif