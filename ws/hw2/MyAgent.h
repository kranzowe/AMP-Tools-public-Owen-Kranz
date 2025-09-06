#ifndef PRIMATIVE_H
#define PRIMATIVE_H
#include "AMPCore.h"
class PointAgent {

    public:
        Eigen::Vector2d x = Eigen::Vector2d(0.0, 0.0); // current position
        Eigen::Vector2d heading = Eigen::Vector2d(1.0, 0.0); // heading. Unit vector. Consider it velocity
        bool collided = False; //todo might be bad??
        bool move(const double dt); //updates x
        void rotate(const double dtheta); //updates heading
        void rotateUntilFreeRH(std::vector<MyObstacles> obstacles, const double dtheta, const double epsilon); //rotates heading until heading and -heading is free and rh is not
        void follow_boundary(const double dt); //moves and rotates around a boundary
        void goToGoal(const Eigen::Vector2d q_goal); // points to goal, moves
};

#endif