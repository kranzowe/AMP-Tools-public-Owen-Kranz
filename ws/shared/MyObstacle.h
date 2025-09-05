#ifndef OBSTACLE_H
#define OBSTACLE_H

#include "AMPCore.h"

class MyObstacle {
    public:
        using Primative = std::<bool(Eigen::Vector2d)>;
        std::vector<Primative> primatives; // vector holding all the primative functions you want


        bool collisionCheck(Eigen::Vector2d, Eigen::Vector2d) const;
};

#endif