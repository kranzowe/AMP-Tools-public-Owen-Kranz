#ifndef MYOBSTACLE_H
#define MYOBSTACLE_H

#include "AMPCore.h"
#include "Primative.h"

class MyObstacle {
    public:
        // vector holding my linear primatives
        std::vector<LinearPrimative> primatives;

        bool collisionCheck(Eigen::Vector2d) const;
        bool MyObstacle::collisionCheckAlongLine(Eigen::Vector2d q_start, Eigen::Vector2d q_end) const;

        void defineWithPoints(const std::vector<Eigen::Vector2d>& vertices);
};

#endif