#ifndef MYOBSTACLE_H
#define MYOBSTACLE_H

#include "AMPCore.h"
#include "Primative.h"

class MyObstacle {
    public:
        // vector holding my linear primatives
        std::vector<LinearPrimative> primatives;

        bool collisionCheck(Eigen::Vector2d) const;

        void defineWithPoints(const std::vector<Eigen::Vector2d>& vertices);
};

#endif