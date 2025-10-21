#ifndef MYOBSTACLE_H
#define MYOBSTACLE_H

#include "AMPCore.h"
#include "Primative.h"

class MyObstacle {
    public:
        // vector holding my linear primatives
        std::vector<Eigen::Vector2d> vertices;
        std::vector<LinearPrimative> primatives;
        MovingCircularPrimative moving_circular_primative;

        bool collisionCheck(Eigen::Vector2d) const;
        bool collisionCheckTranslated(Eigen::Vector2d q, Eigen::Vector2d center) const;
        bool collisionCheckAlongLine(Eigen::Vector2d q_start, Eigen::Vector2d q_end) const;

        void defineWithPoints(const std::vector<Eigen::Vector2d>& vertices);
        void defineWithDiscRadius(const double radius);

        std::pair<double, Eigen::Vector2d> closestDistanceToq(const Eigen::Vector2d q) const;
        std::pair<double, Eigen::Vector2d> distanceToCentroid(const Eigen::Vector2d q) const;
};

#endif