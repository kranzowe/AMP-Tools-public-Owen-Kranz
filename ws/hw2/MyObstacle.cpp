#include "MyObstacle.h"

#include "AMPCore.h"

bool MyObstacle::collisionCheck(Eigen::Vector2d q) const {

    // loop thru da primatives, if we find a o or negative, COLLISIOONNNNN :o
    for (const auto& primative : primatives){
        if (primative.evaluatePoint(q) <= 0) {
            return true;
        }
    }
    // everything >0
    return false;
}