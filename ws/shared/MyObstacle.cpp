#include "Primative.h"
#include "MyObstacle.h"
#include "AMPCore.h"


bool MyObstacle::collisionCheck(Eigen::Vector2d q) const {

    // loop thru da primatives, if we find all are 0 or negative, COLLISIOONNNNN :o
    for (const auto& primative : primatives){
        if (primative.evaluatePoint(q) > 1e-8) { // using some smol number for floating point
            return false;
        }
    }
    // everything <= 0
    return true;
}

bool MyObstacle::collisionCheckAlongLine(Eigen::Vector2d q_start, Eigen::Vector2d q_end) const {

    double delta_gamma = 0.01;
    for (double gamma = 0.0; gamma <=1.0; gamma += delta_gamma){

        Eigen::Vector2d q_new = gamma*q_start + (1.0-gamma) * q_end;

        if (collisionCheck(q_new)){
            return true;
        }

    }

    // all points pass
    return false;
}


void MyObstacle::defineWithPoints(const std::vector<Eigen::Vector2d>& vertices) {
    /* this method loops through a set of CCW vertices and defines primatives and
    stores them in the primative vector. GPT helped me with syntax here cuz c++ loops r hard
    */

    primatives.clear(); //clear existing

    size_t n = vertices.size();
    if (n < 2) return;

    for (size_t i = 0; i < n; ++i) {
        LinearPrimative prim;
        prim.point_a = vertices[i];
        prim.point_b = vertices[(i + 1) % n]; // %n ensures last to first happens
        primatives.push_back(prim);
    }
}
