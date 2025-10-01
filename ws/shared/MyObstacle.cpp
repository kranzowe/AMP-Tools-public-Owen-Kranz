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

    // gotta make sure we get the end
    if (collisionCheck(q_end)){
        return true;
        }

    if (collisionCheck(q_start)){
        return true;
        }

    // all points pass
    return false;
}


void MyObstacle::defineWithPoints(const std::vector<Eigen::Vector2d>& vertices_inp) {
    /* this method loops through a set of CCW vertices and defines primatives and
    stores them in the primative vector. GPT helped me with syntax here cuz c++ loops r hard
    */

    primatives.clear(); //clear existing
    vertices = vertices_inp; // set param
    size_t n = vertices.size();
    if (n < 2) return;

    for (size_t i = 0; i < n; ++i) {
        LinearPrimative prim;
        prim.point_a = vertices[i];
        prim.point_b = vertices[(i + 1) % n]; // %n ensures last to first happens
        primatives.push_back(prim);
    }
}

double MyObstacle::closestDistanceToq(Eigen::Vector2d q) const {

    // using projection, but gotta check if it outside the segment (in which case itll be the start or
    // end point)

    double min_dist = INFINITY;

    for (uint16_t i = 0; i < vertices.size(); i++){
        Eigen::Vector2d a;
        Eigen::Vector2d b;
        
        if (i == vertices.size() - 1){
            //wrap around
            a = vertices[i];
            b = vertices[0];
        }else{
            a = vertices[i];
            b = vertices[i+1];

        }
        Eigen::Vector2d closest_point;
        Eigen::Vector2d ab = b - a;
        Eigen::Vector2d aq = q - a;

        // projection, t. gonna be less than 0 if its behind a
        // greater than 1 if past b
        double t = ab.dot(aq) / ab.squaredNorm();
        
        if (t < 0.0) {
            closest_point = a;
        } else if (t > 1.0) {
            closest_point = b;
        } else {
            // find the point along the segment
            closest_point = a + t * ab;
        }

        double dist = (q - closest_point).norm();

        if (dist < min_dist){
            min_dist = dist;
        }
    }

    return min_dist;

}