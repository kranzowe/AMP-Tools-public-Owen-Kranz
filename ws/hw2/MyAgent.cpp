#include "MyAgent.h"
#include "MyObstacle.h"
#include "AMPCore.h"

void PointAgent::move(double dt){
    // moves the robot with x += heading*dt

    x += heading*dt;
}

void PointAgent::rotate(double dtheta){
    // moves the robot's heading by some dtheta (rhrule about +z)

    Eigen::Matrix2d rot_mat;

    rot_mat << cos(dtheta * M_PI/180.0), -sin(dtheta * M_PI/180.0),
                sin(dtheta * M_PI/180.0), cos(dtheta * M_PI/180.0);

    heading = rot_mat * heading;
    heading.normalize(); //gotta make sure to normalize
}

void PointAgent::rotateToCircumnavigateRH(const std::vector<MyObstacle> obstacles, const double dtheta, const double epsilon){
    /* rotates withe the following schema.

    if heading*epsilon is in collision and -heading*epsilon is not:
        rotate dtheta
    if -heading*epsilon is in collision and heading*epsilon is not:
        rotate -dtheta
    if -heading*epsilon is not collision and heading*epsilon is also not
        and right_hand * epsilon not in collision

        flip heading
    once headings are free and rh is not, break and can move
    */

    double total_rotation = 0.0;

    while (total_rotation < 1000){ // gives it 1000 degrees to converge lol
        bool heading_free = true;
        bool backheading_free = true;
        bool rh_free = true;
        bool lh_free = true;

        for (const auto& ob : obstacles){
            Eigen::Vector2d heading_vec = x + epsilon*heading;
            Eigen::Vector2d backheading_vec = x - epsilon*heading;
            Eigen::Vector2d rh_vec = x + epsilon * Eigen::Vector2d(heading[1], -heading[0]);
            Eigen::Vector2d lh_vec = x + epsilon * Eigen::Vector2d(-heading[1], heading[0]);
            
            if (ob.collisionCheck(heading_vec)){
                heading_free = false;
            };
            if (ob.collisionCheck(backheading_vec)){
                backheading_free = false;
            };
            if (ob.collisionCheck(rh_vec)){
                rh_free = false;
            };
            if (ob.collisionCheck(lh_vec)){
                lh_free = false;
            };
        };
        
        //Check our exit condition.
        if (heading_free && backheading_free && lh_free && !rh_free){
            break;
        }

        int num_free = heading_free + backheading_free + rh_free + lh_free;
        
        if (num_free == 0){
            std::cout << "All vectors are not free. Inside obstacle" << std::endl;
        }

        // std::cout << heading_free << " " << backheading_free << " " << rh_free << total_rotation << std::endl;

        //Conditions when two are not free, gotta move by dtheta
        // goal is to move to the closest solution where only one arm is in. 
        // this is tricky to determine 'closest' so we just rotate dtheta
        else if (num_free == 2){
            rotate(dtheta);
            total_rotation += dtheta;
        }

        // CORNER CONDITIONS or conditions where only one is free (can solve more easily)
        // special cases where only one arm is free, or all are free
        else if (num_free == 4){
            // this is the gross corner condition. we are gonna rotate 45, and evaluate which arm is in the corner
            rotate(45.0);
            total_rotation += 45;
        }
        // if num_free = 1, solve for exact rotation to go right way
        else if (heading_free && backheading_free && rh_free && !lh_free){
            // only left hand is in corner, flip
            rotate(180);
            total_rotation += 180;
        }
        else if (!heading_free && backheading_free && rh_free && lh_free){
            // only heading is in corner, rot 90
            rotate(90);
            total_rotation += 90;
        }
        else if (heading_free && !backheading_free && rh_free && lh_free){
            // only backheading is in corner, rot -90
            rotate(-90);
            total_rotation += 90;
        }
    };

    if (total_rotation > 1000){ //prevent not converging
        std::cout << "ROTATE TO CIRCUMNAV DID NOT CONVERGE " << std::endl;

    }


};

