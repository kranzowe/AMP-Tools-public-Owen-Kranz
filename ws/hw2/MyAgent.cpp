#include "MyAgent.h"
#include "MyObstacle.h"
#include "AMPCore.h"

bool PointAgent::move(const std::vector<MyObstacle> obstacles, const double dt, const double epsilon){
    // moves the robot with x += heading*dt
    Eigen::Vector2d x_test;
    bool collision = false;

    x_test = x + heading*dt;

    for (const auto& ob : obstacles){
        if (ob.collisionCheck(x_test)){
            collision = true;
        };
    }
    
    if (!collision){
        x += heading*dt;
        return collision;
    }
    else{
        //found a collision. need to move within epsilon/3 of target
        Eigen::Vector2d x_last = x;
        Eigen::Vector2d x_next = x;

        // gonna do this iteratively
        //move eps/3 closer each time just until you make contact
        while (true){
            bool contact = false;
            x_next += heading*(epsilon/2.0);
            for (const auto& ob : obstacles){
                if (ob.collisionCheck(x_next)){
                    contact = true;
                    break;
                }
            }
            if (contact){
                break; //breaks the while loop before x last is updated
            }
            
            x_last = x_next;
        };
        x = x_last;
        return collision;
    }
};

void PointAgent::rotate(double dtheta){
    // moves the robot's heading by some dtheta (rhrule about +z)

    Eigen::Matrix2d rot_mat;

    rot_mat << cos(dtheta * M_PI/180.0), -sin(dtheta * M_PI/180.0),
                sin(dtheta * M_PI/180.0), cos(dtheta * M_PI/180.0);

    heading = rot_mat * heading;
    heading.normalize(); //gotta make sure to normalize
}

bool PointAgent::rotateToCircumnavigateRH(const std::vector<MyObstacle> obstacles, const double dtheta, const double epsilon){
    /* rotates withe the following schema.

    draw a circle of radius epsilon going CW from heading
    with dtheta as the step

    compute which of these are in collision and which arent. 

    searching CW, choose the heading that is free when the next one isn't
    */

    const int num_vecs = static_cast<int>(360.0 / dtheta) + 1;
    const int num_points_on_vec = 10;
    std::vector<std::vector<bool>> collision_flags(num_vecs, std::vector<bool>(num_points_on_vec, false));
    Eigen::Vector2d base_heading = heading; // set so we can have something to rotate
    double ray_length = 2*epsilon;

    bool no_collision = true;

    // rotate base heading by dtheta each time and check for collisions

    for (int i = 0; i < num_vecs; ++i) {
        
        double theta = i*-dtheta;
        //define the dtheta rotation matrix
        Eigen::Matrix2d rot_mat;
        rot_mat << cos(theta * M_PI/180.0), -sin(theta * M_PI/180.0),
                sin(theta * M_PI/180.0), cos(theta * M_PI/180.0);
        
        Eigen::Vector2d new_heading = rot_mat * base_heading;
        
        for (int j = 0; j < num_points_on_vec; ++j) {

            double frac = static_cast<double>(j) / (num_points_on_vec - 1);
            Eigen::Vector2d sample_point = x + (frac * ray_length) * new_heading;

            //check collision for this direction
            for (const auto& ob : obstacles) {
                if (ob.collisionCheck(sample_point)) {
                    collision_flags[i][j] = true;
                    if (no_collision) {
                        no_collision = false;
                    }

                    break;
                }
            }
        }
    }
    if (no_collision){ // if we found no collision :/
        std::cout << "NO COLLISION :( \n";
        return false;
    }
    double closest_distance;
    double theta_collision;
    // this loops searchs outwards, finding the first distance that results in collision. 
    // thats our distance to obstacle

    bool done = false;
    for (int j = 0; j < num_points_on_vec && !done; ++j) {
        for (int i = 0; i < num_vecs - 1; ++i) {

            if (collision_flags[i][j]) {
                // this is the closest distance that found collision
                double frac = static_cast<double>(j) / (num_points_on_vec - 1);
                closest_distance = frac * ray_length;
                done = true;
                break; // double break??
            }
        }
    }
    
    // this loops searches for the vector, that at its longest point, collides
    // thats our theta to collision
    for (int i = 0; i < num_vecs - 1; ++i) {
        if (collision_flags[i][num_points_on_vec-1]) { 

            theta_collision = i*-dtheta;

            break;
        }
    }

    // now, compute how much to rotate. 
    // we want to move proportionally towards the theta collision
    // we want to move with respect to the derivative of the closest distance
    double kp_theta = 0.2;
    double kp_dist = -20000.00;
    double kd_dist = -10000.00;
    double drotation;
    if (previous_dist == -1){
        drotation = kp_theta * theta_collision;
        previous_dist = closest_distance;
    }else{
        drotation = (kp_theta * theta_collision) + (kp_dist*(closest_distance - epsilon)) + kd_dist*(closest_distance - previous_dist);
        previous_dist = closest_distance;
    }
    
    rotate(drotation);
    return true;
};

void PointAgent::pointAtGoal(const Eigen::Vector2d q_goal){
    //updates heading to be a unit vector in direction of goal
    Eigen::Vector2d vec_to_goal = q_goal - x;
    heading = vec_to_goal.normalized();

};

