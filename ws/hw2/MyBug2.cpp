#include "MyBug2.h"
#include "MyObstacle.h"
#include "MyAgent.h"

double MyBug2::DistanceToMLine(Eigen::Vector2d q_start, Eigen::Vector2d q_end, Eigen::Vector2d q){
    // a = (y2-y1)
    // b = -(x2-x1)
    // c = x2*y1 - x1*y2

    double a = q_end[1] - q_start[1];
    double b = -(q_end[0] - q_start[0]);
    double c = (q_end[0]*q_start[1]) - (q_start[0]*q_end[1]);
    
    // so by my convention, all points to the left of the vector 
    // point a to point b are negative. therefore I define polygons cCW using vertices
    return abs(a*q[0] + b*q[1] + c)/sqrt((a*a) + (b*b));
}


amp::Path2D MyBug2::plan(const amp::Problem2D& problem) {

    // Your algorithm solves the problem and generates a path. Here is a hard-coded to path for now...
    
    // initialize, from the problem, all the obstacles
    std::vector<MyObstacle> my_obstacles;
    for (const auto& obstacle : problem.obstacles) {
        MyObstacle my_ob;
        my_ob.defineWithPoints(obstacle.verticesCCW());
        my_obstacles.push_back(my_ob);
    };

    // initialize the agent
    PointAgent agent;
    agent.x = problem.q_init;

    // init the path
    amp::Path2D path;

    // Actual algorithm

    int j = 0; //counts iterations
    int qh_i_index = -1; //will save the index of the point on the path at qhi
    int ql_i_index = -1; //ditto but for qli
    bool collided = false; // lets us know if we are in obstacle logic or not
    double dist_qh;

    Eigen::Vector2d last_position = agent.x;
    int stuck_points = 0;
    const int max_stuck = 100;
    
    path.waypoints.push_back(agent.x);

    while (j < 16e5){
        
        // at start, evaluate if we hit goal! if not pushback and continue
        if ((problem.q_goal - agent.x).norm() < epsilon) {
            path.waypoints.push_back(problem.q_goal);
            std::cout << "made it to goal \n";

            break;
        }
            
        if (!collided){
            agent.pointAtGoal(problem.q_goal);
            collided = agent.move(my_obstacles, dt, epsilon);
            //no need to push back useless points
            if (collided){
                agent.rotate(90); // this is because the controller wont update
                                // if the heading is right at the target. so niavely rotate 90 right
            }
        } 
        else {// uh oh! we smacked an object!!!
            path.waypoints.push_back(agent.x); // push all collided points
            bool converged;
            if (qh_i_index == -1){ //define qhi for the first time
                qh_i_index = path.waypoints.size() - 1;
                dist_qh = (problem.q_goal - path.waypoints[qh_i_index]).norm();
            }

            converged = agent.rotateToCircumnavigateRH(my_obstacles, dtheta, epsilon);
            if (!converged){
                return path;
            }
            bool circumnav_collided = agent.move(my_obstacles, dt, epsilon);
            if (circumnav_collided){
                agent.heading *=-1;
            }

            // detect if we made it back to m_line
            double dist_mline = DistanceToMLine(problem.q_init, problem.q_goal, agent.x);
            double dist_togoal = (problem.q_goal - agent.x).norm();

            PointAgent test_agent = agent; // copies agent to check if path to goal is blocked
            test_agent.pointAtGoal(problem.q_goal);
            bool collision_test = test_agent.move(my_obstacles, dt, epsilon);
            
            if (dist_mline <= 3*epsilon && 
                path.waypoints.size() >= qh_i_index + 200 &&
                dist_togoal < dist_qh &&
                !collision_test) {
               
                agent.pointAtGoal(problem.q_goal);
                collided = false;
                agent.previous_dist = -1;

                qh_i_index = -1; //set back to sentinel value

            }
        }

        if ((agent.x - last_position).norm() < epsilon) {
            stuck_points++;
        } else {
            stuck_points = 0;
            last_position = agent.x;
        }
        if (stuck_points > max_stuck) {
            std::cout << "agent is stuck :( \n";
            break;
        }
        
        j += 1;
    };

    return path;
}
