#include "MyBugAlgorithm.h"
#include "MyObstacle.h"
#include "MyAgent.h"

amp::Path2D MyBugAlgorithm::plan(const amp::Problem2D& problem) {

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
    double min_dist;

    Eigen::Vector2d last_position = agent.x;
    int stuck_points = 0;
    const int max_stuck = 100;
    
    path.waypoints.push_back(agent.x);

    while (j < 1e4){
        
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
        } 
        else {// uh oh! we smacked an object!!!
            path.waypoints.push_back(agent.x); // push all collided points
            bool converged;
            agent.heading *= -1; // explain this todo
            if (qh_i_index == -1){ //define qhi for the first time
                qh_i_index = path.waypoints.size() - 1;
                min_dist = (problem.q_goal - path.waypoints[qh_i_index]).norm();
            }

            converged = agent.rotateToCircumnavigateRH(my_obstacles, dtheta, epsilon);
            if (!converged){
                return path;
            }
            (void)agent.move(my_obstacles, dt, epsilon);
            
            //set qli if new minimum found
            if ((problem.q_goal - agent.x).norm() <= min_dist) {
                ql_i_index = path.waypoints.size(); // no minus 1 here because we havent pushed that point yet
                min_dist = (problem.q_goal - agent.x).norm();
            }

            // detect if we made it back to qhi
            if ((path.waypoints[qh_i_index] - agent.x).norm() <= 3*epsilon &&
                path.waypoints.size() >= qh_i_index + 20) { //this is a guard to make sure we have moved far enough past it
                //looks like we made it back
                std::cout << "successful circumnavigation \n";
                //compute which path is shortest to qli
                // option 1 is taking qhi to qli again.
                // option 2 is reversing and going x to qli
                
                //my robot moves pretty linearly, so we are gonna assume the
                // fewer points between the two is the shortest path
                if ((j - ql_i_index) < (ql_i_index - qh_i_index)){
                    //option 2 is shorter
                    // N.B. I needed some AI help to figure out how to reverse
                    std::vector<Eigen::Vector2d> temp(
                        path.waypoints.begin() + ql_i_index,
                        path.waypoints.end()
                    );
                    std::reverse(temp.begin(), temp.end()); // gotta flip to turn around
                    path.waypoints.insert(path.waypoints.end(), temp.begin(), temp.end());
                } else {
                    // option 1 is shorter
                    path.waypoints.insert(
                        path.waypoints.end(),
                        path.waypoints.begin() + qh_i_index,
                        path.waypoints.begin() + ql_i_index);
                }
                agent.x = path.waypoints[ql_i_index];
                agent.pointAtGoal(problem.q_goal);
                collided = false;
                agent.previous_dist = -1;

                qh_i_index = -1; //set back to sentinel value
                ql_i_index = -1;

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
