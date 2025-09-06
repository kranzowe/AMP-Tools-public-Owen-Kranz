#include "MyBugAlgorithm.h"
#include "MyObstacle.h"

// Implement your methods in the `.cpp` file, for example:
amp::Path2D MyBugAlgorithm::plan(const amp::Problem2D& problem) {

    // Your algorithm solves the problem and generates a path. Here is a hard-coded to path for now...
    
    // initialize, from the problem, all the obstacles
    std::vector<MyObstacle> my_obstacles;
    for (const auto& obstacle : problem.obstacles) {
        MyObstacle my_ob;
        my_obstacles.push_back(my_ob.defineWithPoints(obstacle.verticesCCW()))
    };
    
    
    
    amp::Path2D path;
    path.waypoints.push_back(problem.q_init);
    path.waypoints.push_back(Eigen::Vector2d(1.0, 5.0));
    path.waypoints.push_back(Eigen::Vector2d(3.0, 9.0));
    path.waypoints.push_back(problem.q_goal);

    return path;
}
