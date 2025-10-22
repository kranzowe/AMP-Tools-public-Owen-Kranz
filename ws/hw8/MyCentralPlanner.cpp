#include "MyMultiAgentPlanners.h"
#include "MySamplingBasedPlanners.h" 




amp::MultiAgentPath2D MyCentralPlanner::plan(const amp::MultiAgentProblem2D& problem) {
    
    int q_dim = problem.agent_properties[0].q_init.size();
    int num_bots = problem.agent_properties.size();
    LOG("q dim" << q_dim);

    Eigen::VectorXd init_state(problem.numAgents() * q_dim);
    LOG("length of qinit is " << problem.numAgents() * q_dim);
    Eigen::VectorXd goal_state(problem.numAgents() * q_dim);

    std::vector<MyObstacle> my_disc_obstacles;

    int i = 0;
    for (const amp::CircularAgentProperties& property : problem.agent_properties){
        
        init_state[i] = property.q_init[0];
        init_state[i+1] = property.q_init[1];

        goal_state[i] = property.q_goal[0];
        goal_state[i+1] = property.q_goal[1];

        MyObstacle my_ob;
        my_ob.defineWithDiscRadius(property.radius);
        my_disc_obstacles.push_back(my_ob);

        i += 2;
    }


    //create obs, no collision checker here
    std::vector<MyObstacle> my_obstacles;
    for (const auto& obstacle : problem.obstacles) {
        MyObstacle my_ob;
        my_ob.defineWithPoints(obstacle.verticesCCW());
        my_obstacles.push_back(my_ob);
    }
    
    std::vector<Eigen::VectorXd> env_vertices;
    Eigen::VectorXd bottom_left(q_dim*num_bots), top_right(q_dim*num_bots);
    for (int i = 0; i < num_bots; i++){
        bottom_left[2*i] = problem.x_min;
        bottom_left[2*i+1] = problem.y_min;
        top_right[2*i] = problem.x_max;
        top_right[2*i+1] = problem.y_max;
    }
    env_vertices.push_back(bottom_left);
    env_vertices.push_back(top_right);
    // LOG("TOP RIGHT" << top_right);

    //ollision checker
    MyPointAndDiscCollisionChecker col_check(my_obstacles, my_disc_obstacles, env_vertices);

    L2Distance metric;

    //GenericRRT RRT(int max_iterations = 5000, double step_size = 0.5, double goal_bias = 0.1);
    GenericRRT RRT(5000, 0.5, 0.1);

    amp::Path path_rrt;

    path_rrt = RRT.plan(init_state, 
             goal_state, 
             col_check,
             metric);

    LOG("EX WP" << path_rrt.waypoints[0]);
    LOG("len sol" << path_rrt.waypoints.size());

    amp::MultiAgentPath2D path;
    int k = 0;
    for (const amp::CircularAgentProperties& agent : problem.agent_properties) {
        amp::Path2D agent_path;
        agent_path.waypoints.push_back(agent.q_init);
        for (auto waypoint : path_rrt.waypoints){
            Eigen::Vector2d point{waypoint[2*k], waypoint[2*k + 1]};
            agent_path.waypoints.push_back(point);
            //LOG("ADDED POINT " << point);
        }
        agent_path.waypoints.push_back(agent.q_goal);
        path.agent_paths.push_back(agent_path);
        
        k++;
    }

    return path;
}

amp::MultiAgentPath2D MyDecentralPlanner::plan(const amp::MultiAgentProblem2D& problem) {
    amp::MultiAgentPath2D path;
    for (const amp::CircularAgentProperties& agent : problem.agent_properties) {
        amp::Path2D agent_path;
        agent_path.waypoints = {agent.q_init, agent.q_goal};
        path.agent_paths.push_back(agent_path);
    }
    return path;
}