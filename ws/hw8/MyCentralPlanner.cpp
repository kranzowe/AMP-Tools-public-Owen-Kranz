#include "MyMultiAgentPlanners.h"
#include "MySamplingBasedPlanners.h" 

// a new collision check for the robo and obstacles
MyPointAndDiscCollisionChecker::MyPointAndDiscCollisionChecker(const std::vector<MyObstacle>& obstacles, 
                                                 const std::vector<MyObstacle>& moving_circular_obstacles,
                                                 const std::vector<Eigen::VectorXd>& env_vertices)
    : MyPointCollisionChecker(obstacles, env_vertices),
     m_moving_circular_obstacles(moving_circular_obstacles), {
}

// for all obs checks for collsiiosn
bool MyPointAndDiscCollisionChecker::inCollision(const Eigen::VectorXd& config) const {
    
    std::vector<Eigen::Vector2d> disc_centers;
    std::vector<Eigen::Vector2d> points_to_check;

    // this 2 is becaseu i know the config of one robot is 2. kinda gross
    for (size_t i = 0; 2*i < config.size(); i++){
        Eigen::Vector2d point{config[2*i], config[(2*i) + 1]};
        disc_centers.push_back(point);

        for (double theta = 0; theta < 2*M_PI; theta += M_PI/8){
            //sample 16 points on each robot edge.
            Eigen::Vector2d outer_point{point[0] + (cos(theta)*m_moving_circular_obstacles[i].moving_circular_radius.radius*1.1), 
                                        point[1] + (sin(theta)*m_moving_circular_obstacles[i].moving_circular_radius.radius*1.1)};

            points_to_check.push_back(outer_point);

            
        }
    }

    for (auto point : points_to_check){

        // check for collisions with obstacels
        for (const auto& obstacle : m_obstacles) {
            if (obstacle.collisionCheck(point)) {
                return true;
            }
        }
        // check for collisions with other robits
        size_t j = 0;
        for (size_t j = 0; j < m_moving_circular_obstacles.size(); j++) {
            if (m_moving_circular_obstacles[j].collisionCheckTranslated(point, disc_centers[j])) {
                return true;
            }
        }
    }

    return false;
}

bool MyPointAndDiscCollisionChecker::edgeInCollision(const Eigen::VectorXd& config1, const Eigen::VectorXd& config2) const {
    // checks along ray connecting two configs
    uint16_t divs = 50;
    for (size_t i = 0; i <= divs; i++) {
        double t = static_cast<double>(i) / divs;
        Eigen::VectorXd interpolated_config = (1.0 - t) * config1 + t * config2;
        if (inCollision(interpolated_config)){
            return true;
        }
    }
    return false;
}



amp::MultiAgentPath2D MyCentralPlanner::plan(const amp::MultiAgentProblem2D& problem) {
    amp::MultiAgentPath2D path;

    int q_dim = problem.agent_properties[0].q_init.size();
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
    
    // some AI help here cuz vectors are confuzing
    std::vector<Eigen::VectorXd> env_vertices;
    Eigen::VectorXd bottom_left(2), top_right(2);
    bottom_left << problem.x_min, problem.y_min;
    top_right << problem.x_max, problem.y_max;
    env_vertices.push_back(bottom_left);
    env_vertices.push_back(top_right);

    //ollision checker
    MyPointAndDiscCollisionChecker col_check(my_obstacles, my_disc_obstacles, env_vertices);

    L2Distance metric;

    GenericRRT RRT(int max_iterations = 5000, double step_size = 0.5, double goal_bias = 0.1);

    amp::Path path_rrt;

    path_rrt = RRT.plan(init_state, 
             goal_state, 
             col_check,
            metric);

    for (const amp::CircularAgentProperties& agent : problem.agent_properties) {
        amp::Path2D agent_path;
        agent_path.waypoints = {agent.q_init, agent.q_goal};
        path.agent_paths.push_back(agent_path);
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