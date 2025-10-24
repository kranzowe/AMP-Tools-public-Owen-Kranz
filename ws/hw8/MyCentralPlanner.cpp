#include "MyMultiAgentPlanners.h"
#include "MySamplingBasedPlanners.h" 
#include "AMPCore.h"
// #include "hw/HW8.h"

int MyCentralPlanner::getLastTreeSize() const {
    return m_rrt.getLastTreeSize();
}


amp::MultiAgentPath2D MyCentralPlanner::plan(const amp::MultiAgentProblem2D& problem) {
    
    int q_dim = problem.agent_properties[0].q_init.size();
    int num_bots = problem.agent_properties.size();
    //LOG("q dim" << q_dim);

    Eigen::VectorXd init_state(problem.numAgents() * q_dim);
    //LOG("length of qinit is " << problem.numAgents() * q_dim);
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
    GenericRRT RRT(7500, 0.5, 0.05);

    amp::Path path_rrt;

    
    path_rrt = RRT.plan(init_state, 
        goal_state, 
        col_check,
        metric);
        
    m_rrt = RRT;
    // LOG("EX WP" << path_rrt.waypoints[0]);
    // LOG("len sol" << path_rrt.waypoints.size());

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

int MyDecentralPlanner::getLastTreeSize() const {
    
    int total_nodes = 0;
    for (GenericRRT rrt : m_all_rrt){
        total_nodes += rrt.getLastTreeSize();
    }
    return total_nodes;
}



amp::MultiAgentPath2D MyDecentralPlanner::plan(const amp::MultiAgentProblem2D& problem) {

    m_all_rrt.clear();
    amp::MultiAgentPath2D final_path;

    
    int q_dim = problem.agent_properties[0].q_init.size();
    int num_bots = problem.agent_properties.size();

    for (int attempts = 0; attempts < 10; attempts ++){
        // priority indexes randomized
        std::vector<int> priorities;
        for (int i = 0; i < num_bots; i++){
            priorities.push_back(i);
        }
        //shuffled dem
        std::random_device rd;
        std::mt19937 gen(rd());
        std::shuffle(priorities.begin(), priorities.end(), gen);

        // DEBUG: Validate priorities
        for (size_t i = 0; i < priorities.size(); i++) {
            if (priorities[i] >= num_bots || priorities[i] < 0) {
                LOG("ERROR: Invalid priority index " << priorities[i] << " at position " << i);
                continue; // Skip this attempt
            }
        }

        // create env obs at the start
        std::vector<MyObstacle> my_obstacles;
        for (const auto& obstacle : problem.obstacles) {
            MyObstacle my_ob;
            my_ob.defineWithPoints(obstacle.verticesCCW());
            my_obstacles.push_back(my_ob);
        }
        
        std::vector<amp::Path> all_paths;

        for (size_t bot_num = 0; bot_num < num_bots; bot_num++){

            Eigen::VectorXd init_state(q_dim);
            //LOG("length of qinit is " << problem.numAgents() * q_dim);
            Eigen::VectorXd goal_state(q_dim);
            
            
            amp::CircularAgentProperties property = problem.agent_properties[priorities[bot_num]];
            init_state[0] = property.q_init[0];
            init_state[1] = property.q_init[1];
            
            goal_state[0] = property.q_goal[0];
            goal_state[1] = property.q_goal[1];
            
            // LOG("q init" << property.q_init);
            
            std::vector<MyObstacle> my_disc_obstacles;
            for (int i = 0; i < all_paths.size(); i++){
                MyObstacle my_ob;
                amp::CircularAgentProperties robo_property = problem.agent_properties[priorities[i]];
                my_ob.defineWithDiscRadius(robo_property.radius);
                my_disc_obstacles.push_back(my_ob);
            }
            // gotta define this robo as an ob
            MyObstacle my_ob;
            amp::CircularAgentProperties robo_property = problem.agent_properties[priorities[bot_num]];
            my_ob.defineWithDiscRadius(robo_property.radius);
            my_disc_obstacles.push_back(my_ob);
            
            std::vector<Eigen::VectorXd> env_vertices;
            Eigen::VectorXd bottom_left(q_dim), top_right(q_dim);
            for (int i = 0; i < num_bots; i++){
                bottom_left[0] = problem.x_min;
                bottom_left[1] = problem.y_min;
                top_right[0] = problem.x_max;
                top_right[1] = problem.y_max;
            }
            env_vertices.push_back(bottom_left);
            env_vertices.push_back(top_right);
            
            // LOG("TOP RIGHT" << top_right);
            
            //ollision checker
            // LOG("Creating collision checker with "<< my_obstacles.size() <<" , " << my_disc_obstacles.size());
            MyPointAndDiscCollisionChecker col_check(my_obstacles, my_disc_obstacles, env_vertices);
            
            // std::cin.get();
            L2Distance metric;

            //GenericRRT RRT(int max_iterations = 5000, double step_size = 0.5, double goal_bias = 0.1);
            GenericRRT RRT(17500, 0.5, 0.05);

            amp::Path path_rrt;

            path_rrt = RRT.plan_with_moving_obs(init_state, 
                goal_state, 
                col_check,
                metric,
                all_paths);
                
            m_all_rrt.push_back(RRT);


            all_paths.push_back(path_rrt);
        }
    
        // LOG("EX WP" << path_rrt.waypoints[0]);
        // LOG("len sol" << path_rrt.waypoints.size());

        amp::MultiAgentPath2D path;
        path.agent_paths.resize(num_bots);
        int k = 0;
        for (size_t bot_num = 0; bot_num < num_bots; bot_num++) {
            const amp::CircularAgentProperties& agent = problem.agent_properties[priorities[bot_num]];
            amp::Path2D agent_path;
            agent_path.waypoints.push_back(agent.q_init);
            if (k == all_paths.size()){     
                LOG("bad k");
                break;
            }
            for (auto point : all_paths[k].waypoints){
                //Eigen::Vector2d point{waypoint[2*k], waypoint[2*k + 1]};
                agent_path.waypoints.push_back(point);
                //LOG("ADDED POINT " << point);
            }
            agent_path.waypoints.push_back(agent.q_goal);
            path.agent_paths[priorities[bot_num]] = agent_path;
            
            k++;
        }

        // check if works
        std::vector<std::vector<Eigen::Vector2d>> collision_states;
        bool isValid = amp::HW8::check(path, problem, collision_states);
        final_path = path;
        
        if(isValid){
            break;
        }

    }

    return final_path;
}