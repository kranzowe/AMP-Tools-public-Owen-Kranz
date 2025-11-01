#include "MyKinoRRT.h"
#include "MySamplingBasedPlanners.h"

namespace amp {
    std::vector<std::vector<Eigen::VectorXd>> points_to_check;
    std::vector<Eigen::VectorXd> sub_states;
}

Eigen::Matrix<double,2,2> MySingleIntegrator::A(Eigen::VectorXd state){
    Eigen::Matrix<double, 2,2> A_mat;
    A_mat << 0, 0, 0, 0;
    return A_mat;
}

Eigen::Matrix<double,2,2> MySingleIntegrator::B(Eigen::VectorXd state){
    Eigen::Matrix<double, 2,2> B_mat;
    B_mat << 1, 0, 0, 1;
    return B_mat;
}

void MySingleIntegrator::propagate(Eigen::VectorXd& state, Eigen::VectorXd& control, double dt) {
    auto getA = [this](Eigen::VectorXd s) { return this->A(s); };
    auto getB = [this](Eigen::VectorXd s) { return this->B(s); };
    
    RK4Integrate(state, control, dt, getA, getB, true, 0.001);
}

Eigen::Matrix<double,3,3> MyFirstOrderUnicycle::A(Eigen::VectorXd state){
    Eigen::Matrix<double, 3,3> A_mat;
    A_mat << 0, 0, 0,
             0, 0, 0,
             0, 0, 0;
    return A_mat;
}

Eigen::Matrix<double,3,2> MyFirstOrderUnicycle::B(Eigen::VectorXd state){
    Eigen::Matrix<double, 3,2> B_mat;
    B_mat << 0.25 * cos(state[2]), 0,
             0.25 * sin(state[2]), 0,
             0, 1;
    return B_mat;
}

void MyFirstOrderUnicycle::propagate(Eigen::VectorXd& state, Eigen::VectorXd& control, double dt) {
    auto getA = [this](Eigen::VectorXd s) { return this->A(s); };
    auto getB = [this](Eigen::VectorXd s) { return this->B(s); };
    
    RK4Integrate(state, control, dt, getA, getB, true, 0.001);
}

MyKinoRRT::MyKinoRRT(double dt, int num_control_samples, int max_iterations, double stop_radius)
    : m_dt(dt), m_num_control_samples(num_control_samples), m_max_iterations(max_iterations), m_stop_radius(stop_radius) {}

amp::KinoPath MyKinoRRT::plan(const amp::KinodynamicProblem2D& problem, amp::DynamicAgent& agent) {
    
    // Setup collision checker and metric
    std::vector<MyObstacle> my_obstacles;
    for (const auto& obstacle : problem.obstacles) {
        MyObstacle my_ob;
        my_ob.defineWithPoints(obstacle.verticesCCW());
        my_obstacles.push_back(my_ob);
    }
    
    std::vector<Eigen::VectorXd> env_vertices;
    Eigen::VectorXd bottom_left(2), top_right(2);
    bottom_left << problem.x_min, problem.y_min;
    top_right << problem.x_max, problem.y_max;
    env_vertices.push_back(bottom_left);
    env_vertices.push_back(top_right);

    MyPointCollisionChecker collision_checker(my_obstacles, env_vertices);
    L2Distance metric;
    
    // FIX: Use problem.q_init.size() to get correct dimension
    int dim = problem.q_init.size();
    Eigen::VectorXd init_state = problem.q_init;  // ← Just copy directly!
    
    // Goal is a region (box), so we use the center for sampling bias
    Eigen::VectorXd goal_state(dim);
    for (int i = 0; i < dim; i++) {
        goal_state[i] = (problem.q_goal[i].first + problem.q_goal[i].second) / 2.0;
    }

    // Bounds for generating random samples 
    Eigen::VectorXd min_vector = collision_checker.lowerBounds();
    Eigen::VectorXd max_vector = collision_checker.upperBounds();
    
    std::shared_ptr<amp::Graph<double>> graphPtr = std::make_shared<amp::Graph<double>>();
    
    // Tree data structures
    std::vector<Eigen::VectorXd> points;
    std::vector<Eigen::VectorXd> controls;
    std::vector<double> durations;
    std::vector<std::tuple<amp::Node, amp::Node, double>> edges = {};

    points.insert(points.begin(), init_state);
    
    int samples = 0;

    static std::random_device rd;
    static std::mt19937 gen(rd());

    amp::Node goal_node_id = -1;  
    bool found_goal = false;

    // Main RRT loop
    while (samples < m_max_iterations){
        Eigen::VectorXd random_point(dim);
        
        // Goal-biased sampling (8% chance)
        std::uniform_real_distribution<double> uniform_dist(0.0, 1.0);
        double rando_0to1 = uniform_dist(gen);
        if (rando_0to1 < 0.08){
            random_point = goal_state;
        } else {
            // Uniform random sampling
            for (int i = 0; i < dim; ++i) {
                std::uniform_real_distribution<double> dist(min_vector[i], max_vector[i]);
                random_point[i] = dist(gen);
            }
        }
        
        // Find nearest neighbor in tree
        double min_distance = 1e8;
        Eigen::VectorXd nearest_point(dim);
        amp::Node nearest_node_id;

        for (int i = 0; i < points.size(); i++) {
            double dist = metric.distance(random_point, points[i]);
            if (dist < min_distance){
                nearest_point = points[i];
                nearest_node_id = i;
                min_distance = dist;
            }
        }

        // Try multiple random controls and keep the best one
        Eigen::VectorXd proposed_point(dim);
        Eigen::VectorXd proposed_control(problem.u_bounds.size());  // ← Use u_bounds size
        double proposed_duration;
        double min_dist = 1e8;
        
        for (int i = 0; i < m_num_control_samples; i++) {
            // Sample random control within bounds
            Eigen::VectorXd control(problem.u_bounds.size());
            for (int c = 0; c < problem.u_bounds.size(); c++) {
                std::uniform_real_distribution<double> control_dist(
                    problem.u_bounds[c].first, 
                    problem.u_bounds[c].second
                );
                control[c] = control_dist(gen);
            }
            
            Eigen::VectorXd test_point = nearest_point;

            std::uniform_real_distribution<double> duration_dist(
                problem.dt_bounds.first, 
                problem.dt_bounds.second
            );
            double sampled_dt = duration_dist(gen);
            
            // Propagate for the full time step
            agent.propagate(test_point, control, sampled_dt);
            
            // Check collision at the final state
            bool trajectory_valid = true;
            if (collision_checker.inCollision(test_point)) {
                trajectory_valid = false;
            }

            // add points in points to check to collision checking
            // add points in points to check to collision checking
            if (!amp::points_to_check.empty()) {
                for (size_t step = 0; step < amp::points_to_check.size(); step++) {
                    // Check each corner point at this step
                    for (const auto& corner_point : amp::points_to_check[step]) {
                        if (collision_checker.inCollision(corner_point)) {
                            trajectory_valid = false;
                            break;
                        }
                    }
                    if (!trajectory_valid) break;
                    
                    // Also check the center point (sub_state) if available
                    if (step < amp::sub_states.size()) {
                        if (collision_checker.inCollision(amp::sub_states[step])) {
                            trajectory_valid = false;
                            break;
                        }
                    }
                }
            }
            
            // Check bounds violations
            for (int b = 0; b < test_point.rows(); b++) {
                if (test_point[b] < problem.q_bounds[b].first || 
                    test_point[b] > problem.q_bounds[b].second) {
                    trajectory_valid = false;
                    break;
                }
            }
            
            // Keep the control that gets closest to the random sample without collision
            if (trajectory_valid) {
                double dist = metric.distance(test_point, random_point);
                if (dist < min_dist) {
                    proposed_point = test_point;
                    proposed_control = control;
                    proposed_duration = sampled_dt;
                    min_dist = dist;
                }
            }
        }

        // Add the best proposal to the tree if found
        if (min_dist < 1e8) {
            // Double-check bounds before adding (safety net)
            bool within_bounds = true;
            for(int i = 0; i < proposed_point.rows(); i++){
                if(proposed_point[i] < problem.q_bounds[i].first || 
                   proposed_point[i] > problem.q_bounds[i].second){
                    within_bounds = false;
                    break;
                }
            }
            
            if (!within_bounds) {
                samples++;
                continue;
            }
            
            points.push_back(proposed_point);
            controls.push_back(proposed_control);
            durations.push_back(proposed_duration);
            amp::Node proposed_node_id = points.size() - 1;

            double edge_weight = metric.distance(nearest_point, proposed_point);
            edges.push_back({nearest_node_id, proposed_node_id, edge_weight}); 

            //std::cout << "Added point " << proposed_node_id << ": (" << proposed_point[0] << ", " << proposed_point[1] << ")" << std::endl;

            // Check if we've reached the goal region
            bool at_goal = true;
            for(int i = 0; i < proposed_point.rows(); i++){
                if(i == 2 && dim >= 3){
                    continue; // Skip angle checking for now
                }
                
                if(problem.q_goal[i].first > proposed_point[i] || 
                   problem.q_goal[i].second < proposed_point[i]){
                    at_goal = false;
                    break;
                }
            }

            if(at_goal){
                std::cout << "GOAL REACHED! Setting goal_node_id = " << proposed_node_id << std::endl;
                std::cout << "Goal point: (" << proposed_point[0] << ", " << proposed_point[1] << ")" << std::endl;
                goal_node_id = proposed_node_id;
                found_goal = true;
                break;
            }
        }
        samples++;
    }

    // DEBUG: Print all points
    // std::cout << "\n=== ALL POINTS IN TREE ===" << std::endl;
    // for (size_t i = 0; i < points.size(); i++) {
    //     std::cout << "Point " << i << ": (" << points[i][0] << ", " << points[i][1] << ")" << std::endl;
    // }

    if (!found_goal) {
        std::cout << "WARNING: Goal not reached!" << std::endl;
        amp::KinoPath empty_path;
        empty_path.valid = false;
        return empty_path;
    }

    // Build path by tracing back from goal
    std::map<amp::Node, amp::Node> parent_map;
    for (const auto& [parent, child, weight] : edges) {
        parent_map[child] = parent;
    }

    std::vector<amp::Node> node_path;
    amp::Node current_node_id = goal_node_id;
    
    std::cout << "\n=== BUILDING PATH ===" << std::endl;
    std::cout << "Starting from goal_node_id: " << goal_node_id << std::endl;
    
    while (current_node_id != 0) {
        node_path.push_back(current_node_id);
        //std::cout << "Added node " << current_node_id << " to path" << std::endl;
        
        if (parent_map.find(current_node_id) == parent_map.end()) {
            std::cout << "ERROR: No parent found for node " << current_node_id << std::endl;
            break;
        }
        current_node_id = parent_map[current_node_id];
    }
    node_path.push_back(0); // Add root
    std::reverse(node_path.begin(), node_path.end());

    std::cout << "\n=== FINAL NODE PATH ===" << std::endl;
    for (size_t i = 0; i < node_path.size(); i++) {
        std::cout << "Step " << i << ": Node " << node_path[i] << std::endl;
    }

    // Build KinoPath
    amp::KinoPath path;

    std::cout << "\n=== BUILDING KINOPATH WAYPOINTS ===" << std::endl;
    for (size_t i = 0; i < node_path.size(); i++) {
        amp::Node node_id = node_path[i];
        
        if (node_id >= points.size()) {
            std::cout << "ERROR: Invalid node_id " << node_id << " (points.size() = " << points.size() << ")" << std::endl;
            continue;
        }
        
        Eigen::VectorXd waypoint = points[node_id];
        std::cout << "Waypoint " << i << " (Node " << node_id << "): (" << waypoint[0] << ", " << waypoint[1] << ")" << std::endl;
        path.waypoints.push_back(waypoint);
        
        // Control and duration pairing:
        // - Initial waypoint (i=0): gets zero control/duration (nothing needed to get there)
        // - Subsequent waypoints: get the control that was applied to reach them
        if (i == 0) {
            // Initial waypoint gets zero control and duration
            Eigen::VectorXd zero_control = Eigen::VectorXd::Zero(problem.u_bounds.size());
            path.controls.push_back(zero_control);
            path.durations.push_back(0.0);
            std::cout << "Initial waypoint: adding zero control and duration" << std::endl;
        } else {
            // For waypoint i, use the control that was applied to reach it
            // This control was stored at the node_id when it was created
            if (node_id < controls.size()) {
                std::cout << "Adding control[" << node_id << "] - control TO REACH node " << node_id << std::endl;
                path.controls.push_back(controls[node_id]);
                path.durations.push_back(durations[node_id]);
            } else {
                std::cout << "ERROR: No control for node " << node_id << std::endl;
                // Add zero control as fallback
                Eigen::VectorXd zero_control = Eigen::VectorXd::Zero(problem.u_bounds.size());
                path.controls.push_back(zero_control);
                path.durations.push_back(0.0);
            }
        }
    }

    std::cout << "\n=== FINAL PATH INFO ===" << std::endl;
    std::cout << "Number of waypoints: " << path.waypoints.size() << std::endl;
    std::cout << "Number of controls: " << path.controls.size() << std::endl;
    std::cout << "Number of durations: " << path.durations.size() << std::endl;
    if (!path.waypoints.empty()) {
        std::cout << "First waypoint: (" << path.waypoints[0][0] << ", " << path.waypoints[0][1] << ")" << std::endl;
        std::cout << "Last waypoint: (" << path.waypoints.back()[0] << ", " << path.waypoints.back()[1] << ")" << std::endl;
    }
    std::cout << "Goal region: [" << problem.q_goal[0].first << ", " << problem.q_goal[0].second 
              << "] x [" << problem.q_goal[1].first << ", " << problem.q_goal[1].second << "]" << std::endl;

    path.valid = true;
    return path;
}