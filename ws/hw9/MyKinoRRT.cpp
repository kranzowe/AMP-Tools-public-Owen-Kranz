#include "MyKinoRRT.h"
#include "MySamplingBasedPlanners.h"

Eigen::Matrix<double,2,2> MySingleIntegrator::A(Eigen::VectorXd state){
    Eigen::Matrix<double, 2,2> A;
    A << 0,0,0,0;
    return A;
}

Eigen::Matrix<double,2,2> MySingleIntegrator::B(Eigen::VectorXd state){
    Eigen::Matrix<double, 2,2> B;
    B << 1,0,0,1;
    return B;
}

void MySingleIntegrator::propagate(Eigen::VectorXd& state, Eigen::VectorXd& control, double dt) {
    
    // amp::points_to_check.clear();
    // amp::sub_states.clear();

    double sub_dt = 0.001;

    double t = 0;

    while (t + sub_dt < dt){
        
        Eigen::VectorXd k1 = A(state)* state + B(state) * control;
        Eigen::VectorXd k2 = A(state + k1 * sub_dt/2) * (state + k1 * sub_dt / 2) + B(state + k1 * sub_dt/2) * control;
        Eigen::VectorXd k3 = A(state + k2 * sub_dt/2) * (state + k2 * sub_dt / 2) + B(state + k2 * sub_dt/2) * control;
        Eigen::VectorXd k4 = A(state + k3 * sub_dt) * (state + k3 * sub_dt) + B(state + k3 * sub_dt) * control;

        state += ((k1 + 2.0*k2 + 2.0*k3 + k4) / 6.0)* sub_dt;
        
        // by reccomendation, adding buffer on all states for checking 0l001 works
        // Eigen::Vector2d pos(state[0] +  new_state[0]  + 0.001, state[1] + new_state[1] + 0.001);
        // Eigen::Vector2d pos1(state[0] + new_state[0]  - 0.001, state[1] + new_state[1] + 0.001);
        // Eigen::Vector2d pos2(state[0] + new_state[0]  + 0.001, state[1] + new_state[1] - 0.001);
        // Eigen::Vector2d pos3(state[0] + new_state[0]  - 0.001, state[1] + new_state[1] - 0.001);
        
        // std::vector<Eigen::VectorXd> points;
        // points.push_back(pos);
        // points.push_back(pos1);
        // points.push_back(pos2);
        // points.push_back(pos3);
        // amp::points_to_check.push_back(points);
        // amp::sub_states.push_back(state);

        t += sub_dt;
    }

    // last step needs to be precise
    double last_dt = dt - t; // gauranteed to be less than sub dt

    Eigen::VectorXd k1 = A(state)* state + B(state) * control;
    Eigen::VectorXd k2 = A(state + k1 * last_dt/2) * (state + k1 * last_dt / 2) + B(state + k1 * last_dt/2) * control;
    Eigen::VectorXd k3 = A(state + k2 * last_dt/2) * (state + k2 * last_dt / 2) + B(state + k2 * last_dt/2) * control;
    Eigen::VectorXd k4 = A(state + k3 * last_dt) * (state + k3 *   last_dt) + B(state + k3 *     last_dt) * control;

    state += ((k1 + 2.0*k2 + 2.0*k3 + k4) / 6.0)* last_dt;

    return;

};

MyKinoRRT::MyKinoRRT(double dt, int num_control_samples, int max_iterations, double stop_radius)
    : m_dt(dt), m_num_control_samples(num_control_samples), m_max_iterations(max_iterations), m_stop_radius(stop_radius) {}

amp::KinoPath MyKinoRRT::plan(const amp::KinodynamicProblem2D& problem, amp::DynamicAgent& agent) {
    
    /// need to make collision checker and metric

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
    
    Eigen::VectorXd init_state(2), goal_state(2);
    init_state[0] = problem.q_init[0];
    init_state[1] = problem.q_init[1];

    // some AI assist here
    // q_goal is a vector of pairs, so access .first and .second
    goal_state[0] = (problem.q_goal[0].first + problem.q_goal[0].second) / 2.0;  // center of goal region
    goal_state[1] = (problem.q_goal[1].first + problem.q_goal[1].second) / 2.0; 

     // bounds for generating random samples 
    Eigen::VectorXd min_vector = collision_checker.lowerBounds();
    //LOG("MIN vec " << min_vector);
    Eigen::VectorXd max_vector = collision_checker.upperBounds();
    //LOG("Max vec " << max_vector);
    
    std::shared_ptr<amp::Graph<double>> graphPtr = std::make_shared<amp::Graph<double>>();
    
    // graph stuff
    std::vector<Eigen::VectorXd> points;
    std::vector<Eigen::VectorXd> controls;
    std::vector<std::tuple<amp::Node, amp::Node, double>> edges = {};
    // STILL GOTTA PUSH BACK THIS INIT POINT

    points.insert(points.begin(), init_state); // rrt starts from init and branches from there
    
    int dim = init_state.size();
    int samples = 0;

    static std::random_device rd;
    static std::mt19937 gen(rd());

    //start samplign
    while (samples < m_max_iterations){
        Eigen::VectorXd random_point(dim);
        
        // AI helped with randomness here
        std::uniform_real_distribution<double> uniform_dist(0.0, 1.0);
        double rando_0to1 = uniform_dist(gen);
        if (rando_0to1 < 0.08){
            random_point = goal_state;
        }else{ // do traditional sampling
            for (int i = 0; i < dim; ++i) {
                std::uniform_real_distribution<double> dist(min_vector[i], max_vector[i]);
                random_point[i] = dist(gen);
            }
        }
        
        // LOG("Rando point" << random_point);
        // nearest neighbor
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

        // LOG("NEAREST " << nearest_point);

        Eigen::VectorXd proposed_point(dim);
        Eigen::VectorXd proposed_control(dim);

        // const MyPointAndDiscCollisionChecker* my_checker = dynamic_cast<const MyPointAndDiscCollisionChecker*>(&collision_checker);

        //apply random controls
        double min_dist = 1e8;
        for (int i = 0; i < m_num_control_samples; i++) {
            Eigen::VectorXd control = Eigen::VectorXd::Random(problem.q_init.size());
            Eigen::VectorXd test_point = nearest_point;
            
            bool trajectory_collision = false;
            double trajectory_dt = m_dt / 30.0;
            
            for (int j = 0; j < 30; j++) {
                agent.propagate(test_point, control, trajectory_dt);
                
                if (collision_checker.inCollision(test_point)) {
                    trajectory_collision = true;
                    break;
                }
            }
            
            if (!trajectory_collision) {
                double dist = metric.distance(test_point, random_point);
                if (dist < min_dist) {
                    proposed_point = test_point;
                    proposed_control = control;
                    min_dist = dist;
                }
            }
        }

        if (min_dist < 1e8) {
            points.push_back(proposed_point);
            controls.push_back(proposed_control);
            amp::Node proposed_node_id = points.size() - 1;

            double edge_weight = metric.distance(nearest_point, proposed_point);
            edges.push_back({nearest_node_id, proposed_node_id, edge_weight}); 

            double dist_to_goal = metric.distance(proposed_point, goal_state);
            bool at_goal = true;

        // since goal is now a box, gotta check a lil diff
        for(int i = 0; i < proposed_point.rows(); i++){
            if(i == 2){ // kinda hardcoded sorry
                // put into region -pi to pi
                double temp = atan2(sin(proposed_point[i]), cos(proposed_point[i]));
                
                // check outside box
                if(problem.q_goal[i].first > temp || problem.q_goal[i].second < temp){
                    at_goal = false;
                }
                continue;
            }
            
            // positional parts of the state
            if(problem.q_goal[i].first > proposed_point[i] || problem.q_goal[i].second < proposed_point[i]){
                at_goal = false;
            }
        }

        if(at_goal){
            // Add goal connection
            points.push_back(proposed_point);  // or use the center of goal bounds if you want
            controls.push_back(proposed_control);
            amp::Node goal_node_id = points.size() - 1;
            
            double goal_edge_weight = metric.distance(proposed_point, proposed_point);  // or your distance function
            edges.push_back({proposed_node_id, goal_node_id, goal_edge_weight}); 
            
            break;
        }
            }
            //else{LOG("FOUND POINT COLLISION");}
            samples++;
            // std::cin.get();
        }

    // now only for visualizations
    for (const auto& [from, to, weight] : edges) { // add to graph
        graphPtr->connect(from, to, weight);
    }

    // AI helped me store stuff to visualize later so i can see the pretty graphs
    //LOG("Points size before: " << points.size());
    // m_last_graph = graphPtr;
    // m_last_nodes.clear();
    // for (amp::Node i = 0; i < points.size(); ++i) {
    //     m_last_nodes[i] = Eigen::Vector2d(points[i][0], points[i][1]);
    // }
    //LOG("m_last_nodes size after storing: " << m_last_nodes.size());
    // add points
    amp::KinoPath path;
    std::map<amp::Node, amp::Node> parent_map;
    for (const auto& [parent, child, weight] : edges) {
        parent_map[child] = parent;
    }

    std::vector<amp::Node> node_path;
    amp::Node current_node_id = points.size() - 1;
    while (current_node_id != 0) {
        node_path.push_back(current_node_id);
        current_node_id = parent_map[current_node_id];
    }
    node_path.push_back(0);
    std::reverse(node_path.begin(), node_path.end());

    for (size_t i = 0; i < node_path.size(); i++) {
        path.waypoints.push_back(points[node_path[i]]);
        
        if (i < node_path.size() - 1) {
            amp::Node current = node_path[i];
            path.controls.push_back(controls[current]);
            path.durations.push_back(m_dt);
        }
    }

    path.valid = true;
    return path;
}

