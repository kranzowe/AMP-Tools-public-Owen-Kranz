#include "MySamplingBasedPlanners.h"
#include <random>
#include <algorithm>
#include <queue>
#include "AMPCore.h"
#include "MyAStar.h"
#include "MyObstacle.h"

// NOTE on AI Use, I used AI to help structure the new header and .cpp file to match
// the given hint

// methods for getting the bounds of the prob
Eigen::VectorXd MyPointCollisionChecker::calculateLowerBounds(const std::vector<Eigen::VectorXd>& env_vertices) {
    
    Eigen::VectorXd min_bounds = env_vertices[0];
    for (size_t i = 1; i < env_vertices.size(); ++i) {
        // find min of all indexes
        min_bounds = min_bounds.cwiseMin(env_vertices[i]);
    }
    return min_bounds;
}

Eigen::VectorXd MyPointCollisionChecker::calculateUpperBounds(const std::vector<Eigen::VectorXd>& env_vertices) {

    Eigen::VectorXd max_bounds = env_vertices[0];
    for (size_t i = 1; i < env_vertices.size(); ++i) {
        // find min of all indexes
        max_bounds = max_bounds.cwiseMax(env_vertices[i]);
    }
    return max_bounds;
}

// my custom collision checker
MyPointCollisionChecker::MyPointCollisionChecker(const std::vector<MyObstacle>& obstacles, 
                                                 const std::vector<Eigen::VectorXd>& env_vertices)
    : amp::ConfigurationSpace(calculateLowerBounds(env_vertices), calculateUpperBounds(env_vertices))
    , m_obstacles(obstacles), m_env_vertices(env_vertices) {
}

// for all obs checks for collsiiosn
bool MyPointCollisionChecker::inCollision(const Eigen::VectorXd& config) const {
    // this still uses only 2d obs and 2d points. CHANGE FOR N dim
    Eigen::Vector2d point2d(config[0], config[1]);
    for (const auto& obstacle : m_obstacles) {
        if (obstacle.collisionCheck(point2d)) {
            return true;
        }
    }
    return false;
}

bool MyPointCollisionChecker::edgeInCollision(const Eigen::VectorXd& config1, const Eigen::VectorXd& config2) const {
    // checks allong ray, uses old func
    Eigen::Vector2d point2d1(config1[0], config1[1]);
    Eigen::Vector2d point2d2(config2[0], config2[1]);
    for (const auto& obstacle : m_obstacles) {
        if (obstacle.collisionCheckAlongLine(point2d1, point2d2)){
            return true;
        }
    }
    return false;
}


/////// New collision checks /////////////////////////////

// a new collision check for the robo and obstacles
MyPointAndDiscCollisionChecker::MyPointAndDiscCollisionChecker(const std::vector<MyObstacle>& obstacles, 
                                                 const std::vector<MyObstacle>& moving_circular_obstacles,
                                                 const std::vector<Eigen::VectorXd>& env_vertices)
    : MyPointCollisionChecker(obstacles, env_vertices),
     m_moving_circular_obstacles(moving_circular_obstacles) {
}

// for all obs checks for collsiiosn
bool MyPointAndDiscCollisionChecker::inCollision(const Eigen::VectorXd& config) const {
    
    std::vector<Eigen::Vector2d> disc_centers;
    std::vector<Eigen::Vector2d> points_to_check;

    int num_samples = 16;

    // this 2 is becaseu i know the config of one robot is 2. kinda gross
    for (size_t i = 0; 2*i < config.size(); i++){
        Eigen::Vector2d point{config[2*i], config[(2*i) + 1]};
        disc_centers.push_back(point);

        for (double theta = 0; theta < 2*M_PI; theta += 2*M_PI/(num_samples)){
            //sample 16 points on each robot edge.
            Eigen::Vector2d outer_point{point[0] + (cos(theta)*m_moving_circular_obstacles[i].moving_circular_primative.radius*1.1), 
                                        point[1] + (sin(theta)*m_moving_circular_obstacles[i].moving_circular_primative.radius*1.1)};

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
    }

    for (size_t i = 0; 2*i < config.size(); i++){
        std::vector<Eigen::Vector2d> points_to_check_minus_current;

        // pushback only takes one so gotta use insert(at end or begining, what to insert, end of insert)
        points_to_check_minus_current.insert(points_to_check_minus_current.end(),
                                     points_to_check.begin(), 
                                     points_to_check.begin() + (i*num_samples));


        points_to_check_minus_current.insert(points_to_check_minus_current.end(),
                                            points_to_check.begin() + (i*num_samples) + num_samples, 
                                            points_to_check.end());
        // LOG("JUST SLICED 0 to " << i*num_samples);
        // LOG("and  SLICED " << (i*num_samples) + num_samples << " to end");
        for (auto point : points_to_check_minus_current){
            // check for collisions with other robits
            // ooo gotta be sneaky here. Dont want to check collision with self.
            if (m_moving_circular_obstacles[i].collisionCheckTranslated(point, disc_centers[i])) {
                return true;
            }
        }
    }

    return false;
}

bool MyPointAndDiscCollisionChecker::edgeInCollision(const Eigen::VectorXd& config1, const Eigen::VectorXd& config2) const{
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


// n dim
GenericPRM::GenericPRM(int num_samples, double connection_radius) 
    : m_num_samples(num_samples), m_connection_radius(connection_radius) {}

//N dim
amp::Path GenericPRM::plan(const Eigen::VectorXd& init_state, 
                           const Eigen::VectorXd& goal_state, 
                           const amp::ConfigurationSpace& collision_checker,
                           const DistanceMetric& metric) {
    
    // Needed AI help to Cast so that it would use the custom function
    const MyPointCollisionChecker* my_checker = dynamic_cast<const MyPointCollisionChecker*>(&collision_checker);
    
    // bounds for generating random samples 
    Eigen::VectorXd min_vector = collision_checker.lowerBounds();
    Eigen::VectorXd max_vector = collision_checker.upperBounds();
    
    
    std::shared_ptr<amp::Graph<double>> graphPtr = std::make_shared<amp::Graph<double>>();
    
    // graph stuff
    std::vector<Eigen::VectorXd> points;
    std::vector<std::tuple<amp::Node, amp::Node, double>> edges = {};
    
    int dim = init_state.size();
    int samples = 0;
    int collision_free_count = 0;

    static std::random_device rd;
    static std::mt19937 gen(rd());

    //start samplign
    while (samples < m_num_samples){
        Eigen::VectorXd random_point(dim);
        
        // AI helped with randomness here
        for (int i = 0; i < dim; ++i) {
            std::uniform_real_distribution<double> dist(min_vector[i], max_vector[i]);
            random_point[i] = dist(gen);
        }
        
        // if node collision free canadd
        if (!collision_checker.inCollision(random_point)){
            points.push_back(random_point);
            collision_free_count++;
            amp::Node random_node_id = points.size() - 1;
            

            // connect but only if in radius and no collisions
            int connections_made = 0;
            for (amp::Node node_id = 0; node_id < random_node_id; ++node_id) {
                double dist = metric.distance(points[node_id], random_point);
                if (dist < m_connection_radius){
                    
                    if (!my_checker->edgeInCollision(points[node_id], random_point)) {
                        edges.push_back({node_id, random_node_id, dist});
                        edges.push_back({random_node_id, node_id, dist});
                        connections_made++;
                    }
                }
            }
        }
        samples++;
    }


    // adding init and final and connecting them
    // guess u could add em earlier but i think for multiquery this would be cleaner but idk
    points.insert(points.begin(), init_state); // astar needs em at the front
    points.insert(points.begin() + 1, goal_state);

    
    // adjusting edges since added at the start
    for (auto& [from, to, weight] : edges) {
        from += 2;
        to += 2;
    }

    // connections to init and goal
    for (amp::Node node_id = 2; node_id < points.size(); ++node_id) {
        double dist = metric.distance(init_state, points[node_id]);
        if (dist < m_connection_radius){

  
            if (!my_checker->edgeInCollision(init_state, points[node_id])){
                edges.push_back({0, node_id, dist});
                edges.push_back({node_id, 0, dist});

            }
        }
    }


    for (amp::Node node_id = 2; node_id < points.size(); ++node_id) {
        double dist = metric.distance(goal_state, points[node_id]);
        if (dist < m_connection_radius){

            if (!my_checker->edgeInCollision(goal_state, points[node_id])){
                edges.push_back({1, node_id, dist});
                edges.push_back({node_id, 1, dist});
            }
        }
    }



    for (const auto& [from, to, weight] : edges) { // add to graph
        graphPtr->connect(from, to, weight);
    }

    //astar needs shortest path problem
    amp::ShortestPathProblem spp;
    spp.graph = graphPtr;
    spp.init_node = 0;
    spp.goal_node = 1;
    
    MyLookupSearchHeuristic heuristic;
    std::map<amp::Node, double> heuristic_values; 
    
    //adding my heuristic vals just as the distance. should be a perfect heuristic??
    for (amp::Node i = 0; i < points.size(); ++i) {
        heuristic_values[i] = metric.distance(points[i], goal_state);

    }
    heuristic.heuristic_values = heuristic_values;
    MyAStarAlgo algo;
    MyAStarAlgo::GraphSearchResult result = algo.search(spp, heuristic);

    amp::Path path;
    for (const auto& node : result.node_path) {
        path.waypoints.push_back(points[node]);
    }

    // AI helped me store stuff to visualize later so i can see the pretty graphs
    m_last_graph = graphPtr;
    m_last_nodes.clear();
    for (amp::Node i = 0; i < points.size(); ++i) {
        m_last_nodes[i] = Eigen::Vector2d(points[i][0], points[i][1]);
    }


    bool smooth = false;

    if (smooth){
        for (int i = 0; i < 100; i++){
            if (path.waypoints.size() <= 3) break; //checking if there even is a path
            
            std::uniform_int_distribution<size_t> dist1(0, path.waypoints.size()-3);
            size_t index1 = dist1(gen);
            
            std::uniform_int_distribution<size_t> dist2(index1+2, path.waypoints.size()-1);
            size_t index2 = dist2(gen);

            //can connect?
            if (!my_checker->edgeInCollision(path.waypoints[index1], path.waypoints[index2])){
                // erase all inbetween
                path.waypoints.erase(path.waypoints.begin() + index1 + 1, 
                                path.waypoints.begin() + index2);
            }
        } 
    }

    return path;
}

// AI --- Add getter methods for visualization
std::shared_ptr<amp::Graph<double>> GenericPRM::getLastGraph() const {
    return m_last_graph;
}

std::map<amp::Node, Eigen::Vector2d> GenericPRM::getLastNodes() const {
    return m_last_nodes;
}

// NDim rrt
GenericRRT::GenericRRT(int max_iterations, double step_size, double goal_bias)
    : m_max_iterations(max_iterations), m_step_size(step_size), m_goal_bias(goal_bias) {}

amp::Path GenericRRT::plan(const Eigen::VectorXd& init_state, 
                           const Eigen::VectorXd& goal_state, 
                           const amp::ConfigurationSpace& collision_checker,
                           const DistanceMetric& metric) {
    
    
    // bounds for generating random samples 
    Eigen::VectorXd min_vector = collision_checker.lowerBounds();
    //LOG("MIN vec " << min_vector);
    Eigen::VectorXd max_vector = collision_checker.upperBounds();
    //LOG("Max vec " << max_vector);
    
    std::shared_ptr<amp::Graph<double>> graphPtr = std::make_shared<amp::Graph<double>>();
    
    // graph stuff
    std::vector<Eigen::VectorXd> points;
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
        if (rando_0to1 < m_goal_bias){
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

        // steer towards

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

        
        if (min_distance < m_step_size){
            proposed_point = random_point;
        }else{
            // need to just move it a bit by step
            Eigen::VectorXd unit_vec_dir = (random_point - nearest_point).normalized();
            proposed_point = nearest_point + (m_step_size * unit_vec_dir);
        }

        // LOG("PROPOSED " << proposed_point);
        const MyPointAndDiscCollisionChecker* my_checker = dynamic_cast<const MyPointAndDiscCollisionChecker*>(&collision_checker);
        

        // check collisionsss
        if (!collision_checker.inCollision(proposed_point)){
            // LOG("NO POINT COLL");

            bool edge_collision = false;
            if (my_checker) {
                edge_collision = my_checker->edgeInCollision(nearest_point, proposed_point);
            }else{
                LOG("CAST ERRRORRR");
            }
            
            if (!edge_collision) {
                // LOG("NO EDGE COLL");
                points.push_back(proposed_point);
                amp::Node proposed_node_id = points.size() - 1;

                // calculate the correct edge weight first
                double edge_weight = metric.distance(nearest_point, proposed_point);
                edges.push_back({nearest_node_id, proposed_node_id, edge_weight}); 

                double dist_to_goal = metric.distance(proposed_point, goal_state);
                if(dist_to_goal < 0.25){
                    // good enough for me
                    points.push_back(goal_state);
                    amp::Node goal_node_id = points.size() - 1;
                    
                    // Add edge from proposed_point to goal_state
                    double goal_edge_weight = metric.distance(proposed_point, goal_state);
                    edges.push_back({proposed_node_id, goal_node_id, goal_edge_weight}); 

                    break;
                }
            }
        }
        samples++;
        // std::cin.get();
    }

    // now only for visualizations
    for (const auto& [from, to, weight] : edges) { // add to graph
        graphPtr->connect(from, to, weight);
    }

    // AI helped me store stuff to visualize later so i can see the pretty graphs
    m_last_graph = graphPtr;
    m_last_nodes.clear();
    for (amp::Node i = 0; i < points.size(); ++i) {
        m_last_nodes[i] = Eigen::Vector2d(points[i][0], points[i][1]);
    }

    // add points
    amp::Path path;
    std::map<amp::Node, amp::Node> parent_map;
    for (const auto& [parent, child, weight] : edges) {
        parent_map[child] = parent;
    }

    amp::Node current_node_id = points.size() - 1; // goal is last
    // std::cout << "current id:" << current_node_id << std::endl;
    while (current_node_id != 0) {
        path.waypoints.push_back(points[current_node_id]);

        // std::cout << "added:" <<points[current_node_id] << std::endl;
        current_node_id = parent_map[current_node_id]; // parent from map
        // std::cout << "current id:" << current_node_id << std::endl;

    }
    path.waypoints.push_back(init_state);
    std::reverse(path.waypoints.begin(), path.waypoints.end());

    return path;
}

// AI --- Add getter methods for visualization
std::shared_ptr<amp::Graph<double>> GenericRRT::getLastGraph() const {
    return m_last_graph;
}

std::map<amp::Node, Eigen::Vector2d> GenericRRT::getLastNodes() const {
    return m_last_nodes;
}
MyPRM::MyPRM() : m_generic_prm(200, 2.0) {}

MyPRM::MyPRM(int num_samples, double connection_radius) 
    : m_generic_prm(num_samples, connection_radius) {}

amp::Path2D MyPRM::plan(const amp::Problem2D& problem) {
    Eigen::VectorXd init_nd(2);
    init_nd << problem.q_init.x(), problem.q_init.y();
    
    Eigen::VectorXd goal_nd(2);
    goal_nd << problem.q_goal.x(), problem.q_goal.y();
    
    std::vector<MyObstacle> obstacles;
    for (const auto& obstacle : problem.obstacles) {
        MyObstacle obs;
        obs.defineWithPoints(obstacle.verticesCCW());
        obstacles.push_back(obs);
    }
    
    std::vector<Eigen::VectorXd> env_vertices;
    env_vertices.push_back(Eigen::Vector2d(problem.x_min, problem.y_min));
    env_vertices.push_back(Eigen::Vector2d(problem.x_max, problem.y_min));
    env_vertices.push_back(Eigen::Vector2d(problem.x_max, problem.y_max));
    env_vertices.push_back(Eigen::Vector2d(problem.x_min, problem.y_max));
    
    MyPointCollisionChecker col_check(obstacles, env_vertices);
    L2Distance metric;
    
    amp::Path path_nd = m_generic_prm.plan(init_nd, goal_nd, col_check, metric);
    
    //  back to 2D
    amp::Path2D path_2d;
    for (const auto& waypoint : path_nd.waypoints) {
        path_2d.waypoints.push_back(Eigen::Vector2d(waypoint[0], waypoint[1]));
    }
    
    return path_2d;
}

//for testing
MyRRT::MyRRT() : m_generic_rrt(5000, 0.5, 0.05) {}

MyRRT::MyRRT(int max_iterations, double step_size, double goal_bias) 
    : m_generic_rrt(max_iterations, step_size, goal_bias) {}

amp::Path2D MyRRT::plan(const amp::Problem2D& problem) {
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
    MyPointCollisionChecker col_check(my_obstacles, env_vertices);

    // distance metric def
    L2Distance metric;
    
    // convert 2D points to N-D vectors. Needed AI help here
    Eigen::VectorXd init_nd(2), goal_nd(2);
    init_nd << problem.q_init[0], problem.q_init[1];
    goal_nd << problem.q_goal[0], problem.q_goal[1];
    
    amp::Path path_nd = m_generic_rrt.plan(init_nd, goal_nd, col_check, metric);
    
    // Convert N-D path back to 2D path // AI helped here as well
    amp::Path2D path_2d;
    for (const auto& waypoint : path_nd.waypoints) {
        path_2d.waypoints.push_back(Eigen::Vector2d(waypoint[0], waypoint[1]));
    }
    
    return path_2d;
}