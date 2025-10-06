#include "MyCSConstructors.h"
#include "CSpaceSkeleton.h"



// Override this method for computing all of the boolean collision values for each cell in the cspace  
std::unique_ptr<amp::GridCSpace2D> MyPointAgentCSConstructor::construct(const amp::Environment2D& env) {
    // Create an object of my custom cspace type (e.g. MyGridCSpace2D) and store it in a unique pointer. 
    // Pass the constructor parameters to std::make_unique()
    std::unique_ptr<MyGridCSpace2D> cspace_ptr = std::make_unique<MyGridCSpace2D>(m_cells_per_dim, m_cells_per_dim, env.x_min, env.x_max, env.y_min, env.y_max);
    // In order to use the pointer as a regular GridCSpace2D object, we can just create a reference
    MyGridCSpace2D& cspace = *cspace_ptr;
    std::cout << "Constructing C-space for point agent" << std::endl;
    // Determine if each cell is in collision or not, and store the values the cspace. This `()` operator comes from DenseArray base class
    
    
    std::vector<MyObstacle> my_obstacles;
    for (const auto& obstacle : env.obstacles) {
        MyObstacle my_ob;
        my_ob.defineWithPoints(obstacle.verticesCCW());
        my_obstacles.push_back(my_ob);
    };
    
    double dx = (env.x_max - env.x_min) / m_cells_per_dim;
    double dy = (env.y_max - env.y_min) / m_cells_per_dim;
    for (double x = env.x_min + (dx/2.0); x < env.x_max; x += dx) {
        for (double y = env.y_min + (dy/2.0); y < env.y_max; y += dy) {
            // amp::ManipulatorState state;
            // state.resize(manipulator.nLinks());

            // state[0] = theta_0;
            // state[1] = theta_1;

            bool collision = false;
            
            // for (uint32_t i = 0; i <= manipulator.nLinks()-1; ++i) {

            Eigen::Vector2d point(x,y);

            for (const auto& ob : my_obstacles){
                if (ob.collisionCheck(point)){
                    collision = true;
                    break;
                }
            }

            if (collision){
                auto cell_collided = cspace.getCellFromPoint(x, y);
                cspace(cell_collided.first, cell_collided.second) = true;

            }
        }
    }


    // Returning the object of type std::unique_ptr<MyGridCSpace2D> can automatically cast it to a polymorphic base-class pointer of type std::unique_ptr<amp::GridCSpace2D>.
    // The reason why this works is not super important for our purposes, but if you are curious, look up polymorphism!
    return cspace_ptr;
}

amp::Path2D MyWaveFrontAlgorithm::planInCSpace(const Eigen::Vector2d& q_init, const Eigen::Vector2d& q_goal, const amp::GridCSpace2D& grid_cspace, bool isManipulator) {
    // Implement your WaveFront algorithm here

    MyGridCSpace2D mygrid_cspace(grid_cspace);

    auto grid_size = mygrid_cspace.size();
    int n = grid_size.first;
    int m = grid_size.second; 
    
    Eigen::MatrixXi wavefront_grid = Eigen::MatrixXi::Zero(n, m);

    // set goal to 2
    auto cell_goal = mygrid_cspace.getCellFromPoint(q_goal[0], q_goal[1]);
    auto cell_init = mygrid_cspace.getCellFromPoint(q_init[0], q_init[1]);
    wavefront_grid(cell_goal.first, cell_goal.second) = 2;

    // set all obstacles to 1
    for (uint64_t i = 0; i < n; i++){
        for (uint64_t j = 0; j < m; j++){
            if (mygrid_cspace(i, j)){
                wavefront_grid(i, j) = 1;
            }
        }
    }

    uint64_t run_count = 0;
    int searching_for = 2; // 2 cuz we looking for the goal to start

    bool hit_goal = false;

    while (run_count < 1e6 && !hit_goal){
        run_count ++;

        for (uint64_t i = 0; i < n; i++){
            for (uint64_t j = 0; j < m; j++){
                
                bool set_ij = false;
                if (i != 0){ // allowed to move left
                    if (wavefront_grid(i-1, j) == searching_for && wavefront_grid(i, j) == 0){
                        wavefront_grid(i, j) = searching_for + 1;
                        set_ij = true;
                    }
                }
                if (i != n-1){ // allowed to move right
                    if (wavefront_grid(i+1, j) == searching_for && wavefront_grid(i, j) == 0){
                        wavefront_grid(i, j) = searching_for + 1;
                        set_ij = true;
                    }
                }
                if (j != 0){ // allowed to move down
                    if (wavefront_grid(i, j-1) == searching_for && wavefront_grid(i, j) == 0){
                        wavefront_grid(i, j) = searching_for + 1;
                        set_ij = true;
                    }
                }
                if (j != m-1){ // allowed to move up
                    if (wavefront_grid(i, j+1) == searching_for && wavefront_grid(i, j) == 0){
                        wavefront_grid(i, j) = searching_for + 1;
                        set_ij = true;
                    }
                }

                if (set_ij){ // gotta check if we set the init
                    if (i == cell_init.first && j == cell_init.second){
                        hit_goal = true; // should break the while loop
                    }

                }
            }
        }
        searching_for ++;
    }

    if (run_count >= 1e6){
        std::cout << "loop 1 took too long" << std::endl;
    }

    // AI helped me here to visualize
    std::cout << "Wavefront Grid:" << std::endl;
    for (int j = m-1; j >= 0; j--) {  // Print from top to bottom
        for (int i = 0; i < n; i++) {
            int val = wavefront_grid(i, j);
            if (val == 1) {
                std::cout << " # ";  // Obstacle
            } else if (val == 2) {
                std::cout << " G ";  // Goal
            } else if (val > 2) {
                std::cout << std::setw(2) << val << " ";  // Distance values
            } else {
                std::cout << " . ";  // Unvisited
            }
        }
        std::cout << std::endl;
    }

    
    // now do gradient descent on dis thing
    
    amp::Path2D path;
    path.waypoints.push_back(q_init);
    
    Eigen::Vector2i current_cell(cell_init.first, cell_init.second);

    run_count = 0;

    while ((current_cell[0] != cell_goal.first || current_cell[1] != cell_goal.second) && run_count < 1e5){
        run_count ++;
        Eigen::Vector2i lowest_cell;
        int lowest_value = 1e6;
        if (current_cell[0] != 0){ // can look left
            if (wavefront_grid(current_cell[0] - 1, current_cell[1]) < lowest_value && 
            wavefront_grid(current_cell[0] - 1, current_cell[1]) > 1){
                lowest_value = wavefront_grid(current_cell[0] - 1, current_cell[1]);
                lowest_cell = Eigen::Vector2i(current_cell[0] - 1, current_cell[1]);
            }
        }
        if (current_cell[0] != n-1){ // can look right
            if (wavefront_grid(current_cell[0] + 1, current_cell[1]) < lowest_value && 
            wavefront_grid(current_cell[0] + 1, current_cell[1]) > 1){
                lowest_value = wavefront_grid(current_cell[0] + 1, current_cell[1]);
                lowest_cell = Eigen::Vector2i(current_cell[0] + 1, current_cell[1]);
            }
        }
        if (current_cell[1] != 0){ // can look down
            if (wavefront_grid(current_cell[0], current_cell[1] - 1) < lowest_value && 
            wavefront_grid(current_cell[0], current_cell[1] - 1) > 1){
                lowest_value = wavefront_grid(current_cell[0], current_cell[1] - 1);
                lowest_cell = Eigen::Vector2i(current_cell[0], current_cell[1] - 1);
            }
        }
        if (current_cell[1] != m-1){ // can look up
            if (wavefront_grid(current_cell[0], current_cell[1] + 1) < lowest_value && 
            wavefront_grid(current_cell[0], current_cell[1] + 1) > 1){
                lowest_value = wavefront_grid(current_cell[0], current_cell[1] + 1);
                lowest_cell = Eigen::Vector2i(current_cell[0], current_cell[1] + 1);
            }
        }

        // move to that lowest cell. 

        current_cell = lowest_cell;

        Eigen::Vector2d new_point = mygrid_cspace.getPointFromCell(current_cell[0], current_cell[1]);

        path.waypoints.push_back(new_point);
    }

    if (run_count >= 1e5){
        std::cout << "loop 2 took too long" << std::endl;
    }

    path.waypoints.push_back(q_goal);

    if (isManipulator) {
        Eigen::Vector2d bounds0 = Eigen::Vector2d(0.0, 0.0);
        Eigen::Vector2d bounds1 = Eigen::Vector2d(2*M_PI, 2*M_PI);
        amp::unwrapWaypoints(path.waypoints, bounds0, bounds1);
    }
    return path;
}
