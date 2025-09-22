#include "CSpaceSkeleton.h"
#include "MyObstacle.h"
// Override this method for returning whether or not a point is in collision

std::pair<std::size_t, std::size_t> MyGridCSpace2D::getCellFromPoint(double x0, double x1) const {
    // Implment your discretization procedure here, such that the point (x0, x1) lies within the returned cell
    double x0_min = m_x0_bounds.first; //gotta get a bunch o variaubles
    double x0_max = m_x0_bounds.second;
    double x1_min = m_x0_bounds.first;
    double x1_max = m_x0_bounds.second;

    auto grid_size = size();
    std::size_t x0_cells = grid_size.first;
    std::size_t x1_cells = grid_size.second;

    double delta_x0 = (x0_max - x0_min) / x0_cells;
    double delta_x1 = (x1_max - x1_min) / x1_cells;

    double unrounded_index_x0 = (x0 - x0_min) / delta_x0;
    double unrounded_index_x1 = (x1 - x1_min) / delta_x1;
    
    std::size_t cell_x0 = static_cast<std::size_t>(std::max(0.0, std::floor(unrounded_index_x0)));
    std::size_t cell_x1 = static_cast<std::size_t>(std::max(0.0, std::floor(unrounded_index_x1)));
   
    if (cell_x0 > x0_cells - 1){
        std::cout << "x0 cell outta range \n"; 
    }
    if (cell_x1 > x1_cells - 1){
        std::cout << "x1 cell outta range \n"; 
    }
    return {cell_x0, cell_x1};
}

// Override this method for computing all of the boolean collision values for each cell in the cspace
std::unique_ptr<amp::GridCSpace2D> MyManipulatorCSConstructor::construct(const amp::LinkManipulator2D& manipulator, const amp::Environment2D& env) {
    // Create an object of my custom cspace type (e.g. MyGridCSpace2D) and store it in a unique pointer. 
    // Pass the constructor parameters to std::make_unique()
    std::unique_ptr<MyGridCSpace2D> cspace_ptr = std::make_unique<MyGridCSpace2D>(m_cells_per_dim, m_cells_per_dim, env.x_min, env.x_max, env.y_min, env.y_max);
    // In order to use the pointer as a regular GridCSpace2D object, we can just create a reference
    MyGridCSpace2D& cspace = *cspace_ptr;

    // Determine if each cell is in collision or not, and store the values the cspace. This `()` operator comes from DenseArray base class
    
    std::vector<MyObstacle> my_obstacles;
    for (const auto& obstacle : env.obstacles) {
        MyObstacle my_ob;
        my_ob.defineWithPoints(obstacle.verticesCCW());
        my_obstacles.push_back(my_ob);
    };

    double theta_1 = env.x_min;
    

    double dtheta = 0.1;

    while (theta_1 < env.x_max){

        double theta_2 = env.y_min;

        while (theta_2 < env.y_max){
            amp::ManipulatorState state;
            state.resize(2);

            state[1] = theta_1;
            state[2] = theta_2;
            
            for (uint32_t i = 0; i < manipulator.nLinks(); ++i) {

                points_to_check = 

                Eigen::Vector2d joint_pos = manipulator.getJointLocation(ninety_state, i + 1);
                std::cout << "Joint " << i + 1 << " position: [" << joint_pos[0] << ", " << joint_pos[1] << "]\n";
            }



        }

    }
    
    cspace(1, 3) = true;
    cspace(3, 3) = true;
    cspace(0, 1) = true;
    cspace(1, 0) = true;
    cspace(2, 0) = true;
    cspace(3, 0) = true;
    cspace(4, 1) = true;

    // Returning the object of type std::unique_ptr<MyGridCSpace2D> can automatically cast it to a polymorphic base-class pointer of type std::unique_ptr<amp::GridCSpace2D>.
    // The reason why this works is not super important for our purposes, but if you are curious, look up polymorphism!
    return cspace_ptr;
}
