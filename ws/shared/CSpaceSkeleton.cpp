#include "CSpaceSkeleton.h"
#include "MyObstacle.h"

// Override this method for returning whether or not a point is in collision

std::pair<std::size_t, std::size_t> MyGridCSpace2D::getCellFromPoint(double x0, double x1) const {
    // Implment your discretization procedure here, such that the point (x0, x1) lies within the returned cell
    double x0_min = m_x0_bounds.first; //gotta get a bunch o variaubles
    double x0_max = m_x0_bounds.second;
    double x1_min = m_x1_bounds.first;
    double x1_max = m_x1_bounds.second;

    auto grid_size = size();
    std::size_t x0_cells = grid_size.first;
    std::size_t x1_cells = grid_size.second;

    while (x0 < x0_min) x0 += 2*M_PI;
    while (x0 >= x0_max) x0 -= 2*M_PI;
    while (x1 < x1_min) x1 += 2*M_PI;
    while (x1 >= x1_max) x1 -= 2*M_PI;

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

Eigen::Vector2d MyGridCSpace2D::getPointFromCell(double c0, double c1) const {
    // just the inverse of the above...
    double x0_min = m_x0_bounds.first; //gotta get a bunch o variaubles
    double x0_max = m_x0_bounds.second;
    double x1_min = m_x1_bounds.first;
    double x1_max = m_x1_bounds.second;

    auto grid_size = size();
    std::size_t x0_cells = grid_size.first;
    std::size_t x1_cells = grid_size.second;

    double delta_x0 = (x0_max - x0_min) / x0_cells;
    double delta_x1 = (x1_max - x1_min) / x1_cells;

    double x = ((delta_x0 * c0) + (delta_x0/2.0)) + x0_min;
    double y = ((delta_x1 * c1) + (delta_x1/2.0)) + x1_min; // adding half so we are in the middle of the cells

    Eigen::Vector2d point(x,y);
    
    return point;
}


// Override this method for computing all of the boolean collision values for each cell in the cspace
std::unique_ptr<amp::GridCSpace2D> MyManipulatorCSConstructor::construct(const amp::LinkManipulator2D& manipulator, const amp::Environment2D& env) {
    // Create an object of my custom cspace type (e.g. MyGridCSpace2D) and store it in a unique pointer. 
    // Pass the constructor parameters to std::make_unique()
    double x0_min = 0.0;
    double x0_max = 2.0*M_PI;
    double x1_min = 0.0;
    double x1_max = 2.0*M_PI;
    std::unique_ptr<MyGridCSpace2D> cspace_ptr = std::make_unique<MyGridCSpace2D>(m_cells_per_dim, m_cells_per_dim, x0_min, x0_max, x1_min, x1_max);
    // In order to use the pointer as a regular GridCSpace2D object, we can just create a reference
    MyGridCSpace2D& cspace = *cspace_ptr;

    // Determine if each cell is in collision or not, and store the values the cspace. This `()` operator comes from DenseArray base class
    
    std::vector<MyObstacle> my_obstacles;
    for (const auto& obstacle : env.obstacles) {
        MyObstacle my_ob;
        my_ob.defineWithPoints(obstacle.verticesCCW());
        my_obstacles.push_back(my_ob);
    };
    
    double dtheta_0 = (x0_max - x0_min) / m_cells_per_dim;
    double dtheta_1 = (x1_max - x1_min) / m_cells_per_dim;
    for (double theta_0 = x0_min + (dtheta_0/2.0); theta_0 < x0_max; theta_0 += dtheta_0) {
        for (double theta_1 = x1_min + (dtheta_1/2.0); theta_1 < x1_max; theta_1 += dtheta_1) {
            amp::ManipulatorState state;
            state.resize(manipulator.nLinks());

            state[0] = theta_0;
            state[1] = theta_1;

            bool collision = false;
            
            for (uint32_t i = 0; i <= manipulator.nLinks()-1; ++i) {

                Eigen::Vector2d joint_start = manipulator.getJointLocation(state, i);
                Eigen::Vector2d joint_end = manipulator.getJointLocation(state, i + 1);
                


                for (const auto& ob : my_obstacles){
                    if (ob.collisionCheckAlongLine(joint_start, joint_end)){
                        collision = true;
                        break;
                    }}
                if (collision){
                    break;
                }
            }
            if (collision){
                auto cell_collided = cspace.getCellFromPoint(theta_0, theta_1);
                cspace(cell_collided.first, cell_collided.second) = true;

            }
        
        }
    }


    // cspace(1, 3) = true;
    // cspace(3, 3) = true;
    // cspace(0, 1) = true;
    // cspace(1, 0) = true;
    // cspace(2, 0) = true;
    // cspace(3, 0) = true;
    // cspace(4, 1) = true;

    // Returning the object of type std::unique_ptr<MyGridCSpace2D> can automatically cast it to a polymorphic base-class pointer of type std::unique_ptr<amp::GridCSpace2D>.
    // The reason why this works is not super important for our purposes, but if you are curious, look up polymorphism!
    return cspace_ptr;
}
