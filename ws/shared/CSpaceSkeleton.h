#pragma once

// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"
#include "MyObstacle.h"

// Include the correct homework header
#include "hw/HW4.h"

// Derive the amp::GridCSpace2D class and override the missing method
class MyGridCSpace2D : public amp::GridCSpace2D {
    public:
        MyGridCSpace2D(std::size_t x0_cells, std::size_t x1_cells, double x0_min, double x0_max, double x1_min, double x1_max)
            : amp::GridCSpace2D(x0_cells, x1_cells, x0_min, x0_max, x1_min, x1_max) // Call base class constructor
        {}

        // Some AI help for this constructor. Needed a way to define a MyGrid with an instance of the grid
        MyGridCSpace2D(const amp::GridCSpace2D& existing_grid)
            : amp::GridCSpace2D(existing_grid.size().first, existing_grid.size().second, 
                               existing_grid.x0Bounds().first, existing_grid.x0Bounds().second,
                               existing_grid.x1Bounds().first, existing_grid.x1Bounds().second)
        {
            // Copy the collision data from the existing grid
            auto grid_size = size(); // size() returns std::pair<std::size_t, std::size_t>
            for (std::size_t i = 0; i < grid_size.first; ++i) {
                for (std::size_t j = 0; j < grid_size.second; ++j) {
                    operator()(i, j) = existing_grid(i, j);
                }
            }
        }

        // Override this method for determining which cell a continuous point belongs to
        virtual std::pair<std::size_t, std::size_t> getCellFromPoint(double x0, double x1) const override;

        Eigen::Vector2d getPointFromCell(double c0, double c1) const;

};

// Derive the HW4 ManipulatorCSConstructor class and override the missing method
class MyManipulatorCSConstructor : public amp::ManipulatorCSConstructor {
    public:
        // To make things easy, add the number of cells as a ctor param so you can easily play around with it
        MyManipulatorCSConstructor(std::size_t cells_per_dim) : m_cells_per_dim(cells_per_dim) {}

        // Override this method for computing all of the boolean collision values for each cell in the cspace
        virtual std::unique_ptr<amp::GridCSpace2D> construct(const amp::LinkManipulator2D& manipulator, const amp::Environment2D& env) override;

    private:
        std::size_t m_cells_per_dim;
};