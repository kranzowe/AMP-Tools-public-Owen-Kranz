#pragma once

// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework headers
#include "hw/HW8.h"

// This is file is loaded from the shared/ directory
// Overwrite with your MySamplingBasedPlanners.h and MySamplingBasedPlanners.cpp from hw7
#include "MySamplingBasedPlanners.h" 



class MyPointAndDiscCollisionChecker : public MyPointCollisionChecker {
    public:
        MyPointAndDiscCollisionChecker(const std::vector<MyObstacle>& obstacles,
                                const std::vector<MyObstacle>& moving_circular_obstacles,
                                const std::vector<Eigen::VectorXd>& env_vertices);
        
        virtual bool inCollision(const Eigen::VectorXd& config) const override;
        bool edgeInCollision(const Eigen::VectorXd& config1, const Eigen::VectorXd& config2) const;
    
    private:
        const std::vector<MyObstacle>& m_obstacles;
        const std::vector<MyObstacle>& m_moving_circular_obstacles;
        const std::vector<Eigen::VectorXd>& m_env_vertices;
};


class MyCentralPlanner : public amp::CentralizedMultiAgentRRT {
    public:
        virtual amp::MultiAgentPath2D plan(const amp::MultiAgentProblem2D& problem) override; 
};


class MyDecentralPlanner : public amp::DecentralizedMultiAgentRRT {
    public:
        virtual amp::MultiAgentPath2D plan(const amp::MultiAgentProblem2D& problem) override;
};