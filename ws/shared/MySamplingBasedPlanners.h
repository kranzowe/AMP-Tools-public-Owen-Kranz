#pragma once

// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"
#include "MyObstacle.h"
#include "MyAStar.h"

// Include the correct homework headers
#include "hw/HW7.h"

// NOTE on AI Use, I used AI to help structure the new header and .cpp file to match
// the given hint



class DistanceMetric {
    public:
        virtual double distance(const Eigen::VectorXd& vec1, const Eigen::VectorXd& vec2) const = 0;
        virtual ~DistanceMetric() = default;
};

// define l2 since im guessing we will need others later
class L2Distance : public DistanceMetric {
    public:
        virtual double distance(const Eigen::VectorXd& vec1, const Eigen::VectorXd& vec2) const override {
            return (vec1 - vec2).norm();
        }
};

class MyPointCollisionChecker : public amp::ConfigurationSpace {
    public:
        MyPointCollisionChecker(const std::vector<MyObstacle>& obstacles, 
                               const std::vector<Eigen::VectorXd>& env_vertices);
        
        virtual bool inCollision(const Eigen::VectorXd& config) const override;
        virtual bool edgeInCollision(const Eigen::VectorXd& config1, const Eigen::VectorXd& config2) const;
        
        // Remove override since getBounds() is not in the base class
        std::pair<Eigen::VectorXd, Eigen::VectorXd> getBounds() const;
        
        static Eigen::VectorXd calculateLowerBounds(const std::vector<Eigen::VectorXd>& env_vertices);
        static Eigen::VectorXd calculateUpperBounds(const std::vector<Eigen::VectorXd>& env_vertices);
    
    protected:
        const std::vector<MyObstacle>& m_obstacles;
        const std::vector<Eigen::VectorXd>& m_env_vertices;
};


class MyPointAndDiscCollisionChecker : public MyPointCollisionChecker {
    public:
        MyPointAndDiscCollisionChecker(const std::vector<MyObstacle>& obstacles,
                                const std::vector<MyObstacle>& moving_circular_obstacles,
                                const std::vector<Eigen::VectorXd>& env_vertices);
        
        virtual bool inCollision(const Eigen::VectorXd& config) const override;
        virtual bool edgeInCollision(const Eigen::VectorXd& config1, const Eigen::VectorXd& config2) const override;
    
    private:
        const std::vector<MyObstacle>& m_moving_circular_obstacles;
};


// Fixed: Renamed and properly inherit from amp::SearchHeuristic
struct MyLookupSearchHeuristic : public amp::SearchHeuristic {
    /// @brief Get the heuristic value stored in `heuristic_values`. 
    /// @param node Node to get the heuristic value h(node) for. 
    /// @return Heuristic value
    virtual double operator()(amp::Node node) const override {
        return heuristic_values.at(node);
    }

    /// @brief Store the heuristic values for each node in a map
    std::map<amp::Node, double> heuristic_values; 
};

class GenericPRM {
    public:
        GenericPRM(int num_samples = 1000, double connection_radius = 1.0);
        
        amp::Path plan(const Eigen::VectorXd& init_state, 
                       const Eigen::VectorXd& goal_state, 
                       const amp::ConfigurationSpace& collision_checker,
                       const DistanceMetric& metric);

        // for viz
        std::shared_ptr<amp::Graph<double>> getLastGraph() const;
        std::map<amp::Node, Eigen::Vector2d> getLastNodes() const;
    
    private:
        int m_num_samples;
        double m_connection_radius;
        
        // viz storage
        std::shared_ptr<amp::Graph<double>> m_last_graph;
        std::map<amp::Node, Eigen::Vector2d> m_last_nodes;
};

class GenericRRT {
    public:
        GenericRRT(int max_iterations = 5000, double step_size = 0.5, double goal_bias = 0.1);
        
        amp::Path plan(const Eigen::VectorXd& init_state, 
                       const Eigen::VectorXd& goal_state, 
                       const amp::ConfigurationSpace& collision_checker,
                       const DistanceMetric& metric);

                       
        // for viz
        std::shared_ptr<amp::Graph<double>> getLastGraph() const;
        std::map<amp::Node, Eigen::Vector2d> getLastNodes() const;
    private:
        int m_max_iterations;
        double m_step_size;
        double m_goal_bias;

        // viz storage
        std::shared_ptr<amp::Graph<double>> m_last_graph;
        std::map<amp::Node, Eigen::Vector2d> m_last_nodes;
};


class MyPRM : public amp::PRM2D {
    public:
        MyPRM();
        MyPRM(int num_samples, double connection_radius);
        
        virtual amp::Path2D plan(const amp::Problem2D& problem) override;
        
        std::shared_ptr<amp::Graph<double>> getLastGraph() const {
            return m_generic_prm.getLastGraph();
        }
        
        std::map<amp::Node, Eigen::Vector2d> getLastNodes() const {
            return m_generic_prm.getLastNodes();
        }
        
    private:
        GenericPRM m_generic_prm;
};

class MyRRT : public amp::GoalBiasRRT2D {
    public:
        MyRRT();
        MyRRT(int max_iterations, double step_size, double goal_bias); // for passing in vals
        virtual amp::Path2D plan(const amp::Problem2D& problem) override;
        
        std::shared_ptr<amp::Graph<double>> getLastGraph() const {
            return m_generic_rrt.getLastGraph();
        }
        
        std::map<amp::Node, Eigen::Vector2d> getLastNodes() const {
            return m_generic_rrt.getLastNodes();
        }
        
    private:
        GenericRRT m_generic_rrt;
};