#pragma once

// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework headers
#include "hw/HW9.h"

// Global containers for collision checking during propagation
namespace amp {
    extern std::vector<std::vector<Eigen::VectorXd>> points_to_check;
    extern std::vector<Eigen::VectorXd> sub_states;
}

class MyKinoRRT : public amp::KinodynamicRRT {
    public:
        MyKinoRRT(double dt = 0.1, int num_control_samples = 10, int max_iterations = 5000, double stop_radius = 0.25);
        virtual amp::KinoPath plan(const amp::KinodynamicProblem2D& problem, amp::DynamicAgent& agent) override;

    private:
        double m_dt;
        int m_num_control_samples;
        int m_max_iterations;
        double m_stop_radius;
};  


class MySingleIntegrator : public amp::DynamicAgent {
    public:
        virtual void propagate(Eigen::VectorXd& state, Eigen::VectorXd& control, double dt) override;

    private:
        Eigen::Matrix<double,2,2> A(Eigen::VectorXd state);
        Eigen::Matrix<double,2,2> B(Eigen::VectorXd state);
        
    template<typename FuncA, typename FuncB>
    void RK4Integrate(
        Eigen::VectorXd& state, 
        const Eigen::VectorXd& control, 
        double dt,
        FuncA getA,
        FuncB getB,
        bool track_collision_points = false,
        double safety_buffer = 0.001
    ) {
        if (track_collision_points) {
            amp::points_to_check.clear();
            amp::sub_states.clear();
        }

        double sub_dt = 0.001;
        double t = 0;
        
        // Process with fixed substeps
        while (t < dt - sub_dt/2) {  // Use t < dt - epsilon to avoid floating point issues
            // RK4 integration
            Eigen::VectorXd k1 = getA(state) * state + getB(state) * control;
            Eigen::VectorXd k2 = getA(state + k1 * sub_dt/2) * (state + k1 * sub_dt/2) + 
                                getB(state + k1 * sub_dt/2) * control;
            Eigen::VectorXd k3 = getA(state + k2 * sub_dt/2) * (state + k2 * sub_dt/2) + 
                                getB(state + k2 * sub_dt/2) * control;
            Eigen::VectorXd k4 = getA(state + k3 * sub_dt) * (state + k3 * sub_dt) + 
                                getB(state + k3 * sub_dt) * control;

            Eigen::VectorXd d_state = ((k1 + 2.0*k2 + 2.0*k3 + k4) / 6.0) * sub_dt;
            
            if (track_collision_points) {
                // Create corners at NEXT position with safety buffer
                std::vector<Eigen::VectorXd> corner_points;
                Eigen::Vector2d top_right(state[0] + d_state[0] + safety_buffer, 
                                        state[1] + d_state[1] + safety_buffer);
                Eigen::Vector2d top_left(state[0] + d_state[0] - safety_buffer, 
                                        state[1] + d_state[1] + safety_buffer);
                Eigen::Vector2d bottom_right(state[0] + d_state[0] + safety_buffer, 
                                            state[1] + d_state[1] - safety_buffer);
                Eigen::Vector2d bottom_left(state[0] + d_state[0] - safety_buffer, 
                                        state[1] + d_state[1] - safety_buffer);
                
                corner_points.push_back(top_right);
                corner_points.push_back(top_left);
                corner_points.push_back(bottom_right);
                corner_points.push_back(bottom_left);
                
                amp::points_to_check.push_back(corner_points);
                amp::sub_states.push_back(state);  // Save CURRENT state
            }

            state += d_state;
            t += sub_dt;
        }
        
        // Handle remaining time if any
        double remaining = dt - t;
        if (remaining > 1e-9) {
            Eigen::VectorXd k1 = getA(state) * state + getB(state) * control;
            Eigen::VectorXd k2 = getA(state + k1 * remaining/2) * (state + k1 * remaining/2) + 
                                getB(state + k1 * remaining/2) * control;
            Eigen::VectorXd k3 = getA(state + k2 * remaining/2) * (state + k2 * remaining/2) + 
                                getB(state + k2 * remaining/2) * control;
            Eigen::VectorXd k4 = getA(state + k3 * remaining) * (state + k3 * remaining) + 
                                getB(state + k3 * remaining) * control;

            Eigen::VectorXd d_state = ((k1 + 2.0*k2 + 2.0*k3 + k4) / 6.0) * remaining;
            
            if (track_collision_points) {
                std::vector<Eigen::VectorXd> corner_points;
                Eigen::Vector2d top_right(state[0] + d_state[0] + safety_buffer, 
                                        state[1] + d_state[1] + safety_buffer);
                Eigen::Vector2d top_left(state[0] + d_state[0] - safety_buffer, 
                                        state[1] + d_state[1] + safety_buffer);
                Eigen::Vector2d bottom_right(state[0] + d_state[0] + safety_buffer, 
                                            state[1] + d_state[1] - safety_buffer);
                Eigen::Vector2d bottom_left(state[0] + d_state[0] - safety_buffer, 
                                        state[1] + d_state[1] - safety_buffer);
                
                corner_points.push_back(top_right);
                corner_points.push_back(top_left);
                corner_points.push_back(bottom_right);
                corner_points.push_back(bottom_left);
                
                amp::points_to_check.push_back(corner_points);
                amp::sub_states.push_back(state);
            }
            
            state += d_state;
        }
    }
};

class MyFirstOrderUnicycle : public amp::DynamicAgent {
    public:
        virtual void propagate(Eigen::VectorXd& state, Eigen::VectorXd& control, double dt) override;
        
    private:
        Eigen::Matrix<double,3,3> A(Eigen::VectorXd state);
        Eigen::Matrix<double,3,2> B(Eigen::VectorXd state);
        
        template<typename FuncA, typename FuncB>
        void RK4Integrate(
            Eigen::VectorXd& state, 
            const Eigen::VectorXd& control, 
            double dt,
            FuncA getA,
            FuncB getB,
            bool track_collision_points = false,
            double safety_buffer = 0.1
        ) {
            if (track_collision_points) {
                amp::points_to_check.clear();
                amp::sub_states.clear();
            }

            double sub_dt = 0.001;
            double t = 0;

            while (t + sub_dt <= dt) {
                Eigen::VectorXd k1 = getA(state) * state + getB(state) * control;
                Eigen::VectorXd k2 = getA(state + k1 * sub_dt/2) * (state + k1 * sub_dt/2) + 
                                     getB(state + k1 * sub_dt/2) * control;
                Eigen::VectorXd k3 = getA(state + k2 * sub_dt/2) * (state + k2 * sub_dt/2) + 
                                     getB(state + k2 * sub_dt/2) * control;
                Eigen::VectorXd k4 = getA(state + k3 * sub_dt) * (state + k3 * sub_dt) + 
                                     getB(state + k3 * sub_dt) * control;

                Eigen::VectorXd d_state = ((k1 + 2.0*k2 + 2.0*k3 + k4) / 6.0) * sub_dt;
                
                if (track_collision_points) {
                    std::vector<Eigen::VectorXd> corner_points;
                    Eigen::Vector2d top_right(state[0] + d_state[0] + safety_buffer, 
                                              state[1] + d_state[1] + safety_buffer);
                    Eigen::Vector2d top_left(state[0] + d_state[0] - safety_buffer, 
                                             state[1] + d_state[1] + safety_buffer);
                    Eigen::Vector2d bottom_right(state[0] + d_state[0] + safety_buffer, 
                                                 state[1] + d_state[1] - safety_buffer);
                    Eigen::Vector2d bottom_left(state[0] + d_state[0] - safety_buffer, 
                                                state[1] + d_state[1] - safety_buffer);
                    
                    corner_points.push_back(top_right);
                    corner_points.push_back(top_left);
                    corner_points.push_back(bottom_right);
                    corner_points.push_back(bottom_left);
                    
                    amp::points_to_check.push_back(corner_points);
                    amp::sub_states.push_back(state);
                }

                state += d_state;
                t += sub_dt;
            }

            double last_dt = dt - t;
            if (last_dt > 1e-9) {
                Eigen::VectorXd k1 = getA(state) * state + getB(state) * control;
                Eigen::VectorXd k2 = getA(state + k1 * last_dt/2) * (state + k1 * last_dt/2) + 
                                     getB(state + k1 * last_dt/2) * control;
                Eigen::VectorXd k3 = getA(state + k2 * last_dt/2) * (state + k2 * last_dt/2) + 
                                     getB(state + k2 * last_dt/2) * control;
                Eigen::VectorXd k4 = getA(state + k3 * last_dt) * (state + k3 * last_dt) + 
                                     getB(state + k3 * last_dt) * control;

                Eigen::VectorXd d_state = ((k1 + 2.0*k2 + 2.0*k3 + k4) / 6.0) * last_dt;
                
                if (track_collision_points) {
                    std::vector<Eigen::VectorXd> corner_points;
                    Eigen::Vector2d top_right(state[0] + d_state[0] + safety_buffer, 
                                              state[1] + d_state[1] + safety_buffer);
                    Eigen::Vector2d top_left(state[0] + d_state[0] - safety_buffer, 
                                             state[1] + d_state[1] + safety_buffer);
                    Eigen::Vector2d bottom_right(state[0] + d_state[0] + safety_buffer, 
                                                 state[1] + d_state[1] - safety_buffer);
                    Eigen::Vector2d bottom_left(state[0] + d_state[0] - safety_buffer, 
                                                state[1] + d_state[1] - safety_buffer);
                    
                    corner_points.push_back(top_right);
                    corner_points.push_back(top_left);
                    corner_points.push_back(bottom_right);
                    corner_points.push_back(bottom_left);
                    
                    amp::points_to_check.push_back(corner_points);
                    amp::sub_states.push_back(state);
                }
                
                state += d_state;
            }
        }
};

class MySecondOrderUnicycle : public amp::DynamicAgent {
    public:
        virtual void propagate(Eigen::VectorXd& state, Eigen::VectorXd& control, double dt) override {};
};

class MySimpleCar : public amp::DynamicAgent {
    public:
        virtual void propagate(Eigen::VectorXd& state, Eigen::VectorXd& control, double dt) override {};
};