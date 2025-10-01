#pragma once

// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"
#include "MyObstacle.h"
// Include the correct homework header
#include "hw/HW5.h"


class MyGDAlgorithm : public amp::GDAlgorithm {
	public:
		// Consider defining a class constructor to easily tune parameters, for example: 
		MyGDAlgorithm(double d_star, double zetta, double Q_star, double eta) :
			d_star(d_star),
			zetta(zetta),
			Q_star(Q_star),
			eta(eta) {}

		// Override this method to solve a given problem.
		virtual amp::Path2D plan(const amp::Problem2D& problem) override;
	private:
		double d_star, zetta, Q_star, eta;
		// Add additional member variables here...
};

class MyPotentialFunction : public amp::PotentialFunction2D {
	private:
		// gonna need some stuff to compute potential
		Eigen::Vector2d m_q_goal;
		std::vector<MyObstacle> m_obstacles;
		double m_d_star, m_zetta, m_Q_star, m_eta;
    public:
		// constructor to set params
		MyPotentialFunction(const Eigen::Vector2d& q_goal,
							const std::vector<MyObstacle>& obstacles, 
							double d_star, double zetta, double Q_star, double eta) :
			m_q_goal(q_goal), m_obstacles(obstacles), m_d_star(d_star), m_zetta(zetta), m_Q_star(Q_star), m_eta(eta)
			{}
		
		// Returns the potential function value (height) for a given 2D point. 
        virtual double operator()(const Eigen::Vector2d& q) const override {
            return getPotential(q);
        }

		virtual Eigen::Vector2d getGradient(const Eigen::Vector2d& q) const override;

		void addObstacle(const MyObstacle& ob) {  // method to quickly add an ob
			m_obstacles.push_back(ob);
		}

	private:
		double getPotential(const Eigen::Vector2d& q) const;

};