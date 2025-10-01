#include "MyGDAlgorithm.h"
#include "MyObstacle.h"

// Implement your plan method here, similar to HW2:
amp::Path2D MyGDAlgorithm::plan(const amp::Problem2D& problem) {
    amp::Path2D path;

    // initialize, from the problem, all the obstacles
    std::vector<MyObstacle> my_obstacles;
    for (const auto& obstacle : problem.obstacles) {
        MyObstacle my_ob;
        my_ob.defineWithPoints(obstacle.verticesCCW());
        my_obstacles.push_back(my_ob);
    };

    MyPotentialFunction potential(problem.q_goal,
							my_obstacles, 
							d_star, zetta, Q_star, eta);

    double epsilon = 0.25;
    double dt = 0.1;

    Eigen::Vector2d current_pos = problem.q_init;
    path.waypoints.push_back(problem.q_init);

    uint64_t i = 0;

    while ((problem.q_goal - current_pos).norm() > epsilon && i < 1e5){
        i++;
        Eigen::Vector2d grad = potential.getGradient(current_pos);

        current_pos += -grad*dt;
        path.waypoints.push_back(current_pos);

    }
    
    path.waypoints.push_back(problem.q_goal);
    return path;
}

double MyPotentialFunction::getPotential(const Eigen::Vector2d& q) const {



    double dist = (q - m_q_goal).norm() ;

    double attr;

    if (dist <= m_d_star){
        attr = 0.5*m_zetta*(q - m_q_goal).squaredNorm();
    } else{
        attr = m_d_star*m_zetta*dist - (0.5*m_zetta*m_d_star*m_d_star);
    }

    double rep;

    for (const auto& obstacle : my_obstacles) {
        MyObstacle my_ob;
        my_ob.defineWithPoints(obstacle.verticesCCW());
        my_obstacles.push_back(my_ob);
    };


    return attr;
}


Eigen::Vector2d MyPotentialFunction::getGradient(const Eigen::Vector2d& q) const {

    double dist = (q - m_q_goal).norm();

    Eigen::Vector2d delta_attr;

    if (dist <= m_d_star){
        delta_attr = m_zetta*(q - m_q_goal);
    } else{
        delta_attr = (m_d_star*m_zetta*(q - m_q_goal)) / dist;
    }

    return delta_attr;


    }