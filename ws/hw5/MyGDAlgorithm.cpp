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
    double dt = 0.01;

    Eigen::Vector2d current_pos = problem.q_init;
    path.waypoints.push_back(problem.q_init);

    uint64_t i = 0;

    // adding some stuff to detect local minima...
    std::vector<double> dists_to_goal;

    while ((problem.q_goal - current_pos).norm() > epsilon && i < 1e5){
        i++;
        dists_to_goal.push_back((problem.q_goal - current_pos).norm());
        Eigen::Vector2d grad = potential.getGradient(current_pos);

        current_pos += -grad*dt;
        path.waypoints.push_back(current_pos);


        if (dists_to_goal.size() > 1001 && false){
            // time to detect if we are at a minima        
            // im sure there are a bunch of ways, but im going to use 
            // the standard deviation of the last 100 points
            // some AI help here to find stddev but using std dev is all me
            std::vector<double> recent_dists(dists_to_goal.end() - 1000, dists_to_goal.end());
            double mean = std::accumulate(recent_dists.begin(), recent_dists.end(), 0.0) / recent_dists.size();
    
            // Calculate variance
            double variance = std::accumulate(recent_dists.begin(), recent_dists.end(), 0.0,
                [mean](double acc, double x) {
                    return acc + (x - mean) * (x - mean);
                }) / recent_dists.size();
            
            // Return standard deviation
            double stdev = std::sqrt(variance);

            if (stdev < 1.0){
                std::cout << "Hit a local minimum"<< std::endl;
                // gonna backtrack to the start. Add an obstacle at the mean 
                // with rough radius of 2x the stdev
                std::vector<Eigen::Vector2d> recent_points(path.waypoints.end() - 1000, path.waypoints.end());
                Eigen::Vector2d mean_local_min = Eigen::Vector2d::Zero();
                for (const auto& point : recent_points) {
                    mean_local_min += point;
                }
                mean_local_min /= static_cast<double>(recent_points.size());
                        
                // path.waypoints.clear();
                path.waypoints.push_back(problem.q_init);
                current_pos = problem.q_init;

                // define a new, virtual obstacle
                MyObstacle virt_ob;
                std::vector<Eigen::Vector2d> vertices = {
                            mean_local_min + Eigen::Vector2d(1.0, 1.0),
                            mean_local_min + Eigen::Vector2d(-1.0, 1.0),
                            mean_local_min + Eigen::Vector2d(-1.0, -1.0),
                            mean_local_min + Eigen::Vector2d(1.0, -1.0),
                            };
                virt_ob.defineWithPoints(vertices);
                potential.addObstacle(virt_ob);

                
                dists_to_goal.clear();
            } 
        }
        


    }
    
    path.waypoints.push_back(problem.q_goal);
    return path;
}

double MyPotentialFunction::getPotential(const Eigen::Vector2d& q) const {



    double dist = (q - m_q_goal).norm() ;

    double attr = 0.0;

    if (dist <= m_d_star){
        attr = 0.5*m_zetta*(q - m_q_goal).squaredNorm();
    } else{
        attr = m_d_star*m_zetta*dist - (0.5*m_zetta*m_d_star*m_d_star);
    }

    double rep = 0.0;

    for (const auto& obstacle : m_obstacles) {
        auto [di, point] = obstacle.closestDistanceToq(q);

        if (di>m_Q_star){
            rep += 0.0;
        } else if (di < 1e-2){
            rep +=100.0;
        }else{
            rep += (0.5)*(m_eta)*(((1/di) - (1/m_Q_star))*((1/di) - (1/m_Q_star)));
        }
    };

    if (rep > 100.0){
        rep = 100.0;
    }
    return attr + rep;
}


Eigen::Vector2d MyPotentialFunction::getGradient(const Eigen::Vector2d& q) const {

    double dist = (q - m_q_goal).norm();

    Eigen::Vector2d delta_attr = Eigen::Vector2d::Zero();

    if (dist <= m_d_star){
        delta_attr = m_zetta*(q - m_q_goal);
    } else{
        delta_attr = (m_d_star*m_zetta*(q - m_q_goal)) / dist;
    }

    Eigen::Vector2d delta_rep = Eigen::Vector2d::Zero();

    for (const auto& obstacle : m_obstacles) {
        auto [di, c] = obstacle.closestDistanceToq(q);
        auto [di_cent, cent] = obstacle.distanceToCentroid(q);
        if (di<=m_Q_star){
            if (di > 1e-2){
                Eigen::Vector2d delta_d = (q - c) / di;
                delta_rep += (m_eta)*((1/m_Q_star) - (1/di))*(delta_d/(di*di));

                Eigen::Vector2d delta_d_cent = (q - cent) / di_cent;
                delta_rep += (m_eta)*((1/m_Q_star) - (1/di_cent))*(delta_d_cent/(di_cent*di_cent));
            }
        }
    };
    if (delta_rep.norm() > 10.0){
        delta_rep = (delta_rep/delta_rep.norm()) *10.0;
    }
    return delta_attr + delta_rep;


    }