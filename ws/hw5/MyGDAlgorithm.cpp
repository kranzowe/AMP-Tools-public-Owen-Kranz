#include "MyGDAlgorithm.h"
#include "MyObstacle.h"
#include "AMPCore.h"

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
    //define the boundaries as obstacles
    std::vector<Eigen::Vector2d> vertices1_env = {
        Eigen::Vector2d(problem.x_min, problem.y_min),
        Eigen::Vector2d(problem.x_min, problem.y_max),
        Eigen::Vector2d(problem.x_min - 1.0, problem.y_max),
        Eigen::Vector2d(problem.x_min - 1.0, problem.y_min),
    };

    MyObstacle env_ob1;
    env_ob1.defineWithPoints(vertices1_env);
    my_obstacles.push_back(env_ob1);

    std::vector<Eigen::Vector2d> vertices2_env = {
        Eigen::Vector2d(problem.x_min, problem.y_min),
        Eigen::Vector2d(problem.x_min, problem.y_min - 1.0),
        Eigen::Vector2d(problem.x_max, problem.y_min - 1.0),
        Eigen::Vector2d(problem.x_max, problem.y_min),
    };

    MyObstacle env_ob2;
    env_ob2.defineWithPoints(vertices2_env);
    my_obstacles.push_back(env_ob2);

    std::vector<Eigen::Vector2d> vertices3_env = {
        Eigen::Vector2d(problem.x_max, problem.y_min),
        Eigen::Vector2d(problem.x_max + 1.0, problem.y_min),
        Eigen::Vector2d(problem.x_max + 1.0, problem.y_max),
        Eigen::Vector2d(problem.x_max, problem.y_max),
    };

    MyObstacle env_ob3;
    env_ob3.defineWithPoints(vertices3_env);
    my_obstacles.push_back(env_ob3);

    std::vector<Eigen::Vector2d> vertices4_env = {
        Eigen::Vector2d(problem.x_min, problem.y_max),
        Eigen::Vector2d(problem.x_max, problem.y_max),
        Eigen::Vector2d(problem.x_max, problem.y_max + 1.0),
        Eigen::Vector2d(problem.x_min, problem.y_max + 1.0),
    };

    MyObstacle env_ob4;
    env_ob4.defineWithPoints(vertices4_env);
    my_obstacles.push_back(env_ob4);




    MyPotentialFunction potential(problem.q_goal,
							my_obstacles, 
							d_star, zetta, Q_star, eta, Q_star_cent, eta_cent);

    double epsilon = 0.25;
    double dt = 0.01;

    Eigen::Vector2d current_pos = problem.q_init;
    path.waypoints.push_back(problem.q_init);

    uint64_t i = 0;

    // adding some stuff to detect local minima...
    std::vector<double> dists_to_goal;

    bool random_walk_time = false;
    uint64_t i_random_walk_start = 0;

    // normal dist for random walk 
    std::random_device rd;
    std::mt19937 gen(rd());

    // Create normal distribution (mean=0.0, std_dev=1.0)
    std::normal_distribution<double> normal_dist(0.0, 0.1);

    while ((problem.q_goal - current_pos).norm() > epsilon && i < 4e5){
        i++;
        dists_to_goal.push_back((problem.q_goal - current_pos).norm());

        if (!random_walk_time){
            Eigen::Vector2d grad = potential.getGradient(current_pos);

            current_pos += -grad*dt;
            path.waypoints.push_back(current_pos);
        }else{
            // time to random walk for 10 steps
            if (i_random_walk_start == 0){
                i_random_walk_start = i;
            }else if (i - i_random_walk_start > 20){
                i_random_walk_start = 0;
                random_walk_time = false;
    
            }else{
                // select random walk positions from covariance
                double x_sample = normal_dist(gen);
                double y_sample = normal_dist(gen);

                Eigen::Vector2d point(x_sample, y_sample);

                // check collision 
                bool collision = false;
                for (const auto& ob : my_obstacles){
                    if (ob.collisionCheckAlongLine(current_pos, current_pos + point)){
                        collision = true;
                        break;
                    }}
                if (!collision){
                    current_pos += point;
                }
            }
        }

        if (dists_to_goal.size() > 1001 && !random_walk_time){
            // time to detect if we are at a minima        
            // im sure there are a bunch of ways, but im going to use 
            // the standard deviation of the last 1000 points
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

            if (stdev < 0.01){
                // gonna backtrack to the start. Add an obstacle at the mean 
                random_walk_time = true;

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
                delta_rep += (m_eta_cent)*((1/m_Q_star_cent) - (1/di_cent))*(delta_d_cent/(di_cent*di_cent));
            }
        }
    };

    Eigen::Vector2d total = delta_attr + delta_rep;
    if (total.norm() > 25.0){
        total = (total/total.norm()) *25.0;
    }
    return total;


    }