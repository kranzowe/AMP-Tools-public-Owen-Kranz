// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework header
#include "hw/HW5.h"

// Include any custom headers you created in your workspace
#include "MyGDAlgorithm.h"

using namespace amp;

void test_dist_to_ob(){

    std::vector<Eigen::Vector2d> vertices2 = {
        Eigen::Vector2d(2.0, 5.0),
        Eigen::Vector2d(2.0, 6.0),
        Eigen::Vector2d(0.0, 6.0),
        Eigen::Vector2d(0.0, 5.0),
    };
    MyObstacle test_ob3;
    test_ob3.defineWithPoints(vertices2);

    auto [dist, point] = test_ob3.closestDistanceToq(Eigen::Vector2d(0.0, 0.0));
    std::cout << "dist should be 5 " << dist << std::endl;
    auto [dist2, point2] = test_ob3.closestDistanceToq(Eigen::Vector2d(-1.0, 4.0));
    std::cout << "dist should be sqrt(2) " << dist2 << std::endl;
    auto [dist3, point3] = test_ob3.closestDistanceToq(Eigen::Vector2d(-4.0, 5.0));
    std::cout << "dist should be 4 " << dist3 << std::endl;
    auto [dist4, point4] = test_ob3.closestDistanceToq(Eigen::Vector2d(3.0, 5.5));
    std::cout << "dist should be 1 " << dist4 << std::endl;

    auto [dist5, point5] = test_ob3.distanceToCentroid(Eigen::Vector2d(1.0, 0.0));
    std::cout << "dist should be 5.5 " << dist5 << std::endl;
    


}

int main(int argc, char** argv) {

    // tests
    test_dist_to_ob();

    /* Include this line to have different randomized environments every time you run your code (NOTE: this has no affect on grade()) */
    amp::RNG::seed(amp::RNG::randiUnbounded());


    // Test your gradient descent algorithm on a random problem.
    double d_star = 0.622678;
    double zetta =  4.36968;
    double Q_star = 4.13851;
    double eta = 1.03226;

    MyGDAlgorithm algo(d_star, zetta, Q_star, eta);
    
    // Problem2D prob = HW5::getWorkspace1();
    // amp::Path2D path = algo.plan(prob);
    // //     // Check your path to make sure that it does not collide with the environment 
    // bool success = HW5::check(path, prob);

    // int max_success = 0;
    
    // for (int j = 0; j < 50; j ++){
    //     int success_count = 0;

    //     double d_star = amp::RNG::randf(0.01, 5.0);
    //     double zetta = amp::RNG::randf(0.01, 5.0);   
    //     double Q_star = amp::RNG::randf(0.01, 5.0);
    //     double eta = amp::RNG::randf(0.01, 5.0);
    //     // double stdev = amp::RNG::randf(0.01, 5.0);
    //     // double virtual_ob_size = amp::RNG::randf(0.01, 5.0);

    //     MyGDAlgorithm algo(d_star, zetta, Q_star, eta);
        

    //     for (int i = 0; i < 40; i ++){
    //         Path2D path;
    //         Problem2D prob;
    //         bool success = HW5::generateAndCheck(algo, path, prob, false);
    //         if (success){
    //             success_count ++;
    //         }
    //     }

    //     if (success_count > max_success){
    //         max_success = success_count;
    //         std::cout <<"new max" << success_count<< ", "  << d_star << ", " << zetta << ", " << Q_star << ", " << eta << std::endl;
    //     }
    // }

    Path2D path;
    Problem2D prob;
    bool success = HW5::generateAndCheck(algo, path, prob);
    Visualizer::makeFigure(prob, path);

    // Visualize your potential function
    std::vector<MyObstacle> my_obstacles;
    for (const auto& obstacle : prob.obstacles) {
        MyObstacle my_ob;
        my_ob.defineWithPoints(obstacle.verticesCCW());
        my_obstacles.push_back(my_ob);
    };
    Visualizer::makeFigure(MyPotentialFunction{prob.q_goal, my_obstacles, d_star, zetta, Q_star, eta}, prob, 30);
    Visualizer::saveFigures(true, "hw5_figs");
    
    // Arguments following argv correspond to the constructor arguments of MyGDAlgorithm:
    HW5::grade<MyGDAlgorithm>("nonhuman.biologic@myspace.edu", argc, argv, d_star, zetta, Q_star, eta);
    return 0;
}