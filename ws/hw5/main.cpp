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

    double dist = test_ob3.closestDistanceToq(Eigen::Vector2d(0.0, 0.0));
    std::cout << "dist should be 5 " << dist << std::endl;
    dist = test_ob3.closestDistanceToq(Eigen::Vector2d(-1.0, 4.0));
    std::cout << "dist should be sqrt(2) " << dist << std::endl;
    dist = test_ob3.closestDistanceToq(Eigen::Vector2d(-4.0, 5.0));
    std::cout << "dist should be 4 " << dist << std::endl;
    dist = test_ob3.closestDistanceToq(Eigen::Vector2d(3.0, 5.5));
    std::cout << "dist should be 1 " << dist << std::endl;
    


}

int main(int argc, char** argv) {

    // tests
    test_dist_to_ob();

    /* Include this line to have different randomized environments every time you run your code (NOTE: this has no affect on grade()) */
    amp::RNG::seed(amp::RNG::randiUnbounded());

    // Test your gradient descent algorithm on a random problem.
    double d_star = 3.0;
    double zetta = 1.0;
    double Q_star = 1.0;
    double eta = 1.0;

    MyGDAlgorithm algo(d_star, zetta, Q_star, eta);
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
    HW5::grade<MyGDAlgorithm>("nonhuman.biologic@myspace.edu", argc, argv, 1.0, 1.0, 1.0, 1.0);
    return 0;
}