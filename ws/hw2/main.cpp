// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework header
#include "hw/HW2.h"

// Include any custom headers you created in your workspace
#include "MyBugAlgorithm.h"
#include "Primative.h"
#include "MyObstacle.h"
#include "MyAgent.h"

using namespace amp;

int main(int argc, char** argv) {
    /*TESTS TESTS TESTS TESTS */
    // tests that this primative thing works
    LinearPrimative test_prim;
    test_prim.point_a = Eigen::Vector2d(0.0, 0.0);
    test_prim.point_b = Eigen::Vector2d(1.0, 1.0);
    std::cout << "test point should be >0: " << test_prim.evaluatePoint(Eigen::Vector2d(1.0, 0.0)) << std::endl;
    std::cout << "test point should be 0: " << test_prim.evaluatePoint(Eigen::Vector2d(1.0, 1.0)) << std::endl;
    std::cout << "test point should be <0: " << test_prim.evaluatePoint(Eigen::Vector2d(0.0, 1.0)) << std::endl;

    LinearPrimative test_prim2;
    test_prim2.point_a = Eigen::Vector2d(5, -5);
    test_prim2.point_b = Eigen::Vector2d(5, 5);
    std::cout << "test point should be <0: " << test_prim2.evaluatePoint(Eigen::Vector2d(0, 0)) << std::endl;
    std::cout << "test point should be 0: " << test_prim2.evaluatePoint(Eigen::Vector2d(5, 0)) << std::endl;
    std::cout << "test point should be >0: " << test_prim2.evaluatePoint(Eigen::Vector2d(5.0001, 0)) << std::endl;


    // testing an obstacle
    MyObstacle test_ob; // gonna make a square
    LinearPrimative prim1;
    prim1.point_a = Eigen::Vector2d(5.0, 5.0);
    prim1.point_b = Eigen::Vector2d(-5.0, 5.0);
    LinearPrimative prim2;
    prim2.point_a = Eigen::Vector2d(-5.0, 5.0);
    prim2.point_b = Eigen::Vector2d(-5.0, -5.0);
    LinearPrimative prim3;
    prim3.point_a = Eigen::Vector2d(-5.0, -5.0);
    prim3.point_b = Eigen::Vector2d(5.0, -5.0);
    LinearPrimative prim4;
    prim4.point_a = Eigen::Vector2d(5.0, -5.0);
    prim4.point_b = Eigen::Vector2d(5.0, 5.0);

    test_ob.primatives.push_back(prim1);
    test_ob.primatives.push_back(prim2);
    test_ob.primatives.push_back(prim3);
    test_ob.primatives.push_back(prim4);

    std::cout << "test collision should be TRUE: " << test_ob.collisionCheck(Eigen::Vector2d(0.0, 0.0)) << std::endl;
    std::cout << "test collision should be TRUE: " << test_ob.collisionCheck(Eigen::Vector2d(5.0, 5.0)) << std::endl;
    std::cout << "test collision should be TRUE: " << test_ob.collisionCheck(Eigen::Vector2d(-5.0, -5.0)) << std::endl;
    std::cout << "test collision should be FALSE: " << test_ob.collisionCheck(Eigen::Vector2d(15.00001, 0.0)) << std::endl;
    std::cout << "test collision should be FALSE: " << test_ob.collisionCheck(Eigen::Vector2d(-15.000001, -5.000001)) << std::endl;
    std::cout << "test collision should be FALSE: " << test_ob.collisionCheck(Eigen::Vector2d(5.000001, 5.000001)) << std::endl;
    std::cout << "test collision should be FALSE: " << test_ob.collisionCheck(Eigen::Vector2d(-5.00001, 0.0)) << std::endl;

    // tests obstacle with vertice definition
    std::vector<Eigen::Vector2d> vertices = {
        Eigen::Vector2d(0.0, 0.0),
        Eigen::Vector2d(1.0, 0.0),
        Eigen::Vector2d(0.5, 1.0)
    };
    MyObstacle test_ob2;
    test_ob2.defineWithPoints(vertices);
    std::cout << "test collision should be TRUE: " << test_ob2.collisionCheck(Eigen::Vector2d(0.0, 0.0)) << std::endl;
    std::cout << "test collision should be TRUE: " << test_ob2.collisionCheck(Eigen::Vector2d(0.5, 0.5)) << std::endl;
    std::cout << "test collision should be TRUE: " << test_ob2.collisionCheck(Eigen::Vector2d(0.5, 1.0)) << std::endl;
    std::cout << "test collision should be FALSE: " << test_ob2.collisionCheck(Eigen::Vector2d(0.05, -0.001)) << std::endl;
    std::cout << "test collision should be FALSE: " << test_ob2.collisionCheck(Eigen::Vector2d(0.1, 0.2001)) << std::endl;
    std::cout << "test collision should be FALSE: " << test_ob2.collisionCheck(Eigen::Vector2d(0.5, 1.0001)) << std::endl;

    // testing the agent and its basic functions
    PointAgent test_agent;
    // test rotate
    test_agent.heading = Eigen::Vector2d(1.0, 1.0);
    test_agent.rotate(45.0);
    std::cout << "test point should be 0.0, 1.0: " << test_agent.heading.transpose() << std::endl;
    test_agent.rotate(-45.0);
    std::cout << "test point should be 0.707, 0.707: " << test_agent.heading.transpose() << std::endl;

    // test rotate until free
    // gonna use that old test_ob which is a 2d square
    std::vector test_obs = {test_ob};
    double test_eps = 1e-4;
    // double test_dtheta = 1e-5;
    // test_agent.x = Eigen::Vector2d(-5.0 - test_eps/3.0, -5.0 - test_eps/3.0);
    // bool converged = test_agent.rotateToCircumnavigateRH(test_obs, test_dtheta, test_eps);
    // std::cout << "converge?" << converged <<"heading should close to -0.707, 0.707: " << test_agent.heading.transpose() << std::endl;
    // test_agent.x = Eigen::Vector2d(0, 5.0 + test_eps/3.0);
    // converged = test_agent.rotateToCircumnavigateRH(test_obs, test_dtheta, test_eps);
    // std::cout <<"converge? " << converged <<" heading should close to 1.0, 0.0: " << test_agent.heading.transpose() << std::endl;
    // test_agent.x = Eigen::Vector2d(-5.0 - test_eps/3.0, 1.0);
    // converged = test_agent.rotateToCircumnavigateRH(test_obs, test_dtheta, test_eps);
    // std::cout <<"converge? " << converged <<" heading should close to 0.0, 1.0: " << test_agent.heading.transpose() << std::endl;

    // now gonna add an object to make it convex :o. 

    std::vector<Eigen::Vector2d> vertices2 = {
        Eigen::Vector2d(2.0, 5.0),
        Eigen::Vector2d(2.0, 6.0),
        Eigen::Vector2d(0.0, 6.0),
        Eigen::Vector2d(0.0, 5.0),
    };
    // MyObstacle test_ob3;
    // test_ob3.defineWithPoints(vertices2);

    // test_obs = {test_ob, test_ob3};
    // test_agent.x = Eigen::Vector2d(0.0 - test_eps/3.0, 5.0 + test_eps/3.0);
    // converged = test_agent.rotateToCircumnavigateRH(test_obs, test_dtheta, test_eps);
    // std::cout << "converge?" << converged <<"heading should close to 0.0, 1.0: " << test_agent.heading.transpose() << std::endl;
    // test_agent.x = Eigen::Vector2d(2.0 + test_eps/3.0, 5.0 + test_eps/3.0);
    // converged = test_agent.rotateToCircumnavigateRH(test_obs, test_dtheta, test_eps);
    // std::cout << "converge?" << converged <<"heading should close to 1.0, 0.0: " << test_agent.heading.transpose() << std::endl;


    // tests moving puts you just outside an object
    bool collided;
    test_agent.x = Eigen::Vector2d(5.1, 5.1);
    test_agent.heading = Eigen::Vector2d(1.0, 1.0);
    collided = test_agent.move(test_obs, 1.0, test_eps);
    std::cout <<  "collision FALSE: "<< collided <<"test point should be 6.1, 6.1: " << test_agent.x.transpose() << std::endl;
    test_agent.x = Eigen::Vector2d(-10, -10);
    test_agent.heading = Eigen::Vector2d(-1.0, -1.0);
    collided = test_agent.move(test_obs, 0.1, test_eps);
    std::cout <<  "collision FALSE: "<< collided <<"test point should be -10.1414,-10.1414: " << test_agent.x.transpose() << std::endl;
    test_agent.x = Eigen::Vector2d(0, -6);
    test_agent.heading = Eigen::Vector2d(0.0, 2.0);
    collided = test_agent.move(test_obs, 1.0, test_eps);
    std::cout << "collision TRUE: "<< collided <<"test point should be 0.0, -5.00001 " << test_agent.x.transpose() << std::endl;
    test_agent.x = Eigen::Vector2d(3, 6);
    test_agent.heading = Eigen::Vector2d(-1.0, -1.0);
    collided = test_agent.move(test_obs, 1.0, test_eps);
    std::cout << "collision TRUE: "<< collided <<"test point should be 2.00001, 5.00001 " << test_agent.x.transpose() << std::endl;
    
    /*END TESTS END TESTS END TESTS*/


    /*    Include this line to have different randomized environments every time you run your code (NOTE: this has no affect on grade()) */
    amp::RNG::seed(amp::RNG::randiUnbounded());

    /*    Randomly generate the problem     */ 

    // Use WO1 from Exercise 2
    Problem2D problem = HW2::getWorkspace1();

    // Use WO1 from Exercise 2
    /*
    Problem2D problem = HW2::getWorkspace2();
    */

    // Make a random environment spec, edit properties about it such as the number of obstacles
    /*
    Random2DEnvironmentSpecification spec;
    spec.max_obstacle_region_radius = 5.0;
    spec.n_obstacles = 2;
    spec.path_clearance = 0.01;
    spec.d_sep = 0.01;

    //Randomly generate the environment;
    Problem2D problem = EnvironmentTools::generateRandom(spec); // Random environment
    */

    // Declare your algorithm object 
    const double epsilon = 0.001;
    const double dt = 0.0005;
    const double dtheta = 2.0;
    MyBugAlgorithm algo(dt, dtheta, epsilon); 
    
    {
        // Call your algorithm on the problem

        // DEFINE HYPER PARAMETERS
        amp::Path2D path = algo.plan(problem);

        // Check your path to make sure that it does not collide with the environment 
        bool success = HW2::check(path, problem);

        LOG("Found valid solution to workspace 1: " << (success ? "Yes!" : "No :("));

        // Visualize the path and environment
        Visualizer::makeFigure(problem, path);
    }

    // Let's get crazy and generate a random environment and test your algorithm
    {
    int trial = 1;
    while (true) {
        amp::Path2D path; // Make empty path, problem, and collision points, as they will be created by generateAndCheck()
        amp::Problem2D random_prob; 
        std::vector<Eigen::Vector2d> collision_points;
        bool random_trial_success = HW2::generateAndCheck(algo, path, random_prob, collision_points);
        LOG("Random trial #" << trial << " - Found valid solution in random environment: " << (random_trial_success ? "Yes!" : "No :("));
        LOG("path length: " << path.length());

        // Visualize the path environment, and any collision points with obstacles
        
        if (!random_trial_success) {
            Visualizer::makeFigure(random_prob, path, collision_points);
            LOG("Stopping loop: random trial failed.");
            break;
        }
        ++trial;
    }
    }

    Visualizer::saveFigures(true, "hw2_figs");


    HW2::grade(algo, "owen.kranz@coloradu.edu", argc, argv);
    
    /* If you want to reconstruct your bug algorithm object every trial (to reset member variables from scratch or initialize), use this method instead*/
    //HW2::grade<MyBugAlgorithm>("nonhuman.biologic@myspace.edu", argc, argv, constructor_parameter_1, constructor_parameter_2, etc...);
    
    // This will reconstruct using the default constructor every trial
    //HW2::grade<MyBugAlgorithm>("nonhuman.biologic@myspace.edu", argc, argv);

    return 0;
}