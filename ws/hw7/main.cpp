// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"
#include "hw/HW2.h"
#include "hw/HW5.h"
#include "MySamplingBasedPlanners.h"

using namespace amp;

int main(int argc, char** argv) {
    //HW7::hint(); // Consider implementing an N-dimensional planner 

    // // Test PRM on Workspace1 of HW2
    // Problem2D problem = HW2::getWorkspace1();
    // MyPRM prm;
    // Path2D prm_path = prm.plan(problem);
    
    // // Get PRM visualization data
    // auto prm_graph = prm.getLastGraph();
    // auto prm_nodes = prm.getLastNodes();
    
    // // Visualize PRM results
    // Visualizer::makeFigure(problem, prm_path, *prm_graph, prm_nodes);


    // // Problem2D problem1 = HW2::getWorkspace1();
    // MyRRT rrt;
    // Path2D rrt_path = rrt.plan(problem1);

    // bool result = HW7::check(rrt_path, problem1);
    
    // // Get PRM visualization data
    // auto rrt_graph = rrt.getLastGraph();
    // auto rrt_nodes = rrt.getLastNodes();
    
    // // Visualize PRM results
    // Visualizer::makeFigure(problem1, rrt_path, *rrt_graph, rrt_nodes);

    // // Generate a random problem and test RRT
    Problem2D problem2d;
    MyRRT rrt;
    Path2D rrt_path;
    HW7::generateAndCheck(rrt, rrt_path, problem2d);

    auto rrt_graph = rrt.getLastGraph();
    auto rrt_nodes = rrt.getLastNodes();

    Visualizer::makeFigure(problem2d, rrt_path, *rrt_graph, rrt_nodes);
    
    // Create separate visualization for RRT (you can implement similar for RRT later)
    // std::shared_ptr<amp::Graph<double>> rrt_graphPtr = std::make_shared<amp::Graph<double>>();
    // std::map<amp::Node, Eigen::Vector2d> rrt_nodes;
    
    // For now, just visualize the path points
    // for (amp::Node i = 0; i < rrt_path.waypoints.size(); ++i) {
    //     rrt_nodes[i] = rrt_path.waypoints[i];
    // }
    
    // Visualizer::makeFigure(problem, rrt_path, *rrt_graphPtr, rrt_nodes);
    Visualizer::saveFigures(true, "hw7_figs");

    // Grade method
    //HW7::grade<MyPRM, MyRRT>("firstName.lastName@colorado.edu", argc, argv, std::make_tuple(), std::make_tuple());
    return 0;
}