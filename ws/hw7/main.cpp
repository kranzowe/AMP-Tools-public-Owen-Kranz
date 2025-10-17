// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"
#include "hw/HW2.h"
#include "hw/HW5.h"
#include "MySamplingBasedPlanners.h"

#include <chrono>
#include <iostream>

using namespace amp;

int main(int argc, char** argv) {
    //HW7::hint(); // Consider implementing an N-dimensional planner 

    Problem2D problem = HW5::getWorkspace1();
    problem.y_max = 3.0;
    problem.y_min = -3.0;
    MyPRM prm;
    
    std::vector<std::pair<int, double>> prm_settings = {
        {200, 0.5}, {200, 1.0}, {200, 1.5}, {200, 2.0},
        {500, 0.5}, {500, 1.0}, {500, 1.5}, {500, 2.0}
    };
    
    std::list<std::vector<double>> runtime_data;
    std::list<std::vector<double>> path_length_data;
    std::list<std::vector<double>> failure_data;  // Added for failure count
    std::vector<std::string> setting_labels;
    
    for (const auto& setting : prm_settings) {
        int num_samples = setting.first;
        double connection_radius = setting.second;
        
        
        MyPRM prm(num_samples, connection_radius);
        
        std::vector<double> runtimes;
        std::vector<double> path_lengths;
        int failure_count = 0; 
        
        // 100 trials
        for(int i = 0; i < 100; i++){
            auto start = std::chrono::high_resolution_clock::now();
            Path2D prm_path = prm.plan(problem);
            auto end = std::chrono::high_resolution_clock::now();
            
            bool result = HW7::check(prm_path, problem);
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
            
            runtimes.push_back(duration.count() / 1000.0); // millisecods
            
            if (result) {
                path_lengths.push_back(prm_path.length());
                std::cout << "Trial " << i+1 << "/100: Success - " 
                          << duration.count()/1000.0 << " ms, length: " 
                          << prm_path.length() << std::endl;
            } else {
                // path_lengths.push_back(-1.0);
                failure_count++;  // Increment failure counter
                std::cout << "Trial " << i+1 << "/100: Failed - " 
                          << duration.count()/1000.0 << " ms" << std::endl;
            }
        }
        
        runtime_data.push_back(runtimes);
        
        std::vector<double> successful_lengths;
        for (double length : path_lengths) {
            if (length > 0) {
                successful_lengths.push_back(length);
            }
        }
        path_length_data.push_back(successful_lengths);
        
        failure_data.push_back(std::vector<double>{static_cast<double>(100 - failure_count)});
        
        setting_labels.push_back("(" + std::to_string(num_samples) + "," + 
                                std::to_string(connection_radius) + ")");
        
        int successful_runs = successful_lengths.size();
        double success_rate = (double)successful_runs / 100.0 * 100.0;
        std::cout << "Setting (" << num_samples << "," << connection_radius 
                  << "): " << successful_runs << "/100 successful (" 
                  << success_rate << "%), " << failure_count << " failures" << std::endl << std::endl;
    }
    
    
    Visualizer::makeBoxPlot(runtime_data, setting_labels, 
                           "PRM Runtime Performance", 
                           "PRM Settings (samples, radius)", 
                           "Runtime (ms)");
    
    Visualizer::makeBoxPlot(path_length_data, setting_labels, 
                           "PRM Path Length Performance", 
                           "PRM Settings (samples, radius)", 
                           "Path Length");
    
    Visualizer::makeBoxPlot(failure_data, setting_labels, 
                           "PRM Success Count", 
                           "PRM Settings (samples, radius)", 
                           "Number of Successes (out of 100)");

    // // Get PRM visualization data
    // Path2D prm_path = prm.plan(problem);
    // auto prm_graph = prm.getLastGraph();
    // auto prm_nodes = prm.getLastNodes();
    
    // // Visualize PRM results
    // Visualizer::makeFigure(problem, prm_path, *prm_graph, prm_nodes);

    // // double length = 0.0;
    // // for (int i = 0; i < prm_path.waypoints.size() - 1; i++){
    // //     length += (prm_path.waypoints[i+1] - prm_path.waypoints[i]).norm();
    // // }

    // LOG("path length: " << prm_path.length());


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
    // Problem2D problem2d;
    // MyRRT rrt;
    // Path2D rrt_path;
    // HW7::generateAndCheck(rrt, rrt_path, problem2d);

    // auto rrt_graph = rrt.getLastGraph();
    // auto rrt_nodes = rrt.getLastNodes();

    // Visualizer::makeFigure(problem2d, rrt_path, *rrt_graph, rrt_nodes);
    
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
    // HW7::grade<MyPRM, MyRRT>("firstName.lastName@colorado.edu", argc, argv, std::make_tuple(), std::make_tuple());
    return 0;
}