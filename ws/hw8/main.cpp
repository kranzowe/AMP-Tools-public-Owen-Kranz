// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"
#include "hw/HW8.h"
#include "MyMultiAgentPlanners.h"

using namespace amp;

void timer_example() {
    double startTime;
    amp::Timer timer("timer");
    for (int i=0; i < 5; ++i) {
        startTime = timer.now(TimeUnit::ms);  
        std::cout << "Press any key to continue...\n";
        std::cin.get();
        std::cout << "Time since last run: " << timer.now(TimeUnit::ms) - startTime << std::endl;
    }
    timer.stop();
    std::cout << "Total time since last run: " << Profiler::getTotalProfile("timer") << std::endl;
}

void problem1(){
    amp::RNG::seed(amp::RNG::randiUnbounded());

    // Part (b): Solve for m=2 with specific parameters
    MultiAgentProblem2D problem_m2 = HW8::getWorkspace1(2);
    MyCentralPlanner central_planner_b;  // n=7500, r=0.5, p_goal=0.25
    MultiAgentPath2D path_b = central_planner_b.plan(problem_m2);
    std::vector<std::vector<Eigen::Vector2d>> collision_states_b;
    bool isValid_b = HW8::check(path_b, problem_m2, collision_states_b);
    Visualizer::makeFigure(problem_m2, path_b, collision_states_b);

    // Part (c) & (d): Benchmark for m = 2, 3, 4, 5, 6 agents
    std::vector<int> agent_counts = {2, 3, 4, 5, 6};
    std::map<int, std::list<std::vector<double>>> runtime_data_by_m;
    std::map<int, std::list<std::vector<double>>> tree_size_data_by_m;
    std::map<int, std::vector<std::string>> setting_labels_by_m;

    for (int m : agent_counts) {
        std::cout << "\n========== Benchmarking m = " << m << " agents ==========\n" << std::endl;
        
        MultiAgentProblem2D problem = HW8::getWorkspace1(m);
        
        std::list<std::vector<double>> runtime_data;
        std::list<std::vector<double>> tree_size_data;
        std::vector<std::string> setting_labels;
        
        // Single setting for this agent count (you can add more settings if needed)
        // int num_samples = 7500;
        // double connection_radius = 0.5;
        // double goal_bias = 0.25;
        
        MyCentralPlanner central_planner;
        
        std::vector<double> runtimes;
        std::vector<double> tree_sizes;
        int failure_count = 0;
        
        // 100 trials
        for(int trial = 0; trial < 100; trial++){
            auto start = std::chrono::high_resolution_clock::now();
            MultiAgentPath2D path = central_planner.plan(problem);
            auto end = std::chrono::high_resolution_clock::now();
            
            std::vector<std::vector<Eigen::Vector2d>> collision_states;
            bool result = HW8::check(path, problem, collision_states);
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
            
            // Get tree size (you need to implement getLastTreeSize() in your planner)
            int tree_size = central_planner.getLastTreeSize();
            
            runtimes.push_back(duration.count() / 1000.0); // milliseconds
            
            if (result) {
                tree_sizes.push_back(static_cast<double>(tree_size));
                std::cout << "Trial " << trial+1 << "/100: Success - " 
                        << duration.count()/1000.0 << " ms, tree size: " 
                        << tree_size << std::endl;
            } else {
                failure_count++;
                std::cout << "Trial " << trial+1 << "/100: Failed - " 
                        << duration.count()/1000.0 << " ms" << std::endl;
            }
        }
        
        runtime_data.push_back(runtimes);
        tree_size_data.push_back(tree_sizes);
        
        setting_labels.push_back("m=" + std::to_string(m));
        
        int successful_runs = tree_sizes.size();
        double success_rate = (double)successful_runs / 100.0 * 100.0;
        std::cout << "\nm = " << m << ": " << successful_runs << "/100 successful (" 
                << success_rate << "%), " << failure_count << " failures" << std::endl;
        
        // Store data for this m value
        runtime_data_by_m[m] = runtime_data;
        tree_size_data_by_m[m] = tree_size_data;
        setting_labels_by_m[m] = setting_labels;
        
        // Create individual boxplots for each m
        Visualizer::makeBoxPlot(runtime_data, setting_labels, 
                            "Central Planner Runtime (m=" + std::to_string(m) + ")", 
                            "Number of Agents", 
                            "Runtime (ms)");
        
        Visualizer::makeBoxPlot(tree_size_data, setting_labels, 
                            "Central Planner Tree Size (m=" + std::to_string(m) + ")", 
                            "Number of Agents", 
                            "Tree Size (nodes)");
    }

    // Part (e): Compute averages and print for Excel plotting
    std::cout << "\n========== PART (E): AVERAGE DATA FOR EXCEL ==========\n" << std::endl;
    std::cout << "m\tAvg_Runtime_ms\tAvg_Tree_Size" << std::endl;

    for (int m : agent_counts) {
        // Calculate average runtime
        double total_runtime = 0;
        int runtime_count = 0;
        for (const auto& runtime_vec : runtime_data_by_m[m]) {
            for (double runtime : runtime_vec) {
                total_runtime += runtime;
                runtime_count++;
            }
        }
        double avg_runtime = (runtime_count > 0) ? (total_runtime / runtime_count) : 0;
        
        // Calculate average tree size
        double total_tree_size = 0;
        int tree_size_count = 0;
        for (const auto& tree_size_vec : tree_size_data_by_m[m]) {
            for (double tree_size : tree_size_vec) {
                total_tree_size += tree_size;
                tree_size_count++;
            }
        }
        double avg_tree_size = (tree_size_count > 0) ? (total_tree_size / tree_size_count) : 0;
        
        std::cout << m << "\t" << avg_runtime << "\t" << avg_tree_size << std::endl;
    }

    // Combined boxplot for all m values (optional)
    std::list<std::vector<double>> all_runtime_data;
    std::list<std::vector<double>> all_tree_size_data;
    std::vector<std::string> all_labels;

    for (int m : agent_counts) {
        for (const auto& data : runtime_data_by_m[m]) {
            all_runtime_data.push_back(data);
        }
        for (const auto& data : tree_size_data_by_m[m]) {
            all_tree_size_data.push_back(data);
        }
        all_labels.push_back("m=" + std::to_string(m));
    }

    Visualizer::makeBoxPlot(all_runtime_data, all_labels, 
                        "Central Planner Runtime vs Number of Agents", 
                        "Number of Agents (m)", 
                        "Runtime (ms)");

    Visualizer::makeBoxPlot(all_tree_size_data, all_labels, 
                        "Central Planner Tree Size vs Number of Agents", 
                        "Number of Agents (m)", 
                        "Tree Size (nodes)");




}

int main(int argc, char** argv) {
    // Initializing workspace 1 with 3 agents
    //problem1();
    // amp::RNG::seed(amp::RNG::randiUnbounded());
    // MultiAgentPath2D path;
    // MultiAgentProblem2D problem = HW8::getWorkspace1(6);
    // std::vector<std::vector<Eigen::Vector2d>> collision_states;

    // // Solve using a centralized approach
    // MyCentralPlanner central_planner;
    // path = central_planner.plan(problem);
    // bool isValid = HW8::check(path, problem, collision_states);
    // Visualizer::makeFigure(problem, path, collision_states);

    // // Solve using a decentralized approach
    MultiAgentProblem2D problem = HW8::getWorkspace1(2);
    MyDecentralPlanner decentral_planner;
    MultiAgentPath2D path;
    path = decentral_planner.plan(problem);
    std::vector<std::vector<Eigen::Vector2d>> collision_states;
    // collision_states = {{}};
    HW8::generateAndCheck(decentral_planner, path, problem, collision_states);
    Visualizer::makeFigure(problem, path, collision_states);

    // // Visualize and grade methods
    Visualizer::saveFigures(true, "hw8_figs");
    // HW8::grade<MyCentralPlanner, MyDecentralPlanner>("firstName.lastName@colorado.edu", argc, argv, std::make_tuple(), std::make_tuple());
    return 0;
}