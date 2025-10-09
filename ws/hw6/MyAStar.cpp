#include "MyAStar.h"
#include <queue>
#include <unordered_map>
#include <unordered_set>

struct AStarNode { // AI helped here. Was a major speed up to what I had
    amp::Node node;
    amp::Node parent;
    double g_cost;
    double f_cost;
    
    // for the priority queue to work right
    bool operator>(const AStarNode& other) const {
        return f_cost > other.f_cost; 
    }
};

MyAStarAlgo::GraphSearchResult MyAStarAlgo::search(const amp::ShortestPathProblem& problem, const amp::SearchHeuristic& heuristic) {
    std::cout << "Starting A* Graph Search: Init --> goal | " << problem.init_node << " --> " << problem.goal_node << std::endl;
    GraphSearchResult result = {false, {}, 0.0}; // initialize the results object

    // way faster than vectors AI helped optimize this
    std::priority_queue<AStarNode, std::vector<AStarNode>, std::greater<AStarNode>> open_set;
    
    // hash maps for super fast lookup instead of slow vector searching
    // needed AI help for these
    std::unordered_set<amp::Node> closed_set;
    std::unordered_map<amp::Node, amp::Node> parents;
    std::unordered_map<amp::Node, double> g_costs;

    // add first node to open set
    double init_h = heuristic(problem.init_node);
    open_set.push({problem.init_node, problem.init_node, 0.0, init_h});
    g_costs[problem.init_node] = 0.0;

    int attempts = 0;

    while (!open_set.empty() && attempts < 1e5) {
        attempts++;

        // get the best node automatically at top (AI help with these)
        AStarNode current = open_set.top();
        open_set.pop();
        
        // FIXED: check for outdated entries first, before doing anything else
        if (g_costs.count(current.node) && current.g_cost > g_costs[current.node]) {
            continue; // this is an outdated entry, skip it
        }

        // we made it to the goal!
        if (current.node == problem.goal_node) {
            result.success = true;
            result.path_cost = current.g_cost;
            
            // trace back the path using the parents map
            amp::Node trace_node = problem.goal_node;
            while (trace_node != problem.init_node) {
                result.node_path.push_back(trace_node);
                trace_node = parents[trace_node];
            }
            result.node_path.push_back(problem.init_node);
            std::reverse(result.node_path.begin(), result.node_path.end());
            break;
        }

        // check if we already processed this node
        if (closed_set.count(current.node)) continue; // skip if already done
        
        // add to closed set and record the parent
        closed_set.insert(current.node);
        parents[current.node] = current.parent;

        // add all children to open set
        std::vector<amp::Node> children = problem.graph->children(current.node);
        std::vector<double> edge_weights = problem.graph->outgoingEdges(current.node);

        for (size_t i = 0; i < children.size(); i++) {
            amp::Node child = children[i];
            
            // skip if already processed
            if (closed_set.count(child)) continue;
            
            double tentative_g = current.g_cost + edge_weights[i];
            
            // FIXED: simpler logic - just check if we found a better path
            if (!g_costs.count(child) || tentative_g < g_costs[child]) {
                // update the best known cost
                g_costs[child] = tentative_g;
                parents[child] = current.node;
                
                double h_cost = heuristic(child);
                double f_cost = tentative_g + h_cost;
                
                // always add - duplicates will be filtered out when popped
                open_set.push({child, current.node, tentative_g, f_cost});
            }
        }
        
        // MEMORY CLEANUP: periodically check queue size
        if (attempts % 1000 == 0) {
            std::cout << "A* iteration " << attempts << ", queue size: " << open_set.size() 
                      << ", closed set: " << closed_set.size() << std::endl;
        }
    }

    if (attempts >= 1e5) {
        std::cout << "A* reached maximum iterations!" << std::endl;
    }
    
    if (!result.success) {
        std::cout << "NO SOLUTION" << std::endl;
    }

    // CLEANUP: Clear data structures to free memory
    while (!open_set.empty()) open_set.pop();
    closed_set.clear();
    parents.clear();
    g_costs.clear();

    result.print();
    return result;
}