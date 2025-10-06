#include "MyAStar.h"

// Implement the search method for the A* algorithm
MyAStarAlgo::GraphSearchResult MyAStarAlgo::search(const amp::ShortestPathProblem& problem, const amp::SearchHeuristic& heuristic) {
    std::cout << "Starting A* Graph Search: Init --> goal | " << problem.init_node << " --> " << problem.goal_node << std::endl;
    GraphSearchResult result = {false, {}, 0.0}; // initialize the results object

    std::vector<amp::Node> open_set;
    std::vector<amp::Node> open_set_parents;
    std::vector<double> open_set_weights;
    std::vector<double> open_set_heuristics;
    

    std::vector<amp::Node> closed_set;
    std::vector<amp::Node> closed_set_parents;
    std::vector<double> closed_set_weights;

    // add first to open set

    open_set.push_back(problem.init_node);
    open_set_parents.push_back(problem.init_node); // technically has no parent
    open_set_weights.push_back(0.0);
    open_set_heuristics.push_back(heuristic(problem.init_node));

    while (true){

        if (open_set.empty()){
            std::cout<< "NO SOLUTION" << std::endl;
            break;
        }

        // sort? nah i think finding the minimum will be easiest
        std::vector<double> weight_plus_heuristics; // gotta combine weights and heuristic
        for (size_t i = 0; i < open_set.size(); i++){
            weight_plus_heuristics.push_back(open_set_weights[i] + open_set_heuristics[i]);
        }

        // ai helped here. No clue how to search for a min
        auto min_val_it = std::min_element(weight_plus_heuristics.begin(), weight_plus_heuristics.end());
        int min_index = std::distance(weight_plus_heuristics.begin(), min_val_it);

        // pop this node from the list.
        amp::Node current_node = open_set[min_index];
        amp::Node current_parent = open_set_parents[min_index];
        double current_weight = open_set_weights[min_index];
        // dont need the current heuristic cuz that wont go in the closed set

        
        // add to the closed...
        // gotta chekc if its in the closed set already
        bool is_in_closed = false;
        int index_of_duplicate_in_closed;
        for (size_t i = 0; i < closed_set.size(); i++){
            if (closed_set[i] == current_node){
               is_in_closed = true; 
               index_of_duplicate_in_closed = i;
            }
        }

        if (!is_in_closed){
            closed_set.push_back(current_node);
            closed_set_weights.push_back(current_weight);
            closed_set_parents.push_back(current_parent);
        } else{
            // it is in.. gotta check if it improves cost
            if (current_weight < closed_set_weights[index_of_duplicate_in_closed]){
                // cost improved! erase dat duplicate!
                closed_set.erase(closed_set.begin() + index_of_duplicate_in_closed);
                closed_set_parents.erase(closed_set_parents.begin() + index_of_duplicate_in_closed);
                closed_set_weights.erase(closed_set_weights.begin() + index_of_duplicate_in_closed);

                // now its erased, push the current
                closed_set.push_back(current_node);
                closed_set_weights.push_back(current_weight);
                closed_set_parents.push_back(current_parent);
            }
        }
        
        if (current_node == problem.goal_node){
            //we made ittttt
            // add to closed and break
            break;
        }

        // remove from open list
        open_set.erase(open_set.begin() + min_index);
        open_set_heuristics.erase(open_set_heuristics.begin() + min_index);
        open_set_parents.erase(open_set_parents.begin() + min_index);
        open_set_weights.erase(open_set_weights.begin() + min_index);

        // add all children to the open set
        // needed ai for this next line cuz like what
        std::vector<amp::Node> children = problem.graph->children(current_node);
        std::vector<double> edge_weights = problem.graph->outgoingEdges(current_node);

        for (size_t i = 0; i < children.size(); i++){
            open_set.push_back(children[i]);
            open_set_weights.push_back(edge_weights[i] + current_weight);
            open_set_parents.push_back(current_node); //current node is the parent 
            open_set_heuristics.push_back(heuristic(children[i]));
        }

        

    }


    // we made it! trace the parents back, adding the cost together

    // luckily, the last node in the closed set is the goal

    size_t current_index = closed_set.size() - 1;
    result.path_cost = closed_set_weights[current_index];

    while(closed_set[current_index] != problem.init_node){
        
        // add current node
        result.node_path.push_back(closed_set[current_index]);

        amp::Node parent = closed_set_parents[current_index];

        // ai helped here, no clue how to find an element in a list
        auto parent_it = std::find(closed_set.begin(), closed_set.end(), parent);
        if (parent_it != closed_set.end()) {
            current_index = std::distance(closed_set.begin(), parent_it);
        } else {
            std::cout << "Error: Parent not found in closed set!" << std::endl;
            break;
        }
    }

    // breaks before init gets added
    result.node_path.push_back(problem.init_node);

    std::reverse(result.node_path.begin(), result.node_path.end());

    

    result.print();
    return result;
}
