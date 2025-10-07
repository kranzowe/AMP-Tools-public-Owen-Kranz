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

    int attempts = 0;


    while (true && attempts < 1e5){
        attempts++;

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
        
        if (current_node == problem.goal_node){
            //we made ittttt
            // add to closed and break
            // dont need the current heuristic cuz that wont go in the closed set

            closed_set.push_back(current_node);
            closed_set_weights.push_back(current_weight);
            closed_set_parents.push_back(current_parent);
            break;
        }

        // remove from open list
        open_set.erase(open_set.begin() + min_index);
        open_set_heuristics.erase(open_set_heuristics.begin() + min_index);
        open_set_parents.erase(open_set_parents.begin() + min_index);
        open_set_weights.erase(open_set_weights.begin() + min_index);

        //check if we aleady processed this node
        bool already_processed = false;
        for (size_t i = 0; i < closed_set.size(); i++){
            if (closed_set[i] == current_node){
                already_processed = true;
                break;
            }
        }
        if (already_processed) continue; // Skip if already processed

        // dont need the current heuristic cuz that wont go in the closed set

        //add to closed
        closed_set.push_back(current_node);
        closed_set_weights.push_back(current_weight);
        closed_set_parents.push_back(current_parent);

        // add all children to the open set
        // needed ai for this next line cuz like what
        std::vector<amp::Node> children = problem.graph->children(current_node);
        std::vector<double> edge_weights = problem.graph->outgoingEdges(current_node);

        for (size_t i = 0; i < children.size(); i++){

            // add to the opem...
            // gotta chekc if its in the open set already
            bool is_in_open = false;
            int index_of_duplicate_in_open;
            for (size_t i = 0; i < open_set.size(); i++){
                if (open_set[i] == children[i]){
                is_in_open = true; 
                index_of_duplicate_in_open = i;
                }
            }

            if (!is_in_open){
            
                open_set.push_back(children[i]);
                open_set_weights.push_back(edge_weights[i] + current_weight);
                open_set_parents.push_back(current_node); //current node is the parent 
                open_set_heuristics.push_back(heuristic(children[i]));

            } else{
                // it is in.. gotta check if it improves cost
                if (current_weight + edge_weights[i] < open_set_weights[index_of_duplicate_in_open]){
                    // cost improved! erase dat duplicate!
                    open_set.erase(open_set.begin() + index_of_duplicate_in_open);
                    open_set_parents.erase(open_set_parents.begin() + index_of_duplicate_in_open);
                    open_set_weights.erase(open_set_weights.begin() + index_of_duplicate_in_open);
                    open_set_heuristics.erase(open_set_heuristics.begin() + index_of_duplicate_in_open);

                    // now its erased, push the current
                    open_set.push_back(children[i]);
                    open_set_weights.push_back(edge_weights[i] + current_weight);
                    open_set_parents.push_back(current_node); //current node is the parent 
                    open_set_heuristics.push_back(heuristic(children[i]));
                }
            }
        }

        

    }


    // we made it! trace the parents back, adding the cost together

    // luckily, the last node in the closed set is the goal

    size_t current_index = closed_set.size() - 1;
    result.path_cost = closed_set_weights[current_index];
    attempts = 0;
    while(closed_set[current_index] != problem.init_node && attempts <100){
        attempts ++;
        
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
