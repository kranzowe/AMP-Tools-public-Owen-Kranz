// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"
#include "hw/HW9.h"
#include "hw/HW2.h"
#include "MyKinoRRT.h"

using namespace amp;


// Load problems and map agent for quick testing
std::vector<KinodynamicProblem2D> problems = {HW9::getStateIntProblemWS1(), HW9::getStateIntProblemWS2(), HW9::getFOUniProblemWS1(), HW9::getFOUniProblemWS2(), HW9::getSOUniProblemWS1(), HW9::getSOUniProblemWS2(), HW9::getCarProblemWS1(), HW9::getParkingProblem()};
std::unordered_map<AgentType, std::function<std::shared_ptr<amp::DynamicAgent>()>> agentFactory = {
    {AgentType::SingleIntegrator, []() { return std::make_shared<MySingleIntegrator>(); }},
    {AgentType::FirstOrderUnicycle, []() { return std::make_shared<MyFirstOrderUnicycle>(); }},
    {AgentType::SecondOrderUnicycle, []() { return std::make_shared<MySecondOrderUnicycle>(); }},
    {AgentType::SimpleCar, []() { return std::make_shared<MySimpleCar>(); }}
};


void testPropagation() {
    std::cout << "\n========== PROPAGATION TEST ==========\n" << std::endl;
    
    // Test 1: Single Integrator
    {
        std::cout << "TEST 1: Single Integrator" << std::endl;
        MySingleIntegrator agent;
        
        Eigen::VectorXd state(2);
        state << 0.0, 0.0;  // Start at origin
        
        Eigen::VectorXd control(2);
        control << 1.0, 0.0;  // Move right with velocity 1
        
        double dt = 1.0;
        
        std::cout << "Initial state: [" << state.transpose() << "]" << std::endl;
        std::cout << "Control: [" << control.transpose() << "], dt: " << dt << std::endl;
        
        agent.propagate(state, control, dt);
        
        std::cout << "Final state: [" << state.transpose() << "]" << std::endl;
        std::cout << "Expected: [1.0, 0.0] (moved right by 1.0)" << std::endl;
        
        // Check if collision points were generated
        std::cout << "Collision points generated: " << amp::points_to_check.size() << " timesteps" << std::endl;
        std::cout << "Sub-states generated: " << amp::sub_states.size() << std::endl;
        
        bool passed = std::abs(state[0] - 1.0) < 1e-6 && std::abs(state[1] - 0.0) < 1e-6;
        std::cout << "RESULT: " << (passed ? "PASS ✓" : "FAIL ✗") << "\n" << std::endl;
    }
    
    // Test 2: First Order Unicycle
    {
        std::cout << "TEST 2: First Order Unicycle" << std::endl;
        MyFirstOrderUnicycle agent;
        
        Eigen::VectorXd state(3);
        state << 0.0, 0.0, 0.0;  // Start at origin, facing right
        
        Eigen::VectorXd control(2);
        control << 1.0, 0.0;  // Forward velocity, no angular velocity
        
        double dt = 1.0;
        
        std::cout << "Initial state: [" << state.transpose() << "]" << std::endl;
        std::cout << "Control: [" << control.transpose() << "], dt: " << dt << std::endl;
        
        agent.propagate(state, control, dt);
        
        std::cout << "Final state: [" << state.transpose() << "]" << std::endl;
        std::cout << "Expected: [0.25, 0.0, 0.0] (moved forward by 0.25)" << std::endl;
        
        bool passed = std::abs(state[0] - 0.25) < 1e-6 && std::abs(state[1] - 0.0) < 1e-6;
        std::cout << "RESULT: " << (passed ? "PASS ✓" : "FAIL ✗") << "\n" << std::endl;
    }
    
    // Test 3: Unicycle turning
    {
        std::cout << "TEST 3: First Order Unicycle - Turning" << std::endl;
        MyFirstOrderUnicycle agent;
        
        Eigen::VectorXd state(3);
        state << 0.0, 0.0, 0.0;  // Start at origin, facing right
        
        Eigen::VectorXd control(2);
        control << 0.0, M_PI/2;  // No forward velocity, turn 90 degrees/sec
        
        double dt = 1.0;
        
        std::cout << "Initial state: [" << state.transpose() << "]" << std::endl;
        std::cout << "Control: [" << control.transpose() << "], dt: " << dt << std::endl;
        
        agent.propagate(state, control, dt);
        
        std::cout << "Final state: [" << state.transpose() << "]" << std::endl;
        std::cout << "Expected: [0.0, 0.0, " << M_PI/2 << "] (rotated 90 degrees)" << std::endl;
        
        bool passed = std::abs(state[0] - 0.0) < 1e-6 && 
                     std::abs(state[1] - 0.0) < 1e-6 && 
                     std::abs(state[2] - M_PI/2) < 1e-6;
        std::cout << "RESULT: " << (passed ? "PASS ✓" : "FAIL ✗") << "\n" << std::endl;
    }
    
    // Test 4: Multiple small steps vs one big step
    {
        std::cout << "TEST 4: Integration Consistency Check" << std::endl;
        MySingleIntegrator agent;
        
        // One big step
        Eigen::VectorXd state1(2);
        state1 << 0.0, 0.0;
        Eigen::VectorXd control(2);
        control << 1.0, 1.0;
        agent.propagate(state1, control, 1.0);
        
        // Multiple small steps
        Eigen::VectorXd state2(2);
        state2 << 0.0, 0.0;
        for(int i = 0; i < 10; i++) {
            agent.propagate(state2, control, 0.1);
        }
        
        std::cout << "One step result: [" << state1.transpose() << "]" << std::endl;
        std::cout << "Ten steps result: [" << state2.transpose() << "]" << std::endl;
        
        double diff = (state1 - state2).norm();
        std::cout << "Difference: " << diff << std::endl;
        
        bool passed = diff < 1e-6;
        std::cout << "RESULT: " << (passed ? "PASS ✓" : "FAIL ✗") << "\n" << std::endl;
    }
    
    // Test 5: Check collision points generation
    {
        std::cout << "TEST 5: Collision Points Generation" << std::endl;
        MySingleIntegrator agent;
        
        Eigen::VectorXd state(2);
        state << 0.0, 0.0;
        Eigen::VectorXd control(2);
        control << 1.0, 0.0;
        
        agent.propagate(state, control, 0.01);  // Small dt
        
        std::cout << "Number of collision check sets: " << amp::points_to_check.size() << std::endl;
        if (!amp::points_to_check.empty()) {
            std::cout << "Points per set: " << amp::points_to_check[0].size() << std::endl;
            std::cout << "First corner set:" << std::endl;
            for (const auto& pt : amp::points_to_check[0]) {
                std::cout << "  [" << pt.transpose() << "]" << std::endl;
            }
        }
        
        bool passed = !amp::points_to_check.empty() && amp::points_to_check[0].size() == 4;
        std::cout << "RESULT: " << (passed ? "PASS ✓" : "FAIL ✗") << "\n" << std::endl;
    }
    
    std::cout << "========== END OF TESTS ==========\n" << std::endl;
}


void compareWithChecker() {
    std::cout << "\n========== CHECKER COMPARISON TEST ==========\n" << std::endl;
    
    // Test each agent type
    std::vector<std::pair<std::string, AgentType>> test_cases = {
        {"Single Integrator", AgentType::SingleIntegrator},
        {"First Order Unicycle", AgentType::FirstOrderUnicycle}
    };
    
    for (const auto& [name, agent_type] : test_cases) {
        std::cout << "\nTesting: " << name << std::endl;
        std::cout << "----------------------------------------" << std::endl;
        
        // Get a problem that uses this agent
        KinodynamicProblem2D prob;
        if (agent_type == AgentType::SingleIntegrator) {
            prob = HW9::getStateIntProblemWS1();
        } else if (agent_type == AgentType::FirstOrderUnicycle) {
            prob = HW9::getFOUniProblemWS1();
        }
        
        // Create your agent
        auto my_agent = agentFactory[agent_type]();
        
        // Create a simple path with just 2 waypoints
        KinoPath test_path;
        test_path.valid = true;
        
        // Starting state
        Eigen::VectorXd start_state = prob.q_init;
        test_path.waypoints.push_back(start_state);
        
        // Generate some test controls
        std::vector<Eigen::VectorXd> test_controls;
        std::vector<double> test_durations;
        
        // Test a few different control inputs
        for (int test = 0; test < 3; test++) {
            Eigen::VectorXd control(prob.u_bounds.size());
            
            if (test == 0) {
                // Test 1: Minimal control
                for (int i = 0; i < prob.u_bounds.size(); i++) {
                    control[i] = (prob.u_bounds[i].first + prob.u_bounds[i].second) * 0.25;
                }
            } else if (test == 1) {
                // Test 2: Half-max control
                for (int i = 0; i < prob.u_bounds.size(); i++) {
                    control[i] = (prob.u_bounds[i].first + prob.u_bounds[i].second) * 0.5;
                }
            } else {
                // Test 3: Near-max control  
                for (int i = 0; i < prob.u_bounds.size(); i++) {
                    control[i] = prob.u_bounds[i].second * 0.9;
                }
            }
            
            double dt = 0.1;  // Fixed small timestep
            
            std::cout << "\n  Test " << (test+1) << ":" << std::endl;
            std::cout << "    Control: [" << control.transpose() << "]" << std::endl;
            std::cout << "    Duration: " << dt << std::endl;
            
            // YOUR propagation
            Eigen::VectorXd my_state = start_state;
            my_agent->propagate(my_state, control, dt);
            std::cout << "    Your result: [" << my_state.transpose() << "]" << std::endl;
            
            // Build path for checker
            test_path.waypoints.clear();
            test_path.controls.clear();
            test_path.durations.clear();
            
            test_path.waypoints.push_back(start_state);
            test_path.waypoints.push_back(my_state);  // Where YOU think it should end
            test_path.controls.push_back(control);
            test_path.durations.push_back(dt);
            
            // Call HW9::check to see what IT thinks the final state should be
            std::cout << "    Checking with HW9::check..." << std::endl;
            
            // Capture the output (the checker will print the expected vs actual)
            bool check_result = HW9::check(test_path, prob);
            
            if (!check_result) {
                std::cout << "    ❌ MISMATCH DETECTED!" << std::endl;
                
                // Now let's manually compute what the checker would get
                // by creating a path that the checker WOULD accept
                std::cout << "\n    Diagnosing difference:" << std::endl;
                
                // Try slightly different end states to see which direction we're off
                for (double scale = 0.9; scale <= 1.1; scale += 0.05) {
                    Eigen::VectorXd scaled_state = start_state + scale * (my_state - start_state);
                    test_path.waypoints[1] = scaled_state;
                    
                    // Suppress output for these tests
                    std::streambuf* orig_buf = std::cout.rdbuf();
                    std::cout.rdbuf(nullptr);
                    bool scaled_result = HW9::check(test_path, prob);
                    std::cout.rdbuf(orig_buf);
                    
                    if (scaled_result) {
                        std::cout << "    Checker accepts state scaled by " << scale << std::endl;
                        std::cout << "    Accepted state: [" << scaled_state.transpose() << "]" << std::endl;
                        break;
                    }
                }
            } else {
                std::cout << "    ✓ MATCH! Your propagation agrees with checker." << std::endl;
            }
        }
    }
    
    std::cout << "\n========== END COMPARISON ==========\n" << std::endl;
}

// Also add this simpler direct comparison function
void directComparisonTest() {
    std::cout << "\n========== DIRECT STATE COMPARISON ==========\n" << std::endl;
    
    // Create a minimal path and see exactly what the checker expects
    KinodynamicProblem2D prob = HW9::getStateIntProblemWS1();
    
    KinoPath path;
    path.valid = true;
    
    // Very simple: start at init, apply one control
    Eigen::VectorXd start = prob.q_init;
    Eigen::VectorXd control(2);
    control << 1.0, 0.0;  // Simple: move right
    double dt = 0.1;
    
    // What YOUR propagator says
    MySingleIntegrator my_agent;
    Eigen::VectorXd my_end = start;
    my_agent.propagate(my_end, control, dt);
    
    std::cout << "Start state: [" << start.transpose() << "]" << std::endl;
    std::cout << "Control: [" << control.transpose() << "], dt: " << dt << std::endl;
    std::cout << "Your end state: [" << my_end.transpose() << "]" << std::endl;
    
    // Build path with your result
    path.waypoints.push_back(start);
    path.waypoints.push_back(my_end);
    path.controls.push_back(control);
    path.durations.push_back(dt);
    
    std::cout << "\nCalling HW9::check (it will print expected vs actual):" << std::endl;
    bool result = HW9::check(path, prob);
    
    if (!result) {
        std::cout << "\n❌ The checker's RK4 produces a different result than yours!" << std::endl;
        std::cout << "Look at the 'Expected' state printed above - that's what the checker computed." << std::endl;
    } else {
        std::cout << "\n✓ Your RK4 matches the checker's RK4!" << std::endl;
    }
    
    std::cout << "\n========================================\n" << std::endl;
}

void debugPathDetails(const KinoPath& path, const KinodynamicProblem2D& prob) {
    std::cout << "\n========== DETAILED PATH DEBUG ==========\n" << std::endl;
    
    std::cout << "Path validity: " << (path.valid ? "VALID" : "INVALID") << std::endl;
    std::cout << "Number of waypoints: " << path.waypoints.size() << std::endl;
    std::cout << "Number of controls: " << path.controls.size() << std::endl;
    std::cout << "Number of durations: " << path.durations.size() << std::endl;
    
    std::cout << "\nInitial state: [" << prob.q_init.transpose() << "]" << std::endl;
    std::cout << "Goal region: [" << prob.q_goal[0].first << ", " << prob.q_goal[0].second 
              << "] x [" << prob.q_goal[1].first << ", " << prob.q_goal[1].second << "]" << std::endl;
    
    std::cout << "\n--- WAYPOINTS ---" << std::endl;
    for (size_t i = 0; i < path.waypoints.size(); i++) {
        std::cout << "Waypoint " << i << ": [" << path.waypoints[i].transpose() << "]";
        if (i == 0) std::cout << " (START)";
        if (i == path.waypoints.size() - 1) std::cout << " (END)";
        std::cout << std::endl;
    }
    
    std::cout << "\n--- CONTROLS & DURATIONS ---" << std::endl;
    for (size_t i = 0; i < path.controls.size(); i++) {
        std::cout << "Step " << i << ": control=[" << path.controls[i].transpose() 
                  << "], duration=" << path.durations[i] << std::endl;
    }
    
    // Manual propagation check
    std::cout << "\n--- MANUAL PROPAGATION CHECK ---" << std::endl;
    auto agent = agentFactory[prob.agent_type]();
    
    Eigen::VectorXd sim_state = prob.q_init;
    std::cout << "Starting propagation from: [" << sim_state.transpose() << "]" << std::endl;
    
    for (size_t i = 0; i < path.controls.size()-1; i++) {
        Eigen::VectorXd prev_state = sim_state;
        
        // Create a non-const copy of the control vector
        Eigen::VectorXd control_copy = path.controls[i+1];
        
        agent->propagate(sim_state, control_copy, path.durations[i+1]);
        
        std::cout << "Step " << i << ": " << std::endl;
        std::cout << "  From: [" << prev_state.transpose() << "]" << std::endl;
        std::cout << "  Control: [" << path.controls[i+1].transpose() << "], dt=" << path.durations[i+1] << std::endl;
        std::cout << "  To: [" << sim_state.transpose() << "]" << std::endl;
        std::cout << "  Expected waypoint: [" << path.waypoints[i+1].transpose() << "]" << std::endl;
        
        double diff = (sim_state - path.waypoints[i+1]).norm();
        std::cout << "  Difference: " << diff << std::endl;
        
        if (diff > 1e-6) {
            std::cout << "  ❌ MISMATCH!" << std::endl;
        } else {
            std::cout << "  ✓ Match" << std::endl;
        }
        std::cout << std::endl;
    }
    
    std::cout << "Final simulated state: [" << sim_state.transpose() << "]" << std::endl;
    std::cout << "Final path waypoint: [" << path.waypoints.back().transpose() << "]" << std::endl;
    std::cout << "Difference: " << (sim_state - path.waypoints.back()).norm() << std::endl;
    
    // Check if final state is in goal
    bool in_goal = true;
    for (int i = 0; i < sim_state.size(); i++) {
        if (sim_state[i] < prob.q_goal[i].first || sim_state[i] > prob.q_goal[i].second) {
            in_goal = false;
            std::cout << "Dimension " << i << " out of bounds: " << sim_state[i] 
                      << " not in [" << prob.q_goal[i].first << ", " << prob.q_goal[i].second << "]" << std::endl;
        }
    }
    
    std::cout << "Final state in goal region: " << (in_goal ? "YES" : "NO") << std::endl;
    
    std::cout << "\n==========================================\n" << std::endl;
}
// Update your main function
int main(int argc, char** argv) {
    // Run the debugging tests first
    std::cout << "Running debugging tests..." << std::endl;
    directComparisonTest();
    
    // Select problem, plan, check, and visualize
    int select = 0;
    KinodynamicProblem2D prob = problems[select];
    
    std::cout << "\n========== PLANNING ==========\n" << std::endl;
    std::cout << "Problem " << select << " details:" << std::endl;
    std::cout << "Agent type: " << static_cast<int>(prob.agent_type) << std::endl;
    std::cout << "Initial state: [" << prob.q_init.transpose() << "]" << std::endl;
    std::cout << "Goal region: [" << prob.q_goal[0].first << ", " << prob.q_goal[0].second 
              << "] x [" << prob.q_goal[1].first << ", " << prob.q_goal[1].second << "]" << std::endl;
    
    MyKinoRRT kino_planner(0.05, 20, 5000);  // Reduced iterations for faster debugging
    KinoPath path = kino_planner.plan(prob, *agentFactory[prob.agent_type]());
    
    // Debug the path in detail
    debugPathDetails(path, prob);
    
    // Now run the checker
    std::cout << "\n========== CHECKER RESULTS ==========\n" << std::endl;
    bool check_result = HW9::check(path, prob);
    std::cout << "Checker result: " << (check_result ? "PASS" : "FAIL") << std::endl;
    
    if (path.valid) {
        Visualizer::makeFigure(prob, path, false);
    }
    
    Visualizer::saveFigures(true, "hw9_figs");
    
    // Uncomment this when you're ready for grading
    // HW9::grade<MyKinoRRT, MySingleIntegrator, MyFirstOrderUnicycle, MySecondOrderUnicycle, MySimpleCar>("firstName.lastName@colorado.edu", argc, argv, std::make_tuple(), std::make_tuple(), std::make_tuple(), std::make_tuple(), std::make_tuple());
    
    return 0;
}