// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework header
#include "hw/HW4.h"

// Include the headers for HW4 code
#include "CSpaceSkeleton.h"
#include "ManipulatorSkeleton.h"
#include "MyObstacle.h"

// Include the header of the shared class
#include "HelpfulClass.h"

using namespace amp;

// some Tests

void problem_1b(){


    std::vector<Eigen::Vector2d> ob_vertices = {
        Eigen::Vector2d(0.0, 0.0),
        Eigen::Vector2d(1.0, 2.0),
        Eigen::Vector2d(0.0, 2.0),
        Eigen::Vector2d(0.0, 0.0),
    };
    Polygon ob(ob_vertices);

    std::vector<Polygon> all_c_polygons;
    std::vector<double> heights;


    for (double theta = 0.0; theta < (2.0*M_PI); theta += M_PI/6.0){
        heights.push_back(theta);

        std::vector<Eigen::Vector2d> robo_vertices; 
        std::vector<Eigen::Vector2d> a; 
        std::vector<Eigen::Vector2d> c_vertices; 
        
        Eigen::Matrix2d T;

        T << std::cos(theta), -std::sin(theta),
            std::sin(theta),   std::cos(theta);


        for (const auto& point : ob_vertices){
            robo_vertices.push_back(T*point);
        }

        for (const auto& point : robo_vertices){
            a.push_back(-1.0*point);
        }

        // note, I forgot to sort -A to have the lowest y first, that really 
        // messed things up!
        // AI helped quite a lot with this rotation.
        auto start_it = std::min_element(a.begin(), a.end(), 
            [](const Eigen::Vector2d& p1, const Eigen::Vector2d& p2) {
                if (std::abs(p1[1] - p2[1]) < 1e-9) {
                    return p1[0] < p2[0];  // same y, choose leftmost
                }
                return p1[1] < p2[1];  // choose lowest y
            });
        
        // rotates so the bottom-left point is first
        std::rotate(a.begin(), start_it, a.end());
        a.push_back(a[0]);

        int a_index = 0;
        int b_index = 0;

        while (a_index < static_cast<int>(a.size()) && b_index < static_cast<int>(ob_vertices.size())){

            c_vertices.push_back(a[a_index] + ob_vertices[b_index]);

            std::cout<<"Point added: " << a[a_index] + ob_vertices[b_index] << "\n";

            int a_next = (a_index + 1) % static_cast<int>(a.size());
            int b_next = (b_index + 1) % static_cast<int>(ob_vertices.size());

            double a_theta = atan2(a[a_next][1] - a[a_index][1], a[a_next][0] - a[a_index][0]);
            double b_theta = atan2(ob_vertices[b_next][1] - ob_vertices[b_index][1], ob_vertices[b_next][0] - ob_vertices[b_index][0]);
            
            //AI helped with the next 9 lines
            auto normalizeAngle = [](double angle) -> double {
                angle = std::fmod(angle, 2.0 * M_PI);
                if (angle < 0) angle += 2.0 * M_PI;
                return angle;
            };
            
            a_theta = normalizeAngle(a_theta);
            b_theta = normalizeAngle(b_theta);
            std::cout<<"a_theta: " << a_theta << "\n";
            std::cout<<"b_theta: " << b_theta << "\n";
            
            if (b_theta < a_theta && std::abs(a_theta - b_theta) > 1e-9){
                std::cout<<"b plus \n";
                b_index +=1;
            } else if (a_theta < b_theta && std::abs(a_theta - b_theta) > 1e-9){
                std::cout<<"a plus \n";
                a_index +=1;
            } else {
                std::cout<<"both plus \n";
                a_index +=1;
                b_index +=1;
            }

        }

        all_c_polygons.push_back(Polygon(c_vertices));

    }


    Visualizer::makeFigure(all_c_polygons, heights);

    Visualizer::saveFigures(true, "hw4_prob1_figs");

}

void test_new_ob_func(){

    // tests obstacle with vertice definition
    std::vector<Eigen::Vector2d> vertices = {
        Eigen::Vector2d(0.0, 0.0),
        Eigen::Vector2d(1.0, 0.0),
        Eigen::Vector2d(0.5, 1.0)
    };
    MyObstacle test_ob2;
    test_ob2.defineWithPoints(vertices);
    std::cout << "test collision should be TRUE: " << test_ob2.collisionCheckAlongLine(Eigen::Vector2d(0.0, 0.5), Eigen::Vector2d(1.0, 0.5)) << std::endl;
    std::cout << "test collision should be TRUE: " << test_ob2.collisionCheckAlongLine(Eigen::Vector2d(1.0, 0.5), Eigen::Vector2d(0.0, 0.5)) << std::endl;
    std::cout << "test collision should be TRUE: " << test_ob2.collisionCheckAlongLine(Eigen::Vector2d(0.0, 0.0), Eigen::Vector2d(-1.0, -0.5)) << std::endl;
    std::cout << "test collision should be TRUE: " << test_ob2.collisionCheckAlongLine(Eigen::Vector2d(0.0, 0.9), Eigen::Vector2d(1.0, 0.9)) << std::endl;
    std::cout << "test collision should be FALSE: " << test_ob2.collisionCheckAlongLine(Eigen::Vector2d(0.0, 1.1), Eigen::Vector2d(1.0, 1.1)) << std::endl;
    std::cout << "test collision should be FALSE: " << test_ob2.collisionCheckAlongLine(Eigen::Vector2d(-0.01, -0.01), Eigen::Vector2d(-1.0, -0.5)) << std::endl;


}

void test_manipulator(const MyManipulator2D& manipulator) {

    // AI did lightly help with these tests
    // forward kinematics with zero angles
    amp::ManipulatorState zero_state;
    zero_state.setZero(manipulator.nLinks());
    std::cout << "joint angles: [";
    for (size_t i = 0; i < manipulator.nLinks(); ++i) {
        std::cout << zero_state[i];
        if (i < manipulator.nLinks() - 1) std::cout << ", ";
    }
    std::cout << "]\n";
    
    for (uint32_t i = 0; i < manipulator.nLinks(); ++i) {
        Eigen::Vector2d joint_pos = manipulator.getJointLocation(zero_state, i + 1);
        std::cout << "Joint " << i + 1 << " position: [" << joint_pos[0] << ", " << joint_pos[1] << "]\n";
    }
    
    std::cout << "\nTest 2: Forward Kinematics (90 degrees)\n";
    amp::ManipulatorState ninety_state;
    ninety_state.resize(manipulator.nLinks());
    for (size_t i = 0; i < manipulator.nLinks(); ++i) {
        ninety_state[i] = M_PI / 2.0;
    }
    std::cout << "Joint angles: [";
    for (size_t i = 0; i < manipulator.nLinks(); ++i) {
        std::cout << ninety_state[i];
        if (i < manipulator.nLinks() - 1) std::cout << ", ";
    }
    std::cout << "]\n";
    
    for (uint32_t i = 0; i < manipulator.nLinks(); ++i) {
        Eigen::Vector2d joint_pos = manipulator.getJointLocation(ninety_state, i + 1);
        std::cout << "Joint " << i + 1 << " position: [" << joint_pos[0] << ", " << joint_pos[1] << "]\n";
    }
    
 
    std::cout << "\nTest 3: Inverse Kinematics\n";
    
    Eigen::Vector2d target1(1.5, 0.0);
    std::cout << "Target: [" << target1[0] << ", " << target1[1] << "]\n";
    amp::ManipulatorState ik_result1 = manipulator.getConfigurationFromIK(target1);
    std::cout << "IK result angles: [";
    for (size_t i = 0; i < manipulator.nLinks(); ++i) {
        std::cout << ik_result1[i];
        if (i < manipulator.nLinks() - 1) std::cout << ", ";
    }
    std::cout << "]\n";
    
    // Verify by using FK on the IK result
    Eigen::Vector2d fk_check1 = manipulator.getJointLocation(ik_result1, manipulator.nLinks());
    std::cout << "FK verification: [" << fk_check1[0] << ", " << fk_check1[1] << "]\n";
    std::cout << "Error magnitude: " << (target1 - fk_check1).norm() << "\n";
    
    // Test target 2: Point in first quadrant
    Eigen::Vector2d target2(1.0, 1.0);
    std::cout << "\nTarget: [" << target2[0] << ", " << target2[1] << "]\n";
    amp::ManipulatorState ik_result2 = manipulator.getConfigurationFromIK(target2);
    std::cout << "IK result angles: [";
    for (size_t i = 0; i < manipulator.nLinks(); ++i) {
        std::cout << ik_result2[i];
        if (i < manipulator.nLinks() - 1) std::cout << ", ";
    }
    std::cout << "]\n";

    
    Eigen::Vector2d fk_check2 = manipulator.getJointLocation(ik_result2, manipulator.nLinks());
    std::cout << "FK verification: [" << fk_check2[0] << ", " << fk_check2[1] << "]\n";
    std::cout << "Error magnitude: " << (target2 - fk_check2).norm() << "\n";
    
    Eigen::Vector2d target3(5.0, 5.0);
    std::cout << "\nTarget (unreachable): [" << target3[0] << ", " << target3[1] << "]\n";
    amp::ManipulatorState ik_result3 = manipulator.getConfigurationFromIK(target3);
    std::cout << "IK result angles: [";
    for (size_t i = 0; i < manipulator.nLinks(); ++i) {
        std::cout << ik_result3[i];
        if (i < manipulator.nLinks() - 1) std::cout << ", ";
    }
    std::cout << "]\n";
    
    std::cout << "\n=== End Kinematics Tests ===\n\n";
}

void test_3linkmanipulator(const MyManipulator2D& manipulator) {

    // AI did lightly help with these tests
    // forward kinematics with zero angles
    amp::ManipulatorState zero_state;
    zero_state.setZero(manipulator.nLinks());
    std::cout << "joint angles: [";
    for (size_t i = 0; i < manipulator.nLinks(); ++i) {
        std::cout << zero_state[i];
        if (i < manipulator.nLinks() - 1) std::cout << ", ";
    }
    std::cout << "]\n";
    
    for (uint32_t i = 0; i < manipulator.nLinks(); ++i) {
        Eigen::Vector2d joint_pos = manipulator.getJointLocation(zero_state, i + 1);
        std::cout << "Joint " << i + 1 << " position: [" << joint_pos[0] << ", " << joint_pos[1] << "]\n";
    }
    
    std::cout << "\nTest 2: Forward Kinematics (90 degrees)\n";
    amp::ManipulatorState ninety_state;
    ninety_state.resize(manipulator.nLinks());
    for (size_t i = 0; i < manipulator.nLinks(); ++i) {
        ninety_state[i] = M_PI / 2.0;
    }
    std::cout << "Joint angles: [";
    for (size_t i = 0; i < manipulator.nLinks(); ++i) {
        std::cout << ninety_state[i];
        if (i < manipulator.nLinks() - 1) std::cout << ", ";
    }
    std::cout << "]\n";
    
    for (uint32_t i = 0; i < manipulator.nLinks(); ++i) {
        Eigen::Vector2d joint_pos = manipulator.getJointLocation(ninety_state, i + 1);
        std::cout << "Joint " << i + 1 << " position: [" << joint_pos[0] << ", " << joint_pos[1] << "]\n";
    }
    
 
    std::cout << "\nTest 3: Inverse Kinematics\n";
    
    Eigen::Vector2d target1(0.0, 4.0);
    std::cout << "Target: [" << target1[0] << ", " << target1[1] << "]\n";
    amp::ManipulatorState ik_result1 = manipulator.getConfigurationFromIK(target1);
    std::cout << "IK result angles: [";
    for (size_t i = 0; i < manipulator.nLinks(); ++i) {
        std::cout << ik_result1[i];
        if (i < manipulator.nLinks() - 1) std::cout << ", ";
    }
    std::cout << "]\n";
    
    // Verify by using FK on the IK result
    Eigen::Vector2d fk_check1 = manipulator.getJointLocation(ik_result1, manipulator.nLinks());
    std::cout << "FK verification: [" << fk_check1[0] << ", " << fk_check1[1] << "]\n";
    std::cout << "Error magnitude: " << (target1 - fk_check1).norm() << "\n";
    
    // Test target 2: Point in first quadrant
    Eigen::Vector2d target2(1.0, 1.0);
    std::cout << "\nTarget: [" << target2[0] << ", " << target2[1] << "]\n";
    amp::ManipulatorState ik_result2 = manipulator.getConfigurationFromIK(target2);
    std::cout << "IK result angles: [";
    for (size_t i = 0; i < manipulator.nLinks(); ++i) {
        std::cout << ik_result2[i];
        if (i < manipulator.nLinks() - 1) std::cout << ", ";
    }
    std::cout << "]\n";

    
    Eigen::Vector2d fk_check2 = manipulator.getJointLocation(ik_result2, manipulator.nLinks());
    std::cout << "FK verification: [" << fk_check2[0] << ", " << fk_check2[1] << "]\n";
    std::cout << "Error magnitude: " << (target2 - fk_check2).norm() << "\n";
    
    Eigen::Vector2d target3(5.0, 5.0);
    std::cout << "\nTarget (unreachable): [" << target3[0] << ", " << target3[1] << "]\n";
    amp::ManipulatorState ik_result3 = manipulator.getConfigurationFromIK(target3);
    std::cout << "IK result angles: [";
    for (size_t i = 0; i < manipulator.nLinks(); ++i) {
        std::cout << ik_result3[i];
        if (i < manipulator.nLinks() - 1) std::cout << ", ";
    }
    std::cout << "]\n";
    
    std::cout << "\n=== End Kinematics Tests ===\n\n";
}

void problem2() {

    MyManipulator2D manipulator3_a({0.5, 1.0, 0.5});

    // forward kinematics with zero angles
    amp::ManipulatorState a_state;
    a_state.setZero(manipulator3_a.nLinks());
    a_state[0] = M_PI / 6.0;
    a_state[1] = M_PI / 3.0;
    a_state[2] = (7.0 * M_PI) / 4.0;

    Visualizer::makeFigure(manipulator3_a, a_state); 
    


    MyManipulator2D manipulator3_b({1.0, 0.5, 1.0});
    Eigen::Vector2d targetb(2.0, 0.0);
    amp::ManipulatorState b_state;
    b_state.setZero(manipulator3_b.nLinks());

    b_state = manipulator3_b.getConfigurationFromIK(targetb);
    

    Visualizer::makeFigure(manipulator3_b, b_state); 

    Visualizer::saveFigures(true, "hw4_prob2_figs");

    
}


void test_gridspace() {

    std::size_t x0_cells = 100;
    std::size_t x1_cells = 100;
    double x0_min = 0.0;
    double x0_max = 100.0;
    double x1_min = 0.0;
    double x1_max = 100.0;

    MyGridCSpace2D GS(x0_cells, x1_cells, x0_min, x0_max, x1_min, x1_max);

    double x0 = 0.0;
    double x1 = 0.0;
    auto cell1 = GS.getCellFromPoint(x0, x1);
    std::cout << "Cell: [" << cell1.first << ", " << cell1.second << "]\n";
    double x0a = 0.1;
    double x1a = 0.1;
    auto cell2 = GS.getCellFromPoint(x0a, x1a);
    std::cout << "Cell: [" << cell2.first << ", " << cell2.second << "]\n";
    double x0b = 55.5;
    double x1b = 55.5;
    auto cell3 = GS.getCellFromPoint(x0b, x1b);
    std::cout << "Cell: [" << cell3.first << ", " << cell3.second << "]\n";

    double x0c = 55.5;
    double x1c = 5.5;
    auto cell4 = GS.getCellFromPoint(x0c, x1c);
    std::cout << "Cell: [" << cell4.first << ", " << cell4.second << "]\n";
}
//// end test


int main(int argc, char** argv) {
    /* Include this line to have different randomized environments every time you run your code (NOTE: this has no affect on grade()) */
    amp::RNG::seed(amp::RNG::randiUnbounded());
    // problem_1b();
    // problem2();
    // test_new_ob_func();
    // test_gridspace();

    // MyManipulator2D manipulator({1.0, 2.0});
    // test_manipulator(manipulator);

    // MyManipulator2D manipulator3({8.0, 8.0, 9.0});
    // test_3linkmanipulator(manipulator3);
    

    std::size_t n_cells = 1000;

    MyManipulatorCSConstructor cspace_constructor(n_cells);

    MyManipulator2D manipulator_prob_3({1.0, 1.0});
    // test_manipulator(manipulator_prob_3);
   
    std::unique_ptr<amp::GridCSpace2D> cspace = cspace_constructor.construct(manipulator_prob_3, HW4::getEx3Workspace3());
    Visualizer::makeFigure(HW4::getEx3Workspace3());
    Visualizer::makeFigure(*cspace);
    
    Visualizer::saveFigures(true, "hw4_figs");
    
    amp::HW4::grade<MyManipulator2D>(cspace_constructor, "owen.kranz@colorado.edu", argc, argv);
    
    return 0;
}