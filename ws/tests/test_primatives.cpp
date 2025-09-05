#include <iostream>
#include "Primative.h"



int main() {

    // tests that this primative thing works
    LinearPrimative test_prim;
    test_prim.point_a = Eigen::Vector2d(0, 0);
    test_prim.point_b = Eigen::Vector2d(1, 1);
    std::cout << "test point should be <0: " << lp1.evaluatePoint(Eigen::Vector2d(1, 0)) << std::endl;
    std::cout << "test point should be 0: " << lp1.evaluatePoint(Eigen::Vector2d(1, 1)) << std::endl;
    std::cout << "test point should be >0: " << lp1.evaluatePoint(Eigen::Vector2d(0, 1)) << std::endl;


    return 0
}