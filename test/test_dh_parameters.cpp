#include "../include/standard_includes.h"
#include <Eigen/Dense>
#include <iostream>
#include <cassert>
#include <cmath>

int main() {
    const double coxa = 0.05;  // 50 mm
    const double femur = 0.101; // 101 mm
    const double tibia = 0.208; // 208 mm

    Eigen::Matrix4d T1 = createDHMatrix(0.0, 0.0, coxa, -M_PI / 2.0);
    Eigen::Matrix4d T2 = createDHMatrix(0.0, 0.0, femur, 0.0);
    Eigen::Matrix4d T3 = createDHMatrix(0.0, 0.0, tibia, 0.0);

    Eigen::Matrix4d T = T1 * T2 * T3;
    Eigen::Vector3d pos = T.block<3,1>(0,3);

    double expected_x = coxa + femur + tibia;
    assert(std::abs(pos.x() - expected_x) < 1e-6);
    assert(std::abs(pos.y()) < 1e-6);
    assert(std::abs(pos.z()) < 1e-6);

    std::cout << "Tip position: " << pos.transpose() << std::endl;
    return 0;
}
