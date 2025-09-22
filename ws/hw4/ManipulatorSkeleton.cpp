#include "ManipulatorSkeleton.h"


MyManipulator2D::MyManipulator2D()
    : LinkManipulator2D({1.0, 1.0}) // Default to a 2-link with all links of 1.0 length
{}

// other constructor
MyManipulator2D::MyManipulator2D(const std::vector<double>& link_lengths)
    : LinkManipulator2D(link_lengths)
{}

// Override this method for implementing forward kinematics
Eigen::Vector2d MyManipulator2D::getJointLocation(const amp::ManipulatorState& state, uint32_t joint_index) const {
    // Implement forward kinematics to calculate the joint position given the manipulator state (angles)
    if (joint_index == 0 || joint_index > nLinks()) { //error check
        return Eigen::Vector2d(0.0, 0.0);
    }
    std::vector<Eigen::Matrix3d> homogeneous_T_mats; // can init to length n

    //std::cout << "state: "<<state <<"\n";

    for (uint32_t i = 0; i<joint_index; ++i){
        const double theta = state[i];
        const double prev_link_len = (i>0) ? m_link_lengths[i-1] : 0.0;
        //std::cout << "link length: "<<prev_link_len << "\n";
        Eigen::Matrix3d T_prev_to_next;
        T_prev_to_next << std::cos(theta), -std::sin(theta), prev_link_len,
                        std::sin(theta),   std::cos(theta),   0.0,
                        0.0,             0.0,          1.0;

        
        // insert into the vector of T
        homogeneous_T_mats.push_back(T_prev_to_next);
    }

    //gotta add one more matrix. Just translate end effector to end of last link
    Eigen::Matrix3d T_to_end;
    //std::cout << "last ?link length: "<<m_link_lengths[joint_index-1] << "\n";
    T_to_end << 1.0, 0.0, m_link_lengths[joint_index-1],
                    0.0,   1.0,   0.0,
                    0.0, 0.0, 1.0;  
    homogeneous_T_mats.push_back(T_to_end);

    Eigen::Vector3d joint_position_homogeneous(0.0, 0.0, 1.0);

    for (int i = static_cast<int>(homogeneous_T_mats.size()) - 1; i>=0; --i){
        joint_position_homogeneous = homogeneous_T_mats[i] * joint_position_homogeneous;
    }

    return joint_position_homogeneous.head<2>();
}

// Override this method for implementing inverse kinematics
amp::ManipulatorState MyManipulator2D::getConfigurationFromIK(const Eigen::Vector2d& end_effector_location) const {
    // Implement inverse kinematics here

    amp::ManipulatorState joint_angles;
    joint_angles.setZero(nLinks());
    
    // If you have different implementations for 2/3/n link manipulators, you can separate them here
    Eigen::Vector2d end_effector_location_copy = end_effector_location;
    double a1 = m_link_lengths[0];
    double a2 = m_link_lengths[1];
    if (nLinks() == 3) {

        double theta_1 = 0; // need to discretize later
        joint_angles[0] = theta_1;
        a1 = m_link_lengths[1];
        a2 = m_link_lengths[2]; // increment these

        Eigen::Matrix3d T; // transforms the goal pos so we can use 2link eqns

        T << std::cos(theta_1), std::sin(theta_1), -m_link_lengths[0],
                        -std::sin(theta_1),   std::cos(theta_1),   0.0,
                        0.0,             0.0,          1.0;
        
        Eigen::Vector3d goal_position_homogeneous(end_effector_location[0], end_effector_location[1], 1.0);
        goal_position_homogeneous = T * goal_position_homogeneous;
        end_effector_location_copy = goal_position_homogeneous.head<2>();
    } else if (nLinks() > 3) {
         // n joints?? huh?? not in scope of this hw
        return joint_angles;
    }

    double cos_theta_1; // note, if 3 link, we will say theta 1 has already been set
    double cos_theta_2;
    double sin_theta_1; 
    double sin_theta_2;
    double x = end_effector_location_copy[0];
    double y = end_effector_location_copy[1];



    cos_theta_2 = (1.0/(2.0*a1*a2))*((x*x + y*y) - (a1*a1 + a2*a2));

    if (std::abs(cos_theta_2) > 1.0) {
        // no solution exists - robo cant reach dat
        LOG("robo cant reach :/");
        return joint_angles;  // Returns zeros
    }
    sin_theta_2 = std::sqrt(1.0 - cos_theta_2*cos_theta_2); 

    cos_theta_1 = (1.0/(x*x + y*y))*((x*(a1+(a2*cos_theta_2))) + (y*a2*sin_theta_2));
    sin_theta_1 = (1.0/(x*x + y*y))*((y*(a1+(a2*cos_theta_2))) - (x*a2*sin_theta_2));


    double theta_1 = std::atan2(sin_theta_1, cos_theta_1);
    double theta_2 = std::atan2(sin_theta_2, cos_theta_2);

    if (nLinks() == 3) {
        joint_angles[1] = theta_1;
        joint_angles[2] = theta_2;
    }else{
        joint_angles[0] = theta_1;
        joint_angles[1] = theta_2;
        }


    return joint_angles;
}