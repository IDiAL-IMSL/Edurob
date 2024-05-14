#include "kinematik.h"

// Edit this file to change the Kinematiks of the Robots

void mecanum_matrix(Eigen::MatrixXd& kinematik ,Eigen::MatrixXd& kinematikInv, const double l1, const double l2){
    
  kinematik << 1, 1, (l1 + l2),
    1, -1, -(l1 + l2),
    1, 1, -(l1 + l2),
    1, -1, (l1 + l2);

  kinematikInv << 1, 1, 1, 1,
    1, -1, 1, -1,
    1.0 / (l1 + l2), -1.0 / (l1 + l2), -1.0 / (l1 + l2), 1.0 / (l1 + l2);

  kinematik = 1 * kinematik;
  kinematikInv = (1 / 4.0) * kinematikInv;
}

void differential_matrix(Eigen::MatrixXd& kinematik ,Eigen::MatrixXd& kinematikInv, const double l1, const double l2){
  kinematik << 1, 0, l2,
    1, 0, -l2,
    1, 0, -l2,
    1, 0, l2;

  kinematikInv << 1, 1, 1, 1,
    0, 0, 0, 0,
    1.0 / l2, -1.0 / l2, -1.0 / l2, 1.0 / l2;

  kinematik = 1 * kinematik;
  kinematikInv = (1 / 4.0) * kinematikInv;

}

void omni_4_matrix(Eigen::MatrixXd& kinematik ,Eigen::MatrixXd& kinematikInv, const double l1, const double l2){
  kinematik << 1, 1, l1 + l2,
    1, -1, -l1 - l2,
    1, 1, -l1 - l2,
    1, -1, l1 + l2;

  kinematikInv << 1, 1, 1, 1,
    1, -1, 1, -1,
    1 / (l1 + l2), -1 / (l1 + l2), -1 / (l1 + l2), 1 / (l1 + l2);

  kinematik = ((sqrt(2) / (2.0))) * kinematik;
  kinematikInv = (sqrt(2)/(4.0)) * kinematikInv;
}

void omni_3_matrix(Eigen::MatrixXd& kinematik ,Eigen::MatrixXd& kinematikInv, const double l1, const double l2){ //l1 and l2 should be equal when working with 3 Omni Wheels
  kinematik << -sqrt(3)/2.0, -1.0/2.0, -l1,
    sqrt(3)/2.0, -1.0/2.0, -l1,
    0, 1.0, -l1;

  kinematikInv << -sqrt(3), sqrt(3), 0,
    -1.0, -1.0, 2.0,
    -1.0/l1, -1.0/l1, -1.0/l1;

  kinematik = (1) * kinematik;
  kinematikInv = (1/3.0) * kinematikInv;
}



Eigen::Vector3d robot_vel_to_world_vel(const double theta, const Eigen::Vector3d robot_vel){
    Eigen::Matrix3d robot_to_world;
    robot_to_world << cos(theta), -sin(theta), 0,
                    sin(theta), cos(theta), 0,
                    0, 0, 1;
    return robot_to_world * robot_vel;
}

Eigen::Vector3d world_vel_to_robot_vel(const double theta, const Eigen::Vector3d world_vel){
    Eigen::Matrix3d world_to_robot;
    world_to_robot << cos(theta), sin(theta), 0,
                    -sin(theta), cos(theta), 0,
                    0, 0, 1;
    return world_to_robot * world_vel;
}
