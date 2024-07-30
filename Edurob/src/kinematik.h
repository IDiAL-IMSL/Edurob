#ifndef KINEMATIK_H
#define KINEMATIK_H

#include <Eigen.h> 
// #include "parameter.h"

void mecanum_matrix(Eigen::MatrixXd& kinematik ,Eigen::MatrixXd& kinematikInv, const double l1, const double l2);
void differential_matrix(Eigen::MatrixXd& kinematik ,Eigen::MatrixXd& kinematikInv, const double l1, const double l2);
void omni_4_matrix(Eigen::MatrixXd& kinematik ,Eigen::MatrixXd& kinematikInv, const double l1, const double l2);
void omni_3_matrix(Eigen::MatrixXd& kinematik ,Eigen::MatrixXd& kinematikInv, const double l1, const double l2);


Eigen::Vector3d robot_vel_to_world_vel(const double theta, const Eigen::Vector3d);
Eigen::Vector3d world_vel_to_robot_vel(const double theta, const Eigen::Vector3d);

#endif