#include "kinematik.h"
#include "matrix_utils.h"
#include "kinematik.h"
#include "matrix_utils.h"
#include <Eigen/Dense>
#include <stdexcept>

static inline void checkNonZero(double v, const char* name) {
  if (v == 0.0) {
    throw std::invalid_argument(std::string(name) + " must be non-zero");
  }
}

void mecanum_matrix(Eigen::MatrixXd& kinematik, Eigen::MatrixXd& kinematikInv,
                    const double l1, const double l2) {
  const auto Jv = getMecanumMatrix();                // 4x3, sign/template only
  const auto Kv = getInverseMecanumMatrix();         // 3x4, sign/template only
  const double sJ = getScalarMecanumMatrix();
  const double sK = getScalarInverseMecanumMatrix();

  const double L = l1 + l2;
  checkNonZero(L, "l1 + l2");

  // J: copy and scale ω-column by (l1 + l2)
  kinematik.resize(4, 3);
  for (int r = 0; r < 4; ++r)
    for (int c = 0; c < 3; ++c)
      kinematik(r, c) = Jv[r][c];
  kinematik.col(2) *= L;     // ω column
  kinematik *= sJ;

  // J^{-1}: copy and scale ω-row by 1/(l1 + l2)
  kinematikInv.resize(3, 4);
  for (int r = 0; r < 3; ++r)
    for (int c = 0; c < 4; ++c)
      kinematikInv(r, c) = Kv[r][c];
  kinematikInv.row(2) /= (1.0 / L);   // ω row
  kinematikInv *= sK;
}

void differential_matrix(Eigen::MatrixXd& kinematik, Eigen::MatrixXd& kinematikInv,
                         const double l1, const double l2) {
  const auto Jv = getDifferentialMatrix();           // 4x3, sign/template only
  const auto Kv = getInverseDifferentialMatrix();    // 3x4, sign/template only
  const double sJ = getScalarDifferentialMatrix();
  const double sK = getScalarInverseDifferentialMatrix();

  checkNonZero(l2, "l2");

  // J: copy and scale ω-column by l2
  kinematik.resize(4, 3);
  for (int r = 0; r < 4; ++r)
    for (int c = 0; c < 3; ++c)
      kinematik(r, c) = Jv[r][c];
  kinematik.col(2) *= l2;    // ω column
  kinematik *= sJ;

  // J^{-1}: copy and scale ω-row by 1/l2
  kinematikInv.resize(3, 4);
  for (int r = 0; r < 3; ++r)
    for (int c = 0; c < 4; ++c)
      kinematikInv(r, c) = Kv[r][c];
  kinematikInv.row(2) *= (1.0 / l2);  // ω row
  kinematikInv *= sK;
}

void omni_4_matrix(Eigen::MatrixXd& kinematik, Eigen::MatrixXd& kinematikInv,
                   const double l1, const double l2) {
  const auto Jv = getOmniFourMatrix();               // 4x3, sign/template only
  const auto Kv = getInverseOmniFourMatrix();        // 3x4, sign/template only
  const double sJ = getScalarOmniFourMatrix();
  const double sK = getScalarInverseOmniFourMatrix();

  const double L = l1 + l2;
  checkNonZero(L, "l1 + l2");

  // J: copy and scale ω-column by (l1 + l2)
  kinematik.resize(4, 3);
  for (int r = 0; r < 4; ++r)
    for (int c = 0; c < 3; ++c)
      kinematik(r, c) = Jv[r][c];
  kinematik.col(2) *= L;     // ω column
  kinematik *= sJ;

  // J^{-1}: copy and scale ω-row by 1/(l1 + l2)
  kinematikInv.resize(3, 4);
  for (int r = 0; r < 3; ++r)
    for (int c = 0; c < 4; ++c)
      kinematikInv(r, c) = Kv[r][c];
  kinematikInv.row(2) *= (1.0 / L);   // ω row
  kinematikInv *= sK;
}

void omni_3_matrix(Eigen::MatrixXd& kinematik, Eigen::MatrixXd& kinematikInv,
                   const double l1, const double /*l2*/) {
  const auto Jv = getOmniThreeMatrix();              // 3x3, sign/template only
  const auto Kv = getInverseOmniThreeMatrix();       // 3x3, sign/template only
  const double sJ = getScalarOmniThreeMatrix();
  const double sK = getScalarInverseOmniThreeMatrix();

  checkNonZero(l1, "l1");

  // J: copy and scale ω-column by (-l1) like the original
  kinematik.resize(3, 3);
  for (int r = 0; r < 3; ++r)
    for (int c = 0; c < 3; ++c)
      kinematik(r, c) = Jv[r][c];
  // The template Jv should carry the signs; if it matches your original
  // (-l1, -l1, -l1) pattern, multiplying by l1 is enough:
  kinematik.col(2) *= l1;    // ω column
  kinematik *= sJ;

  // J^{-1}: copy and scale ω-row by 1/l1 (template carries the signs)
  kinematikInv.resize(3, 3);
  for (int r = 0; r < 3; ++r)
    for (int c = 0; c < 3; ++c)
      kinematikInv(r, c) = Kv[r][c];
  kinematikInv.row(2) *= (1.0 / l1);  // ω row
  kinematikInv *= sK;
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
