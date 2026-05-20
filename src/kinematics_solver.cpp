#include "sim2d_hardware_interface/kinematics_solver.hpp"
#include <iostream>
#include <cmath>
#include <numeric>
#include <limits>

namespace sim2d_hardware_interface
{

KinematicSolver::KinematicSolver(
  const std::vector<Wheel> & wheels,
  double icr_offset_x,
  double weight_icr)
: wheels_(wheels),
  icr_offset_x_(icr_offset_x),
  weight_icr_(weight_icr),
  num_wheels_(wheels.size())
{
}

void KinematicSolver::solve(
  const std::vector<double> & wheel_speeds,
  const std::vector<double> & steering_angles,
  double & vx,
  double & vy,
  double & omega)
{
  if (wheel_speeds.size() != num_wheels_ || steering_angles.size() != num_wheels_) {
    // Handle error: input vector sizes do not match number of wheels
    vx = 0.0;
    vy = 0.0;
    omega = 0.0;
    return;
  }

  // STEP 2: Calculate path curvature lambda
  double lambda = computeCurvature(wheel_speeds);

  // STEP 3: Calculate effective slip for each wheel
  std::vector<double> alpha_eff = computeEffectiveSlip(lambda);

  // STEP 4-5: Assemble and solve the system for v = [vx, vy, omega]
  Eigen::Vector3d v = assembleAndSolve(wheel_speeds, steering_angles, alpha_eff);

  vx = v(0);
  vy = v(1);
  omega = v(2);
}

double KinematicSolver::computeCurvature(const std::vector<double> & wheel_speeds) const
{
  double omega_left = 0.0, omega_right = 0.0;
  int left_count = 0, right_count = 0;

  for (size_t i = 0; i < num_wheels_; ++i) {
    if (wheels_[i].position_y > 0) { // Left side
      omega_left += wheel_speeds[i];
      left_count++;
    } else if (wheels_[i].position_y < 0) { // Right side
      omega_right += wheel_speeds[i];
      right_count++;
    }
  }

  if (left_count > 0) omega_left /= left_count;
  if (right_count > 0) omega_right /= right_count;

  double omega_outer = std::max(std::abs(omega_left), std::abs(omega_right));
  double omega_inner = std::min(std::abs(omega_left), std::abs(omega_right));

  if (std::abs(omega_outer - omega_inner) < 1e-6) {
    return std::numeric_limits<double>::infinity(); // Driving straight
  }

  return std::abs((omega_outer + omega_inner) / (omega_outer - omega_inner));
}

std::vector<double> KinematicSolver::computeEffectiveSlip(double lambda) const
{
  std::vector<double> alpha_eff(num_wheels_);
  for (size_t i = 0; i < num_wheels_; ++i) {
    if (std::isinf(lambda)) {
      alpha_eff[i] = wheels_[i].alpha_slip;
    } else {
      double correction = wheels_[i].beta1_roc / (1.0 + wheels_[i].beta2_roc * std::sqrt(lambda));
      alpha_eff[i] = wheels_[i].alpha_slip * (1.0 - correction);
    }
  }
  return alpha_eff;
}

Eigen::Vector3d KinematicSolver::assembleAndSolve(
  const std::vector<double> & wheel_speeds,
  const std::vector<double> & steering_angles,
  const std::vector<double> & alpha_eff) const
{
  // System is Av = b, where v = [vx, vy, omega]^T
  // One extra row for the ICR constraint
  Eigen::MatrixXd A(num_wheels_ + 1, 3);
  Eigen::VectorXd b(num_wheels_ + 1);
  Eigen::VectorXd W(num_wheels_ + 1); // Weights

  for (size_t i = 0; i < num_wheels_; ++i) {
    const auto & wheel = wheels_[i];
    double theta_i = wheel.mounting_angle + steering_angles[i];
    double cos_theta = std::cos(theta_i);
    double sin_theta = std::sin(theta_i);

    // Rolling constraint row for wheel i
    A(i, 0) = cos_theta;
    A(i, 1) = sin_theta;
    A(i, 2) = -wheel.position_y * cos_theta + wheel.position_x * sin_theta;

    b(i) = wheel.radius * alpha_eff[i] * wheel_speeds[i];
    W(i) = 1.0; // Weight for rolling constraints
  }

  // ICR constraint row
  A(num_wheels_, 0) = 0.0;
  A(num_wheels_, 1) = 1.0;
  A(num_wheels_, 2) = -icr_offset_x_;
  b(num_wheels_) = 0.0;
  W(num_wheels_) = weight_icr_;

  // Solve the weighted least-squares problem: min ||W(Av - b)||^2
  // This is equivalent to solving (W*A)v = (W*b)
  Eigen::MatrixXd WA = W.asDiagonal() * A;
  Eigen::VectorXd Wb = W.asDiagonal() * b;

  // Using SVD is robust for potentially rank-deficient or ill-conditioned systems
  return WA.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(Wb);
}

} // namespace sim2d_hardware_interface
