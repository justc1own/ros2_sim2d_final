#include <iostream>
#include <vector>
#include <cmath>
#include "sim2d_hardware_interface/kinematics_solver.hpp"

// --- Raptor Mini Unit Test ---
// This main function serves as a simple unit test.
// It will be compiled as a standalone executable.
int main(int argc, char ** argv)
{
  (void)argc;
  (void)argv;

  std::cout << "--- Running KinematicSolver Unit Test: Skid-Steer Straight Motion ---" << std::endl;

  // 1. Configure a 4-wheel skid-steer robot
  // Symmetric positions, all wheels facing forward
  const double wheel_x = 0.2; // distance from center to wheel axle
  const double wheel_y = 0.3; // distance from center to wheel contact
  const double wheel_radius = 0.05;
  const double slip_alpha = 0.95; // 5% slip

  std::vector<sim2d_hardware_interface::Wheel> wheels = {
    // Front-Left
    {wheel_x, wheel_y, 0.0, wheel_radius, slip_alpha, 0.1, 0.1},
    // Front-Right
    {wheel_x, -wheel_y, 0.0, wheel_radius, slip_alpha, 0.1, 0.1},
    // Rear-Left
    {-wheel_x, wheel_y, 0.0, wheel_radius, slip_alpha, 0.1, 0.1},
    // Rear-Right
    {-wheel_x, -wheel_y, 0.0, wheel_radius, slip_alpha, 0.1, 0.1}
  };

  // For skid-steer, ICR is non-zero during turns, but for straight motion, we expect vy=0.
  // Let's set x_icr = 0 for this test.
  double x_icr = 0.0;

  // 2. Create the solver
  sim2d_hardware_interface::KinematicSolver solver(wheels, x_icr);

  // 3. Define commands: all wheels moving at the same speed
  const double target_speed_rad_s = 10.0; // rad/s
  std::vector<double> wheel_speeds = {target_speed_rad_s, target_speed_rad_s, target_speed_rad_s, target_speed_rad_s};
  std::vector<double> steering_angles = {0.0, 0.0, 0.0, 0.0}; // No steering

  // 4. Solve for body velocity
  double vx, vy, omega;
  solver.solve(wheel_speeds, steering_angles, vx, vy, omega);

  // 5. Assert results
  std::cout << "Calculated Velocities:" << std::endl;
  std::cout << "  vx: " << vx << " m/s" << std::endl;
  std::cout << "  vy: " << vy << " m/s" << std::endl;
  std::cout << "  omega: " << omega << " rad/s" << std::endl;

  bool test_passed = true;
  double tolerance = 1e-6;

  // Expected vx = wheel_radius * speed * slip_alpha
  double expected_vx = wheel_radius * target_speed_rad_s * slip_alpha;
  if (std::abs(vx - expected_vx) > tolerance) {
    std::cerr << "[FAIL] vx is incorrect. Expected ~" << expected_vx << std::endl;
    test_passed = false;
  }

  // For straight motion, vy and omega should be near zero
  if (std::abs(vy) > tolerance) {
    std::cerr << "[FAIL] vy should be near zero for straight motion." << std::endl;
    test_passed = false;
  }
  if (std::abs(omega) > tolerance) {
    std::cerr << "[FAIL] omega should be near zero for straight motion." << std::endl;
    test_passed = false;
  }

  std::cout << "--------------------------------------------------------------------" << std::endl;
  if (test_passed) {
    std::cout << "[SUCCESS] Test passed. Skid-steer robot moves straight as expected." << std::endl;
    return 0;
  } else {
    std::cerr << "[FAILURE] Test failed. Kinematics calculation is incorrect." << std::endl;
    return 1;
  }
}
