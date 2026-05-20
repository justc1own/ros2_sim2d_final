#ifndef SIM2D_HARDWARE_INTERFACE__KINEMATICS_SOLVER_HPP_
#define SIM2D_HARDWARE_INTERFACE__KINEMATICS_SOLVER_HPP_

#include <vector>
#include <Eigen/Dense>

namespace sim2d_hardware_interface
{

// Structure to hold all parameters for a single wheel
struct Wheel
{
  // Kinematic parameters from URDF
  double position_x;      // x-position in base_link frame
  double position_y;      // y-position in base_link frame
  double mounting_angle;  // mounting angle (delta_i) in radians
  double radius;          // wheel radius

  // Slip parameters from URDF/config
  double alpha_slip;      // Base longitudinal slip coefficient
  double beta1_roc;       // ROC correction parameter 1
  double beta2_roc;       // ROC correction parameter 2
};

class KinematicSolver
{
public:
  /**
   * @brief Constructor for the KinematicSolver.
   * @param wheels A vector of Wheel structures, defining the geometry and slip params for each wheel.
   * @param icr_offset_x The lateral offset of the Instantaneous Center of Rotation (ICR).
   * @param weight_icr The weight for the ICR constraint in the least-squares problem.
   */
  KinematicSolver(
    const std::vector<Wheel> & wheels,
    double icr_offset_x,
    double weight_icr = 100.0);

  /**
   * @brief Solves the inverse kinematics problem to find the robot's body velocity.
   * @param wheel_speeds A vector of angular velocities (rad/s) for each wheel.
   * @param steering_angles A vector of steering angles (rad) for each wheel (0 for fixed wheels).
   * @param vx Output: calculated longitudinal velocity (m/s).
   * @param vy Output: calculated lateral velocity (m/s).
   * @param omega Output: calculated angular velocity (rad/s).
   */
  void solve(
    const std::vector<double> & wheel_speeds,
    const std::vector<double> & steering_angles,
    double & vx,
    double & vy,
    double & omega);

private:
  /**
   * @brief Computes the path curvature (lambda) based on wheel speeds.
   * @param wheel_speeds Vector of wheel angular velocities.
   * @return The calculated path curvature lambda.
   */
  double computeCurvature(const std::vector<double> & wheel_speeds) const;

  /**
   * @brief Computes the effective slip coefficient for each wheel based on curvature.
   * @param lambda The path curvature.
   * @return A vector of effective slip coefficients (alpha_eff_i).
   */
  std::vector<double> computeEffectiveSlip(double lambda) const;

  /**
   * @brief Assembles and solves the weighted least-squares system Av=b.
   * @param wheel_speeds Vector of wheel angular velocities.
   * @param steering_angles Vector of wheel steering angles.
   * @param alpha_eff Vector of effective slip coefficients.
   * @return An Eigen::Vector3d containing [vx, vy, omega].
   */
  Eigen::Vector3d assembleAndSolve(
    const std::vector<double> & wheel_speeds,
    const std::vector<double> & steering_angles,
    const std::vector<double> & alpha_eff) const;

  std::vector<Wheel> wheels_;
  double icr_offset_x_;
  double weight_icr_;
  size_t num_wheels_;
};

}  // namespace sim2d_hardware_interface

#endif  // SIM2D_HARDWARE_INTERFACE__KINEMATICS_SOLVER_HPP_
