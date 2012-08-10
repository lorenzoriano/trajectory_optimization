/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/** \author Mrinal Kalakrishnan */

#ifndef TRAJECTORY_OPTIMIZER_H_
#define TRAJECTORY_OPTIMIZER_H_

#include <trajectory_optimization/trajectory_parameters.h>
#include <trajectory_optimization/trajectory.h>
#include <trajectory_optimization/trajectory_cost.h>
#include <trajectory_optimization/multivariate_gaussian.h>
#include <planning_models/kinematic_model.h>
#include <collision_proximity/collision_proximity_space.h>

#include <eigen3/Eigen/Core>

#include <vector>

namespace trajopt
{

class TrajectoryOptimizer
{
public:
  TrajectoryOptimizer(Trajectory *trajectory, planning_models::KinematicModel *robot_model,
      const std::string& planning_group, const TrajectoryParameters *parameters,
      const ros::Publisher& vis_marker_array_publisher,
      const ros::Publisher& vis_marker_publisher,
      collision_proximity::CollisionProximitySpace *collision_space);
  TrajectoryOptimizer(Trajectory *trajectory, planning_models::KinematicModel *robot_model,
      const std::string& planning_group, const TrajectoryParameters *parameters,
      const ros::Publisher& vis_marker_array_publisher,
      const ros::Publisher& vis_marker_publisher,
      collision_proximity::CollisionProximitySpace *collision_space,
      std::vector<double> weights);
  virtual ~TrajectoryOptimizer();

  void optimize();

  inline void destroy()
  {
    //Nothing for now.
  }
private:

  inline double getPotential(double field_distance, double radius, double clearence)
  {
    double d = field_distance - radius;
    double potential = 0.0;

    // three cases below:
    if (d >= clearence)
    {
      potential = 0.0;
    }
    else if (d >= 0.0)
    {
      double diff = (d - clearence);
      double gradient_magnitude = diff * clearence; // (diff / clearance)
      potential = 0.5*gradient_magnitude*diff;
    }
    else // if d < 0.0
    {
      potential = -d + 0.5 * clearence;
    }

    return potential;
  }
  template<typename Derived>
  void getJacobian(int trajectoryPoint,Eigen::Vector3d& collision_point_pos, std::string& jointName, Eigen::MatrixBase<Derived>& jacobian) const;

  void getRandomState(const planning_models::KinematicState* currentState, const std::string& groupName,
                      Eigen::VectorXd& state_vec);

  void setRobotStateFromPoint(Trajectory& group_trajectory, int i);

  collision_proximity::CollisionProximitySpace::TrajectorySafety checkCurrentIterValidity();

  Trajectory reference_trajectory_;
  std::vector<double> reference_weights_;

  int num_joints_;
  int num_vars_free_;
  int num_vars_all_;
  int num_collision_points_;
  int free_vars_start_;
  int free_vars_end_;
  int iteration_;
  unsigned int collision_free_iteration_;
  Trajectory *full_trajectory_;
  planning_models::KinematicModel *robot_model_;
  planning_models::KinematicState *robot_state_;
  const std::string& planning_group_;
  const TrajectoryParameters *parameters_;
  collision_proximity::CollisionProximitySpace *collision_space_;
  Trajectory group_trajectory_;
  std::vector<TrajectoryCost> joint_costs_;

  std::vector<std::vector<std::string> > collision_point_joint_names_;
  std::vector<std::vector<Eigen::Vector3d > > collision_point_pos_eigen_;
  std::vector<std::vector<Eigen::Vector3d > > collision_point_vel_eigen_;
  std::vector<std::vector<Eigen::Vector3d > > collision_point_acc_eigen_;
  std::vector<std::vector<double> > collision_point_potential_;
  std::vector<std::vector<double> > collision_point_vel_mag_;
  std::vector<std::vector<Eigen::Vector3d> > collision_point_potential_gradient_;
  std::vector<std::vector<tf::Vector3> > joint_axes_;
  std::vector<std::vector<tf::Vector3> > joint_positions_;
  Eigen::MatrixXd group_trajectory_backup_;
  Eigen::MatrixXd best_group_trajectory_;
  double best_group_trajectory_cost_;
  int last_improvement_iteration_;
  unsigned int num_collision_free_iterations_;

  // HMC stuff:
  Eigen::MatrixXd momentum_;
  Eigen::MatrixXd random_momentum_;
  Eigen::VectorXd random_joint_momentum_; //temporary variable
  std::vector<MultivariateGaussian> multivariate_gaussian_;
  double stochasticity_factor_;

  std::vector<int> state_is_in_collision_;      /**< Array containing a boolean about collision info for each point in the trajectory */
  std::vector<std::vector<int> > point_is_in_collision_;
  bool is_collision_free_;
  double worst_collision_cost_state_;

  Eigen::MatrixXd smoothness_increments_;
  Eigen::MatrixXd collision_increments_;
  Eigen::MatrixXd distance_increments_;
  Eigen::MatrixXd final_increments_;

  // temporary variables for all functions:
  Eigen::VectorXd smoothness_derivative_;
  Eigen::MatrixXd jacobian_;
  Eigen::MatrixXd jacobian_pseudo_inverse_;
  Eigen::MatrixXd jacobian_jacobian_tranpose_;
  Eigen::VectorXd random_state_;
  Eigen::VectorXd joint_state_velocities_;

  ros::Publisher vis_marker_array_pub_;
  ros::Publisher vis_marker_pub_;

  std::vector<std::string> joint_names_;
  std::map<std::string, std::map<std::string, bool> > joint_parent_map_;

  inline bool isParent(const std::string& childLink, const std::string& parentLink) const
  {
    if(childLink == parentLink)
    {
      return true;
    }

    if(joint_parent_map_.find(childLink) == joint_parent_map_.end())
    {
      //ROS_ERROR("%s was not in joint parent map! for lookup of %s", childLink.c_str(), parentLink.c_str());
      return false;
    }
    const std::map<std::string, bool>& parents = joint_parent_map_.at(childLink);
    return (parents.find(parentLink) != parents.end() && parents.at(parentLink));
  }

  void registerParents(const planning_models::KinematicModel::JointModel* model);
  void initialize();
  void calculateSmoothnessIncrements();
  void calculateCollisionIncrements();
  void calculateDistanceIncrements();
  void calculateTotalIncrements();
  void performForwardKinematics();
  void addIncrementsToTrajectory();
  void updateFullTrajectory();
  void debugCost();
  void handleJointLimits();
  void animatePath();
  void animateEndeffector();
  void visualizeState(int index);
  double getTrajectoryCost();
  double getSmoothnessCost();
  double getCollisionCost();
  double getDistanceCost();
  void perturbTrajectory();
  void getRandomMomentum();
  void updateMomentum();
  void updatePositionFromMomentum();
  void calculatePseudoInverse();
  void computeJointProperties(int trajectoryPoint);

  void display_trajectory();

};

}

#endif
