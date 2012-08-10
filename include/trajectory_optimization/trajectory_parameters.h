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

#ifndef TRAJECTORY_PARAMETERS_H_
#define TRAJECTORY_PARAMETERS_H_

#include <ros/ros.h>

namespace trajopt
{

class TrajectoryParameters
{
public:
  TrajectoryParameters();
  virtual ~TrajectoryParameters();

  void initFromNodeHandle();

  double getPlanningTimeLimit() const;
  void setPlanningTimeLimit(double planning_time_limit);
  int getMaxIterations() const;
  int getMaxIterationsAfterCollisionFree() const;
  double getSmoothnessCostWeight() const;
  double getObstacleCostWeight() const;
  bool getAnimatePath() const;
  double getLearningRate() const;
  double getSmoothnessCostVelocity() const;
  double getSmoothnessCostAcceleration() const;
  double getSmoothnessCostJerk() const;
  bool getAddRandomness() const;
  bool getUseHamiltonianMonteCarlo() const;
  double getHmcDiscretization() const;
  double getHmcStochasticity() const;
  double getHmcAnnealingFactor() const;
  double getRidgeFactor() const;
  bool getUsePseudoInverse() const;
  double getPseudoInverseRidgeFactor() const;
  bool getAnimateEndeffector() const;
  std::string getAnimateEndeffectorSegment() const;
  double getJointUpdateLimit() const;
  double getMinClearence() const;
  double getCollisionThreshold() const;
  bool getFilterMode() const;
  void setFilterMode(bool mode);
  double getRandomJumpAmount() const;
  void setRandomJumpAmount(double amount);
  bool getUseStochasticDescent() const;

  double getDistanceCostWeight() const;

private:
  double planning_time_limit_;
  int max_iterations_;
  int max_iterations_after_collision_free_;
  double smoothness_cost_weight_;
  double obstacle_cost_weight_;
  double learning_rate_;
  bool animate_path_;
  double smoothness_cost_velocity_;
  double smoothness_cost_acceleration_;
  double smoothness_cost_jerk_;
  bool add_randomness_;
  bool use_hamiltonian_monte_carlo_;
  bool use_stochastic_descent_;
  double hmc_stochasticity_;
  double hmc_discretization_;
  double hmc_annealing_factor_;
  double ridge_factor_;
  bool use_pseudo_inverse_;
  double pseudo_inverse_ridge_factor_;
  bool animate_endeffector_;
  std::string animate_endeffector_segment_;
  double joint_update_limit_;
  double min_clearence_;
  double collision_threshold_;
  bool filter_mode_;
  double random_jump_amount_;

  double distance_cost_weight_;
};

/////////////////////// inline functions follow ////////////////////////

inline double TrajectoryParameters::getRandomJumpAmount() const
{
  return random_jump_amount_;
}

inline void TrajectoryParameters::setRandomJumpAmount(double amount)
{
  random_jump_amount_ = amount;
}

inline double TrajectoryParameters::getCollisionThreshold() const
{
  return collision_threshold_;
}

inline bool TrajectoryParameters::getFilterMode() const
{
  return filter_mode_;
}

inline void TrajectoryParameters::setFilterMode(bool mode)
{
  filter_mode_ = mode;
}

inline double TrajectoryParameters::getMinClearence() const
{
  return min_clearence_;
}

inline double TrajectoryParameters::getJointUpdateLimit() const
{
  return joint_update_limit_;
}

inline double TrajectoryParameters::getPlanningTimeLimit() const
{
  return planning_time_limit_;
}

inline void TrajectoryParameters::setPlanningTimeLimit(double planning_time_limit)
{
  planning_time_limit_ = planning_time_limit;
}

inline int TrajectoryParameters::getMaxIterations() const
{
  return max_iterations_;
}

inline int TrajectoryParameters::getMaxIterationsAfterCollisionFree() const
{
  return max_iterations_after_collision_free_;
}

inline double TrajectoryParameters::getSmoothnessCostWeight() const
{
  return smoothness_cost_weight_;
}

inline double TrajectoryParameters::getObstacleCostWeight() const
{
  return obstacle_cost_weight_;
}

inline double TrajectoryParameters::getLearningRate() const
{
  return learning_rate_;
}

inline bool TrajectoryParameters::getAnimatePath() const
{
  return animate_path_;
}

inline bool TrajectoryParameters::getAddRandomness() const
{
  return add_randomness_;
}

inline double TrajectoryParameters::getSmoothnessCostVelocity() const
{
  return smoothness_cost_velocity_;
}

inline double TrajectoryParameters::getSmoothnessCostAcceleration() const
{
  return smoothness_cost_acceleration_;
}

inline double TrajectoryParameters::getSmoothnessCostJerk() const
{
  return smoothness_cost_jerk_;
}

inline double TrajectoryParameters::getHmcDiscretization() const
{
  return hmc_discretization_;
}

inline double TrajectoryParameters::getHmcStochasticity() const
{
  return hmc_stochasticity_;
}

inline double TrajectoryParameters::getHmcAnnealingFactor() const
{
  return hmc_annealing_factor_;
}

inline bool TrajectoryParameters::getUseHamiltonianMonteCarlo() const
{
  return use_hamiltonian_monte_carlo_;
}

inline double TrajectoryParameters::getRidgeFactor() const
{
  return ridge_factor_;
}

inline bool TrajectoryParameters::getUsePseudoInverse() const
{
  return use_pseudo_inverse_;
}

inline double TrajectoryParameters::getPseudoInverseRidgeFactor() const
{
  return pseudo_inverse_ridge_factor_;
}

inline bool TrajectoryParameters::getAnimateEndeffector() const
{
  return animate_endeffector_;
}

inline bool TrajectoryParameters::getUseStochasticDescent() const
{
  return use_stochastic_descent_;
}

inline std::string TrajectoryParameters::getAnimateEndeffectorSegment() const
{
  return animate_endeffector_segment_;
}

inline double TrajectoryParameters::getDistanceCostWeight() const
{
	return distance_cost_weight_;
}


}

#endif
