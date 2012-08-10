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

#ifndef TRAJECTORY_PLANNER_NODE_H_
#define TRAJECTORY_PLANNER_NODE_H_

#include <ros/ros.h>

#include <arm_navigation_msgs/GetMotionPlan.h>
#include <arm_navigation_msgs/convert_messages.h>

#include <arm_navigation_msgs/FilterJointTrajectoryWithConstraints.h>


#include <trajectory_optimization/trajectory_parameters.h>
#include <collision_proximity/collision_proximity_space.h>
#include <map>
#include <string>
#include <filters/filter_chain.h>
#include <eigen3/Eigen/Core>

#include <verb_msgs/WeightedTrajectory.h>
#include <tf/transform_listener.h>
#include <planning_environment/monitors/kinematic_model_state_monitor.h>
#include <trajectory_optimization/trajectory.h>

namespace trajopt
{


class TrajectoryOptimizerNode
{
public:
  /**
   * \brief Default constructor
   */
  TrajectoryOptimizerNode(ros::NodeHandle node_handle, collision_proximity::CollisionProximitySpace* space);

  /**
   * \brief Destructor
   */
  virtual ~TrajectoryOptimizerNode();

  /**
   * \brief Initialize the node
   *
   * \return true if successful, false if not
   */
  bool init();

  /**
   * \brief Runs the node
   *
   * \return 0 on clean exit
   */
  int run();

  /**
   * \brief Main entry point for motion planning (callback for the plan_kinematic_path service)
   */
  bool planKinematicPath(arm_navigation_msgs::GetMotionPlan::Request &req, arm_navigation_msgs::GetMotionPlan::Response &res);

  bool followTrajectory(verb_msgs::WeightedTrajectory::Request& req, verb_msgs::WeightedTrajectory::Response& res);
  void normalizeTrajectory(Trajectory& trajectory, std::string group_name);

  bool filterJointTrajectory(arm_navigation_msgs::FilterJointTrajectoryWithConstraints::Request &req, arm_navigation_msgs::FilterJointTrajectoryWithConstraints::Response &res);
  
private:
  ros::NodeHandle node_handle_, root_handle_;                         /**< ROS Node handle */
  ros::ServiceServer plan_kinematic_path_service_;      /**< The planning service */

  ros::ServiceServer filter_joint_trajectory_service_;      /**< The planning service */

  ros::ServiceServer follow_trajectory_service_;      /**< The trajectory service */
  tf::TransformListener listener_;
  planning_environment::KinematicModelStateMonitor* kinematics_monitor_;


  planning_environment::CollisionModels* collision_models_;
  std::string reference_frame_;

  planning_models::KinematicModel* robot_model_;                   /**< Robot Model */
  TrajectoryParameters trajectory_parameters_;                    /**< Parameters */
  collision_proximity::CollisionProximitySpace* collision_proximity_space_;           /**< Collision space */
  double trajectory_duration_;                          /**< Default duration of the planned motion */
  double trajectory_discretization_;                    /**< Default discretization of the planned motion */
  ros::Publisher vis_marker_array_publisher_;           /**< Publisher for marker arrays */
  ros::Publisher vis_marker_publisher_;                 /**< Publisher for markers */
  std::map<std::string, double> joint_velocity_limits_; /**< Map of joints to velocity limits */
  bool use_trajectory_filter_;
  int maximum_spline_points_;
  int minimum_spline_points_;

  std::map<std::string, arm_navigation_msgs::JointLimits> joint_limits_;
  void getLimits(const trajectory_msgs::JointTrajectory& trajectory, 
                 std::vector<arm_navigation_msgs::JointLimits>& limits_out);


  //filters::FilterChain<arm_navigation_msgs::FilterJointTrajectoryWithConstraints::Request> filter_constraints_chain_;
  ros::ServiceClient filter_trajectory_client_;

  inline void jointStateToArray(const sensor_msgs::JointState &joint_state, std::string& planning_group_name, Eigen::MatrixXd::RowXpr joint_array)
  {
    std::map<std::string, planning_models::KinematicModel::JointModelGroup*> groupMap = robot_model_->getJointModelGroupMap();
    const planning_models::KinematicModel::JointModelGroup* group = groupMap[planning_group_name];
    std::vector<const planning_models::KinematicModel::JointModel*> models = group->getJointModels();

    for(unsigned int i=0; i < joint_state.position.size(); i++)
    {
      for(size_t j = 0; j < models.size(); j++)
      {
        if(models[j]->getName() == joint_state.name[i])
        {
          joint_array(0, j) = joint_state.position[i];
        }
      }
    }
  }

};

}

#endif
