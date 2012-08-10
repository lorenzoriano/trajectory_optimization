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

#include <trajectory_optimization/trajectory_optimizer.h>
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <trajectory_optimization/utils.h>
#include <planning_models/kinematic_model.h>
#include <eigen3/Eigen/LU>
#include <eigen3/Eigen/Core>
#include <arm_navigation_msgs/DisplayTrajectory.h>

#include <sstream>

using namespace std;
using namespace Eigen;
using namespace planning_models;
using namespace collision_proximity;
using namespace arm_navigation_msgs;
using namespace trajectory_msgs;

namespace trajopt
{
  double getRandomDouble()
  {
    return ((double)random() / (double)RAND_MAX);
  }

  TrajectoryOptimizer::TrajectoryOptimizer(Trajectory *trajectory, KinematicModel *robot_model,
                                 const string& planning_group,const TrajectoryParameters *parameters, const ros::Publisher& vis_marker_array_publisher,
                                 const ros::Publisher& vis_marker_publisher, CollisionProximitySpace *collision_space
                                 ) :
    full_trajectory_(trajectory), robot_model_(robot_model), planning_group_(planning_group), parameters_(parameters),
        collision_space_(collision_space), group_trajectory_(*full_trajectory_, planning_group_, DIFF_RULE_LENGTH),
        vis_marker_array_pub_(vis_marker_array_publisher), vis_marker_pub_(vis_marker_publisher),
        reference_trajectory_(*trajectory)
  {
    initialize();
  }
  TrajectoryOptimizer::TrajectoryOptimizer(Trajectory *trajectory, KinematicModel *robot_model,
                                 const string& planning_group,const TrajectoryParameters *parameters, const ros::Publisher& vis_marker_array_publisher,
                                 const ros::Publisher& vis_marker_publisher, CollisionProximitySpace *collision_space,
                                 std::vector<double> weights) :
    full_trajectory_(trajectory), robot_model_(robot_model), planning_group_(planning_group), parameters_(parameters),
        collision_space_(collision_space), group_trajectory_(*full_trajectory_, planning_group_, DIFF_RULE_LENGTH),
        vis_marker_array_pub_(vis_marker_array_publisher), vis_marker_pub_(vis_marker_publisher),
        reference_trajectory_(*trajectory, planning_group),
        reference_weights_(weights)
  {
    initialize();
  }

  void TrajectoryOptimizer::initialize()
  {
	  std::stringstream ss;
	  for (std::vector<double>::iterator i = reference_weights_.begin(); i!=reference_weights_.end(); i++) {
		  ss<<*i<<", ";
	  }
	  ROS_INFO_STREAM("Received weights: "<<ss.str());

    robot_state_ = collision_space_->getCollisionModelsInterface()->getPlanningSceneState();
    if (robot_state_ == NULL) {
    	ROS_ERROR("robot state is null!!!");
    	exit(1);
    }
    // init some variables:
    num_vars_free_ = group_trajectory_.getNumFreePoints();
    num_vars_all_ = group_trajectory_.getNumPoints();
    num_joints_ = group_trajectory_.getNumJoints();

    free_vars_start_ = group_trajectory_.getStartIndex();
    free_vars_end_ = group_trajectory_.getEndIndex();
    ROS_INFO("At initialize, free_vars_end_: %d", free_vars_end_);

    if (reference_weights_.size() == 0) {
    	reference_weights_ = std::vector<double>(group_trajectory_.getNumPoints(), 0.0);
    	ROS_INFO("No weights on the trajectory!");
    }

    vector<GradientInfo> infos;
    collision_space_->getStateGradients(infos);

    num_collision_points_ = 0;
    for(size_t i = 0; i < infos.size(); i++)
    {
      GradientInfo& info = infos[i];
      num_collision_points_ += info.sphere_locations.size();

    }

    // set up the joint costs:
    joint_costs_.reserve(num_joints_);

    double max_cost_scale = 0.0;
    ros::NodeHandle nh("~");

    map<string, KinematicModel::JointModelGroup*> jointMap = robot_model_->getJointModelGroupMap();
    KinematicModel::JointModelGroup* jointModelGroup = jointMap[planning_group_];
    vector<const KinematicModel::JointModel*> jointModels = jointModelGroup->getJointModels();
    for(size_t i = 0; i < jointModels.size(); i++)
    {
      const KinematicModel::JointModel* model = jointModels[i];
      double joint_cost = 1.0;
      string joint_name = model->getName();
      nh.param("joint_costs/" + joint_name, joint_cost, 1.0);
      vector<double> derivative_costs(3);
      derivative_costs[0] = joint_cost * parameters_->getSmoothnessCostVelocity();
      derivative_costs[1] = joint_cost * parameters_->getSmoothnessCostAcceleration();
      derivative_costs[2] = joint_cost * parameters_->getSmoothnessCostJerk();

      joint_costs_.push_back(TrajectoryCost(group_trajectory_, i, derivative_costs, parameters_->getRidgeFactor()));
      double cost_scale = joint_costs_[i].getMaxQuadCostInvValue();
      if(max_cost_scale < cost_scale)
        max_cost_scale = cost_scale;
    }

    // scale the smoothness costs
    for(int i = 0; i < num_joints_; i++)
    {
      joint_costs_[i].scale(max_cost_scale);
    }

    // allocate memory for matrices:
    smoothness_increments_ = MatrixXd::Zero(num_vars_free_, num_joints_);
    collision_increments_ = MatrixXd::Zero(num_vars_free_, num_joints_);
    distance_increments_ = MatrixXd::Zero(num_vars_free_, num_joints_);
    final_increments_ = MatrixXd::Zero(num_vars_free_, num_joints_);
    smoothness_derivative_ = VectorXd::Zero(num_vars_all_);
    jacobian_ = MatrixXd::Zero(3, num_joints_);
    jacobian_pseudo_inverse_ = MatrixXd::Zero(num_joints_, 3);
    jacobian_jacobian_tranpose_ = MatrixXd::Zero(3, 3);
    random_state_ = VectorXd::Zero(num_joints_);
    joint_state_velocities_ = VectorXd::Zero(num_joints_);

    group_trajectory_backup_ = group_trajectory_.getTrajectory();
    best_group_trajectory_ = group_trajectory_.getTrajectory();

    collision_point_joint_names_.resize(num_vars_all_, vector<string>(num_collision_points_));
    collision_point_pos_eigen_.resize(num_vars_all_, vector<Vector3d>(num_collision_points_));
    collision_point_vel_eigen_.resize(num_vars_all_, vector<Vector3d>(num_collision_points_));
    collision_point_acc_eigen_.resize(num_vars_all_, vector<Vector3d>(num_collision_points_));
    joint_axes_.resize(num_vars_all_, vector<tf::Vector3>(num_joints_));
    joint_positions_.resize(num_vars_all_, vector<tf::Vector3>(num_joints_));

    collision_point_potential_.resize(num_vars_all_, vector<double>(num_collision_points_));
    collision_point_vel_mag_.resize(num_vars_all_, vector<double>(num_collision_points_));
    collision_point_potential_gradient_.resize(num_vars_all_, vector<Vector3d>(num_collision_points_));

    collision_free_iteration_ = 0;
    is_collision_free_ = false;
    state_is_in_collision_.resize(num_vars_all_);
    point_is_in_collision_.resize(num_vars_all_, vector<int>(num_collision_points_));

    last_improvement_iteration_ = -1;

    // HMC initialization:
    momentum_ = MatrixXd::Zero(num_vars_free_, num_joints_);
    random_momentum_ = MatrixXd::Zero(num_vars_free_, num_joints_);
    random_joint_momentum_ = VectorXd::Zero(num_vars_free_);
    multivariate_gaussian_.clear();
    stochasticity_factor_ = 1.0;
    for(int i = 0; i < num_joints_; i++)
    {
      multivariate_gaussian_.push_back(
                                       MultivariateGaussian(VectorXd::Zero(num_vars_free_),
                                                            joint_costs_[i].getQuadraticCostInverse()));
    }

    map<string, KinematicModel::JointModelGroup*> groupMap = robot_model_->getJointModelGroupMap();
    KinematicModel::JointModelGroup* modelGroup = groupMap[planning_group_];
    map<string, string> fixedLinkResolutionMap;
    for(int i = 0; i < num_joints_; i++)
    {
      joint_names_.push_back(modelGroup->getJointModels()[i]->getName());
//      ROS_INFO("Got joint %s", joint_names_[i].c_str());
      registerParents(modelGroup->getJointModels()[i]);
      fixedLinkResolutionMap[joint_names_[i]] = joint_names_[i];
    }

    for(size_t i = 0; i < modelGroup->getFixedJointModels().size(); i ++)
    {
      const KinematicModel::JointModel* model = modelGroup->getFixedJointModels()[i];
      fixedLinkResolutionMap[model->getName()] = model->getParentLinkModel()->getParentJointModel()->getName();
    }

    for(size_t i = 0; i < modelGroup->getUpdatedLinkModels().size(); i ++)
    {
      if(fixedLinkResolutionMap.find(modelGroup->getUpdatedLinkModels()[i]->getParentJointModel()->getName()) == fixedLinkResolutionMap.end())
      {
        const KinematicModel::JointModel* parentModel = NULL;
        bool foundRoot = false;

        while(!foundRoot)
        {
          if(parentModel == NULL)
          {
            parentModel = modelGroup->getUpdatedLinkModels()[i]->getParentJointModel();
          }
          else
          {
            parentModel = parentModel->getParentLinkModel()->getParentJointModel();
            for(size_t j = 0; j < joint_names_.size(); j++)
            {
              if(parentModel->getName() == joint_names_[j])
              {
                foundRoot = true;
              }
            }
          }
        }
        fixedLinkResolutionMap[modelGroup->getUpdatedLinkModels()[i]->getParentJointModel()->getName()] = parentModel->getName();
      }
    }

    for(map<string, map<string, bool> >::iterator it = joint_parent_map_.begin(); it != joint_parent_map_.end(); it++)
    {
      stringstream ss;
      ss << it->first << " Parents : {";

      for(map<string, bool>::iterator it2 = it->second.begin(); it2 != it->second.end(); it2++)
      {
        ss << it2->first << ",";
      }
      ss << "}";
//      ROS_INFO("%s",ss.str().c_str());
    }

    int start = free_vars_start_;
    int end = free_vars_end_;
    vector<GradientInfo> gradients;
    collision_space_->getStateGradients(gradients);
    for(int i = start; i <= end; ++i)
    {
      size_t j = 0;
      for(size_t g = 0; g < gradients.size(); g++)
      {
        GradientInfo& info = gradients[g];

        for(size_t k = 0; k < info.sphere_locations.size(); k++)
        {
          if(fixedLinkResolutionMap.find(info.joint_name) != fixedLinkResolutionMap.end())
          {
            collision_point_joint_names_[i][j] = fixedLinkResolutionMap[info.joint_name];
          }
          else
          {
            ROS_ERROR("Couldn't find joint %s!", info.joint_name.c_str());
          }
          j++;
        }
      }
    }
  }

  TrajectoryOptimizer::~TrajectoryOptimizer()
  {
    destroy();
  }

  void TrajectoryOptimizer::registerParents(const KinematicModel::JointModel* model)
  {
    const KinematicModel::JointModel* parentModel = NULL;
    bool foundRoot = false;

    if(model == robot_model_->getRoot()) return;

    while(!foundRoot)
    {
      if(parentModel == NULL)
      { 
        if(model->getParentLinkModel() == NULL) {
          ROS_ERROR_STREAM("Model " << model->getName() << " not root but has NULL link model parent");
          return;
        } else if(model->getParentLinkModel()->getParentJointModel() == NULL) {
          ROS_ERROR_STREAM("Model " << model->getName() << " not root but has NULL joint model parent");
          return;
        }
        parentModel = model->getParentLinkModel()->getParentJointModel();
      } else
      {
        if(parentModel == robot_model_->getRoot())
        {
          foundRoot = true;
        } else {
          parentModel = parentModel->getParentLinkModel()->getParentJointModel();
        }
      }
      joint_parent_map_[model->getName()][parentModel->getName()] = true;
    }
  }

  void TrajectoryOptimizer::optimize()
  {
    ros::WallTime start_time = ros::WallTime::now();
    double averageCostVelocity = 0.0;
    int currentCostIter = 0;
    int costWindow = 10;
    vector<double>costs(costWindow, 0.0);
    double minimaThreshold = 0.05;
    bool shouldBreakOut = false;

    if(parameters_->getAnimatePath())
    {
      animatePath();
    }

    // iterate
    for(iteration_ = 0; iteration_ < parameters_->getMaxIterations(); iteration_++)
    {
      ros::WallTime for_time = ros::WallTime::now();
      performForwardKinematics();
      ROS_DEBUG_STREAM("Forward kinematics took " << (ros::WallTime::now()-for_time));
      double cCost = getCollisionCost();
      double sCost = getSmoothnessCost();
      double refCost = getTrajectoryCost();
      double cost = cCost + sCost;

      if(parameters_->getAddRandomness() && currentCostIter != -1)
      {
        costs[currentCostIter] = cCost;
        currentCostIter++;

        if(currentCostIter >= costWindow)
        {
          for(int i = 1; i < costWindow; i++)
          {
            averageCostVelocity += (costs.at(i) - costs.at(i - 1));
          }

          averageCostVelocity /= (double)(costWindow);
          currentCostIter = -1;
        }
      }
      if(iteration_ == 0)
      {
        best_group_trajectory_ = group_trajectory_.getTrajectory();
        best_group_trajectory_cost_ = cost;
      }
      else
      {
        if(cost < best_group_trajectory_cost_)
        {
          best_group_trajectory_ = group_trajectory_.getTrajectory();
          best_group_trajectory_cost_ = cost;
          last_improvement_iteration_ = iteration_;
        }
      }
      calculateSmoothnessIncrements();
      ros::WallTime coll_time = ros::WallTime::now();
      calculateCollisionIncrements();
      ROS_DEBUG_STREAM("Collision increments took " << (ros::WallTime::now()-coll_time));

      calculateDistanceIncrements();
      calculateTotalIncrements();

      if(!parameters_->getUseHamiltonianMonteCarlo())
      {
        // non-stochastic version:
        addIncrementsToTrajectory();
      }
      else
      {
        // hamiltonian monte carlo updates:
        getRandomMomentum();
        updateMomentum();
        updatePositionFromMomentum();
        stochasticity_factor_ *= parameters_->getHmcAnnealingFactor();
      }
      handleJointLimits();
      updateFullTrajectory();

      ROS_INFO("Iteration: %d, Trajectory cost: %3.5f (s=%3.5f, c=%3.5f, d=%3.5f)",
          			iteration_,
          			getTrajectoryCost(),
          			getSmoothnessCost(),
          			getCollisionCost(),
          			getDistanceCost());

      if( (iteration_ % 10 == 0) && (iteration_ > 0) )
      {
        ROS_DEBUG("Trajectory cost: %f (s=%f, c=%f)", getTrajectoryCost(), getSmoothnessCost(), getCollisionCost());
        CollisionProximitySpace::TrajectorySafety safety = checkCurrentIterValidity();
        if(safety == CollisionProximitySpace::MeshToMeshSafe) 
        {
          num_collision_free_iterations_ = 0;
          ROS_INFO("Got mesh to mesh safety at iter %d. Breaking out early.", iteration_);
          is_collision_free_ = true;
          iteration_++;
          shouldBreakOut = true;
        } else if(safety == CollisionProximitySpace::InCollisionSafe) {
          num_collision_free_iterations_ = parameters_->getMaxIterationsAfterCollisionFree();
          ROS_INFO("Got in collision safety at iter %d. Breaking out soon.", iteration_);
          is_collision_free_ = true;
          iteration_++;
          shouldBreakOut = true;
        }
        else
        {
          is_collision_free_ = false;
        }
      }

      if(!parameters_->getFilterMode())
      {
        if(cCost < parameters_->getCollisionThreshold())
        {
          is_collision_free_ = true;
          iteration_++;
          shouldBreakOut = true;
        }
      }


      if((ros::WallTime::now() - start_time).toSec() > parameters_->getPlanningTimeLimit() && !parameters_->getAnimatePath() && !parameters_->getAnimateEndeffector())
      {
        ROS_WARN("Breaking out early due to time limit constraints.");
        ROS_INFO_STREAM("Time limit was: "<<parameters_->getPlanningTimeLimit()<<" and elapsed is: "<< (ros::WallTime::now() - start_time).toSec());
        break;
      }

      if(fabs(averageCostVelocity) < minimaThreshold && currentCostIter == -1 && !is_collision_free_ && parameters_->getAddRandomness())
      {
        ROS_INFO("Detected local minima. Attempting to break out!");
        int iter = 0;
        bool success = false;
        while(iter < 20 && !success)
        {
          performForwardKinematics();
          double original_cost = getTrajectoryCost();
          group_trajectory_backup_ = group_trajectory_.getTrajectory();
          perturbTrajectory();
          handleJointLimits();
          updateFullTrajectory();
          performForwardKinematics();
          double new_cost = getTrajectoryCost();
          iter ++;
          if(new_cost < original_cost)
          {
            ROS_INFO("Got out of minimum in %d iters!", iter);
            averageCostVelocity = 0.0;
            currentCostIter = 0;
            success = true;
          }
          else
          {
            group_trajectory_.getTrajectory() = group_trajectory_backup_;
            updateFullTrajectory();
            currentCostIter = 0;
            averageCostVelocity = 0.0;
            success = false;
          }

        }

        if(!success)
        {
          ROS_INFO("Failed to exit minimum!");
        }
      }
      else if(currentCostIter == -1)
      {
        currentCostIter = 0;
        averageCostVelocity = 0.0;
      }

      if(parameters_->getAnimateEndeffector())
      {
        animateEndeffector();
      }

      if(parameters_->getAnimatePath() && iteration_ % 25 == 0)
      {
        animatePath();
      }

      if(shouldBreakOut)
      {
        collision_free_iteration_++;
        if(num_collision_free_iterations_ == 0) {
          break;
        } else if(collision_free_iteration_ > num_collision_free_iterations_)
        {
          CollisionProximitySpace::TrajectorySafety safety = checkCurrentIterValidity();
          if(safety != CollisionProximitySpace::MeshToMeshSafe &&
             safety != CollisionProximitySpace::InCollisionSafe) {
            ROS_WARN_STREAM("Apparently regressed");
          }
          break;
        }
      }
    }

    if(is_collision_free_)
    {
      ROS_INFO("Path is collision free");
    }
    else
    {
      ROS_ERROR("Path is not collision free!");
    }

    if(parameters_->getAnimatePath())
    {
      animatePath();
    }

    group_trajectory_.getTrajectory() = best_group_trajectory_;
    updateFullTrajectory();

    if(parameters_->getAnimatePath())
      animatePath();

    ROS_INFO("Terminated after %d iterations, using path from iteration %d", iteration_, last_improvement_iteration_);
    ROS_INFO("Optimization core finished in %f sec", (ros::WallTime::now() - start_time).toSec() );
    ROS_INFO_STREAM("Time per iteration " << (ros::WallTime::now() - start_time).toSec()/(iteration_*1.0));
  }

CollisionProximitySpace::TrajectorySafety TrajectoryOptimizer::checkCurrentIterValidity()
{
    JointTrajectory jointTrajectory;
    jointTrajectory.joint_names = joint_names_;
    jointTrajectory.header.frame_id = collision_space_->getCollisionModelsInterface()->getRobotFrameId();
    jointTrajectory.header.stamp = ros::Time::now();
    Constraints goalConstraints;
    Constraints pathConstraints;
    ArmNavigationErrorCodes errorCode;
    vector<ArmNavigationErrorCodes> trajectoryErrorCodes;
    for(int i = 0; i < group_trajectory_.getNumPoints(); i++)
    {
      JointTrajectoryPoint point;
      for(int j = 0; j < group_trajectory_.getNumJoints(); j++)
      {
        point.positions.push_back(best_group_trajectory_(i, j));
      }
      jointTrajectory.points.push_back(point);
    }

    return collision_space_->isTrajectorySafe(jointTrajectory, goalConstraints, pathConstraints, planning_group_);
    /*
    bool valid = collision_space_->getCollisionModelsInterface()->isJointTrajectoryValid(*robot_state_,
                                                                                         jointTrajectory,
                                                                                         goalConstraints,
                                                                                         pathConstraints, errorCode,
                                                                                         trajectoryErrorCodes, false);
                                                                                         */

  }

  void TrajectoryOptimizer::calculateSmoothnessIncrements()
  {
    for(int i = 0; i < num_joints_; i++)
    {
      joint_costs_[i].getDerivative(group_trajectory_.getJointTrajectory(i), smoothness_derivative_);
      smoothness_increments_.col(i)
         = -smoothness_derivative_.segment(group_trajectory_.getStartIndex(), num_vars_free_);
    }
  }

  void TrajectoryOptimizer::calculateDistanceIncrements() {

//	  ROS_INFO("Matrix size: (%d,%d)", distance_increments_.rows(), distance_increments_.cols());
//	  ROS_INFO("Vars free: %d", num_vars_free_);
//	  ROS_INFO("Free vars end: %d", free_vars_end_);
//	  ROS_INFO("Weight vector size: %d", reference_weights_.size());

	  distance_increments_.setZero(num_vars_free_, num_joints_);
	  if (num_vars_free_ != reference_weights_.size())
		  ROS_ERROR("NUM_VARS_FREE: %i, WEIGHTS SIZE: %i", num_vars_free_, reference_weights_.size());
	  if (num_vars_free_ != distance_increments_.rows())
		  ROS_ERROR("NUM_VARS_FREE: %i, distance_increments_ rows: %i", num_vars_free_, distance_increments_.rows());
	  if (full_trajectory_->getNumPoints() != num_vars_free_)
		  ROS_ERROR("NUM_VARS_FREE: %i, full trajectory: %i", num_vars_free_, full_trajectory_->getNumPoints());
//	  return;

	  int startPoint = 0;
	  int endPoint = num_vars_free_;


	  for(int i = startPoint; i < endPoint; i++) {
		  distance_increments_.row(i) = (-1.0) * reference_weights_.at(i) * (full_trajectory_->getTrajectoryPoint(i) -
				  reference_trajectory_.getTrajectoryPoint(i));
	  }

  }

  void TrajectoryOptimizer::calculateCollisionIncrements()
  {
    double potential;
    double vel_mag_sq;
    double vel_mag;
    Vector3d potential_gradient;
    Vector3d normalized_velocity;
    Matrix3d orthogonal_projector;
    Vector3d curvature_vector;
    Vector3d cartesian_gradient;


    collision_increments_.setZero(num_vars_free_, num_joints_);

    int startPoint = 0;
    int endPoint = free_vars_end_;

    // In stochastic descent, simply use a random point in the trajectory, rather than all the trajectory points.
    // This is faster and guaranteed to converge, but it may take more iterations in the worst case.
    if(parameters_->getUseStochasticDescent())
    {
      startPoint =  (int)(((double)random() / (double)RAND_MAX)*(free_vars_end_ - free_vars_start_) + free_vars_start_);
      if(startPoint < free_vars_start_) startPoint = free_vars_start_;
      if(startPoint > free_vars_end_) startPoint = free_vars_end_;
      endPoint = startPoint;
    }
    else
    {
      startPoint = free_vars_start_;
    }

    for(int i = startPoint; i <= endPoint; i++)
    {
      for(int j = 0; j < num_collision_points_; j++)
      {
        potential = collision_point_potential_[i][j]; //this comes from

        if(potential < 0.0001)
          continue;

        potential_gradient = -collision_point_potential_gradient_[i][j];

        vel_mag = collision_point_vel_mag_[i][j];
        vel_mag_sq = vel_mag * vel_mag;

        // all math from the CHOMP paper:

        normalized_velocity = collision_point_vel_eigen_[i][j] / vel_mag;
        orthogonal_projector = Matrix3d::Identity() - (normalized_velocity * normalized_velocity.transpose());
        curvature_vector = (orthogonal_projector * collision_point_acc_eigen_[i][j]) / vel_mag_sq;
        cartesian_gradient = vel_mag * (orthogonal_projector * potential_gradient - potential * curvature_vector);

        // pass it through the jacobian transpose to get the increments
        getJacobian(i, collision_point_pos_eigen_[i][j], collision_point_joint_names_[i][j] ,jacobian_);

        if(parameters_->getUsePseudoInverse())
        {
          calculatePseudoInverse();
          collision_increments_.row(i - free_vars_start_).transpose() -= jacobian_pseudo_inverse_ * cartesian_gradient;
        }
        else
        {

        	collision_increments_.row(i - free_vars_start_).transpose();
			jacobian_.transpose() * cartesian_gradient;

          collision_increments_.row(i - free_vars_start_).transpose() -= jacobian_.transpose() * cartesian_gradient;
        }

        /*
        if(point_is_in_collision_[i][j])
        {
          break;
        }
        */
      }
    }
    //cout << collision_increments_ << endl;
  }

  void TrajectoryOptimizer::calculatePseudoInverse()
  {
    jacobian_jacobian_tranpose_ = jacobian_ * jacobian_.transpose() + MatrixXd::Identity(3, 3)
        * parameters_->getPseudoInverseRidgeFactor();
    jacobian_pseudo_inverse_ = jacobian_.transpose() * jacobian_jacobian_tranpose_.inverse();
  }

  void TrajectoryOptimizer::calculateTotalIncrements()
  {
//    for(int i = 0; i < num_joints_; i++)
//    {
//      final_increments_.col(i) = parameters_->getLearningRate() * (joint_costs_[i].getQuadraticCostInverse()
//          * (parameters_->getSmoothnessCostWeight() * smoothness_increments_.col(i)
//              + parameters_->getObstacleCostWeight() * collision_increments_.col(i)));
//    }

    for(int i = 0; i < num_joints_; i++)
    {
    	final_increments_.col(i) = parameters_->getLearningRate() *
    			joint_costs_[i].getQuadraticCostInverse() *
    			(
				 parameters_->getObstacleCostWeight() *
    					collision_increments_.col(i)
    					+
    			 parameters_->getDistanceCostWeight() *
    			 distance_increments_.col(i)
				)
    			;
    }

  }

  void TrajectoryOptimizer::addIncrementsToTrajectory()
  {
    map<string, KinematicModel::JointModelGroup*> modelGroups = robot_model_->getJointModelGroupMap();
    string group = planning_group_;
    const KinematicModel::JointModelGroup* modelGroup = modelGroups[group];
    const vector<const KinematicModel::JointModel*>& jointModels = modelGroup->getJointModels();

    for(size_t i = 0; i < jointModels.size(); i++)
    {
      double scale = 1.0;
      double max = final_increments_.col(i).maxCoeff();
      double min = final_increments_.col(i).minCoeff();
      double max_scale =  parameters_->getJointUpdateLimit() / fabs(max);
      double min_scale = parameters_->getJointUpdateLimit() / fabs(min);
      if(max_scale < scale)
        scale = max_scale;
      if(min_scale < scale)
        scale = min_scale;
      group_trajectory_.getFreeTrajectoryBlock().col(i) += scale * final_increments_.col(i);
    }
    //ROS_DEBUG("Scale: %f",scale);
    //group_trajectory_.getFreeTrajectoryBlock() += scale * final_increments_;
  }

  void TrajectoryOptimizer::updateFullTrajectory()
  {
    full_trajectory_->updateFromGroupTrajectory(group_trajectory_);
  }

  void TrajectoryOptimizer::debugCost()
  {
    double cost = 0.0;
    for(int i = 0; i < num_joints_; i++)
      cost += joint_costs_[i].getCost(group_trajectory_.getJointTrajectory(i));
    cout << "Cost = " << cost << endl;
  }

  double TrajectoryOptimizer::getTrajectoryCost()
  {
    return getSmoothnessCost() + getCollisionCost() + getDistanceCost();
  }

  double TrajectoryOptimizer::getSmoothnessCost()
  {
    double smoothness_cost = 0.0;
    // joint costs:
    for(int i = 0; i < num_joints_; i++)
      smoothness_cost += joint_costs_[i].getCost(group_trajectory_.getJointTrajectory(i));

    return parameters_->getSmoothnessCostWeight() * smoothness_cost;
  }

  double TrajectoryOptimizer::getDistanceCost()
  {
//	  ROS_INFO_STREAM("Full trajectory\n"<<full_trajectory_->getTrajectory());
//	  ROS_INFO_STREAM("Reference trajectory:\n"<<reference_trajectory_.getTrajectory());
    double distance_cost = 0.0;
    for(int i = 0; i < full_trajectory_->getNumPoints(); i++) {
    	double norm = (full_trajectory_->getTrajectoryPoint(i) -
    				   reference_trajectory_.getTrajectoryPoint(i)).squaredNorm();
    	distance_cost += reference_weights_[i]*norm;
    }
    return parameters_->getDistanceCostWeight() * distance_cost;
  }



  double TrajectoryOptimizer::getCollisionCost()
  {
    double collision_cost = 0.0;

    double worst_collision_cost = 0.0;
    worst_collision_cost_state_ = -1;

    // collision costs:
    for(int i = free_vars_start_; i <= free_vars_end_; i++)
    {
      double state_collision_cost = 0.0;
      for(int j = 0; j < num_collision_points_; j++)
      {
        state_collision_cost += collision_point_potential_[i][j] * collision_point_vel_mag_[i][j];
      }
      collision_cost += state_collision_cost;
      if(state_collision_cost > worst_collision_cost)
      {
        worst_collision_cost = state_collision_cost;
        worst_collision_cost_state_ = i;
      }
    }

    return parameters_->getObstacleCostWeight() * collision_cost;
  }

  void TrajectoryOptimizer::computeJointProperties(int trajectoryPoint)
  {
    tf::Transform inverseWorldTransform = collision_space_->getInverseWorldTransform(*robot_state_);
     for(int j = 0; j < num_joints_; j++)
     {
       string jointName = joint_names_[j];
       const KinematicState::JointState* jointState = robot_state_->getJointState(jointName);
       const KinematicModel::JointModel* jointModel = jointState->getJointModel();
       const KinematicModel::RevoluteJointModel* revoluteJoint = dynamic_cast<const KinematicModel::RevoluteJointModel*>(jointModel);
       const KinematicModel::PrismaticJointModel* prismaticJoint = dynamic_cast<const KinematicModel::PrismaticJointModel*>(jointModel);

       string parentLinkName = jointModel->getParentLinkModel()->getName();
       string childLinkName = jointModel->getChildLinkModel()->getName();
       tf::Transform jointTransform =
           robot_state_->getLinkState(parentLinkName)->getGlobalLinkTransform()
           * (robot_model_->getLinkModel(childLinkName)->getJointOriginTransform()
               * (robot_state_->getJointState(jointModel->getName())->getVariableTransform()));


       jointTransform = inverseWorldTransform * jointTransform;
       tf::Vector3 axis;


       if(revoluteJoint != NULL)
       {
         axis = revoluteJoint->axis_;
       }
       else if(prismaticJoint != NULL)
       {
         axis = prismaticJoint->axis_;
       }

       axis = jointTransform * axis;

       joint_axes_[trajectoryPoint][j] = axis;
       joint_positions_[trajectoryPoint][j] = jointTransform.getOrigin();
     }
  }

  template<typename Derived>
  void TrajectoryOptimizer::getJacobian(int trajectoryPoint, Vector3d& collision_point_pos, string& jointName,
                                   MatrixBase<Derived>& jacobian) const
  {
    for(int j = 0; j < num_joints_; j++)
    {
      if(isParent(jointName, joint_names_[j]))
      {
        tf::Vector3 column = joint_axes_[trajectoryPoint][j].cross(tf::Vector3(collision_point_pos(0),
                                                                           collision_point_pos(1),
                                                                           collision_point_pos(2))
                                                                     - joint_positions_[trajectoryPoint][j]);

        jacobian.col(j)[0] = column.x();
        jacobian.col(j)[1] = column.y();
        jacobian.col(j)[2] = column.z();
      }
      else
      {
        jacobian.col(j)[0] = 0.0;
        jacobian.col(j)[1] = 0.0;
        jacobian.col(j)[2] = 0.0;
      }
    }
  }

  void TrajectoryOptimizer::handleJointLimits()
  {
    map<string, KinematicModel::JointModelGroup*> modelGroups = robot_model_->getJointModelGroupMap();
    string group = planning_group_;
    KinematicModel::JointModelGroup* modelGroup = modelGroups[group];

    for(int joint = 0; joint < num_joints_; joint++)
    {
      const string& jointName = joint_names_[joint];
      const KinematicModel::JointModel* jointModel = modelGroup->getJointModel(jointName);
      const KinematicModel::RevoluteJointModel* revoluteJoint = dynamic_cast<const KinematicModel::RevoluteJointModel*>(jointModel);

      if(revoluteJoint->continuous_)
      {
        continue;
      }
      map<string, pair<double,double> > bounds = jointModel->getAllVariableBounds();

      double joint_max = -10000;
      double joint_min = 10000;

      for(map<string,pair<double,double> >::iterator it = bounds.begin(); it != bounds.end(); it ++)
      {
        if(it->second.first < joint_min)
        {
          joint_min = it->second.first;
        }

        if(it->second.second > joint_max)
        {
          joint_max = it->second.second;
        }
      }


      int count = 0;

      bool violation = false;
      do
      {
        double max_abs_violation = 1e-6;
        double max_violation = 0.0;
        int max_violation_index = 0;
        violation = false;
        for(int i = free_vars_start_; i <= free_vars_end_; i++)
        {
          double amount = 0.0;
          double absolute_amount = 0.0;
          if(group_trajectory_(i, joint) > joint_max)
          {
            amount = joint_max - group_trajectory_(i, joint);
            absolute_amount = fabs(amount);
          }
          else if(group_trajectory_(i, joint) < joint_min)
          {
            amount = joint_min - group_trajectory_(i, joint);
            absolute_amount = fabs(amount);
          }
          if(absolute_amount > max_abs_violation)
          {
            max_abs_violation = absolute_amount;
            max_violation = amount;
            max_violation_index = i;
            violation = true;
          }
        }

        if(violation)
        {
          int free_var_index = max_violation_index - free_vars_start_;
          double multiplier = max_violation / joint_costs_[joint].getQuadraticCostInverse()(free_var_index,
                                                                                            free_var_index);
          group_trajectory_.getFreeJointTrajectoryBlock(joint) += multiplier
              * joint_costs_[joint].getQuadraticCostInverse().col(free_var_index);
        }
        if(++count > 10)
          break;
      }
      while(violation);
    }
  }

  void TrajectoryOptimizer::performForwardKinematics()
  {
	  //For each point in the trajectory calculate the sphere location (collision_point_pos_eigen),
	  //the potential and the gradient. Also determine if a sphere is in collision.
    double invTime = 1.0 / group_trajectory_.getDiscretization();
    double invTimeSq = invTime * invTime;

    // calculate the forward kinematics for the fixed states only in the first iteration:
    int start = free_vars_start_;
    int end = free_vars_end_;
    if(iteration_ == 0)
    {
      start = 0;
      end = num_vars_all_ - 1;
    }

    is_collision_free_ = true;

    // for each point in the trajectory
    for(int i = start; i <= end; ++i)
    {
      // Set Robot state from trajectory point...
      setRobotStateFromPoint(group_trajectory_, i);
      computeJointProperties(i);
      state_is_in_collision_[i] = false;

      vector<GradientInfo> gradients;
      collision_space_->getStateGradients(gradients);
      //Keep vars in scope
      {
        size_t j = 0;
        for(size_t g = 0; g < gradients.size(); g++)
        {
          GradientInfo& info = gradients[g];

          for(size_t k = 0; k < info.sphere_locations.size(); k++)
          {
            collision_point_pos_eigen_[i][j][0] = info.sphere_locations[k].x();
            collision_point_pos_eigen_[i][j][1] = info.sphere_locations[k].y();
            collision_point_pos_eigen_[i][j][2] = info.sphere_locations[k].z();

            collision_point_potential_[i][j] = getPotential(info.distances[k], info.sphere_radii[k], parameters_->getMinClearence());
            collision_point_potential_gradient_[i][j][0] = info.gradients[k].x();
            collision_point_potential_gradient_[i][j][1] = info.gradients[k].y();
            collision_point_potential_gradient_[i][j][2] = info.gradients[k].z();

            point_is_in_collision_[i][j] = (info.distances[k] - info.sphere_radii[k] < info.sphere_radii[k]);

            if(point_is_in_collision_[i][j])
            {
              state_is_in_collision_[i] = true;
              is_collision_free_ = false;
            }
            j ++;
          }
        }
      }
    }

    // now, get the vel and acc for each collision point (using finite differencing)
    for(int i = free_vars_start_; i <= free_vars_end_; i++)
    {
      for(int j = 0; j < num_collision_points_; j++)
      {
        collision_point_vel_eigen_[i][j] = Vector3d(0,0,0);
        collision_point_acc_eigen_[i][j] = Vector3d(0,0,0);
        for(int k = -DIFF_RULE_LENGTH / 2; k <= DIFF_RULE_LENGTH / 2; k++)
        {
          collision_point_vel_eigen_[i][j] += (invTime * DIFF_RULES[0][k + DIFF_RULE_LENGTH / 2]) * collision_point_pos_eigen_[i
              + k][j];
          collision_point_acc_eigen_[i][j] += (invTimeSq * DIFF_RULES[1][k + DIFF_RULE_LENGTH / 2]) * collision_point_pos_eigen_[i
              + k][j];
        }

        // get the norm of the velocity:
        collision_point_vel_mag_[i][j] = collision_point_vel_eigen_[i][j].norm();
      }
    }
  }

  void TrajectoryOptimizer::setRobotStateFromPoint(Trajectory& group_trajectory, int i)
  {
    const MatrixXd::RowXpr& point = group_trajectory.getTrajectoryPoint(i);

    vector<double> jointStates;
    for(int j = 0; j < group_trajectory.getNumJoints(); j ++)
    {
      jointStates.push_back(point(0,j));
    }

    ros::WallTime timer = ros::WallTime::now();
    KinematicState::JointStateGroup* group = (KinematicState::JointStateGroup*)(robot_state_->getJointStateGroup(planning_group_));
    group->setKinematicState(jointStates);
    timer = ros::WallTime::now();
    collision_space_->setCurrentGroupState(*robot_state_);
  }

  void TrajectoryOptimizer::perturbTrajectory()
  {
    //int mid_point = (free_vars_start_ + free_vars_end_) / 2;
    if(worst_collision_cost_state_ < 0)
      return;
    int mid_point = worst_collision_cost_state_;
    getRandomState(robot_state_, planning_group_, random_state_);

    // convert the state into an increment
    random_state_ -= group_trajectory_.getTrajectoryPoint(mid_point).transpose();

    // project the increment orthogonal to joint velocities
    group_trajectory_.getJointVelocities(mid_point, joint_state_velocities_);
    joint_state_velocities_.normalize();
    random_state_ = (MatrixXd::Identity(num_joints_, num_joints_) - joint_state_velocities_
        * joint_state_velocities_.transpose()) * random_state_;

    int mp_free_vars_index = mid_point - free_vars_start_;
    for(int i = 0; i < num_joints_; i++)
    {
      group_trajectory_.getFreeJointTrajectoryBlock(i)
          += joint_costs_[i].getQuadraticCostInverse().col(mp_free_vars_index) * random_state_(i);
    }
  }

  void TrajectoryOptimizer::getRandomState(const KinematicState* currentState, const string& groupName, VectorXd& state_vec)
  {
    const vector<KinematicState::JointState*>& jointStates =
        currentState->getJointStateGroup(groupName)->getJointStateVector();
    for(size_t i = 0; i < jointStates.size(); i++)
    {

      bool continuous = false;
      
      KinematicState::JointState* jointState = jointStates[i];
      const KinematicModel::RevoluteJointModel* revolute_joint 
        = dynamic_cast<const KinematicModel::RevoluteJointModel*>(jointState->getJointModel());
      if(revolute_joint && revolute_joint->continuous_) {
        continuous = true;
      }
      
      map<string, pair<double, double> > bounds = jointState->getJointModel()->getAllVariableBounds();
      int j = 0;
      for(map<string, pair<double, double> >::iterator it = bounds.begin(); it != bounds.end(); it++)
      {
        double randVal = jointState->getJointStateValues()[j] + (getRandomDouble()
                                                                 * (parameters_->getRandomJumpAmount()) - getRandomDouble() * (parameters_->getRandomJumpAmount()));

        if(!continuous) 
        {
          if(randVal > it->second.second)
          {
            randVal = it->second.second;
          }
          else if(randVal < it->second.first)
          {
            randVal = it->second.first;
          }
        }

        ROS_DEBUG_STREAM("Joint " << it->first << " old value " << jointState->getJointStateValues()[j] << " new value " << randVal);
        state_vec(i) = randVal;
        
        j++;
      }
    }
  }

  void TrajectoryOptimizer::getRandomMomentum()
  {
    if(is_collision_free_)
      random_momentum_.setZero(num_vars_free_, num_joints_);
    else
      for(int i = 0; i < num_joints_; ++i)
      {
        multivariate_gaussian_[i].sample(random_joint_momentum_);
        random_momentum_.col(i) = stochasticity_factor_ * random_joint_momentum_;
      }
  }

  void TrajectoryOptimizer::updateMomentum()
  {
    //double alpha = 1.0 - parameters_->getHmcStochasticity();
    double eps = parameters_->getHmcDiscretization();
    if(iteration_ > 0)
      momentum_ = (momentum_ + eps * final_increments_);
    else
      momentum_ = random_momentum_;
    //momentum_ = alpha * (momentum_ + eps*final_increments_) + sqrt(1.0-alpha*alpha)*random_momentum_;
  }

  void TrajectoryOptimizer::updatePositionFromMomentum()
  {
    double eps = parameters_->getHmcDiscretization();
    group_trajectory_.getFreeTrajectoryBlock() += eps * momentum_;
  }

  void TrajectoryOptimizer::animatePath()
  {
    for(int i = free_vars_start_; i <= free_vars_end_; i++)
    {
      visualizeState(i);
      //ros::WallDuration(group_trajectory_.getDiscretization()).sleep();
      ros::WallDuration(.05).sleep();
    }
  }

  void TrajectoryOptimizer::animateEndeffector()
  {
    visualization_msgs::Marker msg;
    msg.points.resize(num_vars_free_);
    int sn = (int)(num_collision_points_ - 1);
    for(int i = 0; i < num_vars_free_; ++i)
    {
      int j = i + free_vars_start_;
      msg.points[i].x = collision_point_pos_eigen_[j][sn][0];
      msg.points[i].y = collision_point_pos_eigen_[j][sn][1];
      msg.points[i].z = collision_point_pos_eigen_[j][sn][2];
    }
    msg.header.frame_id = collision_space_->getCollisionModelsInterface()->getRobotFrameId();
    msg.header.stamp = ros::Time();
    msg.ns = "trajopt_endeffector";
    msg.id = 0;
    msg.type = visualization_msgs::Marker::SPHERE_LIST;
    msg.action = visualization_msgs::Marker::ADD;
    double scale = 0.05;
    msg.scale.x = scale;
    msg.scale.y = scale;
    msg.scale.z = scale;
    msg.color.a = 0.6;
    msg.color.r = 0.5;
    msg.color.g = 1.0;
    msg.color.b = 0.3;
    vis_marker_pub_.publish(msg);
    ros::WallDuration(0.1).sleep();

  }

  void TrajectoryOptimizer::visualizeState(int index)
  {

    visualization_msgs::MarkerArray msg;
    msg.markers.resize(num_collision_points_ + num_joints_);
    int num_arrows = 0;
    double potential_threshold = 1e-10;
    for(int i = 0; i < num_collision_points_; i++)
    {
      msg.markers[i].header.frame_id = collision_space_->getCollisionModelsInterface()->getRobotFrameId();
      msg.markers[i].header.stamp = ros::Time();
      msg.markers[i].ns = "trajopt_collisions";
      msg.markers[i].id = i;
      msg.markers[i].type = visualization_msgs::Marker::SPHERE;
      msg.markers[i].action = visualization_msgs::Marker::ADD;
      msg.markers[i].pose.position.x = collision_point_pos_eigen_[index][i][0];
      msg.markers[i].pose.position.y = collision_point_pos_eigen_[index][i][1];
      msg.markers[i].pose.position.z = collision_point_pos_eigen_[index][i][2];
      msg.markers[i].pose.orientation.x = 0.0;
      msg.markers[i].pose.orientation.y = 0.0;
      msg.markers[i].pose.orientation.z = 0.0;
      msg.markers[i].pose.orientation.w = 1.0;
      double scale = 0.1;
      msg.markers[i].scale.x = scale;
      msg.markers[i].scale.y = scale;
      msg.markers[i].scale.z = scale;
      msg.markers[i].color.a = 0.6;
      msg.markers[i].color.r = 0.5;
      msg.markers[i].color.g = 1.0;
      msg.markers[i].color.b = 0.3;
      if(collision_point_potential_[index][i] > potential_threshold)
        num_arrows++;
    }

    vis_marker_array_pub_.publish(msg);

    // publish arrows for distance field:
    msg.markers.resize(0);
    msg.markers.resize(num_collision_points_);
    for(int i = 0; i < num_collision_points_; i++)
    {
      msg.markers[i].header.frame_id = collision_space_->getCollisionModelsInterface()->getRobotFrameId();
      msg.markers[i].header.stamp = ros::Time();
      msg.markers[i].ns = "trajopt_arrows";
      msg.markers[i].id = i;
      msg.markers[i].type = visualization_msgs::Marker::ARROW;
      msg.markers[i].action = visualization_msgs::Marker::ADD;
      msg.markers[i].points.resize(2);
      msg.markers[i].points[0].x = collision_point_pos_eigen_[index][i](0);
      msg.markers[i].points[0].y = collision_point_pos_eigen_[index][i](1);
      msg.markers[i].points[0].z = collision_point_pos_eigen_[index][i](2);
      msg.markers[i].points[1] = msg.markers[i].points[0];
      double scale = 0.25f;
      if(collision_point_potential_[index][i] <= potential_threshold)
        scale = 0.0;
      msg.markers[i].points[1].x += scale * collision_point_potential_gradient_[index][i](0);
      msg.markers[i].points[1].y += scale * collision_point_potential_gradient_[index][i](1);
      msg.markers[i].points[1].z += scale * collision_point_potential_gradient_[index][i](2);
      msg.markers[i].scale.x = 0.01;
      msg.markers[i].scale.y = 0.03;
      msg.markers[i].color.a = 0.5;
      msg.markers[i].color.r = 0.5;
      msg.markers[i].color.g = 0.5;
      msg.markers[i].color.b = 1.0;
    }
    vis_marker_array_pub_.publish(msg);

  }


  void TrajectoryOptimizer::display_trajectory()
  {
	  arm_navigation_msgs::DisplayTrajectory display_trajectory;

	  // fill in joint names:
	  display_trajectory.trajectory.joint_trajectory.joint_names.resize(joint_names_.size());
	  display_trajectory.trajectory.joint_trajectory.joint_names = joint_names_;

//	  display_trajectory.trajectory.joint_trajectory.header = "";

	  // fill in the entire trajectory
	  display_trajectory.trajectory.joint_trajectory.points.resize(full_trajectory_->getNumPoints());
	  for (int i=0; i < full_trajectory_->getNumPoints(); i++)
	  {
		  display_trajectory.trajectory.joint_trajectory.points[i].positions.resize(full_trajectory_->getNumJoints());
		  for (size_t j=0; j < display_trajectory.trajectory.joint_trajectory.points[i].positions.size(); j++)
		  {
			  display_trajectory.trajectory.joint_trajectory.points[i].positions[j] = full_trajectory_->getTrajectoryPoint(i)(j);
		  }
		  // Setting invalid timestamps.
		  // Further filtering is required to set valid timestamps accounting for velocity and acceleration constraints.
		  display_trajectory.trajectory.joint_trajectory.points[i].time_from_start = ros::Duration(0.0);
	  }

	  display_trajectory.robot_state.joint_state.name = display_trajectory.trajectory.joint_trajectory.joint_names;
	  display_trajectory.robot_state.joint_state.position = display_trajectory.trajectory.joint_trajectory.points[0].positions;

  }

}
