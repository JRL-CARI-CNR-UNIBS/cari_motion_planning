#pragma once
/*
Copyright (c) 2021, Marco Faroni CNR-STIIMA marco.faroni@stiima.cnr.it
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the <organization> nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

//#include <ros/ros.h>
#include <rclcpp/rclcpp.hpp>
#include <graph_core/multi_goal_selection/policies/policy_base.h>
#include <graph_core/multi_goal_selection/rewards/reward_base.h>

namespace multi_goal_selection
{
class GoalSelectionManager;
typedef std::shared_ptr<GoalSelectionManager> GoalSelectionManagerPtr;

class GoalSelectionManager
{
public:
//  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  GoalSelectionManager(const std::string& name,const unsigned int& n_goals,const unsigned int& n_dof);
  std::vector<double> calculateProbabilities(const std::vector<bool>& were_goals_selected, const std::vector<double>& costs, const std::vector<double>& utopias, const double& best_cost);
  void warmStart(const std::vector<double>& costs, const std::vector<double>& utopias, const double& best_cost);
  bool isWarmStartSet(){return do_warm_start_;};

protected:
  ros::NodeHandle nh_;
  int goal_number_;
  std::vector<double> goal_probabilities_;
  std::string policy_type_;
  std::string policy_name_;
  std::string reward_fcn_name_;
  bool do_warm_start_;

  std::shared_ptr<multi_goal_selection::PolicyBase> policy_;
  std::shared_ptr<multi_goal_selection::RewardBase> reward_fcn_;

  rclcpp::Logger logger = rclcpp::get_logger("goal_selection_manager");
};

}
