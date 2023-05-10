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

#include <graph_core/multi_goal_selection/goal_selection_manager.h>
#include <graph_core/multi_goal_selection/policies/policy_base.h>
#include <graph_core/multi_goal_selection/policies/policy_mab.h>
#include <graph_core/multi_goal_selection/policies/policy_mab_example.h>
#include <graph_core/multi_goal_selection/policies/policy_uniform_on_goals.h>
#include <graph_core/multi_goal_selection/policies/policy_custom_example.h>
#include <graph_core/multi_goal_selection/policies/policy_uniform_on_volume.h>

#include <graph_core/multi_goal_selection/policies/policy_mab_egreedy.h>
#include <graph_core/multi_goal_selection/policies/policy_mab_ucb.h>
#include <graph_core/multi_goal_selection/policies/policy_mab_ts.h>

#include <graph_core/multi_goal_selection/rewards/reward_relative_improvement.h>
#include <graph_core/multi_goal_selection/rewards/reward_bernoulli.h>
#include <graph_core/multi_goal_selection/rewards/reward_best_cost.h>


namespace multi_goal_selection
{

GoalSelectionManager::GoalSelectionManager(const std::string& name, const unsigned int& n_goals, const unsigned int& n_dof)
{
  nh_ = ros::NodeHandle(name);
  goal_number_ = n_goals;

  if (!nh_.getParam("policy_type",policy_type_))
  {
    RCLCPP_DEBUG(logger, "policy type not set. Default: MultiArmedBandit");
    policy_type_="MultiArmedBandit";
  }

  if (!nh_.getParam("policy_name",policy_name_))
  {
    RCLCPP_DEBUG(logger, "policy name not set. Deafult: eGreedy");
    policy_name_="eGreedy";
  }

  if (!nh_.getParam("reward_fcn",reward_fcn_name_))
  {
    RCLCPP_DEBUG(logger, "reward fcn not set. Default: RelativeImprovement");
    reward_fcn_name_="RelativeImprovement";
  }

  if (!nh_.getParam("warm_start_reward",do_warm_start_))
  {
    RCLCPP_DEBUG(logger, "warm start not set.");
    do_warm_start_=false;
  }

  if (!policy_type_.compare("MultiArmedBandit"))
  {
    RCLCPP_INFO_STREAM(logger, "Policy type: " << policy_type_);
    if (!policy_name_.compare("eGreedy"))
    {
      policy_ = std::make_shared<multi_goal_selection::PolicyMABEGreedy>(nh_.getNamespace(),goal_number_);
      RCLCPP_INFO_STREAM(logger, "Policy name: " << policy_name_);
    }
    else if (!policy_name_.compare("UCB1"))
    {
      policy_ = std::make_shared<multi_goal_selection::PolicyMABUCB>(nh_.getNamespace(),goal_number_);
      RCLCPP_INFO_STREAM(logger, "Policy name: " << policy_name_);
    }
    else if (!policy_name_.compare("Thomson"))
    {
      policy_ = std::make_shared<multi_goal_selection::PolicyMABTS>(nh_.getNamespace(),goal_number_);
      RCLCPP_INFO_STREAM(logger, "Policy name: " << policy_name_);
    }
    else if (!policy_name_.compare("PolicyUniformOnGoals"))
    {
      policy_ = std::make_shared<multi_goal_selection::PolicyUniformOnGoals>(nh_.getNamespace(),goal_number_);
      RCLCPP_INFO_STREAM(logger, "Policy name: " << policy_name_);
    }
    else if (!policy_name_.compare("PolicyUniformOnVolume"))
    {
      policy_ = std::make_shared<multi_goal_selection::PolicyUniformOnVolume>(nh_.getNamespace(),goal_number_,n_dof);
      if (reward_fcn_name_.compare("BestCost"))
      {
        reward_fcn_name_ = "BestCost";
        RCLCPP_WARN(logger, "Reward fcn automatically set to BestCost because policy required by UniformOnVolume policy.");
      }
      if (!do_warm_start_)
      {
        do_warm_start_ = true;
        RCLCPP_WARN(logger, "Warm start automatically set because required by UniformOnVolume policy.");
      }
      RCLCPP_INFO_STREAM(logger, "Policy name: " << policy_name_);
    }
    else
    {
      RCLCPP_FATAL_STREAM(logger, "unexpected policy_name_ : " << policy_name_);
    }
  }
  else if (!policy_type_.compare("Custom"))
  {
    RCLCPP_INFO_STREAM(logger, "Policy type: " << policy_type_);
    if (!policy_name_.compare("Custom1"))
    {
      policy_ = std::make_shared<multi_goal_selection::PolicyCustomExample>(nh_.getNamespace(),goal_number_);
      RCLCPP_INFO_STREAM(logger, "Policy name: " << policy_name_);
    }
    else
    {
      RCLCPP_FATAL_STREAM(logger, "unexpected policy_name_ : " << policy_name_);
    }
  }
  else
  {
    RCLCPP_FATAL_STREAM(logger, "unexpected policy_type_ : " << policy_type_);
  }

  if (!reward_fcn_name_.compare("RelativeImprovement"))
  {
    reward_fcn_ = std::make_shared<multi_goal_selection::RewardRelativeImprovement>();
    RCLCPP_INFO(logger, "Reward name: RelativeImprovement");
  }
  else if (!reward_fcn_name_.compare("Bernoulli"))
  {
    reward_fcn_ = std::make_shared<multi_goal_selection::RewardBernoulli>();
    RCLCPP_INFO(logger, "Reward name: Bernoulli");
  }
  else if (!reward_fcn_name_.compare("BestCost"))
  {
    reward_fcn_ = std::make_shared<multi_goal_selection::RewardBestCost>();
    RCLCPP_INFO(logger, "Reward name: BestCost");
  }
  else
  {
    RCLCPP_FATAL_STREAM(logger, "unexpected reward_fcn_name_ : " << reward_fcn_name_);
  }
}

std::vector<double> GoalSelectionManager::calculateProbabilities(const std::vector<bool>& were_goals_selected,
                                                               const std::vector<double>& costs,
                                                               const std::vector<double>& utopias,
                                                               const double& best_cost)
{
  double reward = reward_fcn_->getReward(costs,utopias,best_cost);

  for (unsigned int i_goal=0; i_goal<goal_number_;i_goal++)
  {
    if (were_goals_selected.at(i_goal))
    {
      policy_->updateState(i_goal,reward);
    }
  }

//  RCLCPP_WARN(logger, "updated state");

  goal_probabilities_ = policy_->getProbabilities();
  return goal_probabilities_;
}

void GoalSelectionManager::warmStart(const std::vector<double>& costs, const std::vector<double>& utopias, const double& best_cost)
{
  for (unsigned int i_goal=0; i_goal<goal_number_;i_goal++)
  {
    double reward = reward_fcn_->getReward(costs,utopias,utopias.at(i_goal));
    policy_->updateState(i_goal,reward);
  }
}


}
