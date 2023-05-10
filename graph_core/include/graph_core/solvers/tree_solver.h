#pragma once
/*
Copyright (c) 2019, Manuel Beschi CNR-STIIMA manuel.beschi@stiima.cnr.it
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
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS //GoalCostFunctionPtr getGoalCostFunction() constINTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <graph_core/graph/tree.h>
#include <graph_core/graph/path.h>
#include <graph_core/collision_checker.h>
#include <graph_core/metrics.h>
#include <graph_core/sampler.h>
#include <graph_core/goal_cost_function.h>
//#include <ros/ros.h>
//#include <ros/duration.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/duration.hpp>
namespace pathplan
{


class TreeSolver;
typedef std::shared_ptr<TreeSolver> TreeSolverPtr;
class TreeSolver: public std::enable_shared_from_this<TreeSolver>
{
protected:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  std::shared_ptr<rclcpp::Node> nh_;
  MetricsPtr metrics_;
  CollisionCheckerPtr checker_;
  SamplerPtr sampler_;
  GoalCostFunctionPtr goal_cost_fcn_;
  bool solved_ = false;
  bool completed_=false;
  bool init_ = false;
  bool configured_=false;
  TreePtr start_tree_;
  unsigned int dof_;

  double max_distance_=1.0;
  bool extend_ = false;
  double utopia_tolerance_=1.003;
  bool informed_=true;
  bool warp_=false;
  bool first_warp_=false;
  bool use_kdtree_=true;

  NodePtr goal_node_;                                          // if multigoal, it is related the best goal
  double path_cost_;                                           // if multigoal, it is related the best goal
  double goal_cost_=0;                                         // if multigoal, it is related the best goal
  double cost_=0;                                              // if multigoal, it is related the best goal
  PathPtr solution_;                                           // if multigoal, it is related the best goal
  double best_utopia_=std::numeric_limits<double>::infinity(); // if multigoal, it is related the best goal

  rclcpp::Logger logger = rclcpp::get_logger("tree_solver");

protected:
  virtual bool setProblem(const double &max_time = std::numeric_limits<double>::infinity())
  {
    return false;
  }
  virtual void clean(){}

  virtual void printMyself(std::ostream& os) const {}

public:
  TreeSolver(const MetricsPtr& metrics,
             const CollisionCheckerPtr& checker,
             const SamplerPtr& sampler):
    metrics_(metrics),
    checker_(checker),
    sampler_(sampler)
  {
    path_cost_ = std::numeric_limits<double>::infinity();
    goal_cost_ = 0.0;
    cost_ = std::numeric_limits<double>::infinity();
    goal_cost_fcn_=std::make_shared<GoalCostFunction>();
  }

  virtual bool update(PathPtr& solution) = 0;
  virtual bool update(const Eigen::VectorXd& configuration, PathPtr& solution){return false;}
  virtual bool update(const NodePtr& n, PathPtr& solution){return false;}

  virtual bool initGoalSelector(){return false;}

  virtual bool solve(PathPtr& solution, const unsigned int& max_iter = 100, const double &max_time = std::numeric_limits<double>::infinity());
  virtual bool addStart(const NodePtr& start_node, const double &max_time = std::numeric_limits<double>::infinity()) = 0;
  virtual bool addGoal(const NodePtr& goal_node, const double &max_time = std::numeric_limits<double>::infinity()) = 0;
  virtual bool addStartTree(const TreePtr& start_tree, const double &max_time = std::numeric_limits<double>::infinity())=0;

  //virtual bool computePath(const NodePtr &start_node, const NodePtr &goal_node, const ros::NodeHandle& nh, PathPtr &solution, const double &max_time = std::numeric_limits<double>::infinity(), const unsigned int max_iter = 10000);
  virtual bool computePath(const NodePtr &start_node, const NodePtr &goal_node, const std::shared_ptr<rclcpp::Node>& nh, PathPtr &solution, const double &max_time = std::numeric_limits<double>::infinity(), const unsigned int max_iter = 10000);
  virtual void resetProblem()=0;
  virtual TreeSolverPtr clone(const MetricsPtr& metrics, const CollisionCheckerPtr& checker, const SamplerPtr& sampler) = 0;

  virtual bool setSolution(const PathPtr &solution, const bool& solved=false);

  bool importFromSolver(const TreeSolverPtr& solver);

  const double& cost() const
  {
    return cost_;
  }

  //virtual bool config(const ros::NodeHandle& nh);
  virtual bool config(const std::shared_ptr<rclcpp::Node>& nh);

  void setGoalCostFunction(const GoalCostFunctionPtr& goal_cost_fcn)
  {
    goal_cost_fcn_=goal_cost_fcn;
  }

  const bool& completed()const
  {
    return completed_;
  }

  const bool& solved()const
  {
    return solved_;
  }

  const bool& init()const
  {
    return init_;
  }

  const bool& configured()const
  {
    return configured_;
  }

  const unsigned int& dof()const
  {
    return dof_;
  }

  GoalCostFunctionPtr getGoalCostFunction() const
  {
    return goal_cost_fcn_;
  }

  TreePtr getStartTree() const
  {
    return start_tree_;
  }

  virtual std::vector<TreePtr> getGoalTrees()
  {
    RCLCPP_DEBUG(logger, "virtual fcn");
  }

  PathPtr getSolution() const
  {
    return solution_;
  }

  std::shared_ptr<rclcpp::Node> getNodeHandle() const
  {
    return nh_;
  }

  void setSampler(const SamplerPtr& sampler)
  {
    sampler_ = sampler;
  }

  SamplerPtr getSampler() const
  {
    return sampler_;
  }

  double getPathCost() const
  {
    return path_cost_;
  }

  double getGoalCost() const
  {
    return goal_cost_;
  }

  double getCost() const
  {
    return cost_;
  }

  void setSolved(const bool& solved)
  {
    solved_ = solved;
  }

  virtual void setStartTree(const TreePtr& tree)
  {
    start_tree_ = tree;
  }

  void setCompleted(const bool& completed)
  {
    completed_ = completed;
  }

  void setInit(const bool& init)
  {
    init_ = init;
  }

  void setPathCost(const double& path_cost)
  {
    path_cost_ = path_cost;
  }

  void setChecker(const CollisionCheckerPtr& checker)
  {
    checker_ = checker;
  }

  CollisionCheckerPtr getChecker() const
  {
    return checker_;
  }

  void setMetrics(const MetricsPtr& metrics)
  {
    metrics_ = metrics;
  }

  MetricsPtr getMetrics() const
  {
    return metrics_;
  }

  virtual void setUtopia(const double& utopia)
  {
    best_utopia_ = utopia;
  }
  double getUtopia()
  {
    return best_utopia_;
  }

  virtual void setGoal(const NodePtr& goal)
  {
    goal_node_=goal;
  }

  virtual NodePtr getGoal()
  {
    return goal_node_;
  }

  void setMaxDistance(const double& distance)
  {
    max_distance_ = distance;
  }

  double getMaxDistance()
  {
    return max_distance_;
  }


  double updateCost()
  {
    path_cost_ = solution_->cost();
    cost_ = path_cost_+goal_cost_;
    return cost_;
  }

  double updateCost(const NodePtr& goal_node)
  {
    goal_cost_ = goal_cost_fcn_->cost(goal_node);
    return updateCost();
  }

  friend std::ostream& operator<<(std::ostream& os, const TreeSolver& solver);

};
inline std::ostream& operator<<(std::ostream& os, const TreeSolver& solver){solver.printMyself(os);return os;}

}  // namespace pathplan

