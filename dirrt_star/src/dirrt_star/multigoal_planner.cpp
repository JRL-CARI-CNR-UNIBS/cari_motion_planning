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
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/


#include <dirrt_star/multigoal_planner.h>



namespace pathplan {
namespace dirrt_star {

MultigoalPlanner::MultigoalPlanner ( const std::string& name,
                                     const std::string& group,
                                     const moveit::core::RobotModelConstPtr& model ) :
  PlanningContext ( name, group ),
  group_(group)
{
  m_nh=ros::NodeHandle(name);
  m_nh.setCallbackQueue(&m_queue);


  COMMENT("create MultigoalPlanner, name =%s, group = %s", name.c_str(),group.c_str());
  robot_model_=model;
  if (!robot_model_)
  {
    ROS_ERROR("robot model is not initialized");
  }
  COMMENT("model name = %s",robot_model_->getName().c_str());

  COMMENT("get joint model group");

  const moveit::core::JointModelGroup* jmg=robot_model_->getJointModelGroup(group);
  if (jmg==NULL)
    ROS_ERROR("unable to find JointModelGroup for group %s",group.c_str());


  COMMENT("get joint names of JointModelGroup=%s",jmg->getName().c_str());

  joint_names_=jmg->getActiveJointModelNames();
  m_dof=joint_names_.size();
  COMMENT("number of joints  = %u",m_dof);
  m_lb.resize(m_dof);
  m_ub.resize(m_dof);
  m_max_speed_.resize(m_dof);

  COMMENT("read bounds");
  for (unsigned int idx=0;idx<m_dof;idx++)
  {
    COMMENT("joint %s",joint_names_.at(idx).c_str());
    const robot_model::VariableBounds& bounds = robot_model_->getVariableBounds(joint_names_.at(idx));
    if (bounds.position_bounded_)
    {
      m_lb(idx)=bounds.min_position_;
      m_ub(idx)=bounds.max_position_;
      m_max_speed_(idx)=bounds.max_velocity_;
    }
  }


  urdf::Model urdf_model;
  urdf_model.initParam("robot_description");


  if (!m_nh.getParam("display_bubbles",display_flag_))
  {
    ROS_DEBUG("display_flag is not set, default=false");
    display_flag_=false;
  }

  if (!m_nh.getParam("display_tree",display_tree_))
  {
    ROS_DEBUG("display_tree is not set, default=false");
    display_tree_=false;
  }
  if (display_tree_)
  {
    if (!m_nh.getParam("display_tree_period",display_tree_period_))
    {
      ROS_DEBUG("display_tree_rate is not set, default=1.0");
      display_tree_period_=1.0;
    }
  }


  COMMENT("create metrics");

  if (!m_nh.getParam("use_avoidance_path",use_avoidance_metrics_))
  {
    ROS_DEBUG("use_avoidance_path is not set, default=false");
    use_avoidance_metrics_=false;
  }
  if (use_avoidance_metrics_)
  {
    avoidance_metrics_=std::make_shared<pathplan::AvoidanceMetrics>(m_nh);
    metrics_=avoidance_metrics_;
  }
  else
    metrics_=std::make_shared<pathplan::Metrics>();

  if (!m_nh.getParam("use_avoidance_goal",use_avoidance_goal_))
  {
    ROS_DEBUG("use_avoidance is not set, default=false");
    use_avoidance_goal_=false;
  }
  else if (use_avoidance_metrics_)
  {
    ROS_DEBUG("both use_avoidance_goal and use_avoidance_path are set, using use_avoidance_path");
    use_avoidance_goal_=false;
  }

  if (use_avoidance_goal_)
  {
    m_avoidance_goal_cost_fcn=std::make_shared<pathplan::AvoidanceGoalCostFunction>(m_nh);
  }


  std::string detector_topic;
  if (use_avoidance_goal_ || use_avoidance_metrics_)
  {
    if (!m_nh.getParam("detector_topic",detector_topic))
    {
      ROS_DEBUG("detector_topic is not defined, using poses");
      detector_topic="/poses";
    }
    m_centroid_sub=m_nh.subscribe(detector_topic,2,&MultigoalPlanner::centroidCb,this);
  }

  if (!m_nh.getParam("collision_distance",collision_distance_))
  {
    ROS_DEBUG("collision_distance is not set, default=0.04");
    collision_distance_=0.04;
  }

  if (!m_nh.getParam("collision_thread",collision_thread_))
  {
    ROS_DEBUG("collision_thread is not set, default=5");
    collision_thread_=5;
  }



  COMMENT("created MultigoalPlanner");

  double refining_time=0;
  if (!m_nh.getParam("max_refine_time",refining_time))
  {
    ROS_DEBUG("refining_time is not set, default=30");
    refining_time=30;
  }
  m_max_refining_time=ros::WallDuration(refining_time);

  m_solver_performance=m_nh.advertise<std_msgs::Float64MultiArray>("/solver_performance",1000);

}



void MultigoalPlanner::clear()
{

}


void MultigoalPlanner::setSampler(pathplan::SamplerPtr& sampler,
                                  pathplan::TreeSolverPtr& solver)
{
  sampler = std::make_shared<pathplan::InformedSampler>(m_lb, m_ub, m_lb, m_ub);
  solver=std::make_shared<pathplan::MultigoalSolver>(metrics_, checker, sampler);

}

bool MultigoalPlanner::solve ( planning_interface::MotionPlanDetailedResponse& res )
{


  ros::WallDuration max_planning_time=ros::WallDuration(request_.allowed_planning_time);
  ros::WallTime start_time = ros::WallTime::now();
  ros::WallTime refine_time = ros::WallTime::now();
  m_is_running=true;

  std_msgs::Float64MultiArray performace_msg;
  performace_msg.layout.dim.resize(4);
  performace_msg.layout.dim.at(0).label="time";
  performace_msg.layout.dim.at(0).size=1;
  performace_msg.layout.dim.at(1).label="iteration";
  performace_msg.layout.dim.at(1).size=1;
  performace_msg.layout.dim.at(2).label="cost";
  performace_msg.layout.dim.at(2).size=1;
  performace_msg.layout.dim.at(3).label=m_nh.getNamespace();
  performace_msg.layout.dim.at(3).size=0;

  if (!planning_scene_)
  {
    ROS_ERROR("No planning scene available");
    res.error_code_.val=moveit_msgs::MoveItErrorCodes::COLLISION_CHECKING_UNAVAILABLE;
    m_is_running=false;
    return false;
  }


  if (display_flag_ || display_tree_)
  {
    if (!display)
      display=std::make_shared<pathplan::Display>(planning_scene_,group_);
    else
      display->clearMarkers();
  }

  planning_scene::PlanningScenePtr ptr=planning_scene::PlanningScene::clone(planning_scene_);
  checker=std::make_shared<pathplan::ParallelMoveitCollisionChecker>(ptr,group_,collision_thread_,collision_distance_);


  moveit::core::RobotState start_state(robot_model_);
  moveit::core::robotStateMsgToRobotState(request_.start_state,start_state);
  if (request_.start_state.joint_state.position.size()==0)
    start_state=planning_scene_->getCurrentState();
  else
    moveit::core::robotStateMsgToRobotState(request_.start_state,start_state);

  start_state.update();
  start_state.updateCollisionBodyTransforms();

  const moveit::core::JointModelGroup* jmg = start_state.getJointModelGroup(group_);
  if (!start_state.satisfiesBounds(jmg))
  {

    std::vector<const moveit::core::JointModel*> joint_models= jmg->getJointModels();
    for (const moveit::core::JointModel*& jm: joint_models)
    {
      if (!start_state.satisfiesPositionBounds(jm))
      {
         ROS_ERROR("joint %s is out of bound, actual position=%f",jm->getName().c_str(),*start_state.getJointPositions(jm->getName()));
         std::vector<moveit::core::VariableBounds> bound=jm->getVariableBounds();

         for (const moveit::core::VariableBounds& b: bound)
         {
           ROS_ERROR("joint %s has bound pos = [%f, %f]",jm->getName().c_str(),b.min_position_,b.max_position_);
         }
      }
      if (!start_state.satisfiesVelocityBounds(jm))
      {
         ROS_ERROR("joint %s is out of bound, actual velocity=%f",jm->getName().c_str(),*start_state.getJointVelocities(jm->getName()));
         std::vector<moveit::core::VariableBounds> bound=jm->getVariableBounds();
         for (const moveit::core::VariableBounds& b: bound)
         {
           ROS_ERROR("joint %s has bound vel = [%f, %f]",jm->getName().c_str(),b.min_position_,b.max_position_,b.min_velocity_,b.max_velocity_);
         }
      }
    }
    ROS_FATAL("Start point is  Out of bound");
    res.error_code_.val=moveit_msgs::MoveItErrorCodes::START_STATE_IN_COLLISION;
    m_is_running=false;
    return false;
  }

  Eigen::VectorXd start_conf;
  start_state.copyJointGroupPositions(group_,start_conf);

  for (size_t ij=0;ij<joint_names_.size();ij++)
  {
    start_state.setJointPositions(joint_names_.at(ij),&start_conf(ij));
  }

  if (!checker->check(start_conf))
  {
    ROS_ERROR("Start point is in collision");

    collision_detection::CollisionRequest col_req;
    collision_detection::CollisionResult col_res;
    col_req.contacts = true;
    col_req.group_name=group_;
    planning_scene_->checkCollision(col_req,col_res,start_state);
    if (col_res.collision)
    {
      ROS_ERROR("Start state is colliding +++");
      for (const  std::pair<std::pair<std::string, std::string>, std::vector<collision_detection::Contact> >& contact: col_res.contacts)
      {
        ROS_ERROR("contact between %s and %s",contact.first.first.c_str(),contact.first.second.c_str());
      }
    }
    else
    {
      ROS_FATAL("you shouldn't be here!");
      ROS_FATAL_STREAM("start state = " << start_state);
      ROS_FATAL_STREAM("req start state = " << request_.start_state);

    }
    res.error_code_.val=moveit_msgs::MoveItErrorCodes::START_STATE_IN_COLLISION;
    m_is_running=false;
    return false;
  }


  m_start_node=std::make_shared<pathplan::Node>(start_conf);



  pathplan::SamplerPtr sampler;
  pathplan::TreeSolverPtr solver;

  setSampler(sampler,solver);

  if (!solver->config(m_nh))
  {
    ROS_ERROR("Unable to configure the planner");
    res.error_code_.val=moveit_msgs::MoveItErrorCodes::INVALID_GROUP_NAME;
    m_is_running=false;
    return false;
  }
  if (use_avoidance_goal_)
    solver->setGoalCostFunction(m_avoidance_goal_cost_fcn);
  solver->addStart(m_start_node);

  m_queue.callAvailable();
  bool at_least_a_goal=false;

  // joint goal
  for (unsigned int iGoal=0;iGoal<request_.goal_constraints.size();iGoal++)
  {
    ROS_DEBUG("Processing goal %u",iGoal);

    moveit_msgs::Constraints goal=request_.goal_constraints.at(iGoal);
    if (goal.joint_constraints.size()==0)
    {
      ROS_DEBUG("Goal %u is a Cartesian goal",iGoal);

      if (goal.position_constraints.size()!=1 || goal.orientation_constraints.size()!=1)
      {
        ROS_DEBUG("Goal %u has no position or orientation",iGoal);
        continue;
      }

      Eigen::Quaterniond q_w_tool;
      q_w_tool.x()=goal.orientation_constraints.at(0).orientation.x;
      q_w_tool.y()=goal.orientation_constraints.at(0).orientation.y;
      q_w_tool.z()=goal.orientation_constraints.at(0).orientation.z;
      q_w_tool.w()=goal.orientation_constraints.at(0).orientation.w;


      Eigen::Affine3d T_w_tool;
      T_w_tool.setIdentity();
      T_w_tool=q_w_tool;
      T_w_tool.translation()(0)=goal.position_constraints.at(0).target_point_offset.x;
      T_w_tool.translation()(1)=goal.position_constraints.at(0).target_point_offset.y;
      T_w_tool.translation()(2)=goal.position_constraints.at(0).target_point_offset.z;

    }
    else // joint constraints
    {
      ROS_DEBUG("Goal %u is a joint goal",iGoal);
      Eigen::VectorXd goal_configuration( goal.joint_constraints.size() );
      moveit::core::RobotState goal_state(robot_model_);

      for (auto c: goal.joint_constraints)
        goal_state.setJointPositions(c.joint_name,&c.position);
      goal_state.copyJointGroupPositions(group_,goal_configuration);

      goal_state.updateCollisionBodyTransforms();
      COMMENT("check collision on goal %u",iGoal);

      if (!checker->check(goal_configuration))
      {
        ROS_DEBUG("goal %u is in collision",iGoal);

        if (request_.goal_constraints.size()<5)
        {

          if (!goal_state.satisfiesBounds())
          {
            ROS_INFO_STREAM("End state: " << goal_configuration.transpose()<<" is  Out of bound");
          }

          collision_detection::CollisionRequest col_req;
          collision_detection::CollisionResult col_res;
          col_req.contacts = true;
          col_req.group_name=group_;
          planning_scene_->checkCollision(col_req,col_res,goal_state);
          if (col_res.collision)
          {
            ROS_INFO_STREAM("End state: " << goal_configuration.transpose()<<" is colliding");
            for (const  std::pair<std::pair<std::string, std::string>, std::vector<collision_detection::Contact> >& contact: col_res.contacts)
            {
              ROS_INFO("contact between %s and %s",contact.first.first.c_str(),contact.first.second.c_str());
            }
          }
        }
        continue;
      }
      COMMENT("goal is valid");

      pathplan::NodePtr goal_node=std::make_shared<pathplan::Node>(goal_configuration);
      solver->addGoal(goal_node);
      at_least_a_goal=true;
    }
  }


  if (!at_least_a_goal)
  {
    ROS_ERROR("All goals are in collision");
    res.error_code_.val=moveit_msgs::MoveItErrorCodes::GOAL_IN_COLLISION;
    m_is_running=false;
    return false;
  }

  solver->initGoalSelector(); // init multi-goal selection policy

  // ===============================
  // BEGINNING OF THE IMPORTANT PART
  // ===============================


  // searching initial solutions
  pathplan::PathPtr solution;
  bool found_a_solution=false;
  unsigned int iteration=0;
  ros::WallTime display_time = ros::WallTime::now();
  while((ros::WallTime::now()-start_time)<max_planning_time)
  {

    performace_msg.data.push_back((ros::WallTime::now()-start_time).toSec());
    performace_msg.data.push_back(iteration);
    performace_msg.data.push_back(solver->getCost());

    if (display_tree_)
    {
      if ((ros::WallTime::now()-display_time).toSec()>display_tree_period_)
      {
        display_time = ros::WallTime::now();
        display->displayTree(solver->getStartTree());
        std::vector<TreePtr> goal_trees = solver->getGoalTrees();
        for (auto& goal_tree: goal_trees)
          display->displayTree(goal_tree);
      }
    }
    iteration++;
    if (m_stop)
    {
      ROS_INFO("Externally stopped");
      res.error_code_.val=moveit_msgs::MoveItErrorCodes::PREEMPTED;
      m_is_running=false;
      return false;
    }

    solver->update(solution);
    if (!found_a_solution && solver->solved())
    {
      assert(solution);
      ROS_INFO("Find a first solution (cost=%f) in %f seconds",solver->cost(),(ros::WallTime::now()-start_time).toSec());
      ROS_DEBUG_STREAM(*solver);
      found_a_solution=true;
      refine_time = ros::WallTime::now();
    }
    if (solver->completed())
    {
      ROS_INFO("Optimization completed (cost=%f) in %f seconds (%u iterations)",solver->cost(),(ros::WallTime::now()-start_time).toSec(),iteration);
      break;
    }

    if (found_a_solution && ((ros::WallTime::now()-refine_time)>m_max_refining_time))
    {
      ROS_INFO("Refine time expired (cost=%f) in %f seconds (%u iterations)",solver->cost(),(ros::WallTime::now()-start_time).toSec(),iteration);
      break;
    }
  }

  ROS_INFO_STREAM(*solver);

  m_solver_performance.publish( performace_msg);

  if (!found_a_solution)
  {
    ROS_ERROR("unable to find a valid path");
    res.error_code_.val=moveit_msgs::MoveItErrorCodes::PLANNING_FAILED;
    m_is_running=false;
    if (display_flag_)
      display->displayTree(solver->getStartTree());
    return false;
  }
  //  if (display_flag)
  //    display->displayPath(solution);

  if (!solver->completed())
  {
    ROS_INFO("Stopped (cost=%f) after %f seconds (%u iterations)",solver->cost(),(ros::WallTime::now()-start_time).toSec(),iteration);
  }

  // =========================
  // END OF THE IMPORTANT PART
  // =========================
  COMMENT("Get waypoints");
  std::vector<Eigen::VectorXd> waypoints=solution->getWaypoints();// PUT WAY POINTS
  robot_trajectory::RobotTrajectoryPtr trj(new robot_trajectory::RobotTrajectory(robot_model_,group_));

  COMMENT("processing %zu waypoints..", waypoints.size());
  for (const Eigen::VectorXd& waypoint: waypoints )
  {
    COMMENT("processing waypoint");
    moveit::core::RobotState wp_state=start_state;
    wp_state.setJointGroupPositions(group_,waypoint);
    wp_state.update();
    trj->addSuffixWayPoint(wp_state,0);
  }
  ros::WallDuration wd = ros::WallTime::now() - start_time;


  res.processing_time_.push_back(wd.toSec());
  res.description_.emplace_back("plan");
  res.trajectory_.push_back(trj);

  res.error_code_.val=moveit_msgs::MoveItErrorCodes::SUCCESS;
  m_is_running=false;
  COMMENT("ok");


  return true;
}

bool MultigoalPlanner::solve ( planning_interface::MotionPlanResponse& res )
{
  ros::WallTime start_time = ros::WallTime::now();
  planning_interface::MotionPlanDetailedResponse detailed_res;
  bool success = solve(detailed_res);
  if(success)
  {
    res.trajectory_ = detailed_res.trajectory_.at(0);
  }
  ros::WallDuration wd = ros::WallTime::now() - start_time;
  res.planning_time_ = wd.toSec();
  res.error_code_ = detailed_res.error_code_;

  return success;
}

bool MultigoalPlanner::terminate()
{
  m_stop=true;
  ros::Time t0=ros::Time::now();
  ros::Rate lp(50);
  while (ros::ok())
  {
    if (!m_is_running)
      return true;
    lp.sleep();
    if ((ros::Time::now()-t0).toSec()>5)
    {
      ROS_ERROR("Unable to stop planner %s of group %s",name_.c_str(),group_.c_str());
      return false;
    }
  }
  return false;
}

void MultigoalPlanner::centroidCb(const geometry_msgs::PoseArrayConstPtr& msg)
{

  if (!use_avoidance_goal_ && !use_avoidance_metrics_)
    return;
  if (use_avoidance_goal_)
    m_avoidance_goal_cost_fcn->cleanPoints();
  if (use_avoidance_metrics_)
    avoidance_metrics_->cleanPoints();
  Eigen::Vector3d point;
  for (const geometry_msgs::Pose& p: msg->poses)
  {
    point(0)=p.position.x;
    point(1)=p.position.y;
    point(2)=p.position.z;
    if (use_avoidance_goal_)
      m_avoidance_goal_cost_fcn->addPoint(point);
    //    ros::Duration(0.1).sleep();
    if (use_avoidance_metrics_)
      avoidance_metrics_->addPoint(point);
  }
  m_avoidance_goal_cost_fcn->publishPoints();
}


}  // namespace dirrt_star
}  // namespace pathplan
