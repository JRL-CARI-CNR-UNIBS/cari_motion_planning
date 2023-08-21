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



#include <graph_core/parallel_moveit_collision_checker.h>

namespace pathplan
{

ParallelMoveitCollisionChecker::ParallelMoveitCollisionChecker(const planning_scene::PlanningScenePtr &planning_scene,
                                                               const std::string& group_name,
                                                               const int& threads_num,
                                                               const double& min_distance):
  MoveitCollisionChecker(planning_scene,group_name,min_distance),
  threads_num_(threads_num)
{
  min_distance_ = min_distance;
  group_name_ = group_name;

  if (threads_num<=0)
    throw std::invalid_argument("number of thread should be positive");

  at_least_a_collision_=false;
  stop_check_=true;
  thread_iter_=0;

  threads.resize(threads_num_);
  for (int idx=0;idx<threads_num_;idx++)
  {
    planning_scenes_.push_back(planning_scene::PlanningScene::clone(planning_scene_));
    queues_.push_back(std::vector<std::vector<double>>());
  }

  req_.distance=false;
  req_.group_name=group_name;
  req_.verbose=false;
  req_.contacts=false;
  req_.cost=false;

}

void ParallelMoveitCollisionChecker::resetQueue()
{
  at_least_a_collision_=false;
  stop_check_=true;
  thread_iter_=0;
  for (int idx=0;idx<threads_num_;idx++)
  {
    if (threads.at(idx).joinable())
      threads.at(idx).join();
    queues_.at(idx).clear();
  }
}

void ParallelMoveitCollisionChecker::queueUp(const Eigen::VectorXd &q)
{
  if(q.size() != 0)
  {
    std::vector<double> conf(q.size());
    for (unsigned idx=0;idx<q.size();idx++)
      conf.at(idx)=q(idx);


    queues_.at(thread_iter_).push_back(conf);
    thread_iter_ ++;
    if (thread_iter_>=threads_num_)
      thread_iter_=0;
  }
  else
    throw std::invalid_argument("q is empty");
}

bool ParallelMoveitCollisionChecker::checkAllQueues()
{
  stop_check_=false;
  for (int idx=0;idx<threads_num_;idx++)
  {
    if (queues_.at(idx).size()>0)
      threads.at(idx)=std::thread(&ParallelMoveitCollisionChecker::collisionThread,this,idx);
  }
  for (int idx=0;idx<threads_num_;idx++)
  {
    if (threads.at(idx).joinable())
      threads.at(idx).join();
  }
  return  !at_least_a_collision_;
}

void ParallelMoveitCollisionChecker::collisionThread(int thread_idx)
{
  robot_state::RobotStatePtr state=std::make_shared<robot_state::RobotState>(planning_scenes_.at(thread_idx)->getCurrentState());
  const std::vector<std::vector<double>>& queue=queues_.at(thread_idx);
  for (const std::vector<double>& configuration: queue)
  {
    if (stop_check_)
    {
      break;
    }
    assert(configuration.size()>0);
    //state->setJointGroupPositions(group_name_, configuration);

    for (size_t ij=0;ij<joint_names_.size();ij++)
    {
      state_->setJointPositions(joint_models_.at(ij),&configuration.at(ij));
    }


    state->update();
    if (!state->satisfiesBounds(jmg_))
    {
      at_least_a_collision_=true;
      stop_check_=true;
      break;
    }
    state->updateCollisionBodyTransforms();

    if (planning_scenes_.at(thread_idx)->isStateColliding(*state,group_name_))
    {
      at_least_a_collision_=true;
      stop_check_=true;
      break;
    }

    //    if (!planning_scenes_.at(thread_idx)->isStateFeasible(*state))
    //    {
    //      at_least_a_collision_=true;
    //      stop_check_=true;
    //      break;
    //    }
    //    planning_scenes_.at(thread_idx)->checkCollision(req_,res_);
    //    if (res_.collision)
    //    {
    //      at_least_a_collision_=true;
    //      stop_check_=true;
    //      break;
    //    }

  }
}

ParallelMoveitCollisionChecker::~ParallelMoveitCollisionChecker()
{
  ROS_DEBUG("closing collision threads");
  stop_check_=true;
  for (int idx=0;idx<threads_num_;idx++)
  {
    if (threads.at(idx).joinable())
      threads.at(idx).join();
  }
  ROS_DEBUG("collision threads closed");
}

void ParallelMoveitCollisionChecker::setPlanningSceneMsg(const moveit_msgs::PlanningScene& msg)
{
  stop_check_=true;
  for (int idx=0;idx<threads_num_;idx++)
  {
    if (threads.at(idx).joinable())
      threads.at(idx).join();
  }
  if (!planning_scene_->setPlanningSceneMsg(msg))
  {
    ROS_ERROR_THROTTLE(1,"unable to upload scene");
  }
  for (int idx=0;idx<threads_num_;idx++)
  {
    if(!planning_scenes_.at(idx)->setPlanningSceneMsg(msg))
    {
      ROS_ERROR_THROTTLE(1,"unable to upload scene");
    }
  }
}

void ParallelMoveitCollisionChecker::setPlanningScene(planning_scene::PlanningScenePtr &planning_scene)
{
  stop_check_=true;
  for (int idx=0;idx<threads_num_;idx++)
  {
    if (threads.at(idx).joinable())
      threads.at(idx).join();
  }
  planning_scene_ = planning_scene;
  for (int idx=0;idx<threads_num_;idx++)
  {
    planning_scenes_.at(idx)=planning_scene::PlanningScene::clone(planning_scene_);
  }
}

void ParallelMoveitCollisionChecker::queueConnection(const Eigen::VectorXd& configuration1,
                                                     const Eigen::VectorXd& configuration2)
{

  double distance = (configuration2 - configuration1).norm();
  if (distance < min_distance_)
    return;


  Eigen::VectorXd conf(configuration1.size());
  double n = 2;

  while (distance > n * min_distance_)
  {
    for (double idx = 1; idx < n; idx += 2)
    {
      conf = configuration1 + (configuration2 - configuration1) * idx / n;
      queueUp(conf);
    }
    n *= 2;
  }
}

bool ParallelMoveitCollisionChecker::checkPath(const Eigen::VectorXd& configuration1,
                                               const Eigen::VectorXd& configuration2)
{
  resetQueue();
  if (!check(configuration1))
    return false;
  if (!check(configuration2))
    return false;
  queueConnection(configuration1,configuration2);
  return checkAllQueues();
}

bool ParallelMoveitCollisionChecker::checkConnFromConf(const ConnectionPtr& conn,
                                                       const Eigen::VectorXd& this_conf)
{
  resetQueue();
  Eigen::VectorXd parent = conn->getParent()->getConfiguration();
  Eigen::VectorXd child = conn->getChild()->getConfiguration();

  double dist_child = (this_conf-child).norm();
  double dist_parent = (parent-this_conf).norm();
  double dist = (parent-child).norm();

  if((dist-dist_child-dist_parent)>1e-04)
  {
    ROS_ERROR("The conf is not on the connection between parent and child");
    assert(0);
    return false;
  }

  if(!check(this_conf)) return false;
  if(!check(child)) return false;

  double distance = (this_conf - child).norm();
  if(distance < min_distance_)
  {
    return true;
  }

  double this_abscissa = (parent-this_conf).norm()/(parent-child).norm();
  double abscissa;
  double n = 2;
  Eigen::VectorXd conf;
  while (distance > n * min_distance_)
  {
    for (double idx = 1; idx < n; idx += 2)
    {
      abscissa = idx/n;
      if(abscissa>=this_abscissa)
      {
        conf = parent + (child - parent) * abscissa;
        queueUp(conf);
      }
    }
    n *= 2;
  }

  return checkAllQueues();
}
bool ParallelMoveitCollisionChecker::checkConnections(const std::vector<ConnectionPtr>& connections)
{
  resetQueue();
  if (!check(connections.front()->getParent()->getConfiguration()))
    return false;
  for (const ConnectionPtr& c: connections)
  {
    if (!check(c->getChild()->getConfiguration()))
      return false;
    queueConnection(c->getParent()->getConfiguration(),c->getChild()->getConfiguration());
  }
  return checkAllQueues();
}

CollisionCheckerPtr ParallelMoveitCollisionChecker::clone()
{
  planning_scene::PlanningScenePtr planning_scene = planning_scene::PlanningScene::clone(planning_scene_);
  return std::make_shared<ParallelMoveitCollisionChecker>(planning_scene,group_name_,threads_num_,min_distance_);
}

}  // pathplan
