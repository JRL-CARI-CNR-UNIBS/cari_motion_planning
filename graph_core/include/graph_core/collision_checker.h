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
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

//#include <ros/ros.h>
#include <rclcpp/rclcpp.hpp>
#include <eigen3/Eigen/Core>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <graph_core/graph/connection.h>
#include <moveit/planning_scene/planning_scene.h>

namespace pathplan
{
class CollisionChecker;
typedef std::shared_ptr<CollisionChecker> CollisionCheckerPtr;

class CollisionChecker
{
protected:
  double min_distance_ = 0.01;

  rclcpp::Logger logger = rclcpp::get_logger("collision_checker");
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  CollisionChecker(const double& min_distance = 0.01):
    min_distance_(min_distance)
  {

  }

  virtual void setPlanningSceneMsg(const moveit_msgs::msg::PlanningScene& msg){}

  virtual std::string getGroupName()
  {
    return "";
  }

  virtual planning_scene::PlanningScenePtr getPlanningScene() {}

  virtual CollisionCheckerPtr clone()
  {
    return std::make_shared<CollisionChecker>(min_distance_);
  }

  // collision check: true if it is valid
  virtual bool check(const Eigen::VectorXd& configuration)
  {
    return true;
  }

  bool checkPath(const Eigen::VectorXd& configuration1,
                 const Eigen::VectorXd& configuration2,
                 Eigen::VectorXd& conf)
  {
    if (!check(configuration1))
      return false;

    double dist = (configuration2 - configuration1).norm();
    unsigned int npnt = std::ceil(dist / min_distance_);
    if (npnt > 1)
    {
      Eigen::VectorXd step = (configuration2 - configuration1) / npnt;
      for (unsigned int ipnt = 1; ipnt < npnt; ipnt++)
      {
        conf = configuration1 + step * ipnt;
        if (!check(conf))
        {
          // return last feasible configuration
          conf = configuration1 + step * (ipnt - 1);
          return false;
        }
      }
    }

    if (!check(configuration2))
      return false;

    return true;
  }

  virtual bool checkPath(const Eigen::VectorXd& configuration1,
                 const Eigen::VectorXd& configuration2)
  {
    if (!check(configuration1))
    {
      return false;
    }
    if (!check(configuration2))
    {
      return false;
    }
    double distance = (configuration2 - configuration1).norm();
    if (distance < min_distance_)
      return true;

    Eigen::VectorXd conf;
    double n = 2;
    while (distance > n * min_distance_)
    {
      for (double idx = 1; idx < n; idx += 2)
      {
        conf = configuration1 + (configuration2 - configuration1) * idx / n;
        if (!check(conf))
        {
          return false;
        }
      }
      n *= 2;
    }

    return true;
  }


  virtual bool checkConnection(const ConnectionPtr& conn)
  {
    return checkPath(conn->getParent()->getConfiguration(),conn->getChild()->getConfiguration());
  }

  virtual bool checkConnections(const std::vector<ConnectionPtr>& connections)
  {
    for (const ConnectionPtr& c: connections)
      if (!checkConnection(c))
        return false;
    return true;
  }

  virtual bool checkConnFromConf(const ConnectionPtr &conn,
                         const Eigen::VectorXd& this_conf)
  {
    Eigen::VectorXd parent = conn->getParent()->getConfiguration();
    Eigen::VectorXd child = conn->getChild()->getConfiguration();

    double dist_child = (this_conf-child).norm();
    double dist_parent = (parent-this_conf).norm();
    double dist = (parent-child).norm();

    if((dist-dist_child-dist_parent)>1e-04)
    {
      RCLCPP_ERROR(logger, "The conf is not on the connection between parent and child");
      assert(0);
      return false;
    }

    if (!check(this_conf))
    {
      return false;
    }
    if (!check(child))
    {
      return false;
    }

    double distance = (this_conf - child).norm();
    if(distance < min_distance_) return true;

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
          if (!check(conf))
          {
            return false;
          }
        }
      }
      n *= 2;
    }
    return true;
  }
};


class Cube3dCollisionChecker: public CollisionChecker
{
public:
  virtual bool check(const Eigen::VectorXd& configuration)
  {
    return configuration.cwiseAbs().maxCoeff() > 1;
  }
};


}
