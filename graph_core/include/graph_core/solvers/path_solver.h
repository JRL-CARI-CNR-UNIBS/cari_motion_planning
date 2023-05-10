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

#include <graph_core/graph/path.h>
#include <graph_core/collision_checker.h>
#include <graph_core/metrics.h>
#include <rclcpp/rclcpp.hpp>

namespace pathplan
{

class PathLocalOptimizer;
typedef std::shared_ptr<PathLocalOptimizer> PathLocalOptimizerPtr;

class PathLocalOptimizer: public std::enable_shared_from_this<PathLocalOptimizer>
{

protected:
//  ros::NodeHandle nh_;
  std::shared_ptr<rclcpp::Node> nh_;
  CollisionCheckerPtr checker_;
  MetricsPtr metrics_;
  PathPtr path_;
  bool solved_ = false;
  unsigned int max_stall_gen_ = 10;
  unsigned int stall_gen_ = 0;
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  PathLocalOptimizer(const CollisionCheckerPtr& checker,
                     const MetricsPtr& metrics);
  void setPath(const PathPtr& path);
  void config(std::shared_ptr<rclcpp::Node>& nh);
  bool step(PathPtr& solution);
  bool solve(PathPtr& solution, const unsigned int& max_iteration = 100, const double &max_time = std::numeric_limits<double>::infinity());


};

}
