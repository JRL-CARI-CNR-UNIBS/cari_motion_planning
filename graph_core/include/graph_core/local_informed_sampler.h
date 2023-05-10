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


#include <graph_core/sampler.h>


namespace pathplan
{



class LocalInformedSampler: public InformedSampler
{
protected:
  std::vector<Eigen::VectorXd, Eigen::aligned_allocator<Eigen::VectorXd>> centers_;
  std::vector<double> radii_;

  std::uniform_int_distribution<> id_;
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  LocalInformedSampler(const Eigen::VectorXd& start_configuration,
                       const Eigen::VectorXd& stop_configuration,
                       const Eigen::VectorXd& lower_bound,
                       const Eigen::VectorXd& upper_bound,
                       const double& cost = std::numeric_limits<double>::infinity()):
    InformedSampler(start_configuration, stop_configuration, lower_bound, upper_bound, cost)
  {
    logger = logger.get_child("local");
  }

  void addBall(const Eigen::VectorXd& center, const double& radius)
  {
    if (center.size() != start_configuration_.size())
    {
      RCLCPP_INFO(logger, "center size=%zu, start configuration =%zu", center.size(), start_configuration_.size());
    }
    assert(center.size() == start_configuration_.size());
    centers_.push_back(center);
    radii_.push_back(radius);
    id_ = std::uniform_int_distribution<>(0, centers_.size() - 1);
  }

  void clearBalls()
  {
    centers_.clear();
    radii_.clear();
  }

  virtual Eigen::VectorXd sample();

};


typedef std::shared_ptr<LocalInformedSampler> LocalInformedSamplerPtr;

}
