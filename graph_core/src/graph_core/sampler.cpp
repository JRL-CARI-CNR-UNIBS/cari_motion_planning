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


Eigen::VectorXd InformedSampler::sample()
{
  if (inf_cost_)
  {
    return center_bound_ + Eigen::MatrixXd::Random(ndof_, 1).cwiseProduct(bound_width_);
  }
  else
  {
    Eigen::VectorXd ball(ndof_);
    for (int iter = 0; iter < 100; iter++)
    {
      ball.setRandom();
      ball *= std::pow(ud_(gen_), 1.0 / (double)ndof_) / ball.norm();

      Eigen::VectorXd q = rot_matrix_ * ellipse_axis_.asDiagonal() * ball + ellipse_center_;

      bool in_of_bounds = true;
      for (unsigned int iax = 0; iax < ndof_; iax++)
      {
        if (q(iax) > upper_bound_(iax) || q(iax) < lower_bound_(iax))
        {
          in_of_bounds = false;
          break;
        }
      }
      if (in_of_bounds)
        return q;
    }
    // RCLCPP_DEBUG_THROTTLE(logger, 0.1, "unable to find a feasible point in the ellipse");
    RCLCPP_DEBUG_ONCE(logger, "unable to find a feasible point in the ellipse");
    return center_bound_ + Eigen::MatrixXd::Random(ndof_, 1).cwiseProduct(bound_width_);
  }
}

bool InformedSampler::inBounds(const Eigen::VectorXd& q)
{
  for (unsigned int iax = 0; iax < ndof_; iax++)
  {
    if (q(iax) > upper_bound_(iax) || q(iax) < lower_bound_(iax))
    {
      return false;
    }
  }
  if (inf_cost_)
    return true;
  else
    return ((q - start_configuration_).norm() + (q - stop_configuration_).norm()) < cost_;

}

void InformedSampler::setCost(const double &cost)
{
  cost_ = cost;
  inf_cost_ = cost_ >= std::numeric_limits<double>::infinity();

  if (cost_ < focii_distance_)
  {
    RCLCPP_WARN(logger, "cost is %f, focci distance is %f", cost_, focii_distance_);
    RCLCPP_WARN_STREAM(logger, "start_configuration: " << start_configuration_.transpose());
    RCLCPP_WARN_STREAM(logger, "stop_configuration: " << stop_configuration_.transpose());
    cost_ = focii_distance_;
    min_radius_ = 0.0;
  }
  else
  {
    min_radius_ = 0.5 * std::sqrt(std::pow(cost, 2) - std::pow(focii_distance_, 2));
  }
  max_radius_ = 0.5 * cost;
  ellipse_axis_.setConstant(min_radius_);
  ellipse_axis_(0) = max_radius_;

  if (inf_cost_)
  {
    specific_volume_=std::tgamma(ndof_*0.5+1)/std::pow(M_PI,ndof_*0.5);  // inverse of the volume of unit ball

    for (unsigned int idx=0;idx<ndof_;idx++)
    {
      specific_volume_*=(upper_bound_(idx)-lower_bound_(idx));
    }
  }
  else
    specific_volume_=max_radius_*std::pow(min_radius_,ndof_-1);

  if (specific_volume_>0.0)
    specific_volume_=std::pow(specific_volume_,1./ndof_);
}

double InformedSampler::getSpecificVolume()
{
  return specific_volume_;
}

}
