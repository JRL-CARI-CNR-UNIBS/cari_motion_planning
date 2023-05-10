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


#include <graph_core/time_sampler.h>

namespace pathplan
{
TimeBasedInformedSampler::TimeBasedInformedSampler(const Eigen::VectorXd& start_configuration,
                                                   const Eigen::VectorXd& stop_configuration,
                                                   const Eigen::VectorXd& lower_bound,
                                                   const Eigen::VectorXd& upper_bound,
                                                   const Eigen::VectorXd& max_speed,
                                                   const double& cost):
  InformedSampler(start_configuration,stop_configuration,lower_bound,upper_bound,cost),
  max_speed_(max_speed)
{
  logger = logger.get_child("time");

  inv_max_speed_=max_speed_.cwiseInverse();
  utopia_=(stop_configuration_-start_configuration_).cwiseProduct(inv_max_speed_).cwiseAbs().maxCoeff();
  setCost(cost); // it is necessary because InformedSampler could change the cost if less than focii distance
  RCLCPP_DEBUG_STREAM(logger, "max_speed" << max_speed.transpose());
  RCLCPP_DEBUG_STREAM(logger, "inv_max_speed_" << inv_max_speed_.transpose());
  RCLCPP_DEBUG_STREAM(logger, "utopia_" << utopia_);

}

void TimeBasedInformedSampler::setCost(const double &cost)
{
  InformedSampler::setCost(cost*std::sqrt(ndof_)*max_speed_.maxCoeff());
  cost_ = cost;
  inf_cost_ = cost_ >= std::numeric_limits<double>::infinity();

  if (cost_ < utopia_)
  {
    RCLCPP_WARN(logger, "cost is %f, utopia_ is %f", cost_, utopia_);
    cost_ = utopia_;
  }


  specific_volume_=1.0;
  for (unsigned int idx=0;idx<ndof_;idx++)
  {
    if (cost_==inf_cost_)
      specific_volume_*=(upper_bound_(idx)-lower_bound_(idx));
    else
      specific_volume_*=cost_*max_speed_(idx);
  }
}


bool TimeBasedInformedSampler::inBounds(const Eigen::VectorXd& q)
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

  double check_cost=((q-start_configuration_).cwiseProduct(inv_max_speed_)).cwiseAbs().maxCoeff()+
      ((q-stop_configuration_ ).cwiseProduct(inv_max_speed_)).cwiseAbs().maxCoeff();
  return  (check_cost<cost_);

}



Eigen::VectorXd TimeBasedInformedSampler::sample()
{

  if (cost_>=std::numeric_limits<double>::infinity())
    return InformedSampler::sample();

  Eigen::VectorXd q(ndof_);

  unsigned int iteration=0;
  while (true)
  {

    if (iteration++>100000)
    {
      //RCLCPP_ERROR_THROTTLE(logger, 1,"Unable to find a sample in the informed set, cost =%f",cost_);
      RCLCPP_ERROR(logger, "Unable to find a sample in the informed set, cost =%f",cost_);
      return InformedSampler::sample();
    }
    q=ellipse_center_+0.5*cost_*Eigen::MatrixXd::Random(ndof_, 1).cwiseProduct(max_speed_);
    if (inBounds(q))
    {
      return q;
    }

  }

}


}  // namespace pathplan
