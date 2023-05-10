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


#include <graph_core/tube_informed_sampler.h>

namespace pathplan
{

Eigen::VectorXd TubeInformedSampler::sample()
{
   if (ud_(gen_)>local_bias_)
    return sampler_->sample();

  if (length_<=0)
    return sampler_->sample();

  for (int itrial = 0; itrial < 100; itrial++)
  {
    double abscissa = ud_(gen_) * length_;
    Eigen::VectorXd center = pointOnCurvilinearAbscissa(abscissa);
    Eigen::VectorXd ball(ndof_);
    ball.setRandom();
    ball *= std::pow(ud_(gen_), 1.0 / (double)ndof_) / ball.norm();
    Eigen::VectorXd q = radius_ * ball + center;
    if (sampler_->inBounds(q) && couldImprove(q))
    {
      return q;
    }
  }
  return sampler_->sample();
}

bool TubeInformedSampler::setPath(const PathPtr &path)
{
  return setPath(path->getWaypoints());
}

bool TubeInformedSampler::setPath(const std::vector<Eigen::VectorXd>&  path)
{
  if (path.size()==0)
    return false;

  path_=path;
  partial_length_.resize(path.size(),0);
  partial_cost_.resize(path.size(),0);
  for (size_t idx=1;idx<path.size();idx++)
  {
    partial_length_.at(idx)=partial_length_.at(idx-1)+(path.at(idx)-path.at(idx-1)).norm();
    partial_cost_.at(idx)=partial_cost_.at(idx-1)+metrics_->utopia(path.at(idx-1),path.at(idx));
  }
  length_=partial_length_.back();
  return length_>0;
}

bool TubeInformedSampler::setPath(const std::vector<std::vector<double>>& path)
{
  std::vector<Eigen::VectorXd> eigen_path(path.size());
  for (size_t idx=0;idx<path.size();idx++)
  {
    eigen_path.at(idx).resize(path.at(idx).size());
    for (unsigned iax=0;iax<path.at(idx).size();iax++)
      eigen_path.at(idx)(iax)=path.at(idx).at(iax);
  }
  return setPath(eigen_path);
}

bool TubeInformedSampler::setRadius(const double &radius)
{
  if (radius<=0)
  {
    RCLCPP_WARN(logger, "Radius should be positive");
    return false;
  }
  radius_=radius;
  return true;
}

bool TubeInformedSampler::setLocalBias(const double& local_bias)
{
  if ((local_bias<0) || (local_bias>1))
  {
    RCLCPP_WARN(logger, "Local bias should be between 0-1");
    return false;
  }
  local_bias_=local_bias;
  return true;
}

Eigen::VectorXd TubeInformedSampler::pointOnCurvilinearAbscissa(const double& abscissa)
{
  if (abscissa <= 0)
    return path_.at(0);
  else if (abscissa>=length_)
    return path_.back();

  for (size_t idx=1;idx<path_.size();idx++)
  {
    if ( partial_length_.at(idx) > abscissa)
    {
      double ratio = (abscissa - partial_length_.at(idx-1)) /(partial_length_.at(idx) - partial_length_.at(idx-1));
      return path_.at(idx-1) + ratio * (path_.at(idx) - path_.at(idx-1));
    }
  }
  return path_.back();
}

bool TubeInformedSampler::couldImprove(const Eigen::VectorXd& q)
{
  for (size_t idx=1;idx<path_.size()-1;idx++)
  {
    double delta_cost=partial_cost_.at(idx+1)-partial_cost_.at(idx-1);
    double test_cost=metrics_->utopia(q,path_.at(idx+1))+metrics_->utopia(path_.at(idx-1),q);
    if (test_cost<delta_cost)
      return true;
  }
  return false;
}

}  // namespace pathplan
