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
#include <graph_core/graph/path.h>

namespace pathplan {



class TubeInformedSampler: public InformedSampler
{
protected:
  std::vector<Eigen::VectorXd>  path_;
  std::vector<double> partial_length_;
  std::vector<double> partial_cost_;
  double radius_;
  double length_;
  double local_bias_=0.8;
  SamplerPtr sampler_;
  MetricsPtr metrics_;

  bool couldImprove(const Eigen::VectorXd &q);


public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  TubeInformedSampler(const Eigen::VectorXd& start_configuration,
                      const Eigen::VectorXd& stop_configuration,
                      const SamplerPtr& sampler,
                      const MetricsPtr& metrics):
    InformedSampler(start_configuration,stop_configuration,sampler->getLB(),sampler->getUB(),sampler->cost())
  {
    length_ = 0;
    radius_=0;
    sampler_=sampler;
    metrics_=metrics;
    logger = logger.get_child("tube");
  }

  bool setPath(const pathplan::PathPtr& path);
  bool setPath(const std::vector<Eigen::VectorXd>& path);
  bool setPath(const std::vector<std::vector<double>>& path);
  bool setRadius(const double& radius);
  bool setLocalBias(const double& local_bias);
  Eigen::VectorXd pointOnCurvilinearAbscissa(const double& abscissa);
  virtual Eigen::VectorXd sample();

};


typedef std::shared_ptr<TubeInformedSampler> TubeInformedSamplerPtr;

}    // namespace pathplan
