/*
Copyright (c) 2020, JRL-CARI CNR-STIIMA/UNIBS
Manuel Beschi manuel.beschi@unibs.it
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

#pragma once
#include <graph_core/solvers/multigoal.h>
#include <graph_core/time_sampler.h>

namespace pathplan
{



class TimeMultigoalSolver: public MultigoalSolver
{
protected:
  TimeBasedInformedSamplerPtr time_based_sampler_;

public:

  TimeMultigoalSolver(const MetricsPtr& metrics,
             const CollisionCheckerPtr& checker,
             const SamplerPtr& sampler,
             const Eigen::VectorXd& max_speed):
    MultigoalSolver(metrics, checker, sampler)
  {
    time_based_sampler_ =  std::make_shared<TimeBasedInformedSampler>(sampler->getStartConf(),
                                                                      sampler->getStopConf(),
                                                                      sampler->getLB(),
                                                                      sampler->getUB(),
                                                                      max_speed,
                                                                      sampler->getCost());
  }

  bool addGoal(const NodePtr &goal_node, const double &max_time = std::numeric_limits<double>::infinity()) override;
};

typedef std::shared_ptr<TimeMultigoalSolver> TimeMultigoalSolverPtr;


}  // namespace pathplan
