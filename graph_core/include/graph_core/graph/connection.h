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

#include <graph_core/util.h>
#include <graph_core/graph/node.h>
namespace pathplan
{
class Connection : public std::enable_shared_from_this<Connection>
{
protected:
  NodePtr parent_;
  NodePtr child_;
  double cost_;
  bool added_ = false;
  double euclidean_norm_;
  double time_;
  double likelihood_;

  rclcpp::Logger logger = rclcpp::get_logger("graph/connection");

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Connection(const NodePtr& parent, const NodePtr& child, const double& time=0.0);
  ConnectionPtr pointer()
  {
    return shared_from_this();
  }

  virtual void add();
  virtual void remove();

  virtual bool isNet()
  {
    return false;
  }

  void setCost(const double& cost)
  {
    cost_ = cost;
  }
  const double& getCost()
  {
    return cost_;
  }
  double norm()
  {
    return euclidean_norm_;
  }
  const NodePtr& getParent() const
  {
    return parent_;
  }
  const NodePtr& getChild() const
  {
    return child_;
  }

  void setLikelihood(const double& likelihood){likelihood_=likelihood;}

  virtual ConnectionPtr clone();

  void flip();

  bool isParallel(const ConnectionPtr& conn, const double& toll = 1e-06);

  friend std::ostream& operator<<(std::ostream& os, const Connection& connection);
  ~Connection();
};



std::ostream& operator<<(std::ostream& os, const Connection& connection);

}
