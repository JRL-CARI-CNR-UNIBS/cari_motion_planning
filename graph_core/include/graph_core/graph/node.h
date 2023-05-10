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

#include <eigen3/Eigen/Core>
//#include <ros/ros.h>
#include <rclcpp/rclcpp.hpp>
#include <graph_core/util.h>
#include <graph_core/graph/connection.h>

namespace pathplan
{

class Node: public std::enable_shared_from_this<Node>
{
protected:
  Eigen::VectorXd configuration_;
  unsigned int ndof_;
  bool analyzed_;
  bool non_optimal_;

  rclcpp::Logger logger = rclcpp::get_logger("graph/node");
public:
  std::vector<ConnectionPtr> parent_connections_;
  std::vector<ConnectionPtr> child_connections_;
  std::vector<ConnectionPtr> net_parent_connections_;   //CHIEDI COME USARE NETCONNECTIONPTR
  std::vector<ConnectionPtr> net_child_connections_;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Node(const Eigen::VectorXd& configuration);
  NodePtr pointer()
  {
    return shared_from_this();
  }
  void setAnalyzed(const bool& analyzed);
  bool getAnalyzed();
  void setNonOptimal(const bool& nonOptimal);
  bool getNonOptimal();
  void addParentConnection(const ConnectionPtr& connection);
  void addChildConnection(const ConnectionPtr& connection);
  void addNetParentConnection(const ConnectionPtr& connection);
  void addNetChildConnection(const ConnectionPtr& connection);
  std::vector<NodePtr> getChildren() const;
  std::vector<NodePtr> getParents() const;
  std::vector<NodePtr> getNetParents() const;
  std::vector<NodePtr> getNetChildren() const;

  void disconnect();
  void disconnectChildConnections();
  void disconnectParentConnections();
  void disconnectNetParentConnections();
  void disconnectNetChildConnections();
  void remoteParentConnection(const ConnectionPtr& connection);
  void remoteChildConnection(const ConnectionPtr& connection);
  void remoteNetParentConnection(const ConnectionPtr& connection);
  void remoteNetChildConnection(const ConnectionPtr& connection);

  const Eigen::VectorXd& getConfiguration()
  {
    return configuration_;
  }
  ~Node();


  // XmlRpcValue not supported in ROS2
//  XmlRpc::XmlRpcValue toXmlRpcValue() const;
  friend std::ostream& operator<<(std::ostream& os, const Node& path);

//  static NodePtr fromXmlRpcValue(const XmlRpc::XmlRpcValue& x);
};

std::ostream& operator<<(std::ostream& os, const Node& node);

}
