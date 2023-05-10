#pragma once
/*
Copyright (c) 2019, Manuel Beschi CNR-STIIMA manuel.beschi@stiima.cnr.it
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions aprotectednd the following disclaimer.
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
#include <graph_core/metrics.h>
#include <graph_core/collision_checker.h>
#include <graph_core/graph/tree.h>
#include <math.h>


namespace pathplan
{

class Path;
typedef std::shared_ptr<Path> PathPtr;
class Path: public std::enable_shared_from_this<Path>
{
protected:
  std::vector<ConnectionPtr> connections_;
  MetricsPtr metrics_;
  CollisionCheckerPtr checker_;
  double cost_;
  double min_length_ = 0.04;
  TreePtr tree_;

  std::vector<bool> change_warp_;

  rclcpp::Logger logger = rclcpp::get_logger("graph/path");

  void computeCost();
  void setChanged(const unsigned int& connection_idx);
  bool bisection(const unsigned int& connection_idx,
                 const Eigen::VectorXd& center,
                 const Eigen::VectorXd& direction,
                 double max_distance,
                 double min_distance);

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Path(std::vector<ConnectionPtr> connections, const MetricsPtr& metrics, const CollisionCheckerPtr& checker);
  Path(std::vector<NodePtr> nodes, const MetricsPtr& metrics, const CollisionCheckerPtr& checker);
  const double& cost()
  {
    computeCost();
    return cost_;
  }

  PathPtr pointer()
  {
    return shared_from_this();
  }

  //It creates a node corresponding to the configuration and creates the correct connections inside the current_path_
  NodePtr addNodeAtCurrentConfig(const Eigen::VectorXd& configuration, ConnectionPtr &conn, const bool &rewire);
  NodePtr addNodeAtCurrentConfig(const Eigen::VectorXd& configuration, const bool& rewire);

  //Remove unnecessary nodes
  bool removeNodes(const std::vector<NodePtr>& white_list, std::vector<NodePtr>& deleted_nodes);
  bool removeNodes(const std::vector<NodePtr> &white_list);
  bool removeNodes();

  //It gives the connection to which the configuration belongs
  ConnectionPtr findConnection(const Eigen::VectorXd& configuration, int& idx);
  ConnectionPtr findConnection(const Eigen::VectorXd& configuration);


  NodePtr findCloserNode(const Eigen::VectorXd& configuration, double &dist);
  NodePtr findCloserNode(const Eigen::VectorXd& configuration);
  NodePtr findCloserNode(const NodePtr& node);
  NodePtr findCloserNode(const NodePtr& node, double &dist);
  PathPtr getSubpathFromConf(const Eigen::VectorXd& conf, const bool get_copy);
  PathPtr getSubpathToConf(const Eigen::VectorXd& conf, const bool get_copy);
  PathPtr getSubpathFromNode(const NodePtr& node);
  PathPtr getSubpathToNode(const NodePtr& node);
  PathPtr getSubpathFromNode(const Eigen::VectorXd& conf);
  PathPtr getSubpathToNode(const Eigen::VectorXd& conf);
  bool resample(const double& distance);
  double computeEuclideanNorm();
  Eigen::VectorXd pointOnCurvilinearAbscissa(const double& abscissa);
  double curvilinearAbscissaOfPoint(const Eigen::VectorXd& conf);
  double curvilinearAbscissaOfPoint(const Eigen::VectorXd& conf, int& idx);
  double curvilinearAbscissaOfPointGivenConnection(const Eigen::VectorXd& conf,const int& conn_idx);
  double getCostFromConf(const Eigen::VectorXd& conf);
  double getNormFromConf(const Eigen::VectorXd& conf);


  std::vector<NodePtr> getNodes();
  std::vector<Eigen::VectorXd> getWaypoints();


  std::vector<bool> getChangeWarp()
  {
    return change_warp_;
  }

  TreePtr getTree()
  {
    return tree_;
  }

  void setChecker(const CollisionCheckerPtr &checker)
  {
    checker_ = checker;
  }

  void setChangeWarp(const std::vector<bool>& change_warp)
  {
    change_warp_ = change_warp;
  }

  void setTree(const TreePtr& tree)
  {
    tree_ = tree;
  }

  std::vector<ConnectionPtr> getConnections()const
  {
    return connections_;
  }

  const std::vector<ConnectionPtr>& getConnectionsConst()const
  {
    return connections_;
  }

  void setConnections(const std::vector<ConnectionPtr>& conn)
  {
    change_warp_.clear();
    cost_ = 0;

    for(const ConnectionPtr& connection : conn)
    {
      cost_ += connection->getCost();
      change_warp_.push_back(true);
    }
    change_warp_.at(0) = false;

    connections_ = conn;
  }

  PathPtr clone();
  bool simplify(const double& distance = 0.02);
  bool isValid(const CollisionCheckerPtr &this_checker = NULL);
  bool isValidFromConf(const Eigen::VectorXd &conf, const CollisionCheckerPtr &this_checker = NULL);
  bool isValidFromConf(const Eigen::VectorXd &conf, int &pos_closest_obs_from_goal, const CollisionCheckerPtr &this_checker = NULL);
  bool isValidFromConn(const ConnectionPtr &this_conn, const CollisionCheckerPtr &this_checker = NULL);
  Eigen::VectorXd projectOnConnection(const Eigen::VectorXd& point, const ConnectionPtr &conn, double& distance, bool &in_conn);
  const Eigen::VectorXd projectOnClosestConnection(const Eigen::VectorXd& point);
  const Eigen::VectorXd projectOnClosestConnectionKeepingPastPrj(const Eigen::VectorXd& point, const Eigen::VectorXd &past_prj, int &n_conn);
  const Eigen::VectorXd projectOnClosestConnectionKeepingCurvilinearAbscissa(const Eigen::VectorXd& point, Eigen::VectorXd& past_prj, double &new_abscissa,  double &past_abscissa, int &n_conn, const bool &verbose = false);


  // return true if improve
  bool warp(const double& min_dist = 0.1, const double& max_time = std::numeric_limits<double>::infinity());

  void flip();

//  XmlRpc::XmlRpcValue toXmlRpcValue(bool reverse=false) const;
  friend std::ostream& operator<<(std::ostream& os, const Path& path);
};

std::ostream& operator<<(std::ostream& os, const Path& path);
}
