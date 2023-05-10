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
#include <graph_core/collision_checker.h>
#include <graph_core/sampler.h>
#include <graph_core/metrics.h>
#include <graph_core/datastructure/nearest_neighbors.h>
#include <graph_core/datastructure/kdtree.h>
#include <graph_core/datastructure/vector.h>

namespace pathplan
{

class Tree;
typedef std::shared_ptr<Tree> TreePtr;

class Tree: public std::enable_shared_from_this<Tree>
{
protected:
  NodePtr root_;
  bool use_kdtree_;
  double max_distance_=1;
  double tolerance_ = 1e-6;
  double k_rrt_;
  unsigned int maximum_nodes_ = 5000; // legare il massimo numero di punti al volume????
  CollisionCheckerPtr checker_;
  MetricsPtr metrics_;

  NearestNeighborsPtr nodes_;

  rclcpp::Logger logger = rclcpp::get_logger("graph/tree");

  void purgeNodeOutsideEllipsoid(NodePtr& node,
                                 const SamplerPtr& sampler,
                                 const std::vector<NodePtr>& white_list,
                                 unsigned int& removed_nodes);

  void purgeNodeOutsideEllipsoids(NodePtr& node,
                                  const std::vector<SamplerPtr>& samplers,
                                  const std::vector<NodePtr>& white_list,
                                  unsigned int& removed_nodes);

  // add children to the tree. node is not added (throw exception if it is not member of the tree)
  void populateTreeFromNode(const NodePtr& node);
  void populateTreeFromNode(const NodePtr& node, const std::vector<NodePtr>& white_list);

  //add children to the tree if they are inside the ellipsoid
  void populateTreeFromNode(const NodePtr& node, const Eigen::VectorXd& focus1, const Eigen::VectorXd& focus2, const double& cost);
  void populateTreeFromNode(const NodePtr& node, const Eigen::VectorXd& focus1, const Eigen::VectorXd& focus2, const double& cost, const std::vector<NodePtr> &white_list);


public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Tree(const NodePtr& root,
       const double& max_distance,
       const CollisionCheckerPtr& checker,
       const MetricsPtr& metrics,
       const bool& use_kdtree=true);

  const NodePtr& getRoot()
  {
    return root_;
  }
  std::vector<NodePtr> getNodes()
  {
    return nodes_->getNodes();
  }

  bool changeRoot(const NodePtr& node);

  virtual void addNode(const NodePtr& node, const bool& check_if_present = true);
  virtual void removeNode(const NodePtr& node);

  bool tryExtend(const Eigen::VectorXd& configuration,
                 Eigen::VectorXd& next_configuration,
                 NodePtr& closest_node);

  bool tryExtendFromNode(const Eigen::VectorXd& configuration,
                       Eigen::VectorXd& next_configuration,
                       NodePtr& node);

  /* selectNextConfiguration: compute next_configuration as
   * next_configuration = configurarion if configuration is close to the node (less than max_distance_)
   * next_configuration distance is limited to max_distance_ if configuration is far from node
   */
  double selectNextConfiguration(const Eigen::VectorXd& configuration,
                                 Eigen::VectorXd& next_configuration,
                                 const NodePtr &node);

  bool extend(const Eigen::VectorXd& configuration,
              NodePtr& new_node);

  bool extend(const Eigen::VectorXd& configuration,
              NodePtr& new_node,
              ConnectionPtr& connection);

  bool extendOnly(NodePtr &closest_node,
                  NodePtr& new_node,
                  ConnectionPtr& connection);

  bool extendToNode(const NodePtr& node,
                    NodePtr& new_node);

  bool connect(const Eigen::VectorXd& configuration,
               NodePtr& new_node);

  bool informedExtend(const Eigen::VectorXd& configuration,   //Used in AnytimeRRT
                        NodePtr& new_node,
                        const Eigen::VectorXd &goal, const double &cost2beat, const double &bias);

  bool connectToNode(const NodePtr& node,
                     NodePtr& new_node,
                     const double &max_time = std::numeric_limits<double>::infinity());

  bool rewire(const Eigen::VectorXd& configuration,
              double r_rewire);

  bool rewireK(const Eigen::VectorXd& configuration);

  //Useful for replanning:  extend+rewireAndCheckPath -> new sample and rewire considering only nodes with path to them collision-free
  bool rewireWithPathCheck(const Eigen::VectorXd& configuration,
                           std::vector<ConnectionPtr> &checked_connections,
                           double r_rewire,
                           NodePtr& new_node);

  bool rewire(const Eigen::VectorXd& configuration,
              double r_rewire,
              NodePtr& new_node);

  bool rewireToNode(const NodePtr& n,
                    double r_rewire);

  bool rewireToNode(const NodePtr& n,
                    double r_rewire,
                    NodePtr& new_node);

  //if what_rewire is 1 it searches for the best parent for node inside the radius r_rewire, if 2 it verifies if node is a better parent for the other nodes inside r_rewire, if 0 it does both
  bool rewireOnly(NodePtr& node, double r_rewire, const int &what_rewire = 0);
  //Useful for replanning: rewireOnly considering those nodes that have a free path from the root to them
  bool rewireOnlyWithPathCheck(NodePtr& node,std::vector<ConnectionPtr> &checked_connections, double r_rewire, const int& what_rewire = 0);

  bool checkPathToNode(const NodePtr &node, std::vector<ConnectionPtr>& checked_connections);

  NodePtr findClosestNode(const Eigen::VectorXd& configuration);

  double costToNode(NodePtr node);

  std::vector<ConnectionPtr> getConnectionToNode(NodePtr node);  //la &?

  bool keepOnlyThisBranch(const std::vector<ConnectionPtr>& connections);

  bool addBranch(const std::vector<ConnectionPtr>& connections);
  bool addTree(TreePtr& additional_tree, const double &max_time = std::numeric_limits<double>::infinity());
  void cleanTree();
  std::multimap<double, NodePtr> near(const NodePtr& node, const double& r_rewire);
  std::multimap<double, NodePtr> nearK(const NodePtr& node);
  std::multimap<double, NodePtr> nearK(const Eigen::VectorXd& conf);

  bool isInTree(const NodePtr& node);
  unsigned int getNumberOfNodes()const
  {
    return nodes_->size();
  }

  unsigned int purgeNodesOutsideEllipsoid(const SamplerPtr& sampler, const std::vector<NodePtr>& white_list);
  unsigned int purgeNodesOutsideEllipsoids(const std::vector<SamplerPtr>& samplers, const std::vector<NodePtr>& white_list);

  bool purgeFromHere(NodePtr& node);
  bool purgeFromHere(NodePtr& node, const std::vector<NodePtr>& white_list, unsigned int& removed_nodes);
  bool needCleaning(){return nodes_->size()>maximum_nodes_;}

  bool recheckCollision(); //return true if there are no collisions
  bool recheckCollisionFromNode(NodePtr &n); //return true if there are no collisions

  const double& getMaximumDistance() const {return max_distance_;}
  MetricsPtr& getMetrics() {return metrics_;}
  CollisionCheckerPtr& getChecker() {return checker_;}
  bool getUseKdTree(){return use_kdtree_;}

//  XmlRpc::XmlRpcValue toXmlRpcValue() const;
//  void toXmlFile(const std::string& file_name) const;
  friend std::ostream& operator<<(std::ostream& os, const Tree& tree);

//  static TreePtr fromXmlRpcValue(const XmlRpc::XmlRpcValue& x,
//                                 const double& max_distance,
//                                 const CollisionCheckerPtr& checker,
//                                 const MetricsPtr& metrics,
//                                 const bool& lazy=false);
};

std::ostream& operator<<(std::ostream& os, const Tree& tree);


}
