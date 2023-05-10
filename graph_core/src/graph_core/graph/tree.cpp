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

#include <graph_core/graph/tree.h>
#include <fstream>

namespace pathplan
{

Tree::Tree(const NodePtr& root,
           const double &max_distance,
           const CollisionCheckerPtr &checker,
           const MetricsPtr &metrics,
           const bool &use_kdtree):
  root_(root),
  use_kdtree_(use_kdtree),
  max_distance_(max_distance),
  checker_(checker),
  metrics_(metrics)
{

  if (use_kdtree)
  {
    nodes_=std::make_shared<KdTree>();
  }
  else
  {
    nodes_=std::make_shared<Vector>();
  }
  nodes_->insert(root);
  double dimension=root->getConfiguration().size();
  k_rrt_=1.1*std::pow(2.0,dimension+1)*std::exp(1)*(1.0+1.0/dimension);
}

NodePtr Tree::findClosestNode(const Eigen::VectorXd &configuration)
{
  return nodes_->nearestNeighbor(configuration);
}

bool Tree::tryExtend(const Eigen::VectorXd &configuration,
                     Eigen::VectorXd &next_configuration,
                     NodePtr &closest_node)
{
  closest_node = findClosestNode(configuration);
  assert(closest_node);

  return tryExtendFromNode(configuration,next_configuration,closest_node);
}

bool Tree::tryExtendFromNode(const Eigen::VectorXd &configuration,
                             Eigen::VectorXd &next_configuration,
                             NodePtr& node)
{
  assert(node);
  double distance = selectNextConfiguration(configuration,next_configuration,node);

  if (distance < tolerance_)
    return true;
  else
    if (checker_->checkPath(node->getConfiguration(), next_configuration))
      return true;

  return false;
}

double Tree::selectNextConfiguration(const Eigen::VectorXd& configuration,
                                     Eigen::VectorXd& next_configuration,
                                     const NodePtr& node)
{
  assert(node);

  double distance = (node->getConfiguration() - configuration).norm();

  if (distance < tolerance_)
  {
    next_configuration = configuration;
  }
  else if (distance < max_distance_)
  {
    next_configuration = configuration;
  }
  else
  {
    next_configuration = node->getConfiguration() + (configuration - node->getConfiguration()) / distance * max_distance_;
  }

  return distance;
}

bool Tree::extendOnly(NodePtr& closest_node, NodePtr &new_node, ConnectionPtr &connection)
{
  ConnectionPtr conn;
  double cost = metrics_->cost(closest_node, new_node);
  conn = std::make_shared<Connection>(closest_node, new_node);
  conn->add();
  conn->setCost(cost);

  connection = conn;
  addNode(new_node,false);

  return true;
}


bool Tree::extend(const Eigen::VectorXd &configuration, NodePtr &new_node)
{
  ConnectionPtr conn;
  return extend(configuration,new_node,conn);
}

bool Tree::extend(const Eigen::VectorXd &configuration, NodePtr &new_node, ConnectionPtr &connection)
{
  NodePtr closest_node;
  Eigen::VectorXd next_configuration;
  if (!tryExtend(configuration,
                 next_configuration,
                 closest_node))
  {
    connection = NULL;
    return false;
  }

  new_node = std::make_shared<Node>(next_configuration);
  return extendOnly(closest_node,new_node,connection);
}

bool Tree::extendToNode(const NodePtr& node,
                        NodePtr& new_node)
{
  if (isInTree(node))
    return true;

  NodePtr closest_node;
  Eigen::VectorXd next_configuration;
  if (!tryExtend(node->getConfiguration(),
                 next_configuration,
                 closest_node))
  {
    return false;
  }

  bool attached = false;
  if ((next_configuration - node->getConfiguration()).norm() < tolerance_)
  {
    attached = true;
    new_node = node;
    addNode(node);
  }
  else
  {
    new_node = std::make_shared<Node>(next_configuration);
    addNode(new_node,false);
  }

    double cost = metrics_->cost(closest_node, new_node);
    ConnectionPtr conn = std::make_shared<Connection>(closest_node, new_node);
    conn->add();
    conn->setCost(cost);

  return true;
}

bool Tree::connect(const Eigen::VectorXd &configuration, NodePtr &new_node)
{
  bool success = true;
  while (success)
  {
    NodePtr tmp_node;
    success = extend(configuration, tmp_node);
    if(success)
    {
      new_node = tmp_node;
      if ((new_node->getConfiguration() - configuration).norm() < tolerance_)
        return true;
    }
  }
  return false;
}

bool Tree::informedExtend(const Eigen::VectorXd &configuration, NodePtr &new_node,const Eigen::VectorXd &goal, const double& cost2beat, const double& bias)
{
  struct extension
  {
    NodePtr tree_node;
    Eigen::VectorXd new_conf;
    double distance;
  };

  std::multimap<double,NodePtr> closest_nodes_map = nearK(configuration);
  assert(closest_nodes_map.size()>0);

  double heuristic, distance, cost2node;
  Eigen::VectorXd new_configuration;
  std::multimap<double,extension> best_nodes_map;

  for(const std::pair<double,NodePtr>& n: closest_nodes_map)
  {
    distance = selectNextConfiguration(configuration,new_configuration,n.second);
    cost2node = costToNode(n.second);
    heuristic = bias*distance+(1-bias)*cost2node;

    extension ext;
    ext.tree_node = n.second;
    ext.new_conf = new_configuration;
    ext.distance = distance;

    if((cost2node+distance+(goal-new_configuration).norm())<cost2beat)  //if and only if the new conf underestimation of cost is less than the cost to beat it is added to the tree
      best_nodes_map.insert(std::pair<double,extension>(heuristic,ext));  //give priority to parents with the best heuristics
  }

  bool extend_ok = false;
  extension ext;
  for(const std::pair<double,extension>& n: best_nodes_map)
  {
    ext = n.second;
    if (ext.distance < tolerance_)
    {
      extend_ok = true;
      break;
    }
    else
    {
      if (checker_->checkPath(ext.tree_node->getConfiguration(), ext.new_conf))
      {
        extend_ok = true;
        break;
      }
    }
  }

  if(extend_ok)
  {
    ConnectionPtr connection;
    new_node = std::make_shared<Node>(ext.new_conf);
    return extendOnly(ext.tree_node,new_node,connection);
  }
  else
    return false;
}

bool Tree::connectToNode(const NodePtr &node, NodePtr &new_node, const double &max_time)
{
//  ros::WallTime tic = ros::WallTime::now();
  rclcpp::Time tic = rclcpp::Clock().now(); // node needed??

  if(max_time<=0.0) return false;

  bool success = true;
  while (success)
  {
    NodePtr tmp_node;
    RCLCPP_DEBUG(logger, "calling extend");
    success = extendToNode(node, tmp_node);
    if (success)
    {
      new_node = tmp_node;

      if ((new_node->getConfiguration() - node->getConfiguration()).norm() < tolerance_)
        return true;
    }

//    if((ros::WallTime::now()-tic).toSec() >= 0.98*max_time) break;
    if((rclcpp::Clock().now()-tic).seconds() >= 0.98*max_time) break;
  }
  return false;
}

bool Tree::checkPathToNode(const NodePtr& node, std::vector<ConnectionPtr>& checked_connections)
{
  std::vector<ConnectionPtr> connections_to_check;
  std::vector<ConnectionPtr> path_connections = getConnectionToNode(node);

  for(const ConnectionPtr conn: path_connections)
  {
    bool already_checked = false;
    for(const ConnectionPtr checked_conn: checked_connections) //check if the connection considered has not already been verified
    {
      if(checked_conn == conn)
      {
        already_checked = true;
        if(checked_conn->getCost() == std::numeric_limits<double>::infinity()) return false;

        break;
      }
    }

    if(!already_checked) connections_to_check.push_back(conn);
  }

  for(const ConnectionPtr conn: connections_to_check)
  {
    if(!checker_->checkConnection(conn))
    {
      conn->setCost(std::numeric_limits<double>::infinity());
      checked_connections.push_back(conn);

      return false;
    }
    else
    {
      conn->setCost(metrics_->cost(conn->getParent(),conn->getChild()));
      checked_connections.push_back(conn);
    }
  }

  return true;
}

bool Tree::rewireOnly(NodePtr& node, double r_rewire, const int& what_rewire)
{
  if(what_rewire >2 || what_rewire <0)
  {
    RCLCPP_ERROR(logger, "what_rewire parameter should be 0,1 or 2");
    assert(0);
    return false;
  }

  bool rewire_parent;
  bool rewire_children;

  switch(what_rewire)
  {
  case 0:
    rewire_parent   = true ;
    rewire_children = true ;
    break;
  case 1:
    rewire_parent   = true ;
    rewire_children = false;
    break;
  case 2:
    rewire_parent   = false;
    rewire_children = true ;
    break;
  }

  if(node == root_) rewire_parent = false;

  std::multimap<double,NodePtr> near_nodes = near(node, r_rewire);
  double cost_to_node = costToNode(node);
  bool improved = false;

  if(rewire_parent)
  {
    //RCLCPP_DEBUG(logger, "try to find a better parent between %zu nodes", near_nodes.size());
    NodePtr nearest_node = node->getParents().at(0);
    for (const std::pair<double,NodePtr>& p : near_nodes)
    {
      const NodePtr& n = p.second;
      if (n == nearest_node)
        continue;
      if (n == node)
        continue;

      double cost_to_near = costToNode(n);

      if (cost_to_near >= cost_to_node)
        continue;

      double cost_near_to_node = metrics_->cost(n, node);

      if ((cost_to_near + cost_near_to_node) >= cost_to_node)
        continue;

      if (!checker_->checkPath(n->getConfiguration(), node->getConfiguration()))
        continue;

      node->parent_connections_.at(0)->remove();

      ConnectionPtr conn = std::make_shared<Connection>(n, node);
      conn->setCost(cost_near_to_node);
      conn->add();

      //nearest_node = n; PER ME NON CI VA
      cost_to_node = cost_to_near + cost_near_to_node;
      improved = true;
    }
  }

  if(rewire_children)
  {
    //RCLCPP_DEBUG(logger, "try to find a better child between %zu nodes", near_nodes.size());
    for (const std::pair<double,NodePtr>& p : near_nodes)
    {
      const NodePtr& n = p.second;
      if (n == node)
        continue;

      double cost_to_near = costToNode(n);
      if (cost_to_node >= cost_to_near)
        continue;

      double cost_node_to_near = metrics_->cost(node->getConfiguration(), n->getConfiguration());
      if ((cost_to_node + cost_node_to_near) >= cost_to_near)
        continue;

      if (!checker_->checkPath(node->getConfiguration(), n->getConfiguration()))
        continue;

      n->parent_connections_.at(0)->remove();
      ConnectionPtr conn = std::make_shared<Connection>(node, n);
      conn->setCost(cost_node_to_near);
      conn->add();

      improved = true;
    }
  }

  return improved;
}

bool Tree::rewireOnlyWithPathCheck(NodePtr& node, std::vector<ConnectionPtr>& checked_connections, double r_rewire, const int& what_rewire)
{

  if(what_rewire >2 || what_rewire <0)
  {
    RCLCPP_ERROR(logger, "what_rewire parameter should be 0,1 or 2");
    assert(0);
    return false;
  }

  bool rewire_parent;
  bool rewire_children;

  switch(what_rewire)
  {
  case 0:
    rewire_parent   = true ;
    rewire_children = true ;
    break;
  case 1:
    rewire_parent   = true ;
    rewire_children = false;
    break;
  case 2:
    rewire_parent   = false;
    rewire_children = true ;
    break;
  }

  if(node->getParents().size() == 0) rewire_parent = false;
  std::multimap<double,NodePtr> near_nodes = near(node, r_rewire);

  //validate connections to node
  double cost_to_node;
  if(!checkPathToNode(node,checked_connections))
    cost_to_node = std::numeric_limits<double>::infinity();
  else
    cost_to_node= costToNode(node);

  bool improved = false;

  if(rewire_parent)
  {
    //RCLCPP_DEBUG(logger, "try to find a better parent between %zu nodes", near_nodes.size());
    NodePtr nearest_node = node->getParents().at(0);
    for (const std::pair<double,NodePtr>& p : near_nodes)
    {
      const NodePtr& n = p.second;
      if (n == nearest_node)
        continue;
      if (n == node)
        continue;

      double cost_to_near = costToNode(n);

      if (cost_to_near >= cost_to_node)
        continue;

      double cost_near_to_node = metrics_->cost(n, node);

      if ((cost_to_near + cost_near_to_node) >= cost_to_node)
        continue;

      if(!checkPathToNode(n,checked_connections)) //validate connections to n
        continue;

      if (!checker_->checkPath(n->getConfiguration(), node->getConfiguration()))
        continue;

      node->parent_connections_.at(0)->remove();

      ConnectionPtr conn = std::make_shared<Connection>(n, node);
      conn->setCost(cost_near_to_node);
      conn->add();
      checked_connections.push_back(conn);

      //      nearest_node = n; //PER ME NON CI VA
      cost_to_node = cost_to_near + cost_near_to_node;
      improved = true;
    }
  }

  if(cost_to_node == std::numeric_limits<double>::infinity()) rewire_children = false;

  if(rewire_children)
  {
    //RCLCPP_DEBUG(logger, "try to find a better child between %zu nodes", near_nodes.size());
    for (const std::pair<double,NodePtr>& p : near_nodes)
    {
      const NodePtr& n = p.second;
      if (n == node)
        continue;
      if(n->getParents().size() == 0)
      {

        continue;
      }

      double cost_to_near = costToNode(n);
      bool checked = false;

      if (cost_to_node >= cost_to_near)
      {
        if(checkPathToNode(n,checked_connections)) //if path to n is free, cost_to_node >= cost_to_near is really true
          continue;

        checked = true;
      }

      double cost_node_to_near = metrics_->cost(node, n);
      if ((cost_to_node + cost_node_to_near) >= cost_to_near)
      {
        if(checked) continue;
        else
        {
          if(checkPathToNode(n,checked_connections)) //if path to n is free, cost_to_node +cost_node_to_near >= cost_to_near is really true
            continue;
        }
      }

      if (!checker_->checkPath(node->getConfiguration(), n->getConfiguration()))
        continue;

      n->parent_connections_.at(0)->remove();

      ConnectionPtr conn = std::make_shared<Connection>(node, n);
      conn->setCost(cost_node_to_near);
      conn->add();

      checked_connections.push_back(conn);

      improved = true;
    }
  }
  return improved;
}


bool Tree::rewireK(const Eigen::VectorXd &configuration)
{
  NodePtr new_node;
  if (!extend(configuration, new_node))
  {
    return false;
  }
  std::multimap<double,NodePtr> near_nodes = nearK(new_node);
  NodePtr nearest_node = new_node->getParents().at(0);
  double cost_to_new = costToNode(new_node);

  bool improved = false;

  //RCLCPP_DEBUG(logger, "try to find a better parent between %zu nodes", near_nodes.size());
  for (const std::pair<double,NodePtr>& p : near_nodes)
  {
    const NodePtr& node=p.second;

    if (node == nearest_node)
      continue;
    if (node == new_node)
      continue;

    double cost_to_near = costToNode(node);

    if (cost_to_near >= cost_to_new)
      continue;

    double cost_near_to_new = metrics_->cost(node, new_node);

    if ((cost_to_near + cost_near_to_new) >= cost_to_new)
      continue;

    if (!checker_->checkPath(node->getConfiguration(), new_node->getConfiguration()))
      continue;


    new_node->parent_connections_.at(0)->remove();

    ConnectionPtr conn = std::make_shared<Connection>(node, new_node);
    conn->setCost(cost_near_to_new);
    conn->add();
    nearest_node = node;
    cost_to_new = cost_to_near + cost_near_to_new;
    improved = true;
  }

  //RCLCPP_DEBUG(logger, "try to find a better child between %zu nodes", near_nodes.size());
  for (const std::pair<double,NodePtr>& p : near_nodes)
  {
    const NodePtr& n=p.second;
    if (n == new_node)
      continue;

    double cost_to_near = costToNode(n);
    if (cost_to_new >= cost_to_near)
      continue;

    double cost_new_to_near = metrics_->cost(new_node->getConfiguration(), n->getConfiguration());
    if ((cost_to_new + cost_new_to_near) >= cost_to_near)
      continue;

    if (!checker_->checkPath(new_node->getConfiguration(), n->getConfiguration()))
      continue;

    n->parent_connections_.at(0)->remove();
    ConnectionPtr conn = std::make_shared<Connection>(new_node, n);
    conn->setCost(cost_new_to_near);
    conn->add();

    improved = true;
  }

  return improved;
}

bool Tree::rewireWithPathCheck(const Eigen::VectorXd &configuration, std::vector<ConnectionPtr> &checked_connections, double r_rewire, NodePtr& new_node)
{
  ConnectionPtr new_conn;
  if (!extend(configuration,new_node,new_conn))
  {
    return false;
  }

  checked_connections.push_back(new_conn);

  return rewireOnlyWithPathCheck(new_node,checked_connections,r_rewire);
}

bool Tree::rewire(const Eigen::VectorXd &configuration, double r_rewire, NodePtr& new_node)
{
  if (!extend(configuration, new_node))
  {
    return false;
  }

  return rewireOnly(new_node,r_rewire);
}

bool Tree::rewire(const Eigen::VectorXd &configuration, double r_rewire)
{
  NodePtr new_node;

  return rewire(configuration,r_rewire,new_node);
}


bool Tree::rewireToNode(const NodePtr& n, double r_rewire, NodePtr& new_node)
{
  if (!extendToNode(n, new_node))
  {
    return false;
  }

  return rewireOnly(new_node,r_rewire);
}

bool Tree::rewireToNode(const NodePtr& n, double r_rewire)
{
  NodePtr new_node;

  return rewireToNode(n,r_rewire,new_node);
}


std::multimap<double,NodePtr> Tree::near(const NodePtr &node, const double &r_rewire)
{
  return nodes_->near(node->getConfiguration(),r_rewire);
}

std::multimap<double,NodePtr> Tree::nearK(const NodePtr &node)
{
  return nearK(node->getConfiguration());
}

std::multimap<double,NodePtr> Tree::nearK(const Eigen::VectorXd &conf)
{
  size_t k=std::ceil(k_rrt_*std::log(nodes_->size()+1));
  return nodes_->kNearestNeighbors(conf,k);
}

double Tree::costToNode(NodePtr node)
{
  double cost = 0;
    while (node != root_)
    {
      if (node->parent_connections_.size() != 1)
      {
        RCLCPP_ERROR_STREAM(logger, "a tree node should have exactly a parent. this node has 0: "<<*node);
        return std::numeric_limits<double>::infinity();
      }

      if (node->parent_connections_.at(0)->getParent() == node)
      {
        RCLCPP_FATAL_STREAM(logger, "node "<< node.get() <<"=\n" << *node);
        RCLCPP_FATAL_STREAM(logger, "to parent\n" << * (node->parent_connections_.at(0)));
        RCLCPP_FATAL(logger, "connection between the same node");
        assert(0);
      }
      cost += node->parent_connections_.at(0)->getCost();
      node = node->parent_connections_.at(0)->getParent();

    }
  return cost;
}

std::vector<ConnectionPtr> Tree::getConnectionToNode(NodePtr node)
{
  std::vector<ConnectionPtr> connections;

    while (node != root_)
    {
      if (node->parent_connections_.size() != 1)
      {
        RCLCPP_ERROR(logger, "a tree node should have only a parent");
        RCLCPP_ERROR_STREAM(logger, "node \n" << *node);

        RCLCPP_INFO_STREAM(logger, "current root "<<root_);
        RCLCPP_INFO_STREAM(logger, "node "<<node);

        assert(0);
      }
      connections.push_back(node->parent_connections_.at(0));
      node = node->parent_connections_.at(0)->getParent();

    }
    std::reverse(connections.begin(), connections.end());

  return connections;
}

void Tree::addNode(const NodePtr& node, const bool& check_if_present)
{
  if (!check_if_present || !isInTree(node))
    nodes_->insert(node);
}

void Tree::removeNode(const NodePtr& node)
{
  node->disconnect();
  nodes_->deleteNode(node);
}

bool Tree::keepOnlyThisBranch(const std::vector<ConnectionPtr>& connections)
{
  if (connections.size() == 0)
    return false;

  std::vector<NodePtr> branch_nodes;
  branch_nodes.push_back(connections.at(0)->getParent());
  for (const ConnectionPtr& conn : connections)
    branch_nodes.push_back(conn->getChild());

  nodes_->disconnectNodes(branch_nodes);
  NearestNeighborsPtr nodes;
  for (NodePtr& n : branch_nodes)
    nodes->insert(n);
  nodes_=nodes;

  return true;
}

bool Tree::addBranch(const std::vector<ConnectionPtr> &connections)
{
  if (connections.size() == 0)
    return false;

  NodePtr start_node = connections.at(0)->getParent();
  if (!isInTree(start_node))
  {
    RCLCPP_ERROR(logger, "start node of the branch is not part of the tree");
    return false;
  }

  std::vector<NodePtr> branch_nodes;
  for (const ConnectionPtr& conn : connections)
    branch_nodes.push_back(conn->getChild());

  for (NodePtr& n : branch_nodes)
  {
    if (not nodes_->findNode(n))
      nodes_->insert(n);
  }
  return true;
}

bool Tree::addTree(TreePtr &additional_tree, const double &max_time)
{
  if (not isInTree(additional_tree->getRoot()))
  {
    NodePtr new_node;
    if (not connectToNode(additional_tree->getRoot(),new_node,max_time))
      return false;
  }


  std::vector<NodePtr> additional_nodes=additional_tree->getNodes();
  for (size_t inode=1;inode<additional_nodes.size();inode++)
  {
    addNode(additional_nodes.at(inode),false);
  }
  return true;
}


bool Tree::isInTree(const NodePtr &node)
{
  return nodes_->findNode(node);
}

unsigned int Tree::purgeNodesOutsideEllipsoid(const SamplerPtr& sampler, const std::vector<NodePtr>& white_list)
{
  if (nodes_->size() < maximum_nodes_)
    return 0;
  unsigned int removed_nodes = 0;

  purgeNodeOutsideEllipsoid(root_,sampler,white_list,removed_nodes);
  return removed_nodes;
}

unsigned int Tree::purgeNodesOutsideEllipsoids(const std::vector<SamplerPtr>& samplers, const std::vector<NodePtr>& white_list)
{
  if (nodes_->size() < maximum_nodes_)
    return 0;
  unsigned int removed_nodes = 0;

  purgeNodeOutsideEllipsoids(root_,samplers,white_list,removed_nodes);
  return removed_nodes;
}

void Tree::purgeNodeOutsideEllipsoid(NodePtr& node,
                                     const SamplerPtr& sampler,
                                     const std::vector<NodePtr>& white_list,
                                     unsigned int& removed_nodes)
{
  assert(node);

  // check if it is in the admissible informed set
  if (sampler->inBounds(node->getConfiguration()))
  {
    // if node is inside the admissible set, check its successors
    std::vector<NodePtr> successors;
    successors = node->getChildren();

    for (NodePtr& n : successors)
    {
      assert(n.get()!=node.get());
      purgeNodeOutsideEllipsoid(n,sampler,white_list,removed_nodes);
    }
  }
  else
  {
    // if node is outside the admissible set, remove it and its successors if they are not in the white list.
    if (std::find(white_list.begin(), white_list.end(), node) != white_list.end())
      return;
    purgeFromHere(node, white_list, removed_nodes);
  }
  return;
}


void Tree::purgeNodeOutsideEllipsoids(NodePtr& node,
                                      const std::vector<SamplerPtr>& samplers,
                                      const std::vector<NodePtr>& white_list,
                                      unsigned int& removed_nodes)
{

  if (nodes_->size() < 0.5*maximum_nodes_)
    return;
  assert(node);

  // check if it belongs to a admissible informed set or the white list
  bool inbound=std::find(white_list.begin(), white_list.end(), node) != white_list.end();
  for (const SamplerPtr& sampler: samplers)
    inbound = inbound || sampler->inBounds(node->getConfiguration());

  if (inbound)
  {
    // if node is inside the admissible set, check its successors
    std::vector<NodePtr> successors;
    successors = node->getChildren();

    for (NodePtr& n : successors)
    {
      assert(n.get()!=node.get());
      purgeNodeOutsideEllipsoids(n,samplers,white_list,removed_nodes);
    }
  }
  else
  {
    purgeFromHere(node, white_list, removed_nodes);
  }
  return;
}

bool Tree::purgeFromHere(NodePtr& node)
{
  std::vector<NodePtr> white_list;
  unsigned int removed_nodes;

  return purgeFromHere(node,white_list,removed_nodes);
}

bool Tree::purgeFromHere(NodePtr& node, const std::vector<NodePtr>& white_list, unsigned int& removed_nodes)
{
  if (std::find(white_list.begin(), white_list.end(), node) != white_list.end())
  {
    return false;
  }
  assert(node);
  std::vector<NodePtr> successors;

  successors = node->getChildren();


  for (NodePtr& n : successors)
  {
    assert(n.get()!=node.get());
    if (!purgeFromHere(n,white_list,removed_nodes))
      return false;
  }

  assert(node);
  node->disconnect();
  if (nodes_->deleteNode(node))
  {
    removed_nodes++;
  }
  return true;
}

void Tree::cleanTree()
{
  std::vector<NodePtr> successors;
  std::vector<NodePtr> white_list;
  unsigned int removed_nodes;
  successors=root_->getChildren();
  for (NodePtr& n: successors)
  {
    if (isInTree(n))
      purgeFromHere(n,white_list,removed_nodes);
  }
}

void Tree::populateTreeFromNode(const NodePtr& node)
{
  Eigen::VectorXd focus1 = node->getConfiguration();
  Eigen::VectorXd focus2 = node->getConfiguration();
  populateTreeFromNode(node, focus1, focus2, std::numeric_limits<double>::infinity());
}

void Tree::populateTreeFromNode(const NodePtr& node, const std::vector<NodePtr>& white_list)
{
  Eigen::VectorXd focus1 = node->getConfiguration();
  Eigen::VectorXd focus2 = node->getConfiguration();
  populateTreeFromNode(node, focus1, focus2, std::numeric_limits<double>::infinity(),white_list);
}

void Tree::populateTreeFromNode(const NodePtr& node, const Eigen::VectorXd& focus1, const Eigen::VectorXd& focus2, const double& cost)
{
  std::vector<NodePtr> white_list;
  populateTreeFromNode(node, focus1, focus2, std::numeric_limits<double>::infinity(), white_list);
}


void Tree::populateTreeFromNode(const NodePtr& node, const Eigen::VectorXd& focus1, const Eigen::VectorXd& focus2, const double& cost, const std::vector<NodePtr> &white_list)
{
  if (not nodes_->findNode(node))
  {
    throw std::invalid_argument("node is not member of tree");
  }

    for (const NodePtr& n: node->getChildren())
    {
      std::vector<NodePtr>::const_iterator it = std::find(white_list.begin(), white_list.end(), n);
      if(it != white_list.end())
        continue;

      else
      {
        if(((n->getConfiguration() - focus1).norm() + (n->getConfiguration() - focus2).norm()) < cost)
        {
          nodes_->insert(n);
          populateTreeFromNode(n,focus1,focus2,cost,white_list);
        }
      }
    }

}

//XmlRpc::XmlRpcValue Tree::toXmlRpcValue() const
//{
//  XmlRpc::XmlRpcValue tree;
//  XmlRpc::XmlRpcValue nodes;
//  XmlRpc::XmlRpcValue connections;
//  int iconn=0;

//  std::vector<NodePtr> nodes_vector=nodes_->getNodes();
//  for (size_t inode=0;inode<nodes_vector.size();inode++)
//  {
//    const NodePtr& n=nodes_vector.at(inode);
//    nodes[inode]=n->toXmlRpcValue();
//    std::vector<NodePtr> dest;
//      dest=n->getChildren();

//    for (size_t idest=0;idest<dest.size();idest++)
//    {
//      for (size_t in2=0;in2<nodes_vector.size();in2++)
//      {
//        if (nodes_vector.at(in2)==dest.at(idest))
//        {
//          XmlRpc::XmlRpcValue connection;
//          connection[0]=(int)inode;
//          connection[1]=(int)in2;
//          connections[iconn++]=connection;
//          break;
//        }
//      }
//    }
//  }
//  tree["nodes"]=nodes;
//  tree["connections"]=connections;
//  return tree;
//}


//void Tree::toXmlFile(const std::string& file_name) const
//{
//  std::string xml=toXmlRpcValue().toXml();
//  std::ofstream out(file_name);
//  out << xml;
//  out.close();
//}

std::ostream& operator<<(std::ostream& os, const Tree& tree)
{
  os << "number of nodes = " << tree.nodes_->size() << std::endl;
  os << "root = " << *tree.root_;
  return os;
}

//TreePtr Tree::fromXmlRpcValue(const XmlRpc::XmlRpcValue& x,
//                              const double& max_distance,
//                              const CollisionCheckerPtr& checker,
//                              const MetricsPtr& metrics,
//                              const bool &lazy)
//{
//  if (not x.hasMember("nodes"))
//  {
//    RCLCPP_ERROR(logger, "loading from XmlRpcValue a tree without 'nodes' field");
//    return NULL;
//  }
//  if (not x.hasMember("connections"))
//  {
//    RCLCPP_ERROR(logger, "loading from XmlRpcValue a tree without 'connections' field");
//    return NULL;
//  }

//  XmlRpc::XmlRpcValue nodes=x["nodes"];
//  XmlRpc::XmlRpcValue connections=x["connections"];
//  if (nodes.getType()!= XmlRpc::XmlRpcValue::Type::TypeArray)
//  {
//    RCLCPP_ERROR(logger, "loading from XmlRpcValue a tree where 'nodes' is not an array");
//    return NULL;
//  }
//  if (connections.getType()!= XmlRpc::XmlRpcValue::Type::TypeArray)
//  {
//    RCLCPP_ERROR(logger, "loading from XmlRpcValue a tree where 'connections' is not an array");
//    return NULL;
//  }
//  NodePtr root=Node::fromXmlRpcValue(nodes[0]);
//  if (not lazy)
//  {
//    if (not checker->check(root->getConfiguration()))
//    {
//      RCLCPP_DEBUG(logger, "root is in collision");
//      return NULL;
//    }
//  }
//  assert(root);


//  std::vector<NodePtr> nodes_vector(nodes.size());
//  nodes_vector.at(0)=root;
//  for (int inode=1;inode<nodes.size();inode++)
//  {
//    nodes_vector.at(inode)=Node::fromXmlRpcValue(nodes[inode]);
//  }

//  for (int iconn=0;iconn<connections.size();iconn++)
//  {
//    int in1=connections[iconn][0];
//    int in2=connections[iconn][1];

//    NodePtr& n1=nodes_vector.at(in1);
//    NodePtr& n2=nodes_vector.at(in2);
//    ConnectionPtr conn;
//    conn=std::make_shared<Connection>(n1,n2);
//    conn->setCost(metrics->cost(n1,n2));

//    conn->add();
//  }
//  pathplan::TreePtr tree=std::make_shared<Tree>(root,max_distance,checker,metrics);
//  for (int inode=1;inode<nodes.size();inode++)
//  {
//    tree->addNode(nodes_vector.at(inode),false);
//  }

//  if (not lazy)
//  {
//    tree->recheckCollision();
//  }
//  return tree;
//}


bool Tree::changeRoot(const NodePtr& node)
{
  if (not isInTree(node))
    return false;

  std::vector<ConnectionPtr> connections = getConnectionToNode(node);
  for (ConnectionPtr& conn: connections)
  {
    conn->flip();
  }

  root_=node;

  return true;
}

bool Tree::recheckCollision()
{
  return recheckCollisionFromNode(root_);
}
bool Tree::recheckCollisionFromNode(NodePtr& n)
{
  std::vector<NodePtr> white_list;
  unsigned int removed_nodes;
  for (ConnectionPtr conn: n->child_connections_)
  {
    NodePtr n=conn->getChild();
    if (not checker_->checkConnection(conn))
    {
      purgeFromHere(n,white_list,removed_nodes);
      return false;
    }
    if (not recheckCollisionFromNode(n))
      return false;
  }
  return true;
}

}  // end namespace pathplan
