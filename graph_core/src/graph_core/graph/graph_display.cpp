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

#include <graph_core/graph/graph_display.h>



namespace pathplan
{

Display::Display(const planning_scene::PlanningSceneConstPtr planning_scene,
                 const std::string& group_name,
                 const std::string& last_link):
  planning_scene_(planning_scene),
  group_name_(group_name),
  last_link_(last_link)
{
  if (last_link.empty())
    last_link_=planning_scene->getRobotModel()->getJointModelGroup(group_name)->getLinkModelNames().back();



  node_marker_scale_.resize(3,DEFAULT_NODE_SIZE);
  connection_marker_scale_.resize(3,DEFAULT_CONNECTION_SIZE);
  tree_marker_scale_.resize(3,DEFAULT_TREE_SIZE);
  marker_id_=0;
  state_=std::make_shared<moveit::core::RobotState>(planning_scene_->getCurrentState());
  jmg_ = state_->getJointModelGroup(group_name_);
  joint_names_=jmg_->getActiveJointModelNames();
  joint_models_=jmg_->getActiveJointModels();
//  marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/marker_visualization_topic", 1000,true);
  marker_pub_ = nh_->create_publisher<visualization_msgs::msg::Marker>("/marker_visualization_topic", 1000);
  for (int idx=0;idx<4;idx++)
    clearMarkers();
}

void Display::clearMarkers(const std::string& ns)
{
  visualization_msgs::msg::Marker marker;
  marker.action = visualization_msgs::msg::Marker::DELETEALL;
//  marker.header.stamp=rclcpp::Clock().now();
  marker.header.stamp=rclcpp::Clock().now();

  marker.ns = ns;
  marker.id= marker_id_++;

  marker_pub_->publish(marker);
  rclcpp::Clock().sleep_for(rclcpp::Duration::from_seconds(DISPLAY_TIME));
}
void Display::clearMarker(const int& id,const std::string& ns)
{
  visualization_msgs::msg::Marker marker;
  marker.action = visualization_msgs::msg::Marker::DELETE;
  marker.header.stamp=rclcpp::Clock().now(); //rclcpp::Clock().now();

  marker.ns = ns;
  marker.id= id;

  marker_pub_->publish(marker);
//  rclcpp::Clock().sleep_for(rclcpp::Duration::from_seconds(DISPLAY_TIME));
  rclcpp::Clock().sleep_for(rclcpp::Duration::from_seconds(DISPLAY_TIME));
}

int Display::displayNode(const NodePtr &n,
                         const std::string& ns,
                         const std::vector<double> &marker_color,
                         const bool &plot_state)
{
  int static_id = marker_id_++;

  return displayNode(n,static_id,ns,marker_color,plot_state);
}

int Display::displayNode(const NodePtr &n,
                         const int &static_id,
                         const std::string& ns,
                         const std::vector<double> &marker_color,
                         const bool &plot_state)
{

  visualization_msgs::msg::Marker marker;
  marker.type = visualization_msgs::msg::Marker::SPHERE;

  marker.ns = ns;

  for (size_t ij=0;ij<joint_names_.size();ij++)
    state_->setJointPositions(joint_models_.at(ij),&n->getConfiguration()(ij));
  marker.pose = tf2::toMsg(state_->getGlobalLinkTransform(last_link_));
  //tf::poseEigenToMsg(state_->getGlobalLinkTransform(last_link_),marker.pose);


  marker.header.frame_id=planning_scene_->getRobotModel()->getRootLink()->getName();
  marker.header.stamp=rclcpp::Clock().now();
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.id= static_id;

  marker.scale.x = node_marker_scale_.at(0);
  marker.scale.y = node_marker_scale_.at(1);
  marker.scale.z = node_marker_scale_.at(2);

  marker.color.r = marker_color.at(0);
  marker.color.g = marker_color.at(1);
  marker.color.b = marker_color.at(2);
  marker.color.a = marker_color.at(3);

  marker_pub_->publish(marker);
  rclcpp::Clock().sleep_for(rclcpp::Duration::from_seconds(DISPLAY_TIME));
  return marker.id;
}

int Display::displayConnection(const ConnectionPtr& conn,
                               const std::string& ns,
                               const std::vector<double>& marker_color,
                               const bool& plot_state)
{
  int static_id = marker_id_++;

  return displayConnection(conn,static_id,ns,marker_color,plot_state);
}

int Display::displayConnection(const ConnectionPtr& conn,
                               const int &static_id,
                               const std::string& ns,
                               const std::vector<double>& marker_color,
                               const bool& plot_state)
{
  visualization_msgs::msg::Marker marker;
  marker.ns = ns;
  marker.type = visualization_msgs::msg::Marker::LINE_LIST;
  marker.header.frame_id="world";
  marker.header.stamp=rclcpp::Clock().now();
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.id= static_id;

  marker.scale.x = connection_marker_scale_.at(0);
  marker.scale.y = connection_marker_scale_.at(1);
  marker.scale.z = connection_marker_scale_.at(2);

  marker.color.r = marker_color.at(0);
  marker.color.g = marker_color.at(1);
  marker.color.b = marker_color.at(2);
  marker.color.a = marker_color.at(3);

  Eigen::VectorXd parent = conn->getParent()->getConfiguration();
  Eigen::VectorXd child = conn->getChild()->getConfiguration();

  double length = (parent-child).norm();
  double step = length/SUBDIVISION_FACTOR;

  Eigen::VectorXd v = (child-parent)/length;
  Eigen::VectorXd conf1 = parent;
  Eigen::VectorXd conf2;

  for(unsigned int i=1;i<=SUBDIVISION_FACTOR; i++)
  {
    conf2 = parent + i*step*v;

    geometry_msgs::msg::Pose pose;
    //state_->setJointGroupPositions(group_name_,conf1);
    for (size_t ij=0;ij<joint_names_.size();ij++)
      state_->setJointPositions(joint_models_.at(ij),&conf1(ij));
    pose = tf2::toMsg(state_->getGlobalLinkTransform(last_link_));
    //tf::poseEigenToMsg(state_->getGlobalLinkTransform(last_link_),pose);
    marker.points.push_back(pose.position);

    //state_->setJointGroupPositions(group_name_,conf2);
    for (size_t ij=0;ij<joint_names_.size();ij++)
      state_->setJointPositions(joint_models_.at(ij),&conf2(ij));
    pose = tf2::toMsg(state_->getGlobalLinkTransform(last_link_));
    //tf::poseEigenToMsg(state_->getGlobalLinkTransform(last_link_),pose);
    marker.points.push_back(pose.position);

    conf1 = conf2;
  }

  marker_pub_->publish(marker);
  rclcpp::Clock().sleep_for(rclcpp::Duration::from_seconds(DISPLAY_TIME));
  return marker.id;
}

int Display::displayPath(const PathPtr &path,
                         const std::string& ns,
                         const std::vector<double> &marker_color,
                         const bool &plot_state)
{
  int static_id = marker_id_++;

  return displayPath(path,static_id,ns,marker_color,plot_state);
}

int Display::displayPath(const PathPtr &path,
                         const int &static_id,
                         const std::string& ns,
                         const std::vector<double> &marker_color,
                         const bool &plot_state)
{
  visualization_msgs::msg::Marker marker;
  marker.ns = ns;
  marker.type = visualization_msgs::msg::Marker::LINE_LIST;
  marker.header.frame_id="world";
  marker.header.stamp=rclcpp::Clock().now();
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.id= static_id;

  marker.scale.x = connection_marker_scale_.at(0);
  marker.scale.y = connection_marker_scale_.at(1);
  marker.scale.z = connection_marker_scale_.at(2);

  marker.color.r = marker_color.at(0);
  marker.color.g = marker_color.at(1);
  marker.color.b = marker_color.at(2);
  marker.color.a = marker_color.at(3);

  std::vector<ConnectionPtr> connections=path->getConnections();
  if (connections.size()==0)
    return marker.id;
  for (const ConnectionPtr& conn: connections)
  {
    Eigen::VectorXd parent = conn->getParent()->getConfiguration();
    Eigen::VectorXd child = conn->getChild()->getConfiguration();

    double length = (parent-child).norm();
    double step = length/SUBDIVISION_FACTOR;

    Eigen::VectorXd v = (child-parent)/length;
    Eigen::VectorXd conf1 = parent;
    Eigen::VectorXd conf2;

    for(unsigned int i=1;i<=SUBDIVISION_FACTOR; i++)
    {
      conf2 = parent + i*step*v;

      geometry_msgs::msg::Pose pose;
      //state_->setJointGroupPositions(group_name_,conf1);
      for (size_t ij=0;ij<joint_names_.size();ij++)
        state_->setJointPositions(joint_models_.at(ij),&conf1(ij));
      pose = tf2::toMsg(state_->getGlobalLinkTransform(last_link_));
//      tf::poseEigenToMsg(state_->getGlobalLinkTransform(last_link_),pose);
      marker.points.push_back(pose.position);

      //state_->setJointGroupPositions(group_name_,conf2);
      for (size_t ij=0;ij<joint_names_.size();ij++)
        state_->setJointPositions(joint_models_.at(ij),&conf2(ij));
      pose = tf2::toMsg(state_->getGlobalLinkTransform(last_link_));
      //tf::poseEigenToMsg(state_->getGlobalLinkTransform(last_link_),pose);
      marker.points.push_back(pose.position);

      conf1 = conf2;
    }

  }

  marker_pub_->publish(marker);
  rclcpp::Clock().sleep_for(rclcpp::Duration::from_seconds(DISPLAY_TIME));
  return marker.id;
}

std::vector<int> Display::displayPathAndWaypoints(const PathPtr &path,
                                                  const std::string& ns,
                                                  const std::vector<double> &marker_color,
                                                  const bool &plot_state)
{
  int static_id_path = marker_id_++;
  int static_id_wp = marker_id_++;

  return displayPathAndWaypoints(path,static_id_path,static_id_wp,ns,marker_color,plot_state);
}
std::vector<int> Display::displayPathAndWaypoints(const PathPtr &path,
                                                  const int &static_id_path,
                                                  const int &static_id_wp,
                                                  const std::string& ns,
                                                  const std::vector<double> &marker_color,
                                                  const bool &plot_state)
{
  visualization_msgs::msg::Marker marker;
  marker.ns = ns;
  marker.type = visualization_msgs::msg::Marker::LINE_LIST;
  marker.header.frame_id="world";
  marker.header.stamp=rclcpp::Clock().now();
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.id= static_id_path;

  marker.scale.x = connection_marker_scale_.at(0);
  marker.scale.y = connection_marker_scale_.at(1);
  marker.scale.z = connection_marker_scale_.at(2);

  marker.color.r = marker_color.at(0);
  marker.color.g = marker_color.at(1);
  marker.color.b = marker_color.at(2);
  marker.color.a = marker_color.at(3);

  visualization_msgs::msg::Marker marker_wp;
  marker_wp.ns = ns;
  marker_wp.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  marker_wp.header.frame_id="world";
  marker_wp.header.stamp=rclcpp::Clock().now();
  marker_wp.action = visualization_msgs::msg::Marker::ADD;
  marker_wp.id= static_id_wp;

  marker_wp.scale.x = node_marker_scale_.at(0);
  marker_wp.scale.y = node_marker_scale_.at(1);
  marker_wp.scale.z = node_marker_scale_.at(2);

  marker_wp.color.r = marker_color.at(0);
  marker_wp.color.g = marker_color.at(1);
  marker_wp.color.b = marker_color.at(2);
  marker_wp.color.a = marker_color.at(3);

  std::vector<int> ids;
  ids.push_back(marker.id);
  ids.push_back(marker_wp.id);

  std::vector<ConnectionPtr> connections=path->getConnections();
  if (connections.size()==0)
    return ids;
  for (const ConnectionPtr& conn: connections)
  {
    Eigen::VectorXd parent = conn->getParent()->getConfiguration();
    Eigen::VectorXd child = conn->getChild()->getConfiguration();

    double length = (parent-child).norm();
    double step = length/SUBDIVISION_FACTOR;

    Eigen::VectorXd v = (child-parent)/length;
    Eigen::VectorXd conf1 = parent;
    Eigen::VectorXd conf2;


    for(unsigned int i=1;i<=SUBDIVISION_FACTOR; i++)
    {
      conf2 = parent + i*step*v;

      geometry_msgs::msg::Pose pose;
      //state_->setJointGroupPositions(group_name_,conf1);
      for (size_t ij=0;ij<joint_names_.size();ij++)
        state_->setJointPositions(joint_models_.at(ij),&conf1(ij));
      pose = tf2::toMsg(state_->getGlobalLinkTransform(last_link_));
      //tf::poseEigenToMsg(state_->getGlobalLinkTransform(last_link_),pose);
      marker.points.push_back(pose.position);
      if(i==1) marker_wp.points.push_back(pose.position);

      //state_->setJointGroupPositions(group_name_,conf2);
      for (size_t ij=0;ij<joint_names_.size();ij++)
        state_->setJointPositions(joint_models_.at(ij),&conf2(ij));
      pose = tf2::toMsg(state_->getGlobalLinkTransform(last_link_));
      //tf::poseEigenToMsg(state_->getGlobalLinkTransform(last_link_),pose);
      marker.points.push_back(pose.position);
      if(i==SUBDIVISION_FACTOR) marker_wp.points.push_back(pose.position);

      conf1 = conf2;
    }

  }

  marker_pub_->publish(marker);
  marker_pub_->publish(marker_wp);
  rclcpp::Clock().sleep_for(rclcpp::Duration::from_seconds(DISPLAY_TIME));
  return ids;
}
int Display::displaySubtree(const SubtreePtr &subtree,
                            const std::string &ns,
                            const std::vector<double> &marker_color)
{
  int static_id = marker_id_++;

  return displaySubtree(subtree,static_id,ns,marker_color);
}

int Display::displaySubtree(const SubtreePtr &subtree,
                            const int &static_id,
                            const std::string &ns,
                            const std::vector<double> &marker_color)
{
  visualization_msgs::msg::Marker marker;
  marker.ns = ns;
  marker.type = visualization_msgs::msg::Marker::LINE_LIST;
  marker.header.frame_id="world";
  marker.header.stamp=rclcpp::Clock().now();
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.id= static_id;

  marker.scale.x = tree_marker_scale_.at(0);
  marker.scale.y = tree_marker_scale_.at(1);
  marker.scale.z = tree_marker_scale_.at(2);

  marker.color.r = marker_color.at(0);
  marker.color.g = marker_color.at(1);
  marker.color.b = marker_color.at(2);
  marker.color.a = marker_color.at(3);
  displayTreeNode(subtree->getRoot(),subtree,marker.points,true);

  marker_pub_->publish(marker);
  rclcpp::Clock().sleep_for(rclcpp::Duration::from_seconds(DISPLAY_TIME));
  return marker.id;

}

int Display::displayTree(const TreePtr &tree,
                         const std::string &ns,
                         const std::vector<double> &marker_color)
{
  int static_id = marker_id_++;

  return displayTree(tree,static_id,ns,marker_color);
}

int Display::displayTree(const TreePtr &tree,
                         const int &static_id,
                         const std::string &ns,
                         const std::vector<double> &marker_color)
{
  visualization_msgs::msg::Marker marker;
  marker.ns = ns;
  marker.type = visualization_msgs::msg::Marker::LINE_LIST;
  marker.header.frame_id="world";
//  marker.header.stamp=rclcpp::Clock().now();
  marker.header.stamp=rclcpp::Clock().now();
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.id= static_id;

  marker.scale.x = tree_marker_scale_.at(0);
  marker.scale.y = tree_marker_scale_.at(1);
  marker.scale.z = tree_marker_scale_.at(2);

  marker.color.r = marker_color.at(0);
  marker.color.g = marker_color.at(1);
  marker.color.b = marker_color.at(2);
  marker.color.a = marker_color.at(3);
  displayTreeNode(tree->getRoot(),tree,marker.points,false);

  marker_pub_->publish(marker);
  rclcpp::Clock().sleep_for(rclcpp::Duration::from_seconds(DISPLAY_TIME));
  return marker.id;

}

void Display::displayTreeNode(const NodePtr &n,
                              const TreePtr& tree,
                              std::vector<geometry_msgs::msg::Point>& points,
                              const bool check_in_tree)
{
  std::vector<ConnectionPtr> connections;
    connections=n->child_connections_;

  if (connections.size()==0)
    return;

  bool add_points;
  for (const ConnectionPtr& conn: connections)
  {
    add_points = true;
    if(check_in_tree)
    {
      if(tree->isInTree(conn->getChild())) //CHIEDI SE DEVO DISTINGUERE PER DIRECTIONS
        add_points = true;
      else
        add_points = false;
    }

    if(add_points)
    {
      geometry_msgs::msg::Pose pose;
      //state_->setJointGroupPositions(group_name_,conn->getParent()->getConfiguration());
      for (size_t ij=0;ij<joint_names_.size();ij++)
        state_->setJointPositions(joint_models_.at(ij),&conn->getParent()->getConfiguration()(ij));
      pose = tf2::toMsg(state_->getGlobalLinkTransform(last_link_));
//      tf::poseEigenToMsg(state_->getGlobalLinkTransform(last_link_),pose);
      points.push_back(pose.position);

//      state_->setJointGroupPositions(group_name_,conn->getChild()->getConfiguration());
      for (size_t ij=0;ij<joint_names_.size();ij++)
        state_->setJointPositions(joint_models_.at(ij),&conn->getChild()->getConfiguration()(ij));
      pose = tf2::toMsg(state_->getGlobalLinkTransform(last_link_));
//      tf::poseEigenToMsg(state_->getGlobalLinkTransform(last_link_),pose);
      points.push_back(pose.position);

      displayTreeNode(conn->getChild(),tree,points,check_in_tree);
    }
  }
}

int Display::displayNet(const NetPtr &net,
                        const std::string &ns,
                        const std::vector<double> &marker_color)
{
  int static_id = marker_id_++;

  return displayNet(net,static_id,ns,marker_color);
}

int Display::displayNet(const NetPtr &net,
                        const int &static_id,
                        const std::string &ns,
                        const std::vector<double> &marker_color)
{
  visualization_msgs::msg::Marker marker;
  marker.ns = ns;
  marker.type = visualization_msgs::msg::Marker::LINE_LIST;
  marker.header.frame_id="world";
//  marker.header.stamp=rclcpp::Clock().now();
  marker.header.stamp=rclcpp::Clock().now();
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.id= static_id;

  marker.scale.x = tree_marker_scale_.at(0);
  marker.scale.y = tree_marker_scale_.at(1);
  marker.scale.z = tree_marker_scale_.at(2);

  marker.color.r = marker_color.at(0);
  marker.color.g = marker_color.at(1);
  marker.color.b = marker_color.at(2);
  marker.color.a = marker_color.at(3);
  displayNetNode(net->getTree()->getRoot(),net,marker.points);

  marker_pub_->publish(marker);
//  rclcpp::Clock().sleep_for(rclcpp::Duration::from_seconds(DISPLAY_TIME));
  rclcpp::Clock().sleep_for(rclcpp::Duration::from_seconds(DISPLAY_TIME));
  return marker.id;

}

void Display::displayNetNode(const NodePtr &n,
                             const NetPtr& net,
                             std::vector<geometry_msgs::msg::Point>& points)
{
  TreePtr tree = net->getTree();
  std::vector<ConnectionPtr> connections, net_connections;
    connections=n->child_connections_;
    net_connections=n->net_child_connections_;
    connections.insert(connections.end(),net_connections.begin(),net_connections.end());
  if (connections.size()==0)
    return;

  for (const ConnectionPtr& conn: connections)
  {
    geometry_msgs::msg::Pose pose;
//    state_->setJointGroupPositions(group_name_,conn->getParent()->getConfiguration());
    for (size_t ij=0;ij<joint_names_.size();ij++)
      state_->setJointPositions(joint_models_.at(ij),&conn->getParent()->getConfiguration()(ij));
    pose = tf2::toMsg(state_->getGlobalLinkTransform(last_link_));
//    tf::poseEigenToMsg(state_->getGlobalLinkTransform(last_link_),pose);
    points.push_back(pose.position);

//    state_->setJointGroupPositions(group_name_,conn->getChild()->getConfiguration());
    for (size_t ij=0;ij<joint_names_.size();ij++)
      state_->setJointPositions(joint_models_.at(ij),&conn->getChild()->getConfiguration()(ij));
    pose = tf2::toMsg(state_->getGlobalLinkTransform(last_link_));
//    tf::poseEigenToMsg(state_->getGlobalLinkTransform(last_link_),pose);
    points.push_back(pose.position);

    displayNetNode(conn->getChild(),net,points);
  }
}

void Display::nextButton(const std::string& string)
{
//  moveit_visual_tools::MoveItVisualToolsPtr visual_tools = std::make_shared<moveit_visual_tools::MoveItVisualTools>("/rviz_visual_tools");
  moveit_visual_tools::MoveItVisualToolsPtr visual_tools = std::make_shared<moveit_visual_tools::MoveItVisualTools>(nh_, "/world", "/rviz_visual_tools");
  visual_tools->loadRemoteControl();
  visual_tools->prompt(string);
}
}
