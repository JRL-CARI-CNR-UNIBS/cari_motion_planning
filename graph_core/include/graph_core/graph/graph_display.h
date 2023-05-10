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

#pragma once
#include <graph_core/graph/net.h>
#include <graph_core/graph/subtree.h>
#include <graph_core/graph/path.h>
//#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/msg/marker_array.hpp>
#include <moveit/planning_scene/planning_scene.h>
//#include <eigen_conversions/eigen_msg.h>
#include <tf2_eigen/tf2_eigen.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
//#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/msg/display_trajectory.hpp>

namespace pathplan
{
#define DISPLAY_TIME 0.0001
#define DEFAULT_NODE_SIZE 0.02
#define DEFAULT_CONNECTION_SIZE 0.005
#define DEFAULT_TREE_SIZE 0.005
#define SUBDIVISION_FACTOR 30
class Display;
typedef std::shared_ptr<Display> DisplayPtr;
class Display: public std::enable_shared_from_this<Display>
{
protected:
  planning_scene::PlanningSceneConstPtr planning_scene_;
  std::string group_name_;
  std::string last_link_;
  int marker_id_;
  std::vector<double> node_marker_scale_;
  std::vector<double> connection_marker_scale_;
  std::vector<double> tree_marker_scale_;
//  ros::Publisher marker_pub_;
//  ros::NodeHandle nh_;
  std::shared_ptr<rclcpp::Publisher<visualization_msgs::msg::Marker>> marker_pub_;
  std::shared_ptr<rclcpp::Node> nh_;
  moveit::core::RobotStatePtr state_;

  const moveit::core::JointModelGroup* jmg_;
  std::vector<std::string> joint_names_;
  std::vector<const moveit::core::JointModel*> joint_models_;

  void displayTreeNode(const NodePtr& n,
                       const TreePtr& tree,
                       std::vector<geometry_msgs::msg::Point> &points,
                       const bool check_in_tree = false);
  void displayNetNode(const NodePtr& n,
                      const NetPtr& net,
                      std::vector<geometry_msgs::msg::Point> &points);
public:
  Display(const planning_scene::PlanningSceneConstPtr planning_scene,
          const std::string& group_name,
          const std::string& last_link="");

  DisplayPtr pointer()
  {
    return shared_from_this();
  }

  int getMarkerCounter()
  {
    return marker_id_;
  }

  void changeNodeSize(const std::vector<double>& marker_size)
  {
    node_marker_scale_ = marker_size;
  }
  void defaultNodeSize()
  {
    node_marker_scale_ = {DEFAULT_NODE_SIZE, DEFAULT_NODE_SIZE, DEFAULT_NODE_SIZE};
  }

  void changeConnectionSize(const std::vector<double>& marker_size)
  {
    connection_marker_scale_ = marker_size;
  }
  void defaultConnectionSize()
  {
    connection_marker_scale_ = {DEFAULT_CONNECTION_SIZE, DEFAULT_CONNECTION_SIZE, DEFAULT_CONNECTION_SIZE};
  }

  void clearMarkers(const std::string &ns="pathplan");
  void clearMarker(const int& id,const std::string& ns="pathplan");
  int displayNode(const NodePtr& n,
                  const std::string& ns="pathplan",
                  const std::vector<double>& marker_color = {1,0,0,1.0},
                  const bool& plot_state=false);
  int displayNode(const NodePtr& n,
                  const int &static_id,
                  const std::string& ns="pathplan",
                  const std::vector<double>& marker_color = {1,0,0,1.0},
                  const bool& plot_state=false);
  int displayConnection(const ConnectionPtr& conn,
                        const std::string& ns="pathplan",
                        const std::vector<double>& marker_color= {1,0,0,1.0},
                        const bool& plot_state=false);
  int displayConnection(const ConnectionPtr& conn,
                        const int &static_id,
                        const std::string& ns="pathplan",
                        const std::vector<double>& marker_color= {1,0,0,1.0},
                        const bool& plot_state=false);
  int displayPath(const PathPtr& path,
                  const std::string& ns="pathplan",
                  const std::vector<double>& marker_color= {1,0,0,1.0},
                  const bool& plot_state=false);
  int displayPath(const PathPtr& path,
                  const int &static_id,
                  const std::string& ns="pathplan",
                  const std::vector<double>& marker_color= {1,0,0,1.0},
                  const bool& plot_state=false);
  std::vector<int> displayPathAndWaypoints(const PathPtr& path,
                                           const std::string& ns="pathplan",
                                           const std::vector<double>& marker_color= {1,0,0,1.0},
                                           const bool& plot_state=false);
  std::vector<int> displayPathAndWaypoints(const PathPtr& path,
                                           const int &static_id_path,
                                           const int &static_id_wp,
                                           const std::string& ns="pathplan",
                                           const std::vector<double>& marker_color= {1,0,0,1.0},
                                           const bool& plot_state=false);
  int displayTree(const TreePtr& tree,
                  const std::string& ns="pathplan",
                  const std::vector<double>& marker_color= {1,0,0,1.0});
  int displayTree(const TreePtr& tree,
                  const int &static_id,
                  const std::string& ns="pathplan",
                  const std::vector<double>& marker_color= {1,0,0,1.0});

  int displayNet(const NetPtr& net,
                 const std::string& ns="pathplan",
                 const std::vector<double>& marker_color= {1,0,0,1.0});
  int displayNet(const NetPtr& net,
                 const int &static_id,
                 const std::string& ns="pathplan",
                 const std::vector<double>& marker_color= {1,0,0,1.0});

  int displaySubtree(const SubtreePtr& subtree,
                     const std::string& ns="pathplan",
                     const std::vector<double>& marker_color= {1,0,0,1.0});
  int displaySubtree(const SubtreePtr& subtree,
                     const int &static_id,
                     const std::string& ns="pathplan",
                     const std::vector<double>& marker_color= {1,0,0,1.0});

  void nextButton(const std::string& string="Press Next");

};
}



