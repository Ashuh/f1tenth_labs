// ESE 680
// RRT assignment
// Author: Hongrui Zheng

// This file contains the class definition of tree nodes and RRT
// Before you start, please read: https://arxiv.org/pdf/1105.1186.pdf

// ros
#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>

// standard
#include <math.h>
#include <vector>
#include <array>
#include <iostream>
#include <fstream>
#include <iterator>
#include <string>
#include <algorithm>
#include <boost/algorithm/string.hpp>
#include <random>

// Struct defining the Node object in the RRT tree.
// More fields could be added to thiis struct if more info needed.
// You can choose to use this or not
typedef struct Node
{
  double x, y;
  double cost;  // only used for RRT*
  int parent;   // index of parent node in the tree vector
  bool is_root = false;
} Node;

class RRT
{
public:
  RRT(ros::NodeHandle& nh);
  virtual ~RRT();

private:
  ros::NodeHandle nh_;

  // ros pub/sub
  ros::Publisher grid_pub_;
  ros::Publisher path_pub_;
  ros::Publisher viz_pub_;

  ros::Subscriber pf_sub_;
  ros::Subscriber scan_sub_;
  ros::Subscriber global_path_sub_;

  // tf stuff
  tf::TransformListener listener;

  nav_msgs::OccupancyGrid occ_grid_;
  nav_msgs::Path global_path_;

  int max_iter_ = 10000;

  // random generator, use this
  std::mt19937 gen;
  std::uniform_real_distribution<> x_dist;
  std::uniform_real_distribution<> y_dist;

  // callbacks
  // where rrt actually happens
  void pf_callback(const geometry_msgs::PoseStamped::ConstPtr& pose_msg);
  // updates occupancy grid
  void scan_callback(const sensor_msgs::LaserScan::ConstPtr& scan_msg);
  void path_callback(const nav_msgs::Path::ConstPtr& path_msg);

  // RRT methods
  std::vector<double> sample();
  int nearest(std::vector<Node>& tree, std::vector<double>& sampled_point);
  Node steer(Node& nearest_node, std::vector<double>& sampled_point);
  bool check_collision(Node& nearest_node, Node& new_node);
  bool is_goal(Node& latest_added_node, double goal_x, double goal_y);
  std::vector<Node> find_path(std::vector<Node>& tree, Node& latest_added_node);
  // RRT* methods
  double cost(std::vector<Node>& tree, Node& node);
  double line_cost(Node& n1, Node& n2);
  std::vector<int> near(std::vector<Node>& tree, Node& node);

  geometry_msgs::PointStamped get_goal(const geometry_msgs::Point current_pos);
  void visualize(std::vector<Node> const& tree);
};
