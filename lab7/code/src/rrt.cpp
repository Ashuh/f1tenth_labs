// ESE 680
// RRT assignment
// Author: Hongrui Zheng

// This file contains the class definition of tree nodes and RRT
// Before you start, please read: https://arxiv.org/pdf/1105.1186.pdf
// Make sure you have read through the header file as well

#include <algorithm>
#include <string>

#include "rrt/rrt.h"

// Destructor of the RRT class
RRT::~RRT()
{
  // Do something in here, free up used memory, print message, etc.
  ROS_INFO("RRT shutting down");
}

// Constructor of the RRT class
RRT::RRT(ros::NodeHandle& nh) : nh_(nh), gen((std::random_device())())
{
  std::string pose_topic = "gt_pose";
  std::string scan_topic = "scan";
  std::string global_path_topic = "path/global";

  nh_.getParam("pose_topic", pose_topic);
  nh_.getParam("scan_topic", scan_topic);

  // ROS publishers
  grid_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("grid", 1);
  path_pub_ = nh_.advertise<nav_msgs::Path>("path/local", 1);
  viz_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("tree", 1, true);

  // ROS subscribers
  pf_sub_ = nh_.subscribe(pose_topic, 10, &RRT::pf_callback, this);
  scan_sub_ = nh_.subscribe(scan_topic, 10, &RRT::scan_callback, this);
  global_path_sub_ = nh_.subscribe(global_path_topic, 10, &RRT::path_callback, this);

  occ_grid_.header.frame_id = "laser";
  occ_grid_.info.width = 100;   // x axis
  occ_grid_.info.height = 100;  // y axis
  occ_grid_.info.resolution = 0.3;

  occ_grid_.info.origin.position.y = -50 * 0.3;
  occ_grid_.info.origin.position.x = -50 * 0.3;

  ROS_INFO("Created new RRT Object.");
}

void RRT::scan_callback(const sensor_msgs::LaserScan::ConstPtr& scan_msg)
{
  occ_grid_.data = std::vector<int8_t>(occ_grid_.info.width * occ_grid_.info.height);

  for (int i = 0; i < scan_msg->ranges.size(); ++i)
  {
    double range = scan_msg->ranges.at(i);
    double angle = scan_msg->angle_min + (scan_msg->angle_increment * i);
    double x = range * cos(angle);
    double y = range * sin(angle);

    int grid_x = (x - occ_grid_.info.origin.position.x) / occ_grid_.info.resolution;
    int grid_y = (y - occ_grid_.info.origin.position.x) / occ_grid_.info.resolution;

    if (grid_x >= 0 && grid_x < occ_grid_.info.width && grid_y >= 0 && grid_y < occ_grid_.info.height)
    {
      occ_grid_.data.at(grid_y * occ_grid_.info.width + grid_x) = 100;
    }
  }

  grid_pub_.publish(occ_grid_);
}

void RRT::path_callback(const nav_msgs::Path::ConstPtr& path_msg)
{
  global_path_ = *path_msg;
}

void RRT::pf_callback(const geometry_msgs::PoseStamped::ConstPtr& pose_msg)
{
  // The pose callback when subscribed to particle filter's inferred pose
  // The RRT main loop happens here
  // Args:
  //    pose_msg (*PoseStamped): pointer to the incoming pose message
  // Returns:
  //

  geometry_msgs::PointStamped goal = get_goal(pose_msg->pose.position);
  listener.transformPoint("laser", goal, goal);

  std::vector<Node> tree;

  Node root;
  root.is_root = true;
  root.x = 0.0;
  root.y = 0.0;

  tree.push_back(root);

  for (int i = 0; i < max_iter_; ++i)
  {
    std::vector<double> rand = sample();
    int nearest_id = nearest(tree, rand);
    Node new_node = steer(tree.at(nearest_id), rand);

    if (!check_collision(tree.at(nearest_id), new_node))
    {
      new_node.parent = nearest_id;
      tree.push_back(new_node);

      if (is_goal(new_node, goal.point.x, goal.point.y))
      {
        std::vector<Node> tree_path = find_path(tree, new_node);
        nav_msgs::Path path;
        path.header.frame_id = "map";

        for (auto& node : tree_path)
        {
          geometry_msgs::PointStamped point;
          point.header.frame_id = "laser";
          point.point.x = node.x;
          point.point.y = node.y;
          listener.transformPoint("map", point, point);
          geometry_msgs::PoseStamped pose;
          pose.header.frame_id = "map";
          pose.pose.position = point.point;
          path.poses.push_back(pose);
        }

        path_pub_.publish(path);

        break;
      }
    }
  }

  visualize(tree);
}

std::vector<double> RRT::sample()
{
  std::vector<double> sampled_point;

  x_dist.param(std::uniform_real_distribution<>::param_type(0, 5));
  y_dist.param(std::uniform_real_distribution<>::param_type(-1.5, 1.5));

  double x = x_dist(gen);
  double y = y_dist(gen);

  sampled_point.push_back(x);
  sampled_point.push_back(y);

  return sampled_point;
}

int RRT::nearest(std::vector<Node>& tree, std::vector<double>& sampled_point)
{
  int nearest_node = -1;

  double min_dist_sq = std::numeric_limits<double>::max();

  for (int i = 0; i < tree.size(); ++i)
  {
    double d_x = tree.at(i).x - sampled_point.at(0);
    double d_y = tree.at(i).y - sampled_point.at(1);
    double dist_sq = pow(d_x, 2) + pow(d_y, 2);

    if (dist_sq < min_dist_sq)
    {
      nearest_node = i;
      min_dist_sq = dist_sq;
    }
  }

  return nearest_node;
}

Node RRT::steer(Node& nearest_node, std::vector<double>& sampled_point)
{
  double max_dist = 0.2;
  double d_x = sampled_point.at(0) - nearest_node.x;
  double d_y = sampled_point.at(1) - nearest_node.y;
  double dist = sqrt(pow(d_x, 2) + pow(d_y, 2));
  double ratio = std::min(max_dist / dist, 1.0);

  Node new_node;
  new_node.x = nearest_node.x + ratio * d_x;
  new_node.y = nearest_node.y + ratio * d_y;

  return new_node;
}

bool RRT::check_collision(Node& nearest_node, Node& new_node)
{
  // Bresenham's line algorithm
  int x1 = (nearest_node.x - occ_grid_.info.origin.position.x) / occ_grid_.info.resolution;
  int y1 = (nearest_node.y - occ_grid_.info.origin.position.y) / occ_grid_.info.resolution;

  int x2 = (new_node.x - occ_grid_.info.origin.position.x) / occ_grid_.info.resolution;
  int y2 = (new_node.y - occ_grid_.info.origin.position.y) / occ_grid_.info.resolution;

  const bool steep = (fabs(y2 - y1) > fabs(x2 - x1));
  if (steep)
  {
    std::swap(x1, y1);
    std::swap(x2, y2);
  }

  if (x1 > x2)
  {
    std::swap(x1, x2);
    std::swap(y1, y2);
  }

  const float dx = x2 - x1;
  const float dy = fabs(y2 - y1);

  float error = dx / 2.0f;
  const int ystep = (y1 < y2) ? 1 : -1;
  int y = static_cast<int>(y1);

  const int maxX = static_cast<int>(x2);

  for (int x = static_cast<int>(x1); x <= maxX; x++)
  {
    double grid_x, grid_y;
    if (steep)
    {
      grid_x = y;
      grid_y = x;
    }
    else
    {
      grid_x = x;
      grid_y = y;
    }

    error -= dy;
    if (error < 0)
    {
      y += ystep;
      error += dx;
    }

    if (occ_grid_.data.at(grid_y * occ_grid_.info.width + grid_x) == 100)
    {
      return true;
    }
  }

  return false;
}

bool RRT::is_goal(Node& latest_added_node, double goal_x, double goal_y)
{
  double d_x = goal_x - latest_added_node.x;
  double d_y = goal_y - latest_added_node.y;
  double dist = sqrt(pow(d_x, 2) + pow(d_y, 2));

  return dist <= 0.1;
}

std::vector<Node> RRT::find_path(std::vector<Node>& tree, Node& latest_added_node)
{
  std::vector<Node> found_path;
  Node cur_node = latest_added_node;
  found_path.push_back(cur_node);

  while (!cur_node.is_root)
  {
    cur_node = tree.at(cur_node.parent);
    found_path.push_back(cur_node);
  }

  std::reverse(found_path.begin(), found_path.end());

  return found_path;
}

// RRT* methods
double RRT::cost(std::vector<Node>& tree, Node& node)
{
  // This method returns the cost associated with a node
  // Args:
  //    tree (std::vector<Node>): the current tree
  //    node (Node): the node the cost is calculated for
  // Returns:
  //    cost (double): the cost value associated with the node

  double cost = 0;
  // TODO: fill in this method

  return cost;
}

double RRT::line_cost(Node& n1, Node& n2)
{
  // This method returns the cost of the straight line path between two nodes
  // Args:
  //    n1 (Node): the Node at one end of the path
  //    n2 (Node): the Node at the other end of the path
  // Returns:
  //    cost (double): the cost value associated with the path

  double cost = 0;
  // TODO: fill in this method

  return cost;
}

std::vector<int> RRT::near(std::vector<Node>& tree, Node& node)
{
  // This method returns the set of Nodes in the neighborhood of a
  // node.
  // Args:
  //   tree (std::vector<Node>): the current tree
  //   node (Node): the node to find the neighborhood for
  // Returns:
  //   neighborhood (std::vector<int>): the index of the nodes in the neighborhood

  std::vector<int> neighborhood;
  // TODO:: fill in this method

  return neighborhood;
}

geometry_msgs::PointStamped RRT::get_goal(const geometry_msgs::Point current_pos)
{
  double target_dist = 2.0;

  int nearest_wp_id = -1;
  double min_dist = std::numeric_limits<double>::max();

  for (int i = 0; i < global_path_.poses.size(); ++i)
  {
    double d_x = current_pos.x - global_path_.poses.at(i).pose.position.x;
    double d_y = current_pos.y - global_path_.poses.at(i).pose.position.y;
    double dist = sqrt(pow(d_x, 2) + pow(d_y, 2));

    if (dist < min_dist)
    {
      nearest_wp_id = i;
      min_dist = dist;
    }
  }

  int goal_id = nearest_wp_id;
  double sum_dist = 0.0;

  for (int i = nearest_wp_id + 1; i < global_path_.poses.size(); ++i)
  {
    double d_x = global_path_.poses.at(i).pose.position.x - global_path_.poses.at(i - 1).pose.position.x;
    double d_y = global_path_.poses.at(i).pose.position.y - global_path_.poses.at(i - 1).pose.position.y;
    double dist = sqrt(pow(d_x, 2) + pow(d_y, 2));
    sum_dist += dist;
    goal_id = i;

    if (sum_dist > target_dist)
    {
      break;
    }
  }

  geometry_msgs::PointStamped goal;
  goal.point = global_path_.poses.at(goal_id).pose.position;
  goal.header.frame_id = global_path_.header.frame_id;

  return goal;
}

void RRT::visualize(std::vector<Node> const& tree)
{
  visualization_msgs::Marker edge_marker;

  edge_marker.action = visualization_msgs::Marker::ADD;
  edge_marker.type = visualization_msgs::Marker::LINE_LIST;
  edge_marker.header.frame_id = "laser";
  edge_marker.id = 0;
  edge_marker.lifetime = ros::Duration(0);
  edge_marker.scale.x = 0.01;
  edge_marker.color.r = 0.0;
  edge_marker.color.g = 1.0;
  edge_marker.color.b = 0.0;
  edge_marker.color.a = 1.0;

  visualization_msgs::Marker vertice_marker;
  vertice_marker.action = visualization_msgs::Marker::ADD;
  vertice_marker.type = visualization_msgs::Marker::SPHERE_LIST;
  vertice_marker.header.frame_id = "laser";
  vertice_marker.id = 1;
  vertice_marker.lifetime = ros::Duration(0);
  vertice_marker.scale.x = 0.025;
  vertice_marker.scale.y = 0.025;
  vertice_marker.scale.z = 0.025;
  vertice_marker.color.r = 0.0;
  vertice_marker.color.g = 0.0;
  vertice_marker.color.b = 1.0;
  vertice_marker.color.a = 1.0;

  for (auto& node : tree)
  {
    if (!node.is_root)
    {
      geometry_msgs::Point point_1, point_2;
      point_1.x = node.x;
      point_1.y = node.y;
      point_2.x = tree.at(node.parent).x;
      point_2.y = tree.at(node.parent).y;
      edge_marker.points.push_back(point_1);
      edge_marker.points.push_back(point_2);
      vertice_marker.points.push_back(point_1);
    }
  }

  visualization_msgs::MarkerArray markers;
  markers.markers.push_back(edge_marker);
  markers.markers.push_back(vertice_marker);

  viz_pub_.publish(markers);
}
