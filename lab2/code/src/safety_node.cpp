#include <ros/ros.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Bool.h>

class Safety
{
  // The class that handles emergency braking
private:
  ros::NodeHandle n_;
  ros::Subscriber odom_sub_;
  ros::Subscriber scan_sub_;
  ros::Publisher drive_pub_;
  ros::Publisher brake_pub_;

  double speed_;

public:
  Safety()
  {
    n_ = ros::NodeHandle();
    speed_ = 0.0;
    /*
    One publisher should publish to the /brake topic with an
    ackermann_msgs/AckermannDriveStamped brake message.

    One publisher should publish to the /brake_bool topic with a
    std_msgs/Bool message.

    You should also subscribe to the /scan topic to get the
    sensor_msgs/LaserScan messages and the /odom topic to get
    the nav_msgs/Odometry messages

    The subscribers should use the provided odomCallback and
    scan_callback as callback methods

    NOTE that the x component of the linear velocity in odom is the speed
    */

    odom_sub_ = n_.subscribe("odom", 1, &Safety::odomCallback, this);
    scan_sub_ = n_.subscribe("scan", 1, &Safety::scanCallback, this);
    drive_pub_ = n_.advertise<ackermann_msgs::AckermannDriveStamped>("brake", 1);
    brake_pub_ = n_.advertise<std_msgs::Bool>("brake_bool", 1);
  }
  void odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg)
  {
    speed_ = odom_msg->twist.twist.linear.x;
  }

  void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg)
  {
    ackermann_msgs::AckermannDriveStamped drive_msg;
    std_msgs::Bool brake_msg;

    for (int i = 0; i < scan_msg->ranges.size(); i++)
    {
      double angle = scan_msg->angle_min + (i * scan_msg->angle_increment);
      double range = scan_msg->ranges.at(i);
      double range_rate = speed_ * cos(angle);
      double time_to_collision = (range_rate > 0) ? (range / range_rate) : INFINITY;

      if (time_to_collision <= 0.5)
      {
        ROS_INFO("Emergency Brake Activated");

        brake_msg.data = true;
        brake_pub_.publish(brake_msg);
        drive_pub_.publish(drive_msg);

        return;
      }
    }

    brake_pub_.publish(brake_msg);
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "safety_node");
  Safety sn;
  ros::spin();
  return 0;
}
