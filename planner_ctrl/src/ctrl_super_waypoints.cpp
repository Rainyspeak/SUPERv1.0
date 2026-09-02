#include <ros/ros.h>
#include <algorithm>
#include <cmath>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>

namespace
{
constexpr unsigned short kPositionControl = 0b100111111000;
constexpr unsigned short kTrajectoryControl = 0b001000000000;
constexpr double kControlRate = 50.0;

mavros_msgs::State current_state;
mavros_msgs::PositionTarget current_goal;
quadrotor_msgs::PositionCommand super_command;

bool odom_received = false;
bool trajectory_received = false;

double position_x = 0.0;
double position_y = 0.0;
double position_z = 0.0;
double current_yaw = 0.0;
double hold_position_x = 0.0;
double hold_position_y = 0.0;
double hold_position_z = 0.0;
double hold_yaw = 0.0;

double takeoff_height = 1.0;
double max_speed = 1.0;

void stateCallback(const mavros_msgs::State::ConstPtr &msg)
{
  current_state = *msg;
}

void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
  position_x = msg->pose.pose.position.x;
  position_y = msg->pose.pose.position.y;
  position_z = msg->pose.pose.position.z;
  current_yaw = tf2::getYaw(msg->pose.pose.orientation);
  odom_received = true;
}

void trajectoryCallback(const quadrotor_msgs::PositionCommand::ConstPtr &msg)
{
  trajectory_received = true;
  super_command = *msg;
}

void limitVelocityNorm(double &vx, double &vy, double &vz)
{
  const double speed = std::sqrt(vx * vx + vy * vy + vz * vz);
  if (max_speed > 0.0 && speed > max_speed)
  {
    const double scale = max_speed / speed;
    vx *= scale;
    vy *= scale;
    vz *= scale;
  }
}

void setPositionHold(double x, double y, double z, double yaw)
{
  current_goal.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
  current_goal.header.stamp = ros::Time::now();
  current_goal.type_mask = kPositionControl;
  current_goal.position.x = x;
  current_goal.position.y = y;
  current_goal.position.z = z;
  current_goal.velocity.x = 0.0;
  current_goal.velocity.y = 0.0;
  current_goal.velocity.z = 0.0;
  current_goal.acceleration_or_force.x = 0.0;
  current_goal.acceleration_or_force.y = 0.0;
  current_goal.acceleration_or_force.z = 0.0;
  current_goal.yaw = yaw;
  current_goal.yaw_rate = 0.0;
}

void setTrajectoryCommand()
{
  double vx = super_command.velocity.x;
  double vy = super_command.velocity.y;
  double vz = super_command.velocity.z;
  limitVelocityNorm(vx, vy, vz);

  current_goal.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
  current_goal.header.stamp = ros::Time::now();
  current_goal.type_mask = kTrajectoryControl;
  current_goal.position.x = super_command.position.x;
  current_goal.position.y = super_command.position.y;
  current_goal.position.z = super_command.position.z;
  current_goal.velocity.x = vx;
  current_goal.velocity.y = vy;
  current_goal.velocity.z = vz;
  current_goal.acceleration_or_force.x = super_command.acceleration.x;
  current_goal.acceleration_or_force.y = super_command.acceleration.y;
  current_goal.acceleration_or_force.z = super_command.acceleration.z;
  current_goal.yaw = super_command.yaw;
  current_goal.yaw_rate = super_command.yaw_dot;
}
} // namespace

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ctrl_super_waypoints");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  pnh.param("takeoff_height", takeoff_height, 1.0);
  pnh.param("max_speed", max_speed, 1.0);
  takeoff_height = std::max(0.1, takeoff_height);
  max_speed = std::max(0.1, max_speed);

  ros::Subscriber state_sub = nh.subscribe("/mavros/state", 10, stateCallback);
  ros::Subscriber trajectory_sub = nh.subscribe("/planner_cmd", 10, trajectoryCallback);
  ros::Subscriber odom_sub = nh.subscribe("/mavros/local_position/odom", 10, odomCallback);
  ros::Publisher setpoint_pub = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 1);
  ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
  ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

  ros::Rate rate(kControlRate);
  while (ros::ok() && !current_state.connected)
  {
    ros::spinOnce();
    rate.sleep();
  }

  for (int i = 100; ros::ok() && i > 0; --i)
  {
    setPositionHold(0.0, 0.0, takeoff_height, 0.0);
    setpoint_pub.publish(current_goal);
    ros::spinOnce();
    rate.sleep();
  }

  mavros_msgs::SetMode offboard_mode;
  offboard_mode.request.custom_mode = "OFFBOARD";
  mavros_msgs::CommandBool arm_command;
  arm_command.request.value = true;
  ros::Time last_request = ros::Time::now();

  while (ros::ok())
  {
    if (current_state.mode != "OFFBOARD" && ros::Time::now() - last_request > ros::Duration(5.0))
    {
      if (set_mode_client.call(offboard_mode) && offboard_mode.response.mode_sent)
      {
        ROS_INFO("Offboard enabled");
      }
      last_request = ros::Time::now();
    }
    else if (!current_state.armed && ros::Time::now() - last_request > ros::Duration(5.0))
    {
      if (arming_client.call(arm_command) && arm_command.response.success)
      {
        ROS_INFO("Armed");
      }
      last_request = ros::Time::now();
    }

    if (odom_received && position_z >= takeoff_height - 0.2)
    {
      if (hold_position_x == 0.0 && hold_position_y == 0.0 && hold_position_z == 0.0)
      {
        hold_position_x = position_x;
        hold_position_y = position_y;
        hold_position_z = takeoff_height;
        hold_yaw = current_yaw;
        ROS_INFO("Takeoff complete, ready for waypoints");
      }
    }

    if (trajectory_received && odom_received)
    {
      setTrajectoryCommand();
    }
    else
    {
      setPositionHold(hold_position_x, hold_position_y, hold_position_z, hold_yaw);
    }

    setpoint_pub.publish(current_goal);
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
