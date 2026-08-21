#include <ros/ros.h>
#include <algorithm>
#include <cmath>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include "quadrotor_msgs/PositionCommand.h"
#include<nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


#define VELOCITY2D_CONTROL 0b101111000111 //设置好对应的掩码，从右往左依次对应PX/PY/PZ/VX/VY/VZ/AX/AY/AZ/FORCE/YAW/YAW-RATE
#define POSITION_CONTROL 0b100111111000   //位置起飞：使用PX/PY/PZ/YAW
#define PLANNER_CONTROL 0b100111000000 //轨迹跟踪：使用位置、速度和YAW（加速度位被忽略）

unsigned short velocity_mask = VELOCITY2D_CONTROL;    
unsigned short position_mask = POSITION_CONTROL;

float takeoff_height = 1.0f; //全局起飞高度（米）
mavros_msgs::PositionTarget current_goal;
nav_msgs::Odometry position_msg;
mavros_msgs::State current_state;



int now_yaw = 0;
float position_x, position_y, position_z, current_yaw;
float current_vel_x, current_vel_y, current_vel_z;
float hold_position_x, hold_position_y, hold_position_z, hold_yaw;
bool odom_received = false;
float ego_pos_x, ego_pos_y, ego_pos_z, ego_vel_x, ego_vel_y, ego_vel_z, ego_a_x, ego_a_y, ego_a_z, ego_yaw, ego_yaw_rate; //EGO planner information has position velocity acceleration yaw yaw_dot
bool receive = false;//触发轨迹的条件判断
float pi = 3.14159265;


namespace
{
constexpr double kControlRate = 50.0; //保持ctrl_v1源代码控制频率
constexpr double kSpeedLimit = 1.3; //须与 mid360_real.yaml 的 traj_opt/boundary/max_vel 保持一致

void limitVelocityNorm(double &vx, double &vy, double &vz, double max_speed)
{
  //速度限制
  const double speed = std::sqrt(vx * vx + vy * vy + vz * vz);

  if (max_speed > 0.0 && speed > max_speed)
  {
    const double scale = max_speed / speed;
    vx *= scale;
    vy *= scale;
    vz *= scale;
  }
}
} 

void state_cb(const mavros_msgs::State::ConstPtr& msg){
	current_state = *msg;
}

//read vehicle odometry
void position_cb(const nav_msgs::Odometry::ConstPtr&msg)
{
	position_msg=*msg;
	position_x = position_msg.pose.pose.position.x;
	position_y = position_msg.pose.pose.position.y;
	position_z = position_msg.pose.pose.position.z;
	current_vel_x = position_msg.twist.twist.linear.x;
	current_vel_y = position_msg.twist.twist.linear.y;
	current_vel_z = position_msg.twist.twist.linear.z;

	odom_received = true;
	//四元函数的计算
	tf::Quaternion quat;
	tf::quaternionMsgToTF(msg->pose.pose.orientation, quat);
	double roll,pitch,yaw;
  tf::Matrix3x3(quat).getRPY(roll,pitch,yaw);
	current_yaw = yaw;
}

//读取ego里的位置速度加速度yaw和yaw-dot信息，
quadrotor_msgs::PositionCommand ego;
void twist_planner_cb(const quadrotor_msgs::PositionCommand::ConstPtr& msg)//ego的回调函数
{
	
    receive = true;
	  ego = *msg;
    ego_pos_x = ego.position.x;
    ego_pos_y = ego.position.y;
    ego_pos_z = ego.position.z;
    ego_vel_x = ego.velocity.x;
    ego_vel_y = ego.velocity.y;
    ego_vel_z = ego.velocity.z;
    ego_a_x = ego.acceleration.x;
    ego_a_y = ego.acceleration.y;
    ego_a_z = ego.acceleration.z;
    ego_yaw = ego.yaw;
    ego_yaw_rate = ego.yaw_dot;
}

void Position_Hold()
{
  current_goal.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
  current_goal.header.stamp = ros::Time::now();
  current_goal.type_mask = position_mask;
  current_goal.position.x = hold_position_x;
  current_goal.position.y = hold_position_y;
  current_goal.position.z = hold_position_z;
  current_goal.yaw = hold_yaw;

  current_goal.velocity.x = 0.0;
  current_goal.velocity.y = 0.0;
  current_goal.velocity.z = 0.0;
  current_goal.yaw_rate = 0.0;

  ROS_INFO_THROTTLE(2.0, "hold on：(%.2f, %.2f, %.2f)",
                    hold_position_x, hold_position_y, hold_position_z);
}

void take_off(ros::Publisher &local_pos_pub,ros::ServiceClient &set_mode_client,ros::ServiceClient &arming_client,ros::Rate &rate)
{
  mavros_msgs::SetMode offb_set_mode;
  offb_set_mode.request.custom_mode = "OFFBOARD";

  mavros_msgs::CommandBool arm_cmd;
  arm_cmd.request.value = true;

  // 先发送一段起飞位置 setpoint，再请求 OFFBOARD。
  for (int i = 0; ros::ok() && i < 100; i++)
  {
    current_goal.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    current_goal.header.stamp = ros::Time::now();
    current_goal.type_mask = position_mask;
    current_goal.position.x = 0.0;
    current_goal.position.y = 0.0;
    current_goal.position.z = takeoff_height;
    current_goal.yaw = now_yaw;

    local_pos_pub.publish(current_goal);
    ros::spinOnce();
    rate.sleep(); 
  }

  while (ros::ok())
  {
    current_goal.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    current_goal.header.stamp = ros::Time::now();
    current_goal.type_mask = position_mask;
    current_goal.position.x = 0.0;
    current_goal.position.y = 0.0;
    current_goal.position.z = takeoff_height;
    current_goal.yaw = now_yaw;

    local_pos_pub.publish(current_goal);

    if (current_state.mode != "OFFBOARD")
    {
      if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
      {
        ROS_INFO("Offboard mode enabled");
      }
    }
    else if (!current_state.armed)
    {
      if (arming_client.call(arm_cmd) && arm_cmd.response.success)
      {
        ROS_INFO("arm success, take off");
      }
    }
    else if (position_z <= takeoff_height - 0.2f)
    {
      ROS_INFO_THROTTLE(1.0, "Take off... z=%.2f", position_z);
    }
    else
    {
      hold_position_x = position_x;
      hold_position_y = position_y;
      hold_position_z = takeoff_height;
      hold_yaw = current_yaw;
      ROS_INFO("Takeoff complete, z=%.2f", position_z);
      return;
    }

    ros::spinOnce();
    rate.sleep();
  }
}

void Planner_Control()
{
  current_goal.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
  current_goal.header.stamp = ros::Time::now();
  current_goal.type_mask = PLANNER_CONTROL;

  current_goal.position.x = ego_pos_x;
  current_goal.position.y = ego_pos_y;
  current_goal.position.z = ego_pos_z;

  double velocity_x = ego_vel_x;
  double velocity_y = ego_vel_y;
  double velocity_z = ego_vel_z;
  limitVelocityNorm(velocity_x, velocity_y, velocity_z, kSpeedLimit);
  current_goal.velocity.x = velocity_x;
  current_goal.velocity.y = velocity_y;
  current_goal.velocity.z = velocity_z;

  current_goal.yaw = ego_yaw;
  current_goal.yaw_rate = 0.0;

//   ROS_INFO_THROTTLE(0.5, "EGO trajectory speed: vel_xyz = %.2f",
//                     std::sqrt(std::pow(current_goal.velocity.x, 2) +
//                               std::pow(current_goal.velocity.y, 2) +
//                               std::pow(current_goal.velocity.z, 2)));
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "cxr_egoctrl_v1");
	setlocale(LC_ALL,"");
	ros::NodeHandle nh;
	ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
	("/mavros/state", 10, state_cb);//读取飞控状态的话题
  
	ros::Publisher local_pos_pub = nh.advertise<mavros_msgs::PositionTarget>
	("/mavros/setpoint_raw/local", 1); //这个话题很重要，可以控制无人机的位置速度加速度和yaw以及yaw-rate，里面有个掩码选择，需要注意
	
	ros::service::waitForService("/mavros/cmd/arming");
	ros::service::waitForService("/mavros/set_mode");

	ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
	("/mavros/cmd/arming");//解锁飞机的服务端
	ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
	("/mavros/set_mode");//设置飞机飞行模式的服务端
	
	ros::Subscriber twist_sub = nh.subscribe<quadrotor_msgs::PositionCommand>
	("/planner_cmd", 10, twist_planner_cb);//订阅planner的规划指令话题的

	ros::Subscriber position_sub=nh.subscribe<nav_msgs::Odometry>
  ("/mavros/local_position/odom",10, position_cb);

   ros::Rate rate(kControlRate); //控制频率尽可能高点，大于30hz
	
	take_off(local_pos_pub, set_mode_client, arming_client, rate);

	while(ros::ok())
	{
		//到达判断完全交给 SUPER 原版逻辑（轨迹播完 && 距目标<0.1m 后停发指令）。
		//只要收到过规划指令就一直跟踪：SUPER 到达停发后，这里持续重发
		//最后一条指令采样，等效于在目标点定点悬停。
		if(receive && odom_received)
		{
			Planner_Control();
		}
		else
		{
			Position_Hold(); //起飞后、收到首条规划指令前：原地点定
		}

		local_pos_pub.publish(current_goal);
		ros::spinOnce();
		rate.sleep();
	}

	return 0;
}
