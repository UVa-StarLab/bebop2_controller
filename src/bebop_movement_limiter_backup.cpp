#include "ros/ros.h"
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <std_msgs/Empty.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>

#define DEBUG 1
#define freq 10.0
#define dt 1/freq

// Target velocities
float target_lin = 0.0;  //default to 0.0 in m/s
float target_ang = 0.0;  //default to 0.0 in rad/s

// Odom position data
struct Position {
float x;
float y;
float z;
float roll;
float pitch;
float yaw;
} current_pos;

float x_limit = 1;
float y_limit = 1;
float z_limit = 1.5;

geometry_msgs::Twist constrained_cmd_vel;
geometry_msgs::Twist desired_cmd_vel;
geometry_msgs::WrenchStamped calculated_wrench;
nav_msgs::Odometry odom;

void constrain_movement();
void calc_force_vector(geometry_msgs::Twist);

                                                                      /* -----  Callback functions ----- */
void cmd_callback(const geometry_msgs::Twist::ConstPtr& data)
/*
   Called every time a new Command msg is published 
*/
{
   #if DEBUG
        ROS_INFO("New target velocity");
   #endif
   constrained_cmd_vel = *data;
   constrain_movement();
}

void position_callback(const geometry_msgs::TransformStamped::ConstPtr& data)
/*
   Called every time a new Odometry msg is published 
*/
{
   #if DEBUG
        ROS_INFO("Location Updated");
   #endif

   current_pos.x = data->transform.translation.x;
   current_pos.y = data->transform.translation.y;
   current_pos.z = data->transform.translation.z;

   current_pos.yaw = atan2(2.0*(data->transform.rotation.y*data->transform.rotation.z + data->transform.rotation.w*data->transform.rotation.x), data->transform.rotation.w*data->transform.rotation.w - data->transform.rotation.x*data->transform.rotation.x - data->transform.rotation.y*data->transform.rotation.y + data->transform.rotation.z*data->transform.rotation.z);
   current_pos.pitch = asin(-2.0*(data->transform.rotation.x*data->transform.rotation.z - data->transform.rotation.w*data->transform.rotation.y));
   current_pos.roll = atan2(2.0*(data->transform.rotation.x*data->transform.rotation.y + data->transform.rotation.w*data->transform.rotation.z), data->transform.rotation.w*data->transform.rotation.w + data->transform.rotation.x*data->transform.rotation.x - data->transform.rotation.y*data->transform.rotation.y - data->transform.rotation.z*data->transform.rotation.z);

}

void constrain_movement()
/*
   Calculates new velocity
*/
{
   
   x = current_pos.pitch*cos(current_pos.yaw) + current_pos.roll*sin(current_pos.yaw)
   y = current_pos.pitch*sin(current_pos.yaw) + current_pos.roll*cos(current_pos.yaw)

   if((current_pos.x >= x_limit) && (constrained_cmd_vel.linear.x > 0)){
       constrained_cmd_vel.linear.x = 0;
   }
   if((current_pos.x <= -x_limit) && (constrained_cmd_vel.linear.x < 0)){
       constrained_cmd_vel.linear.x = 0;
   }

   if((current_pos.y >= y_limit) && (constrained_cmd_vel.linear.y > 0)){
       constrained_cmd_vel.linear.y = 0;
   }
   if((current_pos.y <= -y_limit) && (constrained_cmd_vel.linear.y < 0)){
       constrained_cmd_vel.linear.y = 0;
   }

   if((current_pos.z >= z_limit) && (constrained_cmd_vel.linear.z > 0)){
       constrained_cmd_vel.linear.z = 0;
   }
   if((current_pos.z <= -z_limit) && (constrained_cmd_vel.linear.z < 0)){
       constrained_cmd_vel.linear.z = 0;
   }

   calc_force_vector(constrained_cmd_vel);
   return;
}

void calc_force_vector(geometry_msgs::Twist twistmsg){
   std_msgs::Header temp;
   calculated_wrench.header = temp;
   calculated_wrench.wrench.force.x = twistmsg.linear.x;
   calculated_wrench.wrench.force.y = twistmsg.linear.y;
   calculated_wrench.wrench.force.z = twistmsg.linear.z;

   return;
}
                                                                         /* ----- MAIN Function ----- */
int main(int argc, char **argv)
{
   ROS_INFO("Initializing Movement Limiter...");

   ros::init(argc, argv, "movement_limiter_node");
   ros::NodeHandle nh;
   ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("bebop/cmd_vel", 1, true);
   ros::Publisher wrench_pub = nh.advertise<geometry_msgs::WrenchStamped>("bebop/wrench", 1, true);
   ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("vicon/Bebop2/odom", 1, true);
   //ros::Publisher reset_pub = nh.advertise<std_msgs::Empty>("/mobile_base/commands/reset_odometry", 1, true);
   ros::Subscriber position_sub = nh.subscribe("vicon/Bebop2/Bebop2", 10, position_callback);
   ros::Subscriber vel_target = nh.subscribe("bebop/cmd_vel_user", 10, cmd_callback);
   ros::Rate loop_rate = freq;  
/*
   geometry_msgs::Twist new_vel_msg;
   float robot_vel_lin, past_pos, robot_vel_ang, past_ang;
   past_pos = 0.0;
   past_ang = 0.0;
*/

   ROS_INFO("Movement Limiter Initialized!");
   ROS_INFO("Starting Loop...");

   while(ros::ok()) {
      ros::spinOnce();
      
      //ROS_INFO_STREAM(robot_vel_lin);

      //new_vel_msg.linear.x = PID_calc(lin_ptr, robot_vel_lin, dt);
      //new_vel_msg.angular.z = PID_calc(ang_ptr, robot_vel_ang, dt);

      vel_pub.publish(constrained_cmd_vel);
      wrench_pub.publish(calculated_wrench);
      odom_pub.publish(odom);

      loop_rate.sleep();
   }
   return 0;
}

