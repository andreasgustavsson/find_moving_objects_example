/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2018, Andreas Gustavsson.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Andreas Gustavsson
*********************************************************************/

#include <ros/ros.h>
// #include <nav_msgs/Odometry.h>
// #include <std_msgs/Empty.h>
#include <tf/transform_broadcaster.h>
#include <cmath>

#include <find_moving_objects/option.h>

using namespace find_moving_objects;

/* User options */
Option g_options[] = {
  Option(false, "--dx",
         "X coordinate increment",
         0.0, std::numeric_limits<double>::min(), std::numeric_limits<double>::max()),
  Option(false, "--dy",
         "Y coordinate increment",
         0.0, std::numeric_limits<double>::min(), std::numeric_limits<double>::max()),
  Option(false, "--radius",
         "Radius of the circular movement",
         0.0, std::numeric_limits<double>::min(), std::numeric_limits<double>::max()),
  Option(false, "--dtheta",
         "Angle increment",
         0.0, -M_PI, M_PI),
  Option(false, "--reset_time",
         "Send an Empty message on \"/reset_time\"",
         true),
};

typedef enum {
  O_I_DX,
  O_I_DY,
  O_I_RADIUS,
  O_I_DTHETA,
  O_I_RESET_TIME,
  NR_OPTIONS
} option_index_t;


/* Entry point */
int main(int argc, char** argv){
//   scanArgs(argc, argv, options, NR_OPTIONS);


  ros::init(argc, argv, "frame_broadcaster", ros::init_options::AnonymousName);
  ros::NodeHandle node;
  Option::scanArgs(argc, argv, g_options);

//   ros::Subscriber sub = node.subscribe("/clock", 1, callback);


//   bool empty_sent = false;

//   pub = node.advertise<nav_msgs::Odometry>("/odometry/fake", 10);
//   nav_msgs::Odometry msg;
//   msg.header.seq = 0;
//   msg.header.frame_id = "odom";
//   msg.child_frame_id  = "base_link";
//   msg.pose.pose.position.z = 0;
// //   msg.pose.covariance = {0};
// //   msg.twist.covariance = {0};
//   msg.twist.twist.linear.y = 0;
//   msg.twist.twist.linear.z = 0;
//   msg.twist.twist.angular.x = 0;
//   msg.twist.twist.angular.y = 0;
//   ros::Time now/*, prev = ros::Time::now()*/;


  tf::TransformBroadcaster br;
  tf::Transform transform;
  const double dx = g_options[O_I_DX].getDoubleValue();
  const double dy = g_options[O_I_DY].getDoubleValue();
  const double radius = g_options[O_I_RADIUS].getDoubleValue();
  const double dtheta = g_options[O_I_DTHETA].getDoubleValue();
  const double PI_HALF = M_PI/2;
  const double TWO_PI  = 2*M_PI;
  double x = radius;
  double y = 0.0;
  double theta = 0.0;
  double dxsum = 0.0;
  double dysum = 0.0;
//   double next_odom_jump = TWO_PI;
//   double odomx = 0.0;
//   double odomy = 0.0;

  ros::Rate rate(30.0);
//   const double ls_angle_shift = -(2.34731f+(-0.776829f))/2; // Weird hack to get the alignment to be -90 to 90 deg
//   const double ls_angle_shift = 0;

//   if (!empty_sent)
//   {
//   if (options[O_I_RESET_TIME].getBoolValue())
//   {
//     ros::Publisher pub_reset_time = node.advertise<std_msgs::Empty>("/reset_time", 1);
//     pub_reset_time.publish(std_msgs::Empty());
//   }

//     empty_sent = true;
//   }

  while (node.ok()){
//     if (next_odom_jump <= theta)
//     {
//       next_odom_jump += TWO_PI;
//       odomx = 1 - odomx;
//       odomy = 1 - odomy;
    //     }
//     now = ros::Time::now();
    br.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(0,0,0,1),
//                                                         tf::Vector3(odomx,odomy,0)),
                                                        tf::Vector3(0,0,0)),
//                                           now,
                                          ros::Time::now(),
                                          "map",
                                          "odom"));

    br.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(tf::Vector3(0,0,1),
                                                                       dtheta == 0 ? 0.0 : theta + PI_HALF),
//                                                                        0.0),
                                                        tf::Vector3(x,y,0)),
//                                           now,
                                          ros::Time::now(),
                                          "odom",
                                          "base_link"));

    theta += dtheta;
//     const double dt = now.toSec() - prev.toSec();
    x = radius * cos(theta);
//     x=0;
    y = radius * sin(theta);
//     y=0;
    dxsum+=dx;
    dysum+=dy;
    x+=dxsum;
    y+=dysum;

//     msg.header.stamp = now;
//     msg.header.seq++;
//     // Pose is given in odom
//     msg.pose.pose.position.x = x;
//     msg.pose.pose.position.y = y;
//     tf::Quaternion q = tf::Quaternion(tf::Vector3(0,0,1), theta);
//     msg.pose.pose.orientation.x = q.x();
//     msg.pose.pose.orientation.y = q.x();
//     msg.pose.pose.orientation.z = q.y();
//     msg.pose.pose.orientation.w = q.w();
//     // Twist is given in base_link
//     msg.twist.twist.angular.z = dtheta / dt;
//     msg.twist.twist.linear.x = radius * msg.twist.twist.angular.z;
//
//     pub.publish(msg);

//     prev = now;

    // transform.setOrigin( tf::Vector3(0.0, 2.0, 0.0) );
//     transform.setOrigin( tf::Vector3(0.5, 0.1, 0.0) );
//     transform.setRotation( tf::Quaternion(0, 0, 0, 1) );
    br.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(0,0,0,1),
//                                                         tf::Quaternion(tf::Vector3(0.0, 0.0, 1.0),
//                                                                        0.3141592654),
                                                        tf::Vector3(0.5,0.1,0)),
//                                           now,
                                          ros::Time::now(),
                                          "base_link",
                                          "zed_left_camera"));
    // Rotate around Z-axis
//     transform.setOrigin( tf::Vector3(0.2,0.0,0.0));
//     transform.setRotation( tf::Quaternion(tf::Vector3(0,0,1), ls_angle_shift) );
//     br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "turtle1", "carrot1"));
//     br.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(tf::Vector3(0,0,1),
//                                                                        ls_angle_shift),
//                                                         tf::Vector3(0.2,0,0)),
//                                           ros::Time::now(),
//                                           "base_link",
//                                           "base_laser_ema"));
    br.sendTransform(tf::StampedTransform(tf::Transform(/*tf::Quaternion(0, -0.707106781, 0, 0.707106781),*/
//                                                         tf::Quaternion(0, -0.447213595, 0, 0.894427191),
                                                        tf::Quaternion(0, 0, 0, 1),
                                                        tf::Vector3(0.2,/*3*sin(15*theta)*/0,0)),
//                                           now,
                                          ros::Time::now(),
                                          "base_link",
                                          "base_laser"));
//     br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "zed_left_camera"));
    rate.sleep();
  }
  return 0;
}
