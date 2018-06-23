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

/* ROS */
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

/* C/C++ */
#include <iostream> 
#include <cmath>
#include <pthread.h>

/* LOCAL INCLUDES */
#include <find_moving_objects/bank.h>

using namespace find_moving_objects;

/*
 * Standard Units of Measure and Coordinate Conventions:   http://www.ros.org/reps/rep-0103.html
 * Coordinate Frames for Mobile Platforms:                 http://www.ros.org/reps/rep-0105.html  
 */


/* HANDLE TO THIS NODE */
ros::NodeHandle * g_node;
bool first_message_received;
const double mounting_angle_shift_z = -(2.34731+(-0.776829))/2;


/* CONFIDENCE CALCULATION FOR BANK */
const double a = -10 / 3;
double find_moving_objects::Bank::calculateConfidence(const find_moving_objects::MovingObject & mo,
                                                      const find_moving_objects::BankArgument & ba,
                                                      const double dt,
                                                      const double mo_old_width,
                                                      const bool transform_old_time_map_frame_success,
                                                      const bool transform_new_time_map_frame_success,
                                                      const bool transform_old_time_fixed_frame_success,
                                                      const bool transform_new_time_fixed_frame_success,
                                                      const bool transform_old_time_base_frame_success,
                                                      const bool transform_new_time_base_frame_success)
{
  return ba.ema_alpha * // Using weighting decay decreases the confidence while,
  (ba.base_confidence + // how much we trust the sensor itself, 
   (transform_old_time_map_frame_success && 
   transform_new_time_map_frame_success && 
   transform_old_time_fixed_frame_success && 
   transform_new_time_fixed_frame_success && 
   transform_old_time_base_frame_success  &&
   transform_new_time_base_frame_success ? 0.5 : 0.0) + // transform success,
   a*(dt*dt-1.2*dt+0.27) + // a well-adapted bank size in relation to the sensor rate and environmental context,
   (-5.0 * fabsf(mo.seen_width - mo_old_width)));// + // and low difference in width between old and new found object,
   // make us more confident
}


/* USER OPTION INDICES FOR EASY ACCESS */
typedef enum {
  O_I_MOUNTING_ANGLE_SHIFT_Z = 0,
  NR_OPTIONS
} option_index_t;


/* BANK AND ARGUMENT */
find_moving_objects::Bank * bank;
find_moving_objects::BankArgument bank_argument;


/* LOCAL MESSAGE POINTER */
sensor_msgs::LaserScan::ConstPtr msg_old;


/* CALLBACK FOR FIRST MESSAGE */
void laserScanCallbackFirst(const sensor_msgs::LaserScan::ConstPtr & msg_org)
{
  // Working copy of msg
  sensor_msgs::LaserScan::ConstPtr msg;
  
  // Should we rotate the reading around the z-axis?
  if (mounting_angle_shift_z != 0.0)
  {
    sensor_msgs::LaserScan msg_copy = *msg_org;
    // Change angles
    msg_copy.angle_min = msg_copy.angle_min + mounting_angle_shift_z;
    msg_copy.angle_max = msg_copy.angle_max + mounting_angle_shift_z;
    // Create ConstPtr by COPYING the message again...
    sensor_msgs::LaserScan::ConstPtr msg_copy_ptr(new sensor_msgs::LaserScan(msg_copy));
    // Update buffer
    msg = msg_copy_ptr;
  }
  else
  {
    msg = msg_org;
  }
  
  // Keep reference to message
  msg_old = msg;
  
  // Init bank
  bank_argument.points_per_scan = msg->ranges.size();
  if (bank->init(bank_argument, msg) == 0)
  {
    first_message_received = true;
  }
}  


/* CALLBACK FOR ALL BUT THE FIRST MESSAGE */
void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr & msg_org)
{
  // Working copy of msg
  sensor_msgs::LaserScan::ConstPtr msg;
  
  // Should we rotate the reading around the z-axis?
  if (mounting_angle_shift_z != 0.0)
  {
    sensor_msgs::LaserScan msg_copy = *msg_org;
    // Change angles
    msg_copy.angle_min = msg_copy.angle_min + mounting_angle_shift_z;
    msg_copy.angle_max = msg_copy.angle_max + mounting_angle_shift_z;
    // Create ConstPtr by COPYING the message again...
    sensor_msgs::LaserScan::ConstPtr msg_copy_ptr(new sensor_msgs::LaserScan(msg_copy));
    // Update buffer
    msg = msg_copy_ptr;
  }
  else
  {
    msg = msg_org;
  }
  
  // Check if duplicate message was received
  if (msg->header.stamp.toSec() == msg_old->header.stamp.toSec())
  {
    bool same_msg = true;
    for (unsigned int i=0; i<bank_argument.points_per_scan; ++i)
    {
      if (msg->ranges[i] != msg_old->ranges[i])
      {
        same_msg = false;
        break;
      }
    }
    // Discard this message?
    if (same_msg)
    {
      return;
    }
  }
  // Message is ok
  
  // Keep reference to message
  msg_old = msg;
  
  // Add message to bank and find and report moving objects
  bank->addMessage(msg);
  bank->findAndReportMovingObjects();
}


/* ENTRY POINT */
int main (int argc, char ** argv)
{
  // Init
  first_message_received = false;  
  
  // Init ROS
  ros::init(argc, argv, "mo_finder_laserscan", ros::init_options::AnonymousName);
  g_node = new ros::NodeHandle;
  bank = new find_moving_objects::Bank;
  
  // Init bank_argument with user options
  bank_argument.ema_alpha = 1.0;
  bank_argument.nr_scans_in_bank = 11;
  bank_argument.object_threshold_edge_max_delta_range = 0.15;
  bank_argument.object_threshold_min_nr_points = 5;
  bank_argument.object_threshold_max_distance = 6.5; // m
  bank_argument.object_threshold_min_speed = 0.03; // m/s
  bank_argument.object_threshold_max_delta_width_in_points = 5;
  bank_argument.object_threshold_bank_tracking_max_delta_distance = 0.2;
  bank_argument.object_threshold_min_confidence = 0;
  bank_argument.base_confidence = 0.3;
  bank_argument.publish_ema = false; 
  bank_argument.publish_objects_closest_point_markers = false; 
  bank_argument.publish_objects_velocity_arrows = false;
  bank_argument.publish_objects_delta_position_lines = false;
  bank_argument.velocity_arrows_use_full_gray_scale = false;
  bank_argument.velocity_arrows_use_sensor_frame = false;
  bank_argument.velocity_arrows_use_base_frame = false;
  bank_argument.velocity_arrows_use_fixed_frame = false;
  bank_argument.delta_position_lines_use_sensor_frame = false;
  bank_argument.delta_position_lines_use_base_frame = false;
  bank_argument.delta_position_lines_use_fixed_frame = false;
  bank_argument.publish_objects = true;
  bank_argument.map_frame = "map";
  bank_argument.fixed_frame = "odom";
  bank_argument.base_frame = "base_link";
  bank_argument.velocity_arrow_ns = "laserscan_velocity_arrow";
  bank_argument.delta_position_line_ns = "laserscan_delta_position_line";
  bank_argument.topic_ema = "/laserscan_ema";
  bank_argument.topic_objects_closest_point_markers = "/laserscan_objects_closest_point_markers";
  bank_argument.topic_objects_velocity_arrows = "/laserscan_objects_velocity_arrows";
  bank_argument.topic_objects_delta_position_lines = "/laserscan_objects_delta_position_lines";
  bank_argument.topic_objects = "/moving_objects";
  bank_argument.publish_buffer_size = 1;
  
  // Receive first message and init bank
  {
    // Subscribe to the sensor topic using first callback
    ros::Subscriber sub = g_node->subscribe("/scan_filtered_echoed",
                                            1,
                                            laserScanCallbackFirst);
    
    // Receive first message
    while (!first_message_received)
    {
      ros::spinOnce();
      if (!g_node->ok())
      {
        return 0;
      }
    }
  } // un-scope sub object
  
  // Subscribe to sensor topic using new callback
  ros::Subscriber sub = g_node->subscribe("/scan_filtered_echoed",  
                                          1, 
                                          laserScanCallback);
  
  // Enter receive loop
  ros::spin();
  
  return 0;
}
