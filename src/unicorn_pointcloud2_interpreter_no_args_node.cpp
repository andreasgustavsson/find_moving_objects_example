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
#include <sensor_msgs/PointCloud2.h>

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
   a*(dt*dt-1.2*dt+0.27) +// a well-adapted bank size in the relation to the sensor rate and environmental context,
   (-5.0 * fabsf(mo.seen_width - mo_old_width))); // and low difference in width between old and new found object,
   // make us more confident
}


/* BANK AND ARGUMENT */
find_moving_objects::Bank * bank;
find_moving_objects::BankArgument bank_argument;


/* LOCAL MESSAGE POINTER */
sensor_msgs::PointCloud2::ConstPtr msg_old;


/* CALLBACK FOR FIRST MESSAGE */
void pointCloud2CallbackFirst(const sensor_msgs::PointCloud2::ConstPtr & msg)
{
  // Keep reference to message
  msg_old = msg;

  // Init bank
  if (bank->init(bank_argument, msg) == 0)
  {
    first_message_received = true;
  }
}


/* CALLBACK FOR ALL BUT THE FIRST MESSAGE */
void pointCloud2Callback(const sensor_msgs::PointCloud2::ConstPtr & msg)
{
  // Keep reference to message
  msg_old = msg;

  // Was message added to bank?
  if (bank->addMessage(msg) != 0)
  {
    // Adding message failed
    return;
  }

  // If so, then find and report objects
  bank->findAndReportMovingObjects();
}


/* ENTRY POINT */
int main (int argc, char ** argv)
{
  // Init
  first_message_received = false;

  // Init ROS
  ros::init(argc, argv, "mo_finder_pointcloud2", ros::init_options::AnonymousName);
  g_node = new ros::NodeHandle;
  bank = new find_moving_objects::Bank;

  // Init bank_argument using user options
  bank_argument.ema_alpha = 1.0;
  bank_argument.nr_scans_in_bank = 5;
  bank_argument.points_per_scan = 360;
  bank_argument.angle_max = M_PI / 2;
  bank_argument.angle_min = -bank_argument.angle_max;
  bank_argument.object_threshold_edge_max_delta_range = 0.15;
  bank_argument.object_threshold_min_nr_points = 3;
  bank_argument.object_threshold_max_distance = 6.5; // m
  bank_argument.object_threshold_min_speed = 0.1; // m/s
  bank_argument.object_threshold_max_delta_width_in_points = 15;
  bank_argument.object_threshold_bank_tracking_max_delta_distance = 0.2;
  bank_argument.object_threshold_min_confidence = 0;
  bank_argument.base_confidence = 0.1;
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
  bank_argument.velocity_arrow_ns = "pointcloud2_velocity_arrow";
  bank_argument.topic_ema = "/pointcloud2_ema";
  bank_argument.topic_objects_closest_point_markers = "/pointcloud2_objects_closest_point_markers";
  bank_argument.topic_objects_velocity_arrows = "/pointcloud2_objects_velocity_arrows";
  bank_argument.topic_objects_delta_position_lines = "/pointcloud2_objects_delta_position_lines";
  bank_argument.topic_objects = "/moving_objects";
  bank_argument.publish_buffer_size = 1;

  // PointCloud2-specific
  bank_argument.PC2_message_x_coordinate_field_name = "x";
  bank_argument.PC2_message_y_coordinate_field_name = "y";
  bank_argument.PC2_message_z_coordinate_field_name = "z";
  bank_argument.PC2_voxel_leaf_size = 0.02;
  bank_argument.PC2_threshold_z_min = 0.0;
  bank_argument.PC2_threshold_z_max = 1.0;

  // Receive first message and init bank
  {
    // Subscribe to the sensor topic using first callback
    ros::Subscriber sub = g_node->subscribe("/cloud_filtered_echoed",
                                            1,
                                            pointCloud2CallbackFirst);

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
  ros::Subscriber sub = g_node->subscribe("/cloud_filtered_echoed",
                                          1,
                                          pointCloud2Callback);

  // Enter receive loop
  ros::spin();

  return 0;
}
