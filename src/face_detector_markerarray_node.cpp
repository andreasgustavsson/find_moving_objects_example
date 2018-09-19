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
#include <people_msgs/PositionMeasurement.h>
#include <people_msgs/PositionMeasurementArray.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

ros::Publisher pub;

void cb(const people_msgs::PositionMeasurementArray::ConstPtr & msg)
{
  visualization_msgs::MarkerArray msg_out;
  ros::Time now = ros::Time::now();
  static unsigned int seq = 0;
  
  unsigned int nr_people = msg->people.size();
  for (unsigned int i=0; i<nr_people; ++i)
  {
    const people_msgs::PositionMeasurement * face_ptr = &msg->people[i];
    visualization_msgs::Marker marker;
    
    marker.header.seq = seq++;
    marker.header.stamp = now;
    marker.header.frame_id = msg->header.frame_id;
    
    marker.ns = "faces";
    marker.id = i;
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.action = 0;
    marker.pose.position.x = msg->people[i].pos.x;
    marker.pose.position.y = msg->people[i].pos.y;
    marker.pose.position.z = msg->people[i].pos.z;
    marker.scale.z = 0.5;
    marker.lifetime = ros::Duration(0.5);
    marker.frame_locked = false;
    marker.text = std::string("face ");// + char(i) + std::string(" (") + marker.id + std::string(")");
//     marker.text = msg->people[i].object_id;
    
    msg_out.markers.push_back(marker);
  }
  
  pub.publish(msg_out);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "face_detector_markerarray_node");
  ros::NodeHandle n;

  pub = n.advertise<visualization_msgs::MarkerArray>("/face_detector/markers", 10);
  ros::Subscriber sub = n.subscribe<people_msgs::PositionMeasurementArray>("/face_detector/people_tracker_measurements_array", 10, cb);

  ros::spin();

  return 0;
}