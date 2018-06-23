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
#include <find_moving_objects/MovingObjectArray.h>
#include <find_moving_objects/MovingObject.h>
#include <find_moving_objects/option.h>

using namespace find_moving_objects;

Option g_options[] = 
{
  Option(false, "--subscribe_topic", 
         "Topic on which moving objects are published", 
         std::string("/moving_objects")),
};

typedef enum
{
  O_I_SUBSCRIBE_TOPIC,
  NR_OPTIONS
} option_index_t;


void callback(find_moving_objects::MovingObjectArray::ConstPtr msg)
{
  std::cout << *msg << std::endl
            << "--------------------------------------------------" << std::endl;
}

int main(int argc, char** argv){
//   scanArgs(argc, argv, options, NR_OPTIONS);
  
  ros::init(argc, argv, "mo_receiver", ros::init_options::AnonymousName);
  ros::NodeHandle node;
  
  Option::scanArgs(argc, argv, g_options);
  
  ros::Subscriber sub = node.subscribe(g_options[O_I_SUBSCRIBE_TOPIC].getStringValue(), 10, callback);
  
  ros::spin();
  
  return 0;
}
