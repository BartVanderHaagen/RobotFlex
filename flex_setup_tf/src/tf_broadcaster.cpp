/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
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
*   * Neither the name of Willow Garage, Inc. nor the names of its
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
* Author: Eitan Marder-Eppstein
*********************************************************************/
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "robot_tf_publisher");
  ros::NodeHandle n;

  ros::Rate r(100);

  tf::TransformBroadcaster broadcaster;

  while(n.ok()){

    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.0, 0.0, 0.06)),
        ros::Time::now(),"odom" ,"base_link"));


    r.sleep();
    
 
    broadcaster.sendTransform(
                              tf::StampedTransform(tf::Transform(tf::Quaternion(0, 0, 180, 1), tf::Vector3(0.0, 0.0, 0.06)), 
                                                   ros::Time::now(), 
                                                   "base_link", 
                                                   "laser"));
    r.sleep();

    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.10, 0.0, -0.06)),
        ros::Time::now(),"base_link" ,"rWheel"));

    r.sleep();

    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(-0.10, 0.0, -0.06)),
        ros::Time::now(),"base_link" ,"lWheel"));

    r.sleep();

    broadcaster.sendTransform(
                              tf::StampedTransform(tf::Transform(tf::Quaternion(0, 0, 180, 1), tf::Vector3(0.0, 0.0, 0.28)), 
                                                   ros::Time::now(), 
                                                   "laser", 
                                                   "head_h"));
    r.sleep();

    broadcaster.sendTransform(
                              tf::StampedTransform(tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.0, 0.0, 0.03)), 
                                                   ros::Time::now(), 
                                                   "head_h", 
                                                   "head_v"));
    r.sleep();

    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.04, 0.0, 0.075)),
        ros::Time::now(),"head_v", "right_camera"));

    r.sleep();

    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(-0.04, 0.0, 0.075)),
        ros::Time::now(),"head_v", "left_camera"));

    r.sleep();

    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.14, 0.0, -0.02)),
        ros::Time::now(),"head_h", "right_shoulder"));

    r.sleep();

    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(-0.14, 0.0, -0.02)),
        ros::Time::now(),"head_h", "left_shoulder"));

    r.sleep();

    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.02, 0.0, -0.075)),
        ros::Time::now(),"right_shoulder", "rElbow"));

    r.sleep();

    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(-0.02, 0.0, -0.075)),
        ros::Time::now(),"left_shoulder", "Elbow"));

    r.sleep();

    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.0, 0.0, -0.065)),
        ros::Time::now(),"rElbow", "llight"));

    r.sleep();

    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.0, 0.0, -0.065)),
        ros::Time::now(),"Elbow", "rlight"));

    r.sleep();
  }
}
