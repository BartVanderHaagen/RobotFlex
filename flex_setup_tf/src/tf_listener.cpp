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
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>

void transformPoint(const tf::TransformListener& listener){
  

  //we'll create a point in the base_laser frame that we'd like to transform to the base_link frame
  geometry_msgs::PointStamped map_point;
  map_point.header.frame_id = "map";

  //we'll just use the most recent transform available for our simple example
  map_point.header.stamp = ros::Time();

  //just an arbitrary point in space
  map_point.point.x = 1.0;
  map_point.point.y = 0.0;
  map_point.point.z = 0.0;
  
    //we'll create a point in the odom frame that we'd like to transform to the base_link frame
  geometry_msgs::PointStamped odom_point;
  odom_point.header.frame_id = "odom";

  //we'll just use the most recent transform available for our simple example
  odom_point.header.stamp = ros::Time();

  //just an arbitrary point in space
  odom_point.point.x = 1.0;
  odom_point.point.y = 0.0;
  odom_point.point.z = 0.0;


  //we'll create a point in the base_laser frame that we'd like to transform to the base_link frame
  geometry_msgs::PointStamped laser_point;
  laser_point.header.frame_id = "laser";

  //we'll just use the most recent transform available for our simple example
  laser_point.header.stamp = ros::Time();

  //just an arbitrary point in space
  laser_point.point.x = 1.0;
  laser_point.point.y = 0.0;
  laser_point.point.z = 0.0;
  
  //we'll create a point in the lWheel frame that we'd like to transform to the map frame
  geometry_msgs::PointStamped lWheel_point;
  lWheel_point.header.frame_id = "lWheel";

  //we'll just use the most recent transform available for our simple example
  lWheel_point.header.stamp = ros::Time();

  //just an arbitrary point in space
  lWheel_point.point.x = 1.0;
  lWheel_point.point.y = 0.0;
  lWheel_point.point.z = 0.0;

  //we'll create a point in the rWheel frame that we'd like to transform to the map frame
  geometry_msgs::PointStamped rWheel_point;
  rWheel_point.header.frame_id = "rWheel";

  //we'll just use the most recent transform available for our simple example
  rWheel_point.header.stamp = ros::Time();

  //just an arbitrary point in space
  rWheel_point.point.x = 1.0;
  rWheel_point.point.y = 0.0;
  rWheel_point.point.z = 0.0;

  //we'll create a point in the lElbow frame that we'd like to transform to the map frame
  geometry_msgs::PointStamped Elbow_point;
  Elbow_point.header.frame_id = "Elbow";

  //we'll just use the most recent transform available for our simple example
  Elbow_point.header.stamp = ros::Time();

  //just an arbitrary point in space
  Elbow_point.point.x = 1.0;
  Elbow_point.point.y = 0.0;
  Elbow_point.point.z = 0.0;

  //we'll create a point in the rElbow frame that we'd like to transform to the map frame
  geometry_msgs::PointStamped rElbow_point;
  rElbow_point.header.frame_id = "rElbow";

  //we'll just use the most recent transform available for our simple example
  rElbow_point.header.stamp = ros::Time();

  //just an arbitrary point in space
  rElbow_point.point.x = 1.0;
  rElbow_point.point.y = 0.0;
  rElbow_point.point.z = 0.0;
  
  //we'll create a point in the head_v frame that we'd like to transform to the base_link frame
  geometry_msgs::PointStamped head1_point;
  head1_point.header.frame_id = "head_v";

  //we'll just use the most recent transform available for our simple example
  head1_point.header.stamp = ros::Time();

  //just an arbitrary point in space
  head1_point.point.x = 1.0;
  head1_point.point.y = 0.0;
  head1_point.point.z = 0.0;

  //we'll create a point in the head_h frame that we'd like to transform to the base_link frame
  geometry_msgs::PointStamped head2_point;
  head2_point.header.frame_id = "head_h";

  //we'll just use the most recent transform available for our simple example
  head2_point.header.stamp = ros::Time();

  //just an arbitrary point in space
  head2_point.point.x = 1.0;
  head2_point.point.y = 0.0;
  head2_point.point.z = 0.0;

  //we'll create a point in the left_camera frame that we'd like to transform to the base_link frame
  geometry_msgs::PointStamped cam1_point;
  cam1_point.header.frame_id = "left_camera";

  //we'll just use the most recent transform available for our simple example
  cam1_point.header.stamp = ros::Time();

  //just an arbitrary point in space
  cam1_point.point.x = 1.0;
  cam1_point.point.y = 0.0;
  cam1_point.point.z = 0.0;

  //we'll create a point in the right_camera frame that we'd like to transform to the base_link frame
  geometry_msgs::PointStamped cam2_point;
  cam2_point.header.frame_id = "right_camera";

  //we'll just use the most recent transform available for our simple example
  cam2_point.header.stamp = ros::Time();

  //just an arbitrary point in space
  cam2_point.point.x = 1.0;

  //just an arbitrary point in space
  cam2_point.point.x = 1.0;
  cam2_point.point.y = 0.0;
  cam2_point.point.z = 0.0;

  //we'll create a point in the right_shoulder frame that we'd like to transform to the base_link frame
  geometry_msgs::PointStamped shol_point;
  shol_point.header.frame_id = "left_shoulder";

  //we'll just use the most recent transform available for our simple example
  shol_point.header.stamp = ros::Time();

  //just an arbitrary point in space
  shol_point.point.x = 1.0;

  //just an arbitrary point in space
  shol_point.point.x = 1.0;
  shol_point.point.y = 0.0;
  shol_point.point.z = 0.0;

  //we'll create a point in the left_sholder frame that we'd like to transform to the base_link frame
  geometry_msgs::PointStamped shor_point;
  shor_point.header.frame_id = "right_shoulder";

  //we'll just use the most recent transform available for our simple example
  shor_point.header.stamp = ros::Time();

  //just an arbitrary point in space
  shor_point.point.x = 1.0;

  //just an arbitrary point in space
  shor_point.point.x = 1.0;
  shor_point.point.y = 0.0;
  shor_point.point.z = 0.0;

  //we'll create a point in the right_light frame that we'd like to transform to the elbow_link frame
  geometry_msgs::PointStamped rlight_point;
  rlight_point.header.frame_id = "right_light";

  //we'll just use the most recent transform available for our simple example
  rlight_point.header.stamp = ros::Time();

  //just an arbitrary point in space
  rlight_point.point.x = 1.0;

  //just an arbitrary point in space
  rlight_point.point.x = 1.0;
  rlight_point.point.y = 0.0;
  rlight_point.point.z = 0.0;

  //we'll create a point in the left_light frame that we'd like to transform to the base_link frame
  geometry_msgs::PointStamped llight_point;
  llight_point.header.frame_id = "left_light";

  //we'll just use the most recent transform available for our simple example
  llight_point.header.stamp = ros::Time();

  //just an arbitrary point in space
  llight_point.point.x = 1.0;

  //just an arbitrary point in space
  llight_point.point.x = 1.0;
  llight_point.point.y = 0.0;
  llight_point.point.z = 0.0;



  try{
    geometry_msgs::PointStamped base_point;
    listener.transformPoint("map", map_point, base_point);
    listener.transformPoint("base_link", laser_point, base_point);
    listener.transformPoint("base_link", head1_point, base_point);
    listener.transformPoint("base_link", head2_point, base_point);
    listener.transformPoint("base_link", shol_point, base_point);
    listener.transformPoint("base_link", shor_point, base_point);
    listener.transformPoint("rWheel", rWheel_point, base_point);
    listener.transformPoint("lWheel", lWheel_point, base_point);
    listener.transformPoint("rElbow", rElbow_point, shor_point);
    listener.transformPoint("Elbow", Elbow_point, shol_point);
    listener.transformPoint("rlight", rlight_point, rElbow_point);
    listener.transformPoint("llight", llight_point, Elbow_point);
    listener.transformPoint("odom", odom_point, base_point);

ROS_INFO("laser: (%.2f, %.2f. %.2f) -----> base_link: (%.2f, %.2f, %.2f) at time %.2f", 
        laser_point.point.x, laser_point.point.y, laser_point.point.z,
        base_point.point.x, base_point.point.y, base_point.point.z, base_point.header.stamp.toSec());
  }
  catch(tf::TransformException& ex){
    ROS_ERROR("Received an exception trying to transform a point from \"laser\" to \"base_link\": %s", ex.what());
  }
}

int main(int argc, char** argv){
  ros::init(argc, argv, "robot_tf_listener");
  ros::NodeHandle n;

  tf::TransformListener listener(ros::Duration(10));

  //we'll transform a point once every second
  ros::Timer timer = n.createTimer(ros::Duration(1.0), boost::bind(&transformPoint, boost::ref(listener)));

  ros::spin();

}
