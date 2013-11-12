/*********************************************************************
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
 *********************************************************************/

/** \author Ioan Sucan */

/**

  @mainpage

  @htmlinclude manifest.html

  @b odom_localization Takes in ground truth pose information for a robot
  base (e.g., from a simulator or motion capture system) and republishes
  it as if a localization system were in use.

  <hr>

  @section usage Usage
  @verbatim
  $ fake_localization
  @endverbatim

  <hr>

  @section topic ROS topics

  Subscribes to (name/type):
  - @b "base_pose_ground_truth" nav_msgs/Odometry : robot's odometric pose.  Only the position information is used (velocity is ignored).

  Publishes to (name / type):
  - @b "amcl_pose" geometry_msgs/PoseWithCovarianceStamped : robot's estimated pose in the map, with covariance
  - @b "particlecloud" geometry_msgs/PoseArray : fake set of poses being maintained by the filter (one paricle only).

  <hr>

  @section parameters ROS parameters

  - "~odom_frame_id" (string) : The odometry frame to be used, default: "odom"

 **/

#include <ros/ros.h>
#include <ros/time.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <angles/angles.h>

#include "ros/console.h"

#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"


class FakeOdomNode
{
  public:
    FakeOdomNode(void);
    ~FakeOdomNode(void);
  private:
    ros::NodeHandle m_nh;
    ros::Publisher m_posePub;
    ros::Publisher m_particlecloudPub;
    message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped>* m_initPoseSub;
    tf::TransformBroadcaster       *m_tfServer;
    tf::TransformListener          *m_tfListener;
    tf::MessageFilter<geometry_msgs::PoseWithCovarianceStamped>* m_initPoseFilter;
    tf::MessageFilter<nav_msgs::Odometry>* filter_;
    ros::Subscriber stuff_sub_; 
    message_filters::Subscriber<nav_msgs::Odometry>* filter_sub_;

    double                         delta_x_, delta_y_, delta_yaw_;
    bool                           m_base_pos_received;
    double transform_tolerance_;

    nav_msgs::Odometry  m_basePosMsg;
    geometry_msgs::PoseArray      m_particleCloud;
    geometry_msgs::PoseWithCovarianceStamped      m_currentPos;
    tf::Transform m_offsetTf;

    //parameter for what odom to use
    std::string odom_frame_id_;
    std::string base_frame_id_;
    std::string global_frame_id_;

  public:
    void stuffFilter(const nav_msgs::OdometryConstPtr& odom_msg);
    void update(const nav_msgs::OdometryConstPtr& message);
    void initPoseReceived(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg);
};

