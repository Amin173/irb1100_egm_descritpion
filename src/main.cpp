//==================================================================================================
//
// Copyright (c) 2021, ABB Schweiz AG
// All rights reserved.
//
// Redistribution and use in source and binary forms, with
// or without modification, are permitted provided that
// the following conditions are met:
//
//    * Redistributions of source code must retain the
//      above copyright notice, this list of conditions
//      and the following disclaimer.
//    * Redistributions in binary form must reproduce the
//      above copyright notice, this list of conditions
//      and the following disclaimer in the documentation
//      and/or other materials provided with the
//      distribution.
//    * Neither the name of ABB nor the names of its
//      contributors may be used to endorse or promote
//      products derived from this software without
//      specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
// THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
//==================================================================================================

#include <cstdlib>
#include <string>

#include <abb_egm_rws_managers/utilities.h>

#include <ros/ros.h>
#include <angles/angles.h>

#include <abb_robot_msgs/GetRobotControllerDescription.h>

std::string robot_controller_id{""};

void SetJoint(
  abb::robot::StandardizedJoint* p_joint,
  const std::string name,
  const double lower_bound,
  const double upper_bound)
{
  p_joint->set_standardized_name(name);
  p_joint->set_rotating_move(true);
  p_joint->set_lower_joint_bound(angles::from_degrees(lower_bound));
  p_joint->set_upper_joint_bound(angles::from_degrees(upper_bound));
}

bool GetRobotControllerDescription(
  abb_robot_msgs::GetRobotControllerDescription::Request& request,
  abb_robot_msgs::GetRobotControllerDescription::Response& response)
{
  (void) request;

  // Description of ABB robot controller.
  abb::robot::RobotControllerDescription description{};

  // Add header.
  auto header{description.mutable_header()};
  header->mutable_robot_ware_version()->set_major_number(7);
  header->mutable_robot_ware_version()->set_minor_number(3);
  header->mutable_robot_ware_version()->set_patch_number(1);

  // Add system indicators.
  auto system_indicators{description.mutable_system_indicators()};
  system_indicators->mutable_options()->set_egm(true);

  // Add single mechanical units group.
  auto mug{description.add_mechanical_units_groups()};
  mug->set_name("");

  // Add single robot to mechanical units group.
  auto robot{mug->mutable_robot()};
  robot->set_type(abb::robot::MechanicalUnit_Type_TCP_ROBOT);
  robot->set_axes_total(6);
  robot->set_mode(abb::robot::MechanicalUnit_Mode_ACTIVATED);

  // Add joints to robot.
  auto id{robot_controller_id.empty() ? "" : robot_controller_id + "_"};
  auto joint_name{id + "rob1_"};
  SetJoint(robot->add_standardized_joints(), joint_name + std::to_string(1), -230.0, 230.0);
  SetJoint(robot->add_standardized_joints(), joint_name + std::to_string(2), -115.0, 113.0);
  SetJoint(robot->add_standardized_joints(), joint_name + std::to_string(3), -205.0, 55.0);
  SetJoint(robot->add_standardized_joints(), joint_name + std::to_string(4), -230.0, 230.0);
  SetJoint(robot->add_standardized_joints(), joint_name + std::to_string(5), -125.0, 120.0);
  SetJoint(robot->add_standardized_joints(), joint_name + std::to_string(6), -400.0, 400.0);

  // Set respone.
  response.description = description.DebugString();

  return true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rc_description_irb1100_example_node");

  // Node handles.
  ros::NodeHandle nh_params{"~"};
  ros::NodeHandle nh_srvs{"rws"};

  // Get node parameter.
  nh_params.param<std::string>("robot_controller_id", robot_controller_id, "");

  // Setup ROS service.
  ros::ServiceServer service{
    nh_srvs.advertiseService("get_robot_controller_description", GetRobotControllerDescription)};

  // Process ROS events.
  ros::spin();

  return EXIT_SUCCESS;
}
