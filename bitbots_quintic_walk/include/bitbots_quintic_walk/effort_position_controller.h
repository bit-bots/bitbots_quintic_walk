// Walking, 2018, Philipp Ruppel

#pragma once

#include "common.h"
#include <moveit/kinematics_base/kinematics_base.h>
#include <moveit_msgs/RobotState.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <tf/transform_broadcaster.h>


struct EffortPositionController {
  std::vector<std::string> joint_names;
  std::vector<double> goal, current, prev, efforts;
  ros::NodeHandle node;
  std_msgs::Float64MultiArray msg;
  RobotStatePublisher rsp = RobotStatePublisher("ik");
  ros::Publisher pub = node.advertise<std_msgs::Float64MultiArray>(
      "/joint_group_position_controller/command", 4);

  EffortPositionController() {
    //node.getParamCached("/wolves/controller/joints", joint_names);
    joint_names = {"LHipYaw","LHipRoll","LHipPitch","LKnee","LAnklePitch","LAnkleRoll","RHipYaw","RHipRoll","RHipPitch","RKnee","RAnklePitch","RAnkleRoll"};
    current.resize(joint_names.size(), 0.0);
    goal.resize(joint_names.size(), 0.0);
    msg.data.resize(joint_names.size(), 0.0);
    prev.resize(joint_names.size(), 0.0);
    efforts.resize(joint_names.size(), 0.0);
    msg.layout.dim.emplace_back();
    msg.layout.dim[0].size = joint_names.size();
    msg.layout.dim[0].stride = 1;
  }

  double p = 50;

  void setGoal(const moveit::core::RobotState &state) {
    for (size_t i = 0; i < joint_names.size(); i++) {
      goal[i] = state.getVariablePosition(joint_names[i]);
      efforts[i] = state.getVariableEffort(joint_names[i]);
    }
    rsp.publish(state);
    for (size_t i = 0; i < joint_names.size(); i++) {
      msg.data[i] = goal[i] + efforts[i] / p;
    }
    pub.publish(msg);
  }
};
