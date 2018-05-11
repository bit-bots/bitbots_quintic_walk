/*
This code is largely based on the original code by Quentin "Leph" Rouxel and Team Rhoban.
The original files can be found at:
https://github.com/Rhoban/model/
*/
#ifndef QUINTICWALKNODE_HPP
#define QUINTICWALKNODE_HPP

#include <iostream>
#include <string>
#include <Eigen/Dense>
#include <chrono>

#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <moveit_msgs/RobotState.h>
#include <humanoid_league_msgs/RobotControlState.h>
#include <bitbots_quintic_walk/WalkingDebug.h>

#include <dynamic_reconfigure/server.h>
#include <eigen_conversions/eigen_msg.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/kinematics_base/kinematics_base.h>
#include <moveit/move_group_interface/move_group_interface.h> 

#include <bitbots_quintic_walk/bitbots_quintic_walk_paramsConfig.h>
#include "bitbots_ik/AnalyticIKSolver.hpp"
#include "bitbots_ik/BioIKSolver.hpp"
#include "bitbots_quintic_walk/QuinticWalk.hpp"
#include "bitbots_quintic_walk/VectorLabel.hpp"
#include "bitbots_quintic_walk/gravity_compensator.h"
#include "bitbots_quintic_walk/effort_position_controller.h"


#include "swri_profiler/profiler.h"


class QuinticWalkingNode {
public:
    QuinticWalkingNode();
    void run();
    void reconf_callback(bitbots_quintic_walk::bitbots_quintic_walk_paramsConfig &config, uint32_t level);

private:
    void publishModelJointStates(std::vector <std::string> joint_names, std::vector<double> positions);
    void publishControllerCommands(std::vector <std::string> joint_names, std::vector<double> positions);
    void publishControllerCommandsTraj(std::vector <std::string> joint_names, std::vector<double> positions);
    void publishDebug(tf::Transform& trunk_to_support_foot, tf::Transform& trunk_to_flying_foot);
    void publishMarkers();
    void publishTrajectoryDebug();
    void publishOdometry();
    void cmdVelCb(const geometry_msgs::Twist msg);
    void imuCb(const sensor_msgs::Imu msg);
    void robStateCb(const humanoid_league_msgs::RobotControlState msg);
    void jointStateCb(const sensor_msgs::JointState msg);
    void walkingReset();
    void calculateWalking();
    void compensateGravity();

    //std::vector<std::string> _joint_ordering {"head_yaw","head_pitch","left_shoulder_pitch","left_shoulder_roll","left_elbow","right_shoulder_pitch","right_shoulder_roll","right_elbow","left_hip_yaw","left_hip_roll","left_hip_pitch","left_knee","left_ankle_pitch","left_ankle_roll","right_hip_yaw","right_hip_roll","right_hip_pitch","right_knee","right_ankle_pitch","right_ankle_roll"};
    std::vector<std::string> _joint_ordering {"HeadYaw","HeadPitch","LShoulderPitch","LShoulderRoll","LElbow","RShoulderPitch","RShoulderRoll","RElbow","LHipYaw","LHipRoll","LHipPitch","LKnee","LAnklePitch","LAnkleRoll","RHipYaw","RHipRoll","RHipPitch","RKnee","RAnklePitch","RAnkleRoll"};
    std::vector<std::string> _left_leg_joint_names {"LHipYaw", "LHipRoll", "LHipPitch", "LKnee", "LAnklePitch", "LAnkleRoll"};
    std::vector<std::string> _right_leg_joint_names {"RHipYaw", "RHipRoll", "RHipPitch", "RKnee", "RAnklePitch", "RAnkleRoll"};

    bool _debugActive;
    bool _pub_model_joint_states;
    bool _imuActive;
    bool _walkActive;
    bool _stopRequest;
    double _engineFrequency;
    int _odomPubFactor;
    std::chrono::time_point<std::chrono::steady_clock> _last_update_time;
    bool _just_started;

    int _robotState;
    int _marker_id;

    Eigen::Vector3d _stepOdom;
    tf::Transform _supportFootOdom;

    std::string _ik_type;
    std::string _robot_type;

    Eigen::Vector3d _trunkPos;
    Eigen::Vector3d _trunkAxis;
    Eigen::Vector3d _footPos;
    Eigen::Vector3d _footAxis;
    bool _isLeftSupport;
    bool _wasLeftSupport;
    
    bool _compensate_gravity;
    bitbots_quintic_walk::GravityCompensator _gravity_compensator;
    std::shared_ptr<EffortPositionController> _inverse_controller;


    bitbots_quintic_walk::VectorLabel _params;
    Eigen::Vector3d _orders;
    bitbots_quintic_walk::QuinticWalk _walkEngine;

    sensor_msgs::JointState _joint_state_msg;
    std_msgs::Float64MultiArray _command_msg;
    nav_msgs::Odometry _odom_msg;
    geometry_msgs::TransformStamped _odom_trans;
    trajectory_msgs::JointTrajectory _joint_traj_msg;
    moveit_msgs::DisplayTrajectory _traj_display_msg;

    ros::NodeHandle _nh;
    ros::Publisher _pubModelJointState;
    ros::Publisher _pubControllerCommand;
    ros::Publisher _pubControllerCommandTraj;
    ros::Publisher _pubOdometry;
    tf::TransformBroadcaster _odom_broadcaster;
    ros::Subscriber _subCmdVel;
    ros::Subscriber _subRobState;
    ros::Subscriber _subJointStates;


    ros::Publisher _pubDebug;
    ros::Publisher _pubDebugMarker;

    // MoveIt!
    robot_model_loader::RobotModelLoader _robot_model_loader;
    robot_model::RobotModelPtr _kinematic_model;
    robot_state::RobotStatePtr _goal_state;
    robot_state::RobotStatePtr _current_state;
    const robot_state::JointModelGroup *_all_joints_group;
    const robot_state::JointModelGroup *_legs_joints_group;
    const robot_state::JointModelGroup *_lleg_joints_group;
    const robot_state::JointModelGroup *_rleg_joints_group;
    
    // IK solver
    bitbots_ik::BioIKSolver _bioIK_solver;
    bitbots_ik::AnalyticIKSolver _analytic_solver;

    bool _ik_x_offset;

};

#endif