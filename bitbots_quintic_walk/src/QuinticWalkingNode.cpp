#include "bitbots_quintic_walk/QuinticWalkingNode.hpp"


QuinticWalkingNode::QuinticWalkingNode(){        
    // init variables
    _robotState = humanoid_league_msgs::RobotControlState::CONTROLABLE;
    _walkEngine = bitbots_quintic_walk::QuinticWalk();
    _stopRequest = true;    
    walkingReset();
    _isLeftSupport = true;
    _supportFootOdom = tf::Transform();
    tf::Quaternion quat = tf::Quaternion();
    quat.setRPY(0,0,0);
    _supportFootOdom.setRotation(quat);   
    _supportFootOdom.setOrigin(tf::Vector3(0,0,0)); 

    _marker_id = 1;
    _odom_broadcaster = tf::TransformBroadcaster();
    
    // read config
    _nh.param<double>("engineFrequency", _engineFrequency, 100.0);
    _nh.param<bool>("imu_active", _imuActive, false);
    _nh.param<std::string>("walking/ik_type", _ik_type, "bio_ik");
    _nh.param<std::string>("/robot_type_name", _robot_type, "notDefined");
    if(_robot_type != "Minibot" && _robot_type != "Wolfgang"){
        ROS_ERROR("robot_type_name parameter was not specified. Has to be either Minibot or Wolfgang. Will shut down now.");
        exit(1);
    }

    /* init publisher and subscriber */
    _joint_state_msg = sensor_msgs::JointState();
    _pubModelJointState = _nh.advertise<sensor_msgs::JointState>("joint_states", 1);
    _command_msg = std_msgs::Float64MultiArray();
    _pubControllerCommand = _nh.advertise<std_msgs::Float64MultiArray>("/JointGroupController/command", 1);
    _joint_traj_msg = trajectory_msgs::JointTrajectory();

    _pubControllerCommandTraj = _nh.advertise<trajectory_msgs::JointTrajectory>("/walking_motor_goals", 1);
    _odom_msg = nav_msgs::Odometry();
    _pubOdometry = _nh.advertise<nav_msgs::Odometry>("walk_odometry", 1);
    _subCmdVel = _nh.subscribe("/cmd_vel", 1, &QuinticWalkingNode::cmdVelCb, this);
    _subRobState = _nh.subscribe("/robot_state", 1, &QuinticWalkingNode::robStateCb, this);
    _subJointStates = _nh.subscribe("/joint_states", 1, &QuinticWalkingNode::jointStateCb, this);

    /* debug publisher */
    _pubDebug = _nh.advertise<bitbots_quintic_walk::WalkingDebug>("walk_debug", 1);
    _pubDebugMarker = _nh.advertise<visualization_msgs::Marker>("walk_debug_marker", 1);
    
    //load MoveIt! model    
    _robot_model_loader = robot_model_loader::RobotModelLoader("/robot_description", false );
    _robot_model_loader.loadKinematicsSolvers(
            kinematics_plugin_loader::KinematicsPluginLoaderPtr(
                    new kinematics_plugin_loader::KinematicsPluginLoader()));
    _kinematic_model = _robot_model_loader.getModel();
    _all_joints_group = _kinematic_model->getJointModelGroup("All");
    _legs_joints_group = _kinematic_model->getJointModelGroup("Legs");
    _lleg_joints_group = _kinematic_model->getJointModelGroup("LeftLeg");
    _rleg_joints_group = _kinematic_model->getJointModelGroup("RightLeg");
    _goal_state.reset( new robot_state::RobotState( _kinematic_model ));
    _goal_state->setToDefaultValues();
    _current_state.reset(new robot_state::RobotState( _kinematic_model ));
    _current_state->setToDefaultValues();

    // initilize IK solvers
    _bioIK_solver = bitbots_ik::BioIKSolver(*_all_joints_group, *_legs_joints_group, *_lleg_joints_group, *_rleg_joints_group);
    _analytic_solver = bitbots_ik::AnalyticIKSolver(_robot_type, *_lleg_joints_group, *_rleg_joints_group);

    // gravity compensator
    _gravity_compensator = bitbots_quintic_walk::GravityCompensator(_kinematic_model);
    _inverse_controller = std::make_shared<EffortPositionController>();
}


void QuinticWalkingNode::run(){
    /* 
    This is the main loop which takes care of stopping and starting of the walking.
    If the walking is active, computeWalking() is called to compute next motor goals.
    */

   int odom_counter = 0;

    while (ros::ok()){
        ros::Rate loopRate(_engineFrequency);
        if(_walkActive){
            // The robot is currently walking
            if((_robotState != humanoid_league_msgs::RobotControlState::FALLING)){
                // The robot is in the right state, let's compute next motor goals
                // First Update orders
                _walkEngine.setOrders(_orders, true, true);
                // Calculate joint positions
                calculateWalking();
            }else{
                // The HCM changed from state walking to something else.
                // this means, we were forced by the HCM to do something else
                // for example standing up.
                // We reset the walking engine and stop
                walkingReset();
            }
        }else{
            // We're not currently running, test if we want to start
            if(!_stopRequest && (_robotState == humanoid_league_msgs::RobotControlState::CONTROLABLE || _robotState == humanoid_league_msgs::RobotControlState::WALKING
                                 || _robotState == humanoid_league_msgs::RobotControlState::MOTOR_OFF)){
                _walkActive = true;
            }
        }
        odom_counter++;
        if (odom_counter > _odomPubFactor){
            publishOdometry();
            odom_counter = 0;
        }                
        ros::spinOnce();
        loopRate.sleep();
    }
}


void QuinticWalkingNode::walkingReset(){
    /* 
    Resets the walking and stops it *imediatly*. This means that it can also stop during a step, thus in an
    unstable position. Should be normally used when the robot is already falling.
    */
    _orders = {0.0, 0.0, 0.0};
    _walkEngine.setOrders(_orders, false, true);
    _walkActive = false;    
    _just_started = true;
}

void QuinticWalkingNode::calculateWalking(){
    /*
    This method computes the next motor goals as well as the odometry if the step was changed.
    */
    if(_debugActive){
        // save splines to file
        _walkEngine.saveSplineCsv("/tmp/spline_export");
    }
    // save last step odometry if support foot changes
    _stepOdom = _walkEngine.getFootstep().getNext();     

    double dt = 0.01;
    std::chrono::time_point<std::chrono::steady_clock> current_time = std::chrono::steady_clock::now();
    // only take real time difference if walking was not stopped before
    // using c++ time since it is more performant than ros time. We only need a local difference, so it doesnt matter
    if(! _just_started){        
        auto time_diff_ms = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - _last_update_time);
        dt = time_diff_ms.count() / 1000.0;        
        if(dt == 0){
            ROS_WARN("dt was 0");
            dt = 0.001;
        }
    }
    _just_started = false;
    _last_update_time = current_time;
    //ROS_INFO("calc1 dt %f", dt);    
    // compute new values from splines
    _walkEngine.update(dt); //0.005); //todo 1.0/_engineFrequency);
    // read the positions and orientations for trunk and fly foot
    _walkEngine.computeCartesianPosition(_trunkPos, _trunkAxis, _footPos, _footAxis, _isLeftSupport);    
    
    // check if support foot has changed
    if(_isLeftSupport != _wasLeftSupport){
        _wasLeftSupport = _isLeftSupport;
        // add odometry change of last step to trunk odom if step was completed    
        // make transform
        tf::Transform step;
        step.setOrigin(tf::Vector3{_stepOdom[0], _stepOdom[1], 0.0});
        tf::Quaternion tf_quat = tf::Quaternion();
        tf_quat.setRPY(0, 0, _stepOdom[2]);
        step.setRotation(tf_quat);

        // transform global odometry
        _supportFootOdom = _supportFootOdom * step;

        //check if the walking came to a complete stop
        if(_stopRequest && _stepOdom[0] == 0){            
            _walkActive = false;
            _just_started = true;
            return;
        }
    }
    // change goals from support foot based coordinate system to trunk based coordinate system 
    tf::Vector3 tf_vec;
    tf::vectorEigenToTF(_trunkPos, tf_vec);
    tf::Quaternion tf_quat = tf::Quaternion();
    tf_quat.setRPY(_trunkAxis[0], _trunkAxis[1], _trunkAxis[2]);
    tf_quat.normalize();
    tf::Transform support_foot_to_trunk(tf_quat, tf_vec);
    tf::Transform trunk_to_support_foot_goal = support_foot_to_trunk.inverse();

    tf::vectorEigenToTF(_footPos, tf_vec);
    tf_quat.setRPY(_footAxis[0], _footAxis[1], _footAxis[2]);
    tf_quat.normalize();
    tf::Transform support_to_flying_foot(tf_quat, tf_vec);
    tf::Transform trunk_to_flying_foot_goal = trunk_to_support_foot_goal * support_to_flying_foot;

    // call ik solver
    bool success = false;
    if (_ik_type == "bio_ik"){
        success = _bioIK_solver.solve(trunk_to_support_foot_goal, trunk_to_flying_foot_goal, _walkEngine.getFootstep().isLeftSupport(), _goal_state);
    }else if(_ik_type == "analytic"){
        success = _analytic_solver.solve(trunk_to_support_foot_goal, trunk_to_flying_foot_goal, _walkEngine.getFootstep().isLeftSupport(), _goal_state);
    }


    // publish goals if sucessfull
    if(success){
        if(_compensate_gravity){
            compensateGravity();
        }

        std::vector<std::string> joint_names = _legs_joints_group->getActiveJointModelNames();
        std::vector<double> joint_goals;
        _goal_state->copyJointGroupPositions(_legs_joints_group, joint_goals);
        publishControllerCommands(joint_names, joint_goals);
        publishControllerCommandsTraj(joint_names, joint_goals);
        if(_pub_model_joint_states){
            publishModelJointStates(joint_names, joint_goals);
        }
    }
    if(_debugActive){
        publishDebug(trunk_to_support_foot_goal, trunk_to_flying_foot_goal);
        publishMarkers();
    }
}

void QuinticWalkingNode::compensateGravity(){
    // this double represents how the balance of the weight is between the feet
    // 0 if right foot is up, 1 if left foot is up
    double balance_left_right = _walkEngine.getWeightBalance();
    //ROS_INFO("1");
    _goal_state->update();
    for (size_t i = 0; i < _goal_state->getVariableCount(); i++) {
            //ROS_INFO("2");
          _goal_state->setVariableEffort(i, 0.0);
        }
        _gravity_compensator.compensateGravity(
            *_goal_state,
            {{
                std::make_pair("l_foot", 1.0 - balance_left_right),
                std::make_pair("r_foot", 0.0 + balance_left_right),
            }});
        //ROS_INFO("3");
        _goal_state->setVariableEffort("LAnkleRoll", 0);
        _goal_state->setVariableEffort("RAnkleRoll", 0);
        _goal_state->setVariableEffort("LAnklePitch", 0);
        _goal_state->setVariableEffort("RAnklePitch", 0);
        //ROS_INFO("4");
        _inverse_controller->setGoal(*_goal_state);
        //ROS_INFO("5");
}

void QuinticWalkingNode::cmdVelCb(const geometry_msgs::Twist msg){
    // we use only 3 values from the twist messages, as the robot is not capable of jumping or spinning around its
    // other axis
    _orders = {msg.linear.x, msg.linear.y, msg.angular.z} ;
    // deactivate walking if goal is 0 movement, else activate it
    _stopRequest = (msg.linear.x == 0 && msg.linear.y == 0 && msg.angular.z == 0);
}

void QuinticWalkingNode::imuCb(const sensor_msgs::Imu msg){
    if(_imuActive) {
       //todo imu is not active
    }
}
void QuinticWalkingNode::robStateCb(const humanoid_league_msgs::RobotControlState msg){    
    _robotState = msg.state;
}

void QuinticWalkingNode::jointStateCb(const sensor_msgs::JointState msg){
    std::vector<std::string> names_vec = msg.name;
    std::string* names = names_vec.data();    

    _current_state->setJointPositions(*names, msg.position.data());
    //todo
    //_current_state->setJointVelocities(*names, msg.velocity.data());
    //_current_state->setJointEfforts(*names, msg.effort.data());
}

void QuinticWalkingNode::reconf_callback(bitbots_quintic_walk::bitbots_quintic_walk_paramsConfig &config, uint32_t level) {
    _params.setOrAppend("freq", config.freq);
    _params.setOrAppend("doubleSupportRatio", config.doubleSupportRatio);
    _params.setOrAppend("footDistance", config.footDistance);
    _params.setOrAppend("footRise", config.footRise);
    _params.setOrAppend("footPutDownZOffset", config.footPutDownZOffset);
    _params.setOrAppend("footPutDownPhase", config.footPutDownPhase);
    _params.setOrAppend("footApexPhase", config.footApexPhase);
    _params.setOrAppend("footOvershootRatio", config.footOvershootRatio);
    _params.setOrAppend("footOvershootPhase", config.footOvershootPhase);
    _params.setOrAppend("trunkHeight", config.trunkHeight);
    _params.setOrAppend("trunkPitch", config.trunkPitch);
    _params.setOrAppend("trunkPhase", config.trunkPhase);
    _params.setOrAppend("trunkXOffset", config.trunkXOffset);
    _params.setOrAppend("trunkYOffset", config.trunkYOffset);
    _params.setOrAppend("trunkSwing", config.trunkSwing);
    _params.setOrAppend("trunkPause", config.trunkPause);
    _params.setOrAppend("trunkXOffsetPCoefForward", config.trunkXOffsetPCoefForward);
    _params.setOrAppend("trunkXOffsetPCoefTurn", config.trunkXOffsetPCoefTurn);
    _params.setOrAppend("trunkPitchPCoefForward", config.trunkPitchPCoefForward);
    _params.setOrAppend("trunkPitchPCoefTurn", config.trunkPitchPCoefTurn);
    _params.setOrAppend("trunkYOnlyInDoubleSupport", config.trunkYOnlyInDoubleSupport);
    _walkEngine.setParameters(_params);
    if(config.useAnalyticSolver){
        _ik_type = "analytic";
    }else{
        _ik_type = "bio_ik";
    }    
    _bioIK_solver.set_bioIK_timeout(config.bioIKTime);
    _bioIK_solver.set_single_chain(config.bioIKSingleChain);
    _bioIK_solver.set_use_approximate(config.bioIKApprox);
    
    _debugActive = config.debugActive;
    _pub_model_joint_states = config.pubModelJointStates;
    _engineFrequency = config.engineFreq;
    _odomPubFactor = config.odomPubFactor;
    _compensate_gravity = config.compensateGravity;
}


void QuinticWalkingNode::publishModelJointStates(std::vector <std::string> joint_names, std::vector<double> positions){
    _joint_state_msg.position = positions;
    _joint_state_msg.name = joint_names;
    _joint_state_msg.header.stamp = ros::Time::now();
    _pubModelJointState.publish(_joint_state_msg);
}

void QuinticWalkingNode::publishControllerCommands(std::vector <std::string> joint_names, std::vector<double> positions){
    // publishes the commands to the GroupedPositionController
    std::vector<double> positions_ordered;
    bool joint_was_set;
    for(int i =0; i < _joint_ordering.size(); i++){  
        joint_was_set = false;
        for(int j =0; j < joint_names.size(); j++){
            if(_joint_ordering[i] == joint_names[j]){
                positions_ordered.push_back(positions[j]);
                joint_was_set = true;
                break;
            }
        }
        if(!joint_was_set){
            ROS_WARN_ONCE("Not all motor positions are set. The rest will be set to 0. This warning will only be displayed once!");
            positions_ordered.push_back(0.0);
        }
        
    }    

    _command_msg.data = positions_ordered;
    _pubControllerCommand.publish(_command_msg);
}

void QuinticWalkingNode::publishControllerCommandsTraj(std::vector <std::string> joint_names, std::vector<double> positions){
    // publishes the commands to the JointTrajectoryController as single point
    // we only use this for our legacy CM730 code, this is not how it should be done

    _joint_traj_msg.joint_names = joint_names;
     trajectory_msgs::JointTrajectoryPoint point = trajectory_msgs::JointTrajectoryPoint();
     point.positions = positions;  
     point.time_from_start = ros::Duration(0.01);//todo hack
     std::vector<trajectory_msgs::JointTrajectoryPoint> points;     
     points.emplace_back(point);
    _joint_traj_msg.points = points;

    _joint_traj_msg.header.stamp = ros::Time::now();

    _pubControllerCommandTraj.publish(_joint_traj_msg);

}


void QuinticWalkingNode::publishOdometry(){
    // transformation from support leg to trunk
    Eigen::Affine3d trunk_to_support;
    if(_walkEngine.getFootstep().isLeftSupport()){
        trunk_to_support = _goal_state->getGlobalLinkTransform("l_sole");
    }else{
        trunk_to_support = _goal_state->getGlobalLinkTransform("r_sole");
    }
    Eigen::Affine3d support_to_trunk = trunk_to_support.inverse();
    //ROS_INFO("sup to trunk x: %f, y: %f,z: %f", support_to_trunk.translation().x(), support_to_trunk.translation().y(), support_to_trunk.translation().z());
    tf::Transform tf_support_to_trunk;
    tf::transformEigenToTF(support_to_trunk, tf_support_to_trunk);
    tf_support_to_trunk.setRotation(tf_support_to_trunk.getRotation().inverse());

    // odometry to trunk is transform to support foot * transform from support to trunk
    //ROS_WARN("odom x: %f, y: %f,z: %f ", _supportFootOdom.getOrigin()[0],  _supportFootOdom.getOrigin()[1],  _supportFootOdom.getOrigin()[2]);
    tf::Transform odom_to_trunk = _supportFootOdom * tf_support_to_trunk;    
    tf::Vector3 pos = odom_to_trunk.getOrigin();
    geometry_msgs::Quaternion quat_msg;
    
    tf::quaternionTFToMsg(odom_to_trunk.getRotation().normalize(), quat_msg);    

    ros::Time current_time = ros::Time::now();
    _odom_trans = geometry_msgs::TransformStamped();
    _odom_trans.header.stamp = current_time;
    _odom_trans.header.frame_id = "odom";
    _odom_trans.child_frame_id = "base_link";

    _odom_trans.transform.translation.x = pos[0];
    _odom_trans.transform.translation.y = pos[1];
    _odom_trans.transform.translation.z = pos[2];
    _odom_trans.transform.rotation = quat_msg;

    //send the transform
    //todo this kills rviz
    _odom_broadcaster.sendTransform(_odom_trans);

    // send the odometry also as message
    _odom_msg.header.stamp = current_time;
    _odom_msg.header.frame_id = "odom";
    _odom_msg.child_frame_id = "base_link";
    _odom_msg.pose.pose.position.x = pos[0];
    _odom_msg.pose.pose.position.y = pos[1];
    _odom_msg.pose.pose.position.z = pos[2];    

    _odom_msg.pose.pose.orientation = quat_msg;
    geometry_msgs::Twist twist;
    twist.linear.x = _orders[0];
    twist.linear.y = _orders[1];
    twist.angular.z = _orders[2];
    _odom_msg.twist.twist = twist;
    _pubOdometry.publish(_odom_msg);
}

void QuinticWalkingNode::publishDebug(tf::Transform& trunk_to_support_foot_goal, tf::Transform& trunk_to_flying_foot_goal) {
    /*
    This method publishes various debug / visualization information.
    */
    bitbots_quintic_walk::WalkingDebug msg;   
    bool is_left_support = _walkEngine.isLeftSupport();
    msg.is_left_support = is_left_support;
    msg.is_double_support = _walkEngine.isDoubleSupport();
    msg.header.stamp = ros::Time::now();

    // times
    msg.phase_time = _walkEngine.getPhase();    
    msg.traj_time = _walkEngine.getTrajsTime();    

    // engine output
    geometry_msgs::Pose pose_msg;
    tf::pointEigenToMsg(_footPos, pose_msg.position);
    pose_msg.orientation = tf::createQuaternionMsgFromRollPitchYaw(_footAxis[0], _footAxis[1], _footAxis[2]);
    msg.engine_fly_goal = pose_msg;
    tf::pointEigenToMsg(_trunkPos, pose_msg.position);
    pose_msg.orientation = tf::createQuaternionMsgFromRollPitchYaw(_trunkAxis[0], _trunkAxis[1], _trunkAxis[2]);
    msg.engine_trunk_goal = pose_msg;

    // goals
    geometry_msgs::Pose pose_support_foot_goal;
    tf::pointTFToMsg (trunk_to_support_foot_goal.getOrigin(), pose_support_foot_goal.position);
    tf::quaternionTFToMsg (trunk_to_support_foot_goal.getRotation(), pose_support_foot_goal.orientation);
    msg.support_foot_goal = pose_support_foot_goal;
    geometry_msgs::Pose pose_fly_foot_goal;
    tf::pointTFToMsg (trunk_to_flying_foot_goal.getOrigin(), pose_fly_foot_goal.position);
    tf::quaternionTFToMsg (trunk_to_flying_foot_goal.getRotation(), pose_fly_foot_goal.orientation);
    msg.fly_foot_goal = pose_fly_foot_goal;
    if(is_left_support){
        msg.left_foot_goal = pose_support_foot_goal;
        msg.right_foot_goal = pose_fly_foot_goal;
    }else{
        msg.left_foot_goal = pose_fly_foot_goal;
        msg.right_foot_goal = pose_support_foot_goal;        
    }

    // IK results     
    geometry_msgs::Pose pose_left_result;
    tf::poseEigenToMsg (_goal_state->getGlobalLinkTransform("l_sole"), pose_left_result);
    msg.left_foot_ik_result = pose_left_result;
    geometry_msgs::Pose pose_right_result;
    tf::poseEigenToMsg (_goal_state->getGlobalLinkTransform("r_sole"), pose_right_result);
    msg.right_foot_ik_result = pose_right_result;
    if(is_left_support){
        msg.support_foot_ik_result = pose_left_result;
        msg.fly_foot_ik_result = pose_right_result;
    }else{
        msg.support_foot_ik_result = pose_right_result;
        msg.fly_foot_ik_result = pose_left_result;
    }    

    // IK offsets
    tf::Vector3 support_off;
    tf::Vector3 fly_off;
    tf::Vector3 tf_vec_left;
    tf::Vector3 tf_vec_right;
    tf::vectorEigenToTF(_goal_state->getGlobalLinkTransform("l_sole").translation(), tf_vec_left);
    tf::vectorEigenToTF(_goal_state->getGlobalLinkTransform("r_sole").translation(), tf_vec_right);
    geometry_msgs::Vector3 vect_msg;
    if (is_left_support) {
        support_off = trunk_to_support_foot_goal.getOrigin() - tf_vec_left;
        fly_off     = trunk_to_flying_foot_goal.getOrigin()  - tf_vec_right;
        tf::vector3TFToMsg(support_off, vect_msg);
        msg.left_foot_ik_offset = vect_msg;
        tf::vector3TFToMsg(fly_off, vect_msg);
        msg.right_foot_ik_offset = vect_msg;
    }else{
        support_off = trunk_to_support_foot_goal.getOrigin() - tf_vec_right;
        fly_off     = trunk_to_flying_foot_goal.getOrigin()  - tf_vec_left;
        tf::vector3TFToMsg(fly_off, vect_msg);
        msg.left_foot_ik_offset = vect_msg;
        tf::vector3TFToMsg(support_off, vect_msg);
        msg.right_foot_ik_offset = vect_msg;
    }
    tf::vector3TFToMsg(support_off, vect_msg);
    msg.support_foot_ik_offset = vect_msg;
    tf::vector3TFToMsg(fly_off, vect_msg);
    msg.fly_foot_ik_offset = vect_msg;

    // actual positions
    geometry_msgs::Pose pose_left_actual;
    tf::poseEigenToMsg (_current_state->getGlobalLinkTransform("l_sole"), pose_left_actual);
    msg.left_foot_position = pose_left_actual;
    geometry_msgs::Pose pose_right_actual;
    tf::poseEigenToMsg (_current_state->getGlobalLinkTransform("r_sole"), pose_right_actual);
    msg.right_foot_position = pose_right_actual;
    if(is_left_support){
        msg.support_foot_position = pose_left_actual;
        msg.fly_foot_position = pose_right_actual;
    }else{
        msg.support_foot_position = pose_right_actual;
        msg.fly_foot_position = pose_left_actual;
    }    

    // actual offsets
    tf::vectorEigenToTF(_current_state->getGlobalLinkTransform("l_sole").translation(), tf_vec_left);
    tf::vectorEigenToTF(_current_state->getGlobalLinkTransform("r_sole").translation(), tf_vec_right);
    if (is_left_support) {
        support_off = trunk_to_support_foot_goal.getOrigin() - tf_vec_left;
        fly_off     = trunk_to_flying_foot_goal.getOrigin()  - tf_vec_right;
        tf::vector3TFToMsg(support_off, vect_msg);
        msg.left_foot_actual_offset = vect_msg;
        tf::vector3TFToMsg(fly_off, vect_msg);
        msg.right_foot_actual_offset = vect_msg;
    }else{
        support_off = trunk_to_support_foot_goal.getOrigin() - tf_vec_right;
        fly_off     = trunk_to_flying_foot_goal.getOrigin()  - tf_vec_left;
        tf::vector3TFToMsg(fly_off, vect_msg);
        msg.left_foot_actual_offset = vect_msg;
        tf::vector3TFToMsg(support_off, vect_msg);
        msg.right_foot_actual_offset = vect_msg;
    }
    tf::vector3TFToMsg(support_off, vect_msg);
    msg.support_foot_actual_offset = vect_msg;
    tf::vector3TFToMsg(fly_off, vect_msg);
    msg.fly_foot_actual_offset = vect_msg;

    _pubDebug.publish(msg);
}

void QuinticWalkingNode::publishMarkers(){
    //publish markers
    visualization_msgs::Marker marker_msg;
    marker_msg.header.stamp = ros::Time::now();
    if(_walkEngine.getFootstep().isLeftSupport()) {
        marker_msg.header.frame_id = "l_sole";
    }else{
        marker_msg.header.frame_id = "r_sole";
    }
    marker_msg.type = marker_msg.CUBE;
    marker_msg.action = 0;
    marker_msg.lifetime = ros::Duration(0.0);
    geometry_msgs::Vector3 scale;
    scale.x = 0.20;
    scale.y = 0.10;
    scale.z = 0.01;
    marker_msg.scale = scale;
    //last step
    marker_msg.ns = "last_step";
    marker_msg.id = 1;
    std_msgs::ColorRGBA color;
    color.r = 0;
    color.g = 0;
    color.b = 0;
    color.a = 1;
    marker_msg.color = color;
    geometry_msgs::Pose pose;
    Eigen::Vector3d step_pos = _walkEngine.getFootstep().getLast();
    geometry_msgs::Point point;
    point.x = step_pos[0];
    point.y = step_pos[1];
    point.z = 0;
    pose.position = point;
    pose.orientation = tf::createQuaternionMsgFromYaw(step_pos[2]);
    marker_msg.pose = pose;
    _pubDebugMarker.publish(marker_msg);
    // next step
    marker_msg.id = _marker_id;
    marker_msg.ns = "next_step";
    color.r = 1;
    color.g = 1;
    color.b = 1;
    marker_msg.color = color;
    step_pos = _walkEngine.getFootstep().getNext();
    point.x = step_pos[0];
    point.y = step_pos[1];
    pose.position = point;
    pose.orientation = tf::createQuaternionMsgFromYaw(step_pos[2]);
    marker_msg.pose = pose;
    _pubDebugMarker.publish(marker_msg);

    _marker_id++;
    if(_marker_id > 1000){
        _marker_id = 1;
    }

}

int main(int argc, char **argv){
    ros::init(argc, argv, "quintic_walking");
    // init node
    QuinticWalkingNode node = QuinticWalkingNode();
    // set the dynamic reconfigure and load standard params
    dynamic_reconfigure::Server<bitbots_quintic_walk::bitbots_quintic_walk_paramsConfig> server;
    dynamic_reconfigure::Server<bitbots_quintic_walk::bitbots_quintic_walk_paramsConfig>::CallbackType f;
    f = boost::bind(&QuinticWalkingNode::reconf_callback,&node, _1, _2);
    server.setCallback(f);
    // run the node
    node.run();
}

