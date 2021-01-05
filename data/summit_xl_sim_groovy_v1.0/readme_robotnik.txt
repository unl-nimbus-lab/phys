
summit_xl_description   : robot description urdf, sdf rviz
summit_xl_gazebo        : launch files and world files to start the models in gazebo
summit_xl_control       : new gazebo 1.9 style robot control
summit_xl_joystick      : node to process the joystick in simulation
summit_xl_robot_control : control the robot joints in all kinematic configurations, publishes odom topic and, if configured, also tf odom->base_link 
summit_xl_2dnav         : launch and configuration files of gmapping, amcl and move_base
summit_xl_waypoints     : pass a set of goals from a file to the move_base stack (still ROSBUILD due to dep on move_base_msgs, action)

summit_xl_joint_state   : test node to publish joint states (as alternative to the joint_state_publisher)



