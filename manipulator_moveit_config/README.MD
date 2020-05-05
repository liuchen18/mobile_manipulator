manipulator_gazeo.launch: spawn model and controllers into gazebo and run moveit at the same time

gazebo.launch: spawn model and controllers(position controller) into gazebo .the gazebo.launch of robot_description has velocity controllers 

manipulator_control.launch: run manipulator_move node and manipulator_state node. the arg is_into_txt decides whether the node write the state of the manipulator into the txt file.if write the the arg is write.the default of the arg is not.the manipulator_move node requires the move_type: desired or noise_added or control. just change the arg in the launch file





----------------------------------------------------------------------------------------------------------------------------------
the robot model in gazebo is controlled by VELOCITY controller.BUT we still need the jacobian matrix provided by moveit.so now we have the moveit_robot_model.launch:

start the robot_model in moveit and run moveit: roslaunch manipulator_moveit_config fake_moveit_robot_model.launch
