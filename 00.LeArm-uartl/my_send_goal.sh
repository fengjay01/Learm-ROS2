#!/bin/bash

# 发送 FollowJointTrajectory 目标
ros2 action send_goal /arm_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{
  trajectory: {
    joint_names: [Arm_Joint1,Arm_Joint2,Arm_Joint3,Arm_Joint4,Arm_Joint5],
    points: [
      { positions: [0.1, 0.1, 0.1, 0.1, 0.1], time_from_start: { sec: 0, nanosec: 500 } },
      { positions: [0.2, 0.5, 0.2 ,0.2, 0.2], time_from_start: { sec: 5, nanosec: 500 } },
      { positions: [0.3, 0.3, 0.7, -0.5, 0.3], time_from_start: { sec: 7, nanosec: 500 } },
      { positions: [0.5, 0.5, 0.5, 0.5, 0.5], time_from_start: { sec: 8, nanosec: 500 } }
    ]
  }
}"
