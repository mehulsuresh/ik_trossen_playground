#!/usr/bin/env python3
"""
Simple IK Example with MPlib

A minimal example showing the essential steps to perform IK.
"""

import numpy as np
import mplib

# 1. Create planner with your URDF
planner = mplib.Planner(
    urdf="/home/mehul/work/ik_playground/wxai_follower.urdf",
    srdf="",
    move_group="arm",
)

# 2. Define your controlled joints
controlled_joints = ["joint_0", "joint_1", "joint_2", "joint_3", "joint_4", "joint_5"]
end_effector = "ee_gripper_link"

# 3. Set initial configuration
home_qpos = np.array([0.0, 0.5, 0.5, 0.0, 0.0, 0.0])

# 4. Define target pose (4x4 transformation matrix)
target_pose = np.eye(4)
target_pose[:3, 3] = [0.3, 0.0, 0.2]  # Target position: x=0.3m, y=0.0m, z=0.2m

# 5. Compute IK
solutions = planner.IK(goal_pose=target_pose, current_qpos=home_qpos)

# 6. Use the solution
if solutions:
    print(f"✓ Found {len(solutions)} IK solution(s)!")
    print(f"Joint angles (radians): {solutions[0]}")
    print(f"Joint angles (degrees): {np.degrees(solutions[0])}")
else:
    print("✗ No IK solution found")

