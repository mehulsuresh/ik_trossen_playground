#!/usr/bin/env python3
"""
Tutorial: Loading URDF and Performing Inverse Kinematics with MPlib

This script demonstrates how to:
1. Load a URDF file into mplib
2. Set up a planning context
3. Perform inverse kinematics (IK)
4. Work with different end-effector poses
5. Handle multiple IK solutions

Author: Tutorial
Date: 2025-10-31
"""

import numpy as np
import mplib

def main():
    print("=" * 70)
    print("MPLIB Inverse Kinematics Tutorial")
    print("=" * 70)
    
    # =========================================================================
    # STEP 1: Load the URDF and Create a Planner
    # =========================================================================
    print("\n[STEP 1] Loading URDF and creating planner...")
    
    # Define the path to your URDF file
    urdf_path = "/home/mehul/work/ik_playground/wxai_follower.urdf"
    
    # Create a planner instance
    # - urdf: path to the URDF file
    # - srdf: optional SRDF file (we don't have one, so use empty string)
    # - move_group: name of the joints to control (we'll use the arm joints)
    planner = mplib.Planner(
        urdf=urdf_path,
        srdf="",  # No SRDF file needed for basic IK
        move_group="arm",  # We'll define this below
    )
    
    # IMPORTANT: For the wxai robot, we need to specify which joints to control
    # The arm has 6 revolute joints: joint_0 through joint_5
    # We can specify these using the user_link_names parameter
    
    print("✓ Planner created successfully!")
    
    # =========================================================================
    # STEP 2: Better Approach - Create Planner with Explicit Joint Names
    # =========================================================================
    print("\n[STEP 2] Creating planner with explicit joint configuration...")
    
    # Define the joints we want to control (the 6 arm joints)
    controlled_joints = [
        "joint_0",  # Base rotation (around Z)
        "joint_1",  # Shoulder pitch (around Y)
        "joint_2",  # Elbow pitch (around Y)
        "joint_3",  # Wrist pitch (around Y)
        "joint_4",  # Wrist roll (around Z)
        "joint_5",  # Wrist twist (around X)
    ]
    
    # The end-effector link (where we want to compute IK for)
    end_effector_link = "ee_gripper_link"
    
    # Create the planner with explicit configuration
    planner = mplib.Planner(
        urdf=urdf_path,
        srdf="",
        user_link_names=controlled_joints,  # Which joints to control
        user_joint_names=controlled_joints,  # Which joints to control
        move_group="arm",
    )
    
    print(f"✓ Controlling {len(controlled_joints)} joints")
    print(f"✓ End-effector: {end_effector_link}")
    
    # =========================================================================
    # STEP 3: Understanding the Robot's Current Configuration
    # =========================================================================
    print("\n[STEP 3] Understanding robot configuration...")
    
    # Set a home/neutral configuration for the robot
    # These are joint angles in radians for [joint_0, joint_1, ..., joint_5]
    home_qpos = np.array([0.0, 0.5, 0.5, 0.0, 0.0, 0.0])
    
    print(f"Home configuration (radians): {home_qpos}")
    
    # Get the forward kinematics (FK) - where is the end-effector given these joint angles?
    # This tells us the position and orientation of the end-effector
    planner.robot.set_qpos(home_qpos)
    ee_pose = planner.robot.get_link_pose(end_effector_link)
    
    print(f"\nEnd-effector pose at home configuration:")
    print(f"  Position (x, y, z): {ee_pose[:3, 3]}")
    print(f"  Rotation matrix:\n{ee_pose[:3, :3]}")
    
    # =========================================================================
    # STEP 4: Performing Inverse Kinematics (IK)
    # =========================================================================
    print("\n[STEP 4] Performing Inverse Kinematics...")
    
    # Let's define a target pose for the end-effector
    # Format: 4x4 transformation matrix [R|t] where R is 3x3 rotation, t is 3x1 position
    
    # Example 1: Simple target - just move forward in X direction
    target_position = np.array([0.3, 0.0, 0.2])  # 30cm forward, 20cm up
    target_orientation = np.eye(3)  # No rotation (identity matrix)
    
    # Create the 4x4 transformation matrix
    target_pose = np.eye(4)
    target_pose[:3, :3] = target_orientation
    target_pose[:3, 3] = target_position
    
    print("\nTarget pose:")
    print(f"  Position: {target_position}")
    print("  Orientation: Identity (no rotation)")
    
    # Compute IK
    # - target_pose: desired end-effector pose (4x4 matrix)
    # - current_qpos: current joint configuration (helps find nearby solutions)
    # Returns: List of IK solutions (each solution is an array of joint angles)
    ik_solutions = planner.IK(
        goal_pose=target_pose,
        current_qpos=home_qpos,
    )
    
    if len(ik_solutions) > 0:
        print(f"\n✓ Found {len(ik_solutions)} IK solution(s)!")
        
        # Display the first solution
        solution = ik_solutions[0]
        print(f"\nFirst solution (radians):")
        for i, angle in enumerate(solution):
            print(f"  joint_{i}: {angle:+.4f} rad ({np.degrees(angle):+.2f}°)")
        
        # Verify the solution by computing forward kinematics
        planner.robot.set_qpos(solution)
        achieved_pose = planner.robot.get_link_pose(end_effector_link)
        achieved_position = achieved_pose[:3, 3]
        
        print("\nVerification (FK of IK solution):")
        print(f"  Target position:   {target_position}")
        print(f"  Achieved position: {achieved_position}")
        print(f"  Position error:    {np.linalg.norm(achieved_position - target_position):.6f} m")
    else:
        print("\n✗ No IK solution found for this target pose!")
    
    # =========================================================================
    # STEP 5: Working with Different Orientations
    # =========================================================================
    print("\n[STEP 5] IK with different orientations...")
    
    # Let's create a target pose with a specific orientation
    # We'll use Euler angles and convert to rotation matrix
    from scipy.spatial.transform import Rotation as R
    
    # Define orientation using Euler angles (roll, pitch, yaw) in radians
    roll = 0.0
    pitch = np.pi / 4  # 45 degrees
    yaw = 0.0
    
    rotation = R.from_euler('xyz', [roll, pitch, yaw]).as_matrix()
    
    target_pose_2 = np.eye(4)
    target_pose_2[:3, :3] = rotation
    target_pose_2[:3, 3] = [0.25, 0.1, 0.25]  # Position
    
    print(f"Target with rotation:")
    print(f"  Position: {target_pose_2[:3, 3]}")
    print(f"  Euler angles (deg): roll={np.degrees(roll):.1f}, pitch={np.degrees(pitch):.1f}, yaw={np.degrees(yaw):.1f}")
    
    ik_solutions_2 = planner.IK(
        goal_pose=target_pose_2,
        current_qpos=home_qpos,
    )
    
    if len(ik_solutions_2) > 0:
        print(f"✓ Found {len(ik_solutions_2)} solution(s) for rotated pose")
        
        # Show all solutions (robots often have multiple ways to reach the same pose)
        for idx, sol in enumerate(ik_solutions_2):
            print(f"\n  Solution {idx + 1}:")
            print(f"    Joint angles (deg): {np.degrees(sol)}")
    else:
        print("✗ No solution found for rotated pose")
    
    # =========================================================================
    # STEP 6: Understanding IK Constraints and Limitations
    # =========================================================================
    print("\n[STEP 6] Understanding IK constraints...")
    
    # Try an unreachable pose (too far away)
    unreachable_pose = np.eye(4)
    unreachable_pose[:3, 3] = [2.0, 0.0, 0.0]  # Way too far!
    
    print(f"\nTrying unreachable target: {unreachable_pose[:3, 3]}")
    
    ik_unreachable = planner.IK(
        goal_pose=unreachable_pose,
        current_qpos=home_qpos,
    )
    
    if len(ik_unreachable) == 0:
        print("✓ Correctly determined this pose is unreachable!")
    else:
        print("✗ Unexpectedly found a solution (robot may be more capable than expected)")
    
    # =========================================================================
    # STEP 7: Tips and Best Practices
    # =========================================================================
    print("\n" + "=" * 70)
    print("TIPS AND BEST PRACTICES")
    print("=" * 70)
    
    tips = [
        "1. Always provide a good initial guess (current_qpos) close to expected solution",
        "2. Check joint limits in the URDF - IK respects these limits",
        "3. Multiple solutions exist - choose based on criteria (joint limits, smoothness, etc.)",
        "4. IK may fail for poses at the edge of workspace or with difficult orientations",
        "5. Use Forward Kinematics (FK) to verify IK solutions",
        "6. Consider adding collision checking for real robot applications",
        "7. The end-effector frame matters - make sure you're using the right link!",
    ]
    
    for tip in tips:
        print(f"  {tip}")
    
    # =========================================================================
    # STEP 8: Interactive Example - Random Reachable Poses
    # =========================================================================
    print("\n[STEP 8] Testing with random reachable poses...")
    
    num_tests = 5
    success_count = 0
    
    for i in range(num_tests):
        # Generate random joint configuration (within limits)
        # These are approximate limits from the URDF
        random_qpos = np.array([
            np.random.uniform(-3.0, 3.0),    # joint_0
            np.random.uniform(0.0, 3.14),     # joint_1
            np.random.uniform(0.0, 2.35),     # joint_2
            np.random.uniform(-1.57, 1.57),   # joint_3
            np.random.uniform(-1.57, 1.57),   # joint_4
            np.random.uniform(-3.14, 3.14),   # joint_5
        ])
        
        # Get the FK for this random configuration
        planner.robot.set_qpos(random_qpos)
        target = planner.robot.get_link_pose(end_effector_link)
        
        # Try to solve IK back to this pose
        solutions = planner.IK(goal_pose=target, current_qpos=home_qpos)
        
        if len(solutions) > 0:
            success_count += 1
            print(f"  Test {i+1}: ✓ Found {len(solutions)} solution(s)")
        else:
            print(f"  Test {i+1}: ✗ No solution found")
    
    print(f"\nSuccess rate: {success_count}/{num_tests}")
    
    print("\n" + "=" * 70)
    print("Tutorial complete! You now know how to:")
    print("  - Load a URDF file with mplib")
    print("  - Configure the planner with specific joints")
    print("  - Compute inverse kinematics for target poses")
    print("  - Work with positions and orientations")
    print("  - Understand IK solutions and limitations")
    print("=" * 70)


if __name__ == "__main__":
    main()

