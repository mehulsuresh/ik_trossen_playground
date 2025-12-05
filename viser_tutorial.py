#!/usr/bin/env python3
"""
Tutorial: Visualizing Robots and IK with Viser

This script demonstrates how to:
1. Load and visualize a URDF robot in Viser
2. Visualize robot configurations
3. Visualize IK solutions interactively
4. Animate robot motion
5. Add interactive controls

Viser is a 3D visualization library that creates a web-based viewer
accessible at http://localhost:8080

Author: Tutorial
Date: 2025-10-31
"""

import numpy as np
import time
import viser
from viser.extras import ViserUrdf
import yourdfpy
import mplib


def basic_viser_setup():
    """
    STEP 1: Basic Viser Setup
    Create a viewer and understand the basics
    """
    print("=" * 70)
    print("STEP 1: Basic Viser Setup")
    print("=" * 70)
    
    # Create a Viser server - this starts a web server
    server = viser.ViserServer(port=8080)
    
    print("\n✓ Viser server started!")
    print("✓ Open your browser to: http://localhost:8080")
    print("✓ You should see a 3D viewer with a grid")
    
    # Add a coordinate frame to show the world origin
    server.scene.add_frame(
        "/world",
        wxyz=(1.0, 0.0, 0.0, 0.0),  # Quaternion (w, x, y, z)
        position=(0, 0, 0),
        axes_length=0.2,
        axes_radius=0.01,
    )
    
    print("\n✓ Added world coordinate frame (RGB = XYZ)")
    
    return server


def load_urdf_in_viser(server):
    """
    STEP 2: Load URDF Robot in Viser
    Display the robot model
    """
    print("\n" + "=" * 70)
    print("STEP 2: Load URDF Robot")
    print("=" * 70)
    
    # Load robot using ViserUrdf
    urdf_path = "/home/mehul/work/ik_playground/wxai_follower.urdf"
    
    # First load with yourdfpy, then pass to ViserUrdf
    urdf = yourdfpy.URDF.load(urdf_path)
    robot_handle = ViserUrdf(server, urdf, root_node_name="/robot")
    
    print(f"\n✓ Loaded URDF: {urdf_path}")
    print("✓ Check your browser - you should see the robot!")
    print("\nControls:")
    print("  • Left click + drag: Rotate view")
    print("  • Right click + drag: Pan view")
    print("  • Scroll: Zoom in/out")
    
    return robot_handle


def visualize_joint_configurations(server, robot_handle):
    """
    STEP 3: Visualize Different Joint Configurations
    Move the robot to different poses
    """
    print("\n" + "=" * 70)
    print("STEP 3: Visualize Joint Configurations")
    print("=" * 70)
    
    # Define some poses
    poses = {
        "home": np.array([0.0, 0.5, 0.5, 0.0, 0.0, 0.0]),
        "ready": np.array([0.0, 0.785, 1.047, 0.0, 0.0, 0.0]),
        "stowed": np.array([0.0, 1.571, 1.571, 0.0, 0.0, 0.0]),
        "upright": np.array([0.0, 1.571, 0.0, 0.0, 0.0, 0.0]),
    }
    
    print("\nCycling through poses (watch in browser)...")
    
    for name, qpos in poses.items():
        print(f"\n  → {name}")
        
        # Update robot configuration (ViserUrdf uses numpy arrays)
        robot_handle.update_cfg(qpos)
        
        time.sleep(2)  # Pause so you can see it
    
    print("\n✓ Cycled through 4 poses!")


def integrate_with_mplib(server, robot_handle):
    """
    STEP 4: Integrate with MPlib
    Compute IK and visualize the solution
    """
    print("\n" + "=" * 70)
    print("STEP 4: Integrate with MPlib for IK")
    print("=" * 70)
    
    # Create MPlib planner
    planner = mplib.Planner(
        urdf="/home/mehul/work/ik_playground/wxai_follower.urdf",
        srdf="",
        move_group="arm",
    )
    
    print("\n✓ Created MPlib planner")
    
    # Define a target pose
    target_position = np.array([0.3, 0.15, 0.2])
    target_pose = np.eye(4)
    target_pose[:3, 3] = target_position
    
    # Visualize the target position
    server.scene.add_icosphere(
        "/target",
        radius=0.02,
        color=(1.0, 0.0, 0.0),  # Red
        position=tuple(target_position),
    )
    
    print(f"✓ Target position: {target_position}")
    print("✓ Red sphere shows target in viewer")
    
    # Compute IK
    home_qpos = np.array([0.0, 0.5, 0.5, 0.0, 0.0, 0.0])
    solutions = planner.IK(goal_pose=target_pose, current_qpos=home_qpos)
    
    if solutions:
        print(f"\n✓ Found {len(solutions)} IK solution(s)")
        
        # Visualize the solution
        solution = solutions[0]
        
        print("\n  Moving robot to IK solution...")
        robot_handle.update_cfg(solution)
        
        # Verify with forward kinematics
        planner.robot.set_qpos(solution)
        ee_pose = planner.robot.get_link_pose("ee_gripper_link")
        ee_position = ee_pose[:3, 3]
        
        # Show end-effector position
        server.scene.add_icosphere(
            "/end_effector",
            radius=0.015,
            color=(0.0, 1.0, 0.0),  # Green
            position=tuple(ee_position),
        )
        
        error = np.linalg.norm(ee_position - target_position)
        print(f"✓ Position error: {error*1000:.2f} mm")
        print("✓ Green sphere shows actual end-effector position")
        
    else:
        print("✗ No IK solution found")
    
    return planner


def animate_trajectory(server, robot_handle, planner):
    """
    STEP 5: Animate Robot Motion
    Smoothly move robot along a trajectory
    """
    print("\n" + "=" * 70)
    print("STEP 5: Animate Trajectory")
    print("=" * 70)
    
    # Generate a straight-line trajectory in task space
    start_pos = np.array([0.25, -0.1, 0.15])
    end_pos = np.array([0.25, 0.1, 0.15])
    num_points = 20
    
    print(f"\nGenerating trajectory: {num_points} waypoints")
    print(f"  Start: {start_pos}")
    print(f"  End:   {end_pos}")
    
    # Clear previous markers
    try:
        server.scene.remove("/target")
        server.scene.remove("/end_effector")
    except:
        pass
    
    # Show trajectory as line
    trajectory_points = []
    for i in range(num_points):
        alpha = i / (num_points - 1)
        pos = (1 - alpha) * start_pos + alpha * end_pos
        trajectory_points.append(pos)
    
    # Draw trajectory line
    for i in range(len(trajectory_points) - 1):
        server.scene.add_spline_catmull_rom(
            f"traj_segment_{i}",
            positions=np.array([trajectory_points[i], trajectory_points[i+1]]),
            color=(0.5, 0.5, 1.0),  # Light blue
            segments=2,
        )
    
    # Compute IK for each waypoint
    print("\n✓ Computing IK for trajectory...")
    home_qpos = np.array([0.0, 0.5, 0.5, 0.0, 0.0, 0.0])
    current_qpos = home_qpos
    
    ik_solutions = []
    for i, pos in enumerate(trajectory_points):
        target = np.eye(4)
        target[:3, 3] = pos
        
        solutions = planner.IK(goal_pose=target, current_qpos=current_qpos)
        if solutions:
            # Choose closest to current
            best = min(solutions, key=lambda s: np.linalg.norm(s - current_qpos))
            ik_solutions.append(best)
            current_qpos = best
        else:
            print(f"  ✗ Failed at waypoint {i}")
            return
    
    print(f"✓ IK solved for all {len(ik_solutions)} waypoints")
    
    # Animate!
    print("\n✓ Animating... (watch in browser)")
    
    for i, qpos in enumerate(ik_solutions):
        robot_handle.update_cfg(qpos)
        
        # Show current target
        server.scene.add_icosphere(
            "/current_target",
            radius=0.015,
            color=(1.0, 0.5, 0.0),  # Orange
            position=tuple(trajectory_points[i]),
        )
        
        time.sleep(0.1)  # Smooth animation
    
    print("✓ Animation complete!")


def interactive_controls(server, robot_handle):
    """
    STEP 6: Add Interactive Controls
    Use sliders to control joint angles
    """
    print("\n" + "=" * 70)
    print("STEP 6: Interactive Controls")
    print("=" * 70)
    
    print("\n✓ Adding interactive sliders...")
    print("✓ Check the sidebar in your browser!")
    
    # Joint limits from URDF
    joint_limits = [
        (-3.054, 3.054),   # joint_0
        (0.0, 3.142),      # joint_1
        (0.0, 2.356),      # joint_2
        (-1.571, 1.571),   # joint_3
        (-1.571, 1.571),   # joint_4
        (-3.142, 3.142),   # joint_5
    ]
    
    # Create sliders for each joint
    joint_sliders = []
    
    with server.gui.add_folder("Joint Control"):
        for i, (lower, upper) in enumerate(joint_limits):
            slider = server.gui.add_slider(
                f"Joint {i}",
                min=lower,
                max=upper,
                step=0.01,
                initial_value=0.0,
            )
            joint_sliders.append(slider)
    
    # Update function
    @server.gui.on_update
    def update_from_sliders():
        """Update robot when sliders change"""
        joint_positions = {
            f"joint_{i}": slider.value 
            for i, slider in enumerate(joint_sliders)
        }
        robot_handle.update_cfg(joint_positions)
    
    print("✓ Interactive controls ready!")
    print("\nYou can now:")
    print("  • Use sliders to control each joint")
    print("  • See the robot update in real-time")


def preset_buttons(server, robot_handle):
    """
    STEP 7: Add Preset Pose Buttons
    Quick buttons to move to common poses
    """
    print("\n" + "=" * 70)
    print("STEP 7: Preset Pose Buttons")
    print("=" * 70)
    
    poses = {
        "Home": np.array([0.0, 0.5, 0.5, 0.0, 0.0, 0.0]),
        "Ready": np.array([0.0, 0.785, 1.047, 0.0, 0.0, 0.0]),
        "Stowed": np.array([0.0, 1.571, 1.571, 0.0, 0.0, 0.0]),
        "Upright": np.array([0.0, 1.571, 0.0, 0.0, 0.0, 0.0]),
    }
    
    with server.gui.add_folder("Preset Poses"):
        for name, qpos in poses.items():
            button = server.gui.add_button(name)
            
            @button.on_click
            def _on_click(qpos=qpos):
                joint_positions = {
                    f"joint_{i}": float(angle) 
                    for i, angle in enumerate(qpos)
                }
                robot_handle.update_cfg(joint_positions)
    
    print("✓ Preset buttons added!")
    print("✓ Click buttons in sidebar to jump to poses")


def main():
    print("=" * 70)
    print("Viser Visualization Tutorial")
    print("=" * 70)
    
    print("""
Viser is a powerful 3D visualization library for robotics.
It creates an interactive web-based viewer where you can:
  • Visualize URDF robots
  • See robot configurations in real-time
  • Display IK solutions
  • Animate trajectories
  • Add interactive controls

This tutorial will show you all of these features!
    """)
    
    input("Press Enter to start (then open http://localhost:8080 in your browser)...")
    
    try:
        # Step 1: Basic setup
        server = basic_viser_setup()
        input("\nPress Enter to load robot...")
        
        # Step 2: Load robot
        robot_handle = load_urdf_in_viser(server)
        input("\nPress Enter to cycle through poses...")
        
        # Step 3: Show different poses
        visualize_joint_configurations(server, robot_handle)
        input("\nPress Enter to demonstrate IK...")
        
        # Step 4: IK integration
        planner = integrate_with_mplib(server, robot_handle)
        input("\nPress Enter to animate trajectory...")
        
        # Step 5: Trajectory animation
        animate_trajectory(server, robot_handle, planner)
        input("\nPress Enter to add interactive controls...")
        
        # Step 6: Interactive controls
        interactive_controls(server, robot_handle)
        input("\nPress Enter to add preset buttons...")
        
        # Step 7: Preset buttons
        preset_buttons(server, robot_handle)
        
        print("\n" + "=" * 70)
        print("Tutorial Complete!")
        print("=" * 70)
        print("""
✓ You've learned how to:
  1. Create a Viser server
  2. Load URDF robots
  3. Visualize joint configurations
  4. Integrate with MPlib for IK
  5. Animate trajectories
  6. Add interactive controls
  7. Create preset pose buttons

The viewer is still running. Try:
  • Moving the sliders
  • Clicking preset buttons
  • Rotating the view
  
Press Ctrl+C to exit.
        """)
        
        # Keep server running
        while True:
            time.sleep(0.1)
            
    except KeyboardInterrupt:
        print("\n\n✓ Shutting down...")
    except Exception as e:
        print(f"\n✗ Error: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()

