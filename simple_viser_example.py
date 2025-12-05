#!/usr/bin/env python3
"""
Simple Viser Example

Minimal example showing robot visualization with Viser.
Open http://localhost:8080 in your browser to see the robot!
"""

import viser
from viser.extras import ViserUrdf
import yourdfpy
import time
import numpy as np

# 1. Create Viser server
server = viser.ViserServer(port=8080)

# 2. Load URDF with yourdfpy, then pass to ViserUrdf
urdf = yourdfpy.URDF.load("/home/mehul/work/ik_playground/wxai_follower.urdf")
robot = ViserUrdf(server, urdf, root_node_name="/robot")

# 3. Add world coordinate frame
server.scene.add_frame(
    "/world",
    wxyz=(1.0, 0.0, 0.0, 0.0),
    position=(0, 0, 0),
    axes_length=0.2,
    axes_radius=0.01,
)

print("✓ Viser server running!")
print("✓ Open browser to: http://localhost:8080")
print("✓ Press Ctrl+C to exit")

# 4. Animate through some poses
poses = {
    "home": np.array([0.0, 0.5, 0.5, 0.0, 0.0, 0.0]),
    "ready": np.array([0.0, 0.785, 1.047, 0.0, 0.0, 0.0]),
    "stowed": np.array([0.0, 1.571, 1.571, 0.0, 0.0, 0.0]),
}

try:
    while True:
        for name, qpos in poses.items():
            print(f"→ {name}")
            
            # Update robot configuration (ViserUrdf uses numpy arrays)
            robot.update_cfg(qpos)
            
            time.sleep(2)
            
except KeyboardInterrupt:
    print("\n✓ Shutting down...")

