# Robot IK with MPlib + Viser

Learn inverse kinematics and 3D visualization for robotics.

## Quick Start

```bash
# Install
pip install -r requirements.txt

# Test IK
python simple_ik_example.py

# Test visualization
python simple_viser_example.py
# Open http://localhost:8080

# Full tutorial
python ik_tutorial.py
python viser_tutorial.py
```

## Files

**Python Scripts:**
- `simple_ik_example.py` - Minimal IK example (25 lines)
- `simple_viser_example.py` - Minimal visualization (40 lines)
- `ik_tutorial.py` - Complete IK tutorial with 8 steps
- `viser_tutorial.py` - Visualization tutorial with 5 steps

**Config:**
- `wxai_follower.urdf` - Robot description
- `wxai_follower.srdf` - Optional semantic info
- `requirements.txt` - Dependencies

## Examples

### Solve IK

```python
import numpy as np
import mplib

# Setup
planner = mplib.Planner(
    urdf="wxai_follower.urdf",
    srdf="",
    move_group="arm"
)

# Target position
target = np.eye(4)
target[:3, 3] = [0.3, 0.0, 0.2]  # x, y, z in meters

# Solve IK
home = np.array([0.0, 0.5, 0.5, 0.0, 0.0, 0.0])
solutions = planner.IK(goal_pose=target, current_qpos=home)

if solutions:
    print(f"Joint angles: {solutions[0]}")
```

### Visualize Robot

```python
import viser
from viser.extras import ViserUrdf
import yourdfpy
import numpy as np

# Create server
server = viser.ViserServer(port=8080)

# Load robot (need yourdfpy)
urdf = yourdfpy.URDF.load("wxai_follower.urdf")
robot = ViserUrdf(server, urdf)

# Set joint angles
robot.update_cfg(np.array([0.0, 0.5, 0.5, 0.0, 0.0, 0.0]))

# Open http://localhost:8080 in browser
input("Press Enter to exit...")
```

### Combine IK + Visualization

```python
import viser
from viser.extras import ViserUrdf
import yourdfpy
import mplib
import numpy as np

# Setup
server = viser.ViserServer()
urdf = yourdfpy.URDF.load("wxai_follower.urdf")
robot_vis = ViserUrdf(server, urdf)
planner = mplib.Planner(urdf="wxai_follower.urdf", srdf="", move_group="arm")

# Solve IK
target = np.eye(4)
target[:3, 3] = [0.3, 0.0, 0.2]
solutions = planner.IK(goal_pose=target, current_qpos=np.array([0, 0.5, 0.5, 0, 0, 0]))

# Visualize
if solutions:
    robot_vis.update_cfg(solutions[0])
    print("Open http://localhost:8080")
    input("Press Enter to exit...")
```

## Your Robot (wxai_follower)

- 6 revolute joints for arm
- 2 prismatic joints for gripper
- ~50cm reach

Joint limits:
- joint_0: -175° to +175° (base rotation)
- joint_1: 0° to +180° (shoulder)
- joint_2: 0° to +135° (elbow)
- joint_3: -90° to +90° (wrist pitch)
- joint_4: -90° to +90° (wrist roll)
- joint_5: -180° to +180° (wrist twist)

## Troubleshooting

**"No IK solution"** - Target out of reach, try closer positions

**"Can't connect to Viser"** - Make sure script is running, open http://localhost:8080

**Import errors** - Run `pip install -r requirements.txt`

## Learn More

- MPlib docs: https://motion-planning-lib.readthedocs.io/
- Viser docs: https://viser.studio/latest/

That's it! Start with `simple_ik_example.py` and `simple_viser_example.py`.


https://github.com/i2rt-robotics/i2rt/tree/main
