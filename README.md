# GPT Control for TIAGo Robot

A ROS 2 package that enables natural language control of the TIAGo mobile manipulator using OpenAI's GPT models. The robot can navigate and manipulate objects. This is an educational example for a high school lesson.

## Features

- **Natural Language Control**: Command the TIAGo robot
- **Arm Manipulation**: Cartesian and joint-space control with MoveIt2
- **Object Grasping**: Automated pick-and-place with coordinate frame transformations
- **Base Navigation**: Move and rotate the mobile base
- **Gazebo Integration**: Simulated object attachment/detachment using Link Attacher plugin

## Prerequisites

### System Requirements
- **ROS 2 Humble** (tested on Ubuntu 22.04)
- **Python 3.10+**
- **TIAGo Simulation Packages** (PAL Robotics)

### Dependencies

### ROS 2 Humble
Follow tutorial on https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html 

#### ROS 2 Packages
```bash
sudo apt install ros-humble-moveit ros-humble-pymoveit2
```

#### Python Libraries
```bash
pip install openai numpy scipy
```

### TIAGo Robot Simulation
Follow installation procedure on https://github.com/pal-robotics/tiago_simulation.git 

### Link Attacher/Detacher
Follow installation procedure on https://github.com/IFRA-Cranfield/IFRA_LinkAttacher.git 

## Installation of the package

1. **Source the workspace**:
    ```bash
    cd ~/tiago_public_ws
    source install/setup.bash
    ```

1. **Clone the repository** into your workspace:
   ```bash
   cd ~/tiago_public_ws/src
   git clone https://github.com/federicobiagi/tiago_gpt_control.git gpt_control
   ```

2. **Build the package**:
   ```bash
   colcon build
   source install/setup.bash
   ```

## Configuration

### OpenAI API Key

Export your API key as an environment variable:
```bash
export OPENAI_Personal_Key="sk-your-api-key-here"
```

Or add it to your `~/.bashrc`:
```bash
echo 'export OPENAI_Personal_Key="sk-your-api-key-here"' >> ~/.bashrc
source ~/.bashrc
```
### Gazebo World Setup
Copy and paste the table.world file into:
install/pal_gazebo_worlds/share/pal_gazebo_worlds/worlds/

Copy and paste the table.world file into:
src/pal_gazebo_worlds/worlds

### Colcon build again
```bash
cd ~/tiago_public_ws/
colcon build
source install/setup.bash
```

## Usage

### 1. Launch the Simulation

Start the TIAGo simulation with the table world:

```bash
ros2 launch tiago_gazebo tiago_gazebo.launch.py is_public_sim:=True world_name:=table
```

Wait for Gazebo to fully load and the robot to initialize.

### 2. Run the GPT Control Node

In a new terminal:

```bash
source ~/tiago_public_ws/install/setup.bash
ros2 run gpt_control gpt_manager
```

### 3. Command the Robot

Once running, you can give natural language commands:

```
User: Move forward 0.5 meters
User: Rotate 90 degrees
User: Grab the cocacola
User: Move the arm to position [0, 0.3, -1.3, 1.5, 0, 0, 0]
```

## Architecture

### Main Components

#### `gpt_manager_node.py`
- Interfaces with OpenAI's GPT API
- Manages conversation history and tool calling
- Translates natural language to robot actions

#### `tiago_arm_control_clean.py`
- Core robot control library
- Implements navigation, manipulation, and grasping primitives
- Handles coordinate frame transformations (world → base_footprint)

#### `tools.json`
- Defines available functions for GPT tool calling
- JSON schema for OpenAI function calling API

### Key Functions

| Function | Description |
|----------|-------------|
| `move_distance(distance, speed)` | Move base forward/backward |
| `rotate_angle(angle, angular_speed)` | Rotate base in place |
| `move_arm(joint_positions)` | Move arm to joint configuration |
| `move_gripper(x, y, z, quat)` | Move gripper in Cartesian space |
| `grasp_object_by_name_front(object_name)` | Automated front grasping sequence |
| `open_gripper()` / `close_gripper()` | Control gripper fingers |
| `get_object_position_in_robot_frame(name)` | Get object coordinates relative to robot |


## Troubleshooting

### "ImportError: cannot import name 'MoveIt2'"
Install pymoveit2:
```bash
sudo apt install ros-humble-pymoveit2
```

### "Service /ATTACHLINK not available"
Ensure the Link Attacher plugin is loaded in your Gazebo world. Check terminal output for:
```
[INFO] [gazebo_link_attacher]: Link attacher node initialized.
```

### "OpenAI API key not found"
Verify your environment variable:
```bash
echo $OPENAI_Personal_Key
```

### Robot collides with table
Adjust the `height_offset` parameter in `grasp_object_by_name_front()`:
```python
self.grasp_object_by_name_front("object_name", height_offset=0.20)
```

## Example Session

```bash
# Terminal 1: Launch simulation
ros2 launch tiago_gazebo tiago_gazebo.launch.py is_public_sim:=True world_name:=table

# Terminal 2: Run control node
ros2 run gpt_control gpt_manager

# Interactive commands:
User: Move towards the table
User: Pick up the coca cola can
User: Lift the object higher
User: Put it down
```

## License

This project is developed for research purposes.

## Acknowledgments

- **PAL Robotics** for the TIAGo simulation packages
- **IFRA Cranfield** for the Link Attacher plugin
- **OpenAI** for GPT API


**Note**: This package requires an active OpenAI API key and may incur usage costs. Monitor your API usage through the OpenAI dashboard.
