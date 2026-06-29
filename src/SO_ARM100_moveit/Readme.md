# SO_ARM100_moveit

> **MoveIt 2 Configuration for LeRobot SO-ARM100**

[ROS 2](https://www.ros.org/)  
[License: MIT](https://opensource.org/licenses/MIT)  
[MoveIt 2](https://moveit.ros.org/)

---

## 📌 **Overview**

This package provides the **MoveIt 2 configuration** for the **LeRobot SO-ARM100 robotic arm**. It includes:

- **Planning scenes** for RViz and Gazebo.
- **Kinematics, joint limits, and controller configurations** for motion planning.
- **Launch files** to quickly set up MoveIt with the SO-ARM100.
- **SRDF (Semantic Robot Description Format)** for defining robot groups, end effectors, and collision environments.

This package was generated using the **MoveIt Setup Assistant** and is ready for **motion planning, collision avoidance, and trajectory execution** in ROS 2.

---

## 🗂️ **File Structure**

```
SO_ARM100_moveit/
├── CMakeLists.txt              # ROS 2 build configuration
├── config/
│   ├── initial_positions.yaml     # Default joint positions for the robot
│   ├── joint_limits.yaml          # Joint velocity, acceleration, and position limits
│   ├── kinematics.yaml            # Kinematics solver configuration (e.g., KDL, IKFast, RobotArmKinematicsPlugin ( from SO_ARM100_kinematics ) )
│   ├── moveit_controllers.yaml   # MoveIt controller configurations
│   ├── moveit.rviz               # RViz configuration for MoveIt
│   ├── pilz_cartesian_limits.yaml # Cartesian limits for Pilz industrial motion planner
│   ├── ros2_controllers.yaml      # ROS 2 controller configurations
│   ├── sensors_3d.yaml            # 3D sensor (e.g., depth camera) configurations
│   ├── so_arm100.ros2_control.xacro # ROS 2 control configuration (e.g., for `ros2_control`)
│   ├── so_arm100.srdf            # Semantic Robot Description Format (groups, end effectors, collision pairs)
│   └── so_arm100.urdf.xacro       # URDF/Xacro file for the robot (used by MoveIt)
├── launch/
│   ├── demo.launch.py               # Main demo launch file (RViz + MoveIt)
│   ├── move_group.launch.py         # Launch file for the MoveGroup interface
│   ├── moveit_rviz.launch.py        # Launch file for RViz with MoveIt
│   ├── rsp.launch.py               # Launch file for the robot state publisher
│   ├── setup_assistant.launch.py   # Launch file for the MoveIt Setup Assistant
│   ├── spawn_controllers.launch.py # Launch file to spawn ROS 2 controllers
│   ├── static_virtual_joint_tfs.launch.py # Launch file for virtual joint transforms
│   └── warehouse_db.launch.py      # Launch file for the MoveIt warehouse database
└── package.xml                    # ROS 2 package manifest
```

---

## 📝 **File Descriptions**

### **config/ Directory**


| File                                                                  | Description                                                                                           |
| --------------------------------------------------------------------- | ----------------------------------------------------------------------------------------------------- |
| [initial_positions.yaml](config/initial_positions.yaml)             | Defines **default joint positions** (e.g., "home", "folded").                                         |
| [joint_limits.yaml](config/joint_limits.yaml)                       | Specifies **joint limits** (velocity, acceleration, position) for the robot.                          |
| [kinematics.yaml](config/kinematics.yaml)                           | Configures the **kinematics solver** (e.g., KDL, IKFast, RobotArmKinematicsPlugin ( from SO_ARM100_kinematics )) for the robot.                               |
| [moveit_controllers.yaml](config/moveit_controllers.yaml)           | Defines **MoveIt controller configurations** (e.g., for `MoveGroup`).                                 |
| [moveit.rviz](config/moveit.rviz)                                   | RViz configuration file for **visualizing MoveIt planning scenes**.                                   |
| [pilz_cartesian_limits.yaml](config/pilz_cartesian_limits.yaml)     | Cartesian limits for the **Pilz industrial motion planner**.                                          |
| [ros2_controllers.yaml](config/ros2_controllers.yaml)               | ROS 2 **controller configurations** (e.g., for `ros2_control`).                                       |
| [sensors_3d.yaml](config/sensors_3d.yaml)                           | Configurations for **3D sensors** (e.g., depth cameras for collision avoidance).                      |
| [so_arm100.ros2_control.xacro](config/so_arm100.ros2_control.xacro) | ROS 2 control configuration (e.g., for `ros2_control` integration).                                   |
| [so_arm100.srdf](config/so_arm100.srdf)                             | **Semantic Robot Description Format** file. Defines robot groups, end effectors, and collision pairs. |
| [so_arm100.urdf.xacro](config/so_arm100.urdf.xacro)                 | URDF/Xacro file for the robot, used by MoveIt.                                                        |


### **launch/ Directory**


| File                                                                              | Description                                                                         |
| --------------------------------------------------------------------------------- | ----------------------------------------------------------------------------------- |
| [demo.launch.py](launch/demo.launch.py)                                         | **Main demo launch file**. Launches RViz with MoveIt and the robot state publisher. |
| [move_group.launch.py](launch/move_group.launch.py)                             | Launches the **MoveGroup** node for motion planning.                                |
| [moveit_rviz.launch.py](launch/moveit_rviz.launch.py)                           | Launches **RViz with MoveIt** for visualization and planning.                       |
| [rsp.launch.py](launch/rsp.launch.py)                                           | Launches the **robot state publisher** to publish the robot's joint states.         |
| [setup_assistant.launch.py](launch/setup_assistant.launch.py)                   | Launches the **MoveIt Setup Assistant** for reconfiguring the robot.                |
| [spawn_controllers.launch.py](launch/spawn_controllers.launch.py)               | Spawns **ROS 2 controllers** for the robot.                                         |
| [static_virtual_joint_tfs.launch.py](launch/static_virtual_joint_tfs.launch.py) | Publishes **static transforms** for virtual joints (e.g., for fixed bases).         |
| [warehouse_db.launch.py](launch/warehouse_db.launch.py)                         | Launches the **MoveIt warehouse database** for storing and retrieving motion plans. |


---

## 🚀 **Usage**

### **1. Visualize the Robot in RViz with MoveIt**

To launch RViz with the MoveIt planning scene:

```bash
ros2 launch so_arm100_moveit demo.launch.py
```

This will:

- Load the robot description (`so_arm100.urdf.xacro`).
- Start the **robot state publisher** (`rsp.launch.py`).
- Launch **RViz** with the MoveIt configuration (`moveit.rviz`).
- Enable **motion planning** via the MoveGroup interface.

### **2. Plan and Execute Motions**

To use the **MoveGroup interface** for planning and execution:

```bash
ros2 launch so_arm_moveit move_group.launch.py
```

This launches the **MoveGroup** node, which provides:

- **Motion planning** (e.g., `move_group` action server).
- **Kinematics solving** (IK/FK).
- **Collision checking**.

### **3. Spawn ROS 2 Controllers**

To spawn the **ROS 2 controllers** for the robot:

```bash
ros2 launch so_arm_moveit spawn_controllers.launch.py
```

This loads the controllers defined in [ros2_controllers.yaml](config/ros2_controllers.yaml).

### **4. Reconfigure the Robot with MoveIt Setup Assistant**

To modify the MoveIt configuration (e.g., add new planning groups or collision objects):

```bash
ros2 launch so_arm_moveit setup_assistant.launch.py
```

This opens the **MoveIt Setup Assistant GUI**, where you can:

- Edit the **SRDF** (`so_arm100.srdf`).
- Configure **kinematics solvers** (`kinematics.yaml`).
- Define **joint limits** (`joint_limits.yaml`).

### **5. Use the Warehouse Database**

To enable the **MoveIt warehouse database** (for storing/loading motion plans):

```bash
ros2 launch so_arm_moveit warehouse_db.launch.py
```

---

## 🔧 **Customization**

### **Add a New Planning Group**

1. Open the **MoveIt Setup Assistant**:
  ```bash
   ros2 launch so_arm_moveit setup_assistant.launch.py
  ```
2. Go to the **"Planning Groups"** tab.
3. Add a new group (e.g., `arm_with_gripper`) and define its joints.
4. Save the configuration to update the **SRDF** (`so_arm100.srdf`).

### **Modify Kinematics Solver**

Edit the [kinematics.yaml](config/kinematics.yaml) file to change the solver (e.g., from RobotArmKinematicsPlugin to KDL):

```yaml
Arm:
  kinematics_solver: SOArm100/Kinematics/RobotArmKinematicsPlugin
  # kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin
```

### **Adjust Joint Limits**

Edit the [joint_limits.yaml](config/joint_limits.yaml) file to modify joint constraints:

```yaml
joint_limits:
  joint_1:
    has_velocity_limits: true
    max_velocity: 1.0
    has_acceleration_limits: true
    max_acceleration: 0.5
```

---

## 📜 **Dependencies**

- **ROS 2** (Humble, Iron, or Rolling)
- **MoveIt 2**
- `**joint_state_publisher**` (for `rsp.launch.py`)
- `**robot_state_publisher**` (for publishing robot states)
- `**rviz2**` (for visualization)
- `**ros2_control**` (for controller spawning)

---

## 🤝 **Contributing**

- Report issues or suggest improvements via [GitHub Issues](https://github.com/HugoCha/SO-ARM100-ROS2/issues).
- Submit pull requests for new features or bug fixes.

---

## 📞 **Contact**

For questions, reach out to:

- **Hugo Charrier** – [hugo.charrier2009@gmail.com](mailto:hugo.charrier2009@gmail.com)

---

## 📜 **License**

This package is licensed under the **MIT License** – see the [LICENSE](../../LICENSE) file for details.
