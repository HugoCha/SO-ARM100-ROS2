# SO_ARM100_description

> **URDF/Xacro Description Package for LeRobot SO-ARM100**

[ROS 2](https://www.ros.org/)  
[License: MIT](https://opensource.org/licenses/MIT)

---

## 📌 **Overview**

This package provides the **URDF (Unified Robot Description Format)** and **Xacro** files for the **LeRobot SO-ARM100 robotic arm**. It includes:

- The **3D model** of the robot arm, with meshes for each component.
- **Xacro macros** for modular and reusable robot descriptions.
- **Transmission definitions** for motor control.
- **RViz configurations** for visualization.

---

## 🗂️ **File Structure**

```
SO_ARM100_description/
├── CMakeLists.txt          # ROS 2 build configuration
├── config/                # Additional configuration files (if any)
├── launch/
│   └── display.launch.py   # Launch file to visualize the robot in RViz
├── meshes/                # STL files for 3D visualization
│   ├── Base_Motor.stl
│   ├── Base.stl
│   ├── Fixed_Jaw_Motor.stl
│   ├── Fixed_Jaw.stl
│   ├── Lower_Arm_Motor.stl
│   ├── Lower_Arm.stl
│   ├── Moving_Jaw.stl
│   ├── Rotation_Pitch_Motor.stl
│   ├── Rotation_Pitch.stl
│   ├── Upper_Arm_Motor.stl
│   ├── Upper_Arm.stl
│   ├── Wrist_Pitch_Roll_Motor.stl
│   └── Wrist_Pitch_Roll.stl
├── package.xml            # ROS 2 package manifest
├── rviz/
│   └── display_config.rviz # RViz configuration for visualizing the robot
└── urdf/
    ├── grippers/
    │   ├── gripper_jaw.xacro    # Xacro file for the gripper jaw
    │   └── gripper_none.xacro   # Xacro file for no gripper (placeholder)
    ├── materials.xacro     # Defines materials for rendering
    ├── so_arm100.urdf.xacro # Main Xacro file for the SO-ARM100 robot model (without tool)
    ├── transmissions.xacro # Defines motor transmissions
    └── utils.xacro         # Utility macros for joints, links, and inertia
```

---

## 📝 **File Descriptions**

### **urdf/ Directory**


| File                                                | Description                                                                                                          |
| --------------------------------------------------- | -------------------------------------------------------------------------------------------------------------------- |
| [materials.xacro](urdf/materials.xacro)           | Defines **materials** (colors, textures) for rendering the robot in RViz/Gazebo.                                     |
| [so_arm100.urdf.xacro](urdf/so_arm100.urdf.xacro) | **Main Xacro file** for the SO-ARM100 robot model. Describes the robot structure **without a tool** (e.g., gripper). |
| [transmissions.xacro](urdf/transmissions.xacro)   | Defines **motor transmissions** (e.g., joint-to-motor mappings).                                                     |
| [utils.xacro](urdf/utils.xacro)                   | **Utility macros** to simplify the definition of joints, links, and inertia properties.                              |


### **grippers/ Directory**


| File                                                     | Description                                                    |
| -------------------------------------------------------- | -------------------------------------------------------------- |
| [gripper_jaw.xacro](urdf/grippers/gripper_jaw.xacro)   | Xacro file for the **gripper jaw** (if a gripper is attached). |
| [gripper_none.xacro](urdf/grippers/gripper_none.xacro) | Xacro file for **no gripper** (placeholder for end-effector).  |


### **meshes/ Directory**

Contains **STL files** for each physical component of the robot:

- Base, Lower Arm, Upper Arm, Wrist, and Gripper parts.
- Motor meshes for visualization.

### **launch/ Directory**


| File                                            | Description                                                                    |
| ----------------------------------------------- | ------------------------------------------------------------------------------ |
| [display.launch.py](launch/display.launch.py) | Launch file to **visualize the robot in RViz** with the default configuration. |


### **rviz/ Directory**


| File                                              | Description                                                  |
| ------------------------------------------------- | ------------------------------------------------------------ |
| [display_config.rviz](rviz/display_config.rviz) | RViz configuration file for **visualizing the robot model**. |


---

## 🚀 **Usage**

### Visualize the Robot in RViz

1. Build the package:
  ```bash
   colcon build --packages-select SO_ARM100_description
   source install/setup.bash
  ```
2. Launch the RViz display:
  ```bash
   ros2 launch so_arm100_description display.launch.py
  ```

### Load the URDF in Your Own Node

To load the robot description in a custom ROS 2 node, use the following:

```python
from ament_index_python.packages import get_package_share_directory
import os

# Load the URDF from the Xacro file
urdf_path = os.path.join(
    get_package_share_directory('so_arm100_description'),
    'urdf',
    'so_arm100.urdf.xacro'
)
```

---

## 🔧 **Customization**

### Adding a New Gripper

1. Create a new Xacro file in the `grippers/` directory (e.g., `my_gripper.xacro`).
2. Include it in your main URDF/Xacro file:
  ```xml
   <xacro:include filename="$(find SO_ARM100_description)/urdf/grippers/my_gripper.xacro" />
  ```
3. Update the `so_arm100.urdf.xacro` to use your new gripper.

### Modifying Materials

Edit the `[materials.xacro](urdf/materials.xacro)` file to change colors or textures:

```xml
<material name="red_plastic">
  <color rgba="0.8 0.1 0.1 1.0"/>
</material>
```

---

## 📜 **Dependencies**

- **ROS 2** (Humble, Iron, or Rolling)
- **urdf** package
- **xacro** package
- **RViz2** (for visualization)

---

## 🤝 **Contributing**

- Report issues or suggest improvements via [GitHub Issues](https://github.com/your-username/SO-ARM100-ROS2/issues).
- Submit pull requests for new features or bug fixes.

---

## 📞 **Contact**

For questions, reach out to:

- **Hugo Charrier** – [hugo.charrier@example.com](mailto:hugo.charrier2009@gmail.com)

---

## 📜 **License**

This package is licensed under the **MIT License** – see the [LICENSE](../../LICENSE) file for details.
