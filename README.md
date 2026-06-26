# SO-ARM100-ROS2

> **A ROS 2 ecosystem for LeRobot SO-ARM100: Description, Planning, Kinematics, and Control**

[ROS 2](https://www.ros.org/)  
[License: MIT](https://opensource.org/licenses/MIT)  
[LeRobot](https://lerobot.org/)

---

## 📌 **Project Overview**

**SO-ARM100-ROS2** is a **ROS 2** project dedicated to the **LeRobot SO-ARM100**, an open-source robotic arm. This repository provides a **modular, production-ready ecosystem** for simulating, planning, and controlling the SO-ARM100 in ROS 2, with a focus on **motion planning, kinematics, and real-world deployment**.

The project is designed to be **extensible**, **research-friendly**, and **industry-compatible**, making it ideal for both academic and professional robotics applications.

---

## 🎯 **Goals**

- Develop **ROS 2 packages** for seamless integration of the LeRobot SO-ARM100.
- Implement **custom motion planners** (e.g., RRT variants) for efficient trajectory generation.
- Provide **simulation environments** (Gazebo) for testing pick-and-place and other applications.
- Enable **real-world control** of the robot, including motor control and linear axis integration.
- Foster **open-source collaboration** for the LeRobot community.

---

## 🗂️ **Repository Structure**


| Package                                                         | Description                                                     | Status                |
| --------------------------------------------------------------- | --------------------------------------------------------------- | --------------------- |
| `[SO_ARM100_description](SO_ARM100_description/)`               | URDF/Xacro description of the SO-ARM100 robotic arm.            | ✅ **Developed**       |
| `[SO_ARM100_moveit](SO_ARM100_moveit/)`                         | MoveIt planning scene configuration for RViz.                   | ✅ **Developed**       |
| `[SO_ARM100_kinematics](SO_ARM100_kinematics/)`                 | MoveIt plugin for fast kinematic solving.                       | ✅ **Developed**       |
| `[SO_ARM100_planners](SO_ARM100_planners/)`                     | C++/Python MoveIt plugins for custom RRT-based motion planners. | 🚧 **In Development** |
| `[SO_ARM100_planners_prototype](SO_ARM100_planners_prototype/)` | Experimental prototypes for hybrid motion planners.             | 🚧 **In Development** |
| `SO_ARM100_control`                                             | ROS 2 package for motor control and hardware interfacing.       | 📅 **Planned**        |
| `SO_ARM100_gazebo`                                              | Gazebo simulation environment for pick-and-place tasks.         | 📅 **Planned**        |


---

## 🔧 **Key Features**

### ✅ **Developed**

- **URDF/Xacro Models**: Accurate description of the SO-ARM100 arm, compatible with ROS 2 and MoveIt.
- **MoveIt Integration**: Pre-configured planning scenes for RViz, ready for motion planning and collision avoidance.
- **Kinematics Plugin**: Optimized inverse kinematics solver for the SO-ARM100, integrated as a MoveIt plugin.

### 🚧 **In Development**

- **Custom Motion Planners**: Implementation of **RRT**, **RRT-Connect**, and hybrid planners (combining sampling-based methods with heuristics) for efficient path planning.
- **MoveIt Plugins**: PlannerManager and PlanningContext plugins to extend MoveIt’s default capabilities.

### 📅 **Future Work**

- **Gazebo Simulation**: Full simulation environment for testing pick-and-place, collision avoidance, and Cartesian control.
- **Hardware Control**: ROS 2 package (`SO_ARM100_control`) for interfacing with the robot’s motors and a **custom linear axis** (mechanical design + integration).
- **3D Printing**: Physical assembly of the SO-ARM100 for real-world testing.

---

## 🛠️ **Getting Started**

### Prerequisites

- **ROS 2** (Humble, Iron, or Rolling)
- **MoveIt 2**
- **Gazebo** (for simulation)
- **LeRobot SO-ARM100** (hardware or simulation)

### Installation

1. Clone the repository:
  ```bash
   git clone https://github.com/your-username/SO-ARM100-ROS2.git
   cd SO-ARM100-ROS2
  ```
2. Build the workspace:
  ```bash
   colcon build
   source install/setup.bash
  ```
3. Launch a demo (e.g., RViz with MoveIt):
  ```bash
   ros2 launch SO_ARM100_moveit demo.launch.py
  ```

---

## 🤝 **Contributing**

Contributions are welcome! Feel free to:

- Report bugs or suggest features via [GitHub Issues](https://github.com/your-username/SO-ARM100-ROS2/issues).
- Submit pull requests for new planners, improvements, or documentation.
- Share your own SO-ARM100 applications or extensions.

---

## 📜 **License**

This project is licensed under the **MIT License** – see the [LICENSE](LICENSE) file for details.

---

## 📞 **Contact**

For questions or collaboration, reach out to:

- **Hugo Charrier** – [hugo.charrier@example.com](mailto:hugo.charrier@example.com) | [LinkedIn](https://www.linkedin.com/in/your-profile/)
- **LeRobot Community** – [Discord](https://discord.gg/lerobot) | [Forum](https://forum.lerobot.org/)

---

## 🏆 **Acknowledgments**

- [LeRobot](https://lerobot.org/) for the open-source SO-ARM100 design.
- [ROS 2](https://www.ros.org/) and [MoveIt](https://moveit.ros.org/) communities for their invaluable tools and documentation.
