# SO_ARM100_kinematics

> **Custom Kinematics Solver Plugin for LeRobot SO-ARM100**

[ROS 2](https://www.ros.org/)  
[License: MIT](https://opensource.org/licenses/MIT)

---

## 📌 **Overview**

This package provides a **custom kinematics solver plugin** for the **LeRobot SO-ARM100 robotic arm** in ROS 2. It is designed to **solve inverse kinematics (IK) efficiently** by combining **numerical methods, heuristic approaches, and topology-aware strategies**. The solver is implemented as a **MoveIt plugin**, allowing seamless integration with the ROS 2 ecosystem.

The plugin supports:

- **Numerical IK solvers** (e.g., Damped Least Squares, FABRIK).
- **Heuristic solvers** tailored to the robot's topology (e.g., planar joints, revolute/prismatic bases, spherical/universal wrists).
- **Pipeline-based solving** to combine seeding, heuristics, and numerical methods.
- **Scoring mechanisms** to evaluate and rank IK solutions.

---

## 🗂️ **Package Structure**

### **Core Components**

#### **Solvers**

- **DLS**: A **Damped Least Squares (DLS)** solver for numerical IK computation. Includes adaptive damping and step size control for stability and convergence.
- **FABRIK**: A **Forward And Backward Reaching Inverse Kinematics (FABRIK)** solver, a heuristic method for position solving.
- **Heuristic**: Topology-aware solvers for specific robot configurations:
  - **Base solvers**: For revolute or prismatic bases.
  - **Planar solvers**: For planar joints (1R, 2R, CCD, NR).
  - **Wrist solvers**: For spherical, universal, or revolute wrists.
- **UniversalSolver**: Specialized solver for **universal articulations** (e.g., Cardan joints).
- **SphericalSolver**: Specialized solver for **spherical articulations** (e.g., ball joints).


#### **Model**

- **Geometry**: Defines geometric primitives (e.g., lines, planes, poses, spheres) for kinematic computations.
- **Joint**: Models joints, joint chains, and their states (e.g., position, velocity, limits).
- **KinematicTopology**: Represents the **arm topology** (e.g., elbow/wrist configurations).
- **ReachableSpace**: Tools to **quickly test if a pose is reachable** (e.g., line, plane, sphere, or chain-based reachability).
- **Skeleton**: Alternative representation of the arm using **bones and articulations**, particularly useful for FABRIK.

#### **Pipeline**

- **PipelineSolver**: Combines **seeding, heuristics, and numerical solvers** into a unified pipeline for robust IK solving.
- **Seed**: Generates **initial seeds** for the solver (e.g., random, opposite, or intelligent seeds).
- **Scorer**: Evaluates IK solutions using metrics like **manipulability, pose error, or consistency with the seed**.

#### **Utilities**

- **ModelAnalyzer**: Automatically **analyzes the arm topology** to determine the best solving strategy.
- **Utils**: Helper classes for **conversions, math, and kinematics utilities**.

---

## 🚀 **Usage**

### **1. Build the Package**

Ensure you have **ROS 2** and **MoveIt 2** installed. Then, build the package:

```bash
colcon build --packages-select so_arm100_kinematics
source install/setup.bash
```

### **2. Load the Plugin in MoveIt**

The plugin is automatically registered with MoveIt via the `so_arm100_kinematics_plugin.xml` file. To use it:

1. Ensure your **MoveIt configuration** (e.g., in `so_arm100_moveit`) references this plugin in the `kinematics.yaml` file:
  ```yaml
  Arm:
    kinematics_solver: SOArm100/Kinematics/RobotArmKinematicsPlugin
  ```
2. Launch MoveIt with your configuration:
  ```bash
   ros2 launch so_arm100_moveit demo.launch.py
  ```

### **3. Use the Solver Programmatically**

You can also use the solver directly in your C++ code:

```cpp
#include <RobotArmKinematicsSolver.hpp>

// Create a solver instance
auto solver = std::make_shared<so_arm100_kinematics::RobotArmKinematicsSolver>();

// Set the robot model (e.g., from URDF)
solver->setRobotModel(robot_model);

// Solve IK for a target pose
std::vector<double> joint_values;
bool success = solver->solveIK(target_pose, joint_values);
```

---

## 🔧 **Customization**

### **Add a New Heuristic Solver**

1. Create a new class in the `Heuristic` directory (e.g., `MyCustomHeuristic.hpp` and `MyCustomHeuristic.cpp`).
2. Inherit from the `IIKHeuristic` interface and implement the required methods:
  ```cpp
   class MyCustomHeuristic : public SOARM100::Kinematics::Heuristic::IIKHeuristic {
   public:
       IKPresolution Presolve( const Solver::IKProblem& problem, const Solver::IKRunContext& context ) const 
   };
  ```
3. Register the heuristic in the `KinematicTopology`.

### **Modify the Pipeline**

Edit the `PipelineSolver` to change the order or priority of solvers:

```cpp
// Example: Add a custom solver to the pipeline
	auto pipeline = Solver::PipelineBuilder{}
  	.WithHeuristic( std::make_unique< Heuristic::MyCustomHeuristic >( model ) ) // model : Model::KinematicModelConstPtr
  	.WithSolver( std::make_unique< Solver::MyCustomSolver >( model ) ) // model : Model::KinematicModelConstPtr
  	.Build();
```

### **Adjust Scoring Metrics**

Customize the `Scorer` classes to prioritize different metrics (e.g., manipulability, joint limits):

```cpp
// Example: Create a weighted scorer
	auto scorer = Scorer::WeightedScorersBuilder{}
  	.Add( 1.0, std::make_unique< Scorer::CloseToCenterScorer >( model ) )
  	.Add( 1.0, std::make_unique< Scorer::CloseToSeedScorer >( model ) )
  	.Add( 1.0, std::make_unique< Scorer::SeedConsistencyScorer >( std::numeric_limits< double >::infinity() ) )
  	.Add( 1.0, std::make_unique< Scorer::ManipulabilityScorer >( model ) )
  	.Add( 1.0, std::make_unique< Scorer::PoseErrorScorer >( model, Scorer::PoseErrorScorer::ScorerParameters() ) )
  	.Build();
```

---

## 🧪 **Testing**

The package includes a **comprehensive test suite** in the `test/` directory. To run the tests:

```bash
colcon test --packages-select SO_ARM100_kinematics
```

Tests cover:

- **Solver correctness** (DLS, FABRIK, heuristics).
- **Model validation** (joints, topology, reachable space).
- **Pipeline integration** (seeding, scoring, solving).
- **Utility functions** (conversions, math, kinematics).

---

## 📜 **Dependencies**

- **ROS 2** (Humble, Iron, or Rolling)
- **MoveIt 2**
- **Eigen3** (for linear algebra)
- **Boost** (for utilities)

---

## 🤝 **Contributing**

- Report issues or suggest improvements via [GitHub Issues](https://github.com/HugoCha/SO-ARM100-ROS2/issues).
- Submit pull requests for new solvers, heuristics, or optimizations.

---

## 📞 **Contact**

For questions, reach out to:

- **Hugo Charrier** – [hugo.charrier2009@gmail.com](mailto:hugo.charrier2009@gmail.com)

---

## 📜 **License**

This package is licensed under the **MIT License** – see the [LICENSE](../../LICENSE) file for details.
