# 🤖 6-DOF Robotic Arm Simulation

## CodeAlpha Internship — Task 2: Robotic Arm Simulation

A complete **6-DOF robotic arm simulation project** designed in **Onshape**, integrated into **MuJoCo**, and controlled using **Python and NumPy**.

The project combines **3D CAD modeling, robotic kinematics, inverse kinematics, Cartesian/TCP control, motion recording and playback, and basic pick-and-place manipulation**.

The current system is also designed as a foundation for future **autonomous robotic manipulation**, including:

* 🤖 Robotic Gripper
* 📷 Camera Integration
* 👁️ Computer Vision / OpenCV
* 🧠 Reinforcement Learning
* 🎯 Vision-Based Pick-and-Place
* 🚀 Autonomous Manipulation

---

# 📌 Project Overview

The robotic arm was initially designed and modeled as a 3D CAD assembly using **Onshape**.

The CAD geometry was then exported as STL meshes and integrated into a **MuJoCo 3.12.0** simulation model.

The robot is controlled using Python and NumPy, with implementations for:

* Joint-space control
* Forward kinematics
* Jacobian calculation
* Numerical inverse kinematics
* Cartesian motion
* TCP/tool-space control
* Pick-and-place
* Motion recording
* Motion playback

The overall development pipeline is:

```text
                    Onshape
                 3D CAD Design
                       │
                       ▼
                   STL Meshes
                       │
                       ▼
                MuJoCo Robot Model
                       │
                       ▼
                Python + NumPy
                       │
          ┌────────────┼────────────┐
          ▼            ▼            ▼
      Kinematics       IK       TCP Control
          │            │            │
          └────────────┼────────────┘
                       ▼
                Cartesian Motion
                       │
                       ▼
                 Pick & Place
                       │
                       ▼
             Motion Recording
                       │
                       ▼
               Motion Playback
```

---

# ⭐ Major Features

## 1. 6-DOF Robotic Arm

The project contains a complete six-degree-of-freedom robotic arm.

```text
Joint 1
   │
Joint 2
   │
Joint 3
   │
Joint 4
   │
Joint 5
   │
Joint 6
   │
  TCP
```

Joint names:

```text
joint_1
joint_2
joint_3
joint_4
joint_5
joint_6
```

Total:

```text
6 DOF
```

---

# ⭐ 2. CAD Design Using Onshape

The robotic arm was designed as a 3D mechanical assembly using **Onshape**.

The CAD design includes:

* Base
* Multiple robotic links
* Six rotational joints
* End-effector/tool components
* TCP/tool reference

The CAD geometry was exported and integrated into the simulation.

```text
Onshape
   ↓
3D Assembly
   ↓
Individual Components
   ↓
STL Export
   ↓
MuJoCo / ROS2 Integration
```

Mesh files are stored inside:

```text
src/robotic_arm_description/meshes/
```

---

# ⭐ 3. MuJoCo Simulation

The primary robotic simulation is implemented using:

```text
MuJoCo 3.12.0
```

Main model:

```text
mujoco/robotic_arm.xml
```

The MuJoCo model contains:

* Robot bodies
* Joints
* Joint limits
* Visual geometry
* Collision geometry
* TCP definition
* Robot configuration
* Simulation parameters

MuJoCo provides the physics and simulation environment used for testing robotic motion.

---

# ⭐ 4. Forward Kinematics

Forward kinematics calculates the TCP position and orientation from the robot's joint configuration.

```text
Joint Angles
     │
     ▼
Robot Kinematic Model
     │
     ▼
Forward Kinematics
     │
     ▼
TCP Position
+
TCP Orientation
```

Testing script:

```text
mujoco/test_fk.py
```

---

# ⭐ 5. Jacobian Calculation

The robot Jacobian is used to relate joint motion to Cartesian motion.

```text
Joint Velocity
      │
      ▼
   Jacobian
      │
      ▼
Cartesian Velocity
```

Related scripts:

```text
mujoco/test_jacobian.py
mujoco/verify_jacobian.py
```

---

# ⭐ 6. Numerical Inverse Kinematics

Numerical inverse kinematics is used to calculate the required joint configuration for a desired TCP position.

```text
Desired TCP Position
         │
         ▼
 Inverse Kinematics
         │
         ▼
   Joint Angles
         │
         ▼
    Robot Motion
```

Main files:

```text
mujoco/ik_solver.py
mujoco/ik_motion_demo.py
mujoco/ik_visualize.py
```

This provides the mathematical foundation required for autonomous target reaching and future vision-based manipulation.

---

# ⭐ 7. Cartesian / TCP Control

The robotic arm can be controlled using the Tool Center Point.

Instead of manually specifying every joint angle, a target can be defined in Cartesian space.

```text
Cartesian Target
       │
       ▼
      TCP
       │
       ▼
Inverse Kinematics
       │
       ▼
Joint Configuration
       │
       ▼
Robot Motion
```

This approach provides the foundation for:

* Target reaching
* Object approach
* Pick-and-place
* Vision-guided positioning
* Autonomous manipulation

---

# ⭐ 8. Motion Recording and Playback

## 🔥 Major Project Feature

One of the major features of this project is the ability to **record robotic-arm motion and reproduce it through playback**.

The motion recording system captures the robot's movement during simulation and stores the trajectory for later use.

Main implementation:

```text
mujoco/motion_recorder.py
```

The concept is:

```text
Robot Controller
       │
       ▼
Joint Motion
       │
       ▼
MuJoCo Simulation
       │
       ▼
Motion Recorder
       │
       ▼
Saved Trajectory
```

The recorded trajectory can then be used for playback:

```text
Saved Trajectory
       │
       ▼
Motion Playback
       │
       ▼
Joint Commands
       │
       ▼
6-DOF Robotic Arm
       │
       ▼
Reproduced Motion
```

### Why Motion Recording Is Important

Motion recording is more than a debugging feature.

It provides a foundation for:

* Repeating robotic movements
* Saving successful manipulation trajectories
* Motion analysis
* Trajectory comparison
* Demonstration-based robotics
* Creating training datasets
* Learning from demonstrations
* Reinforcement-learning data generation
* Trajectory optimization
* Future sim-to-real development

The long-term idea is:

```text
Manual / Programmed Motion
          │
          ▼
     Motion Recording
          │
          ▼
    Trajectory Dataset
          │
          ▼
 Machine Learning / RL
          │
          ▼
 Autonomous Robot Policy
```

This makes the motion-recording system an important bridge between **traditional programmed robotics and future learning-based robotics**.

---

# ⭐ 9. Pick-and-Place

A basic pick-and-place workflow is implemented as part of the current project.

Conceptually:

```text
Home
  │
  ▼
Approach Object
  │
  ▼
Pick
  │
  ▼
Lift
  │
  ▼
Move
  │
  ▼
Place
  │
  ▼
Return
```

Run the demonstration using:

```bash
python3 mujoco/pick_place_demo.py
```

The current pick-and-place system provides the foundation for future gripper-based and vision-based manipulation.

---

# 🦾 Future: Robotic Gripper

The next major development stage is the integration of a dedicated robotic gripper.

Planned capabilities include:

* Gripper modeling
* Gripper joints
* Open/close control
* Object contact
* Grasping
* Object release
* Collision-aware grasping
* Pick-and-place with physical gripping
* Gripper control through ROS2

Future workflow:

```text
6-DOF Arm
    +
Robotic Gripper
    │
    ▼
Object Manipulation
```

The current end-effector/tool structure provides the starting point for this development.

---

# 📷 Future: Camera and Computer Vision

A simulated camera will be integrated into the robotic environment.

Computer vision will be developed using **OpenCV**.

Planned capabilities include:

* Camera simulation
* Image acquisition
* Image processing
* Object detection
* Object localization
* Object tracking
* Object position estimation
* Camera-to-robot coordinate transformation
* Vision-based target generation
* Visual servoing

Future vision pipeline:

```text
Camera
   │
   ▼
Image
   │
   ▼
OpenCV
   │
   ▼
Object Detection
   │
   ▼
Object Localization
   │
   ▼
Target Position
   │
   ▼
Robot
```

---

# 🧠 Future: Reinforcement Learning

Reinforcement Learning will be introduced to move the project toward **learning-based robotic control**.

Potential applications include:

* Robot reaching
* Motion optimization
* Target reaching
* Pick-and-place learning
* Grasping policies
* Reward-based control
* Trajectory optimization
* Autonomous manipulation
* Simulation-based training
* Policy evaluation in MuJoCo
* Future sim-to-real experimentation

The planned RL architecture is:

```text
             Robot State
                  │
                  ▼
              RL Policy
                  │
                  ▼
             Robot Action
                  │
                  ▼
           MuJoCo Simulation
                  │
                  ▼
               Reward
                  │
                  ▼
            RL Training
                  │
                  └──────────► Updated Policy
```

Motion recording will also provide a potential source of trajectory data for future learning systems.

---

# 🚀 Long-Term Autonomous Manipulation

The long-term goal is to combine:

**Camera + Computer Vision + IK + Motion Planning + Reinforcement Learning + Robotic Gripper**

into a single autonomous manipulation pipeline.

```text
                         CAMERA
                            │
                            ▼
                  COMPUTER VISION
                     / OpenCV
                            │
                            ▼
                  OBJECT DETECTION
                            │
                            ▼
                  OBJECT LOCALIZATION
                            │
                            ▼
                     TARGET POSE
                            │
                ┌───────────┴───────────┐
                ▼                       ▼
       INVERSE KINEMATICS              RL
                │                       │
                └───────────┬───────────┘
                            ▼
                       6-DOF ARM
                            │
                            ▼
                         GRIPPER
                            │
                            ▼
                       OBJECT GRASP
                            │
                            ▼
                       OBJECT LIFT
                            │
                            ▼
                       MOVE TO TARGET
                            │
                            ▼
                       OBJECT RELEASE
```

The final objective is to progress from manually programmed robot motion toward **perception-driven, learning-based autonomous manipulation**.

---

# 🧪 Testing and Validation

The repository contains multiple scripts for testing different parts of the robotic system.

## MuJoCo Model

```bash
python3 mujoco/test_mujoco.py
```

## Forward Kinematics

```bash
python3 mujoco/test_fk.py
```

## Inverse Kinematics

```bash
python3 mujoco/test_ik.py
```

## Jacobian

```bash
python3 mujoco/test_jacobian.py
```

## Jacobian Verification

```bash
python3 mujoco/verify_jacobian.py
```

## TCP Inspection

```bash
python3 mujoco/inspect_tcp.py
```

## Individual Joint Testing

```bash
python3 mujoco/test_joint1.py
python3 mujoco/test_joint2.py
python3 mujoco/test_joint3.py
python3 mujoco/test_joint4.py
```

---

# 📂 Project Structure

```text
robotic_arm_ws/
│
├── README.md
├── .gitignore
├── requirements.txt
├── screenshot.png
│
├── mujoco/
│   ├── robotic_arm.xml
│   ├── arm_controller.py
│   ├── hinge_robot.py
│   ├── ik_solver.py
│   ├── ik_motion_demo.py
│   ├── ik_visualize.py
│   ├── motion_recorder.py
│   ├── pick_place_demo.py
│   ├── view_mujoco.py
│   ├── view_robotic_arm.py
│   ├── inspect_model.py
│   ├── inspect_tcp.py
│   ├── test_fk.py
│   ├── test_ik.py
│   ├── test_jacobian.py
│   └── ...
│
└── src/
    │
    ├── robotic_arm_description/
    │   ├── meshes/
    │   ├── urdf/
    │   ├── launch/
    │   └── config/
    │
    ├── robotic_arm_controller/
    │   └── ...
    │
    └── arm_diagnostics/
        └── ...
```

---

# 🛠️ Software and Technologies

| Technology                 | Purpose                                   |
| -------------------------- | ----------------------------------------- |
| **Onshape**                | 3D CAD design and robotic-arm modeling    |
| **MuJoCo 3.12.0**          | Physics and robotic simulation            |
| **Python 3**               | Controller and simulation programming     |
| **NumPy**                  | Numerical calculations, kinematics and IK |
| **XML**                    | MuJoCo robot model configuration          |
| **ROS2 Humble**            | Robotics middleware and integration       |
| **Gazebo**                 | Additional robotics simulation            |
| **OpenCV**                 | Planned computer-vision integration       |
| **Reinforcement Learning** | Planned learning-based robot control      |
| **Ubuntu Linux**           | Development environment                   |

---

# 🖥️ Environment

Development environment:

```text
Operating System : Ubuntu 22.04
ROS              : ROS2 Humble
Python           : Python 3
MuJoCo           : 3.12.0
Numerical Library: NumPy
CAD              : Onshape
```

---

# ⚙️ Installation

Clone the repository:

```bash
git clone <YOUR-GITHUB-REPOSITORY-URL>
cd robotic_arm_ws
```

Install Python dependencies:

```bash
python3 -m pip install -r requirements.txt
```

Or:

```bash
python3 -m pip install mujoco numpy
```

---

# ▶️ Running the Simulation

Navigate to the project:

```bash
cd robotic_arm_ws
```

Run the main MuJoCo visualization:

```bash
python3 mujoco/view_robotic_arm.py
```

Alternative viewer:

```bash
python3 mujoco/view_mujoco.py
```

---

# 🎯 Running Pick-and-Place

Run:

```bash
python3 mujoco/pick_place_demo.py
```

This demonstrates the current pick-and-place motion workflow.

---

# 🤖 ROS2 Support

The repository also contains ROS2 packages for future robotics-system integration.

```text
src/
├── robotic_arm_description/
├── robotic_arm_controller/
└── arm_diagnostics/
```

### Robotic Arm Description

```text
src/robotic_arm_description/
```

Contains:

* URDF
* Xacro
* STL meshes
* Gazebo configuration
* ROS2 control configuration
* Launch files

### Robotic Arm Controller

```text
src/robotic_arm_controller/
```

Contains the ROS2 controller implementation.

### Arm Diagnostics

```text
src/arm_diagnostics/
```

Contains diagnostic and monitoring functionality.

The primary CodeAlpha demonstration is currently based on **MuJoCo + Python**, while the ROS2 components provide a foundation for further integration.

---

# 📊 Current Results

The current implementation demonstrates:

* ✅ Complete 6-DOF robotic-arm model
* ✅ Onshape CAD integration
* ✅ STL mesh integration
* ✅ MuJoCo simulation
* ✅ Joint-space control
* ✅ Forward kinematics
* ✅ Jacobian calculation
* ✅ Numerical inverse kinematics
* ✅ Cartesian/TCP control
* ✅ Basic pick-and-place
* ✅ Motion recording
* ✅ Motion playback
* ✅ ROS2 robot description
* ✅ Gazebo integration support

---

# 🗺️ Development Roadmap

```text
                    CURRENT
                       │
                       ▼
              6-DOF ROBOTIC ARM
                       │
                       ▼
                 CAD + MuJoCo
                       │
                       ▼
              FK / IK / JACOBIAN
                       │
                       ▼
              CARTESIAN / TCP
                       │
                       ▼
                 PICK & PLACE
                       │
                       ▼
              ⭐ MOTION RECORDING
                       │
                       ▼
               ⭐ MOTION PLAYBACK
                       │
                       ▼
              ┌────────────────┐
              │ FUTURE SYSTEM  │
              └────────────────┘
                       │
             ┌─────────┼─────────┐
             ▼         ▼         ▼
          GRIPPER    CAMERA      RL
             │         │         │
             │         ▼         │
             │      OpenCV       │
             │         │         │
             └─────────┼─────────┘
                       ▼
             AUTONOMOUS MANIPULATION
```

---

# 🔮 Future Scope

The project will progressively evolve toward a more complete autonomous robotics platform.

### Robotics

* Robotic gripper
* Improved end-effector
* Collision-aware manipulation
* Motion planning
* MoveIt 2 integration
* ROS2 control
* Real robot hardware integration

### Computer Vision

* Simulated camera
* OpenCV processing
* Object detection
* Object tracking
* Object pose estimation
* Vision-based target generation
* Visual servoing

### Artificial Intelligence

* Reinforcement Learning
* Learning from demonstrations
* Trajectory datasets
* Learned manipulation policies
* Reward-based motion optimization
* Autonomous task execution
* Sim-to-real experimentation

### Advanced Manipulation

```text
Camera
   +
Computer Vision
   +
Motion Recording
   +
IK
   +
Reinforcement Learning
   +
Gripper
   =
Autonomous Robotic Manipulation
```

---

# 📸 Project Screenshot

A simulation screenshot is included:

```text
screenshot.png
```

Additional simulation screenshots and demonstration videos can be added as the project develops.

---

# 🎓 CodeAlpha Internship

**Internship:** CodeAlpha

**Task:** Task 2 — Robotic Arm Simulation

**Project:** Design and Simulation of a 6-DOF Robotic Arm

**CAD:** Onshape

**Simulation:** MuJoCo 3.12.0

**Programming:** Python / NumPy

**Robotics:** Kinematics, Inverse Kinematics, Cartesian Control, TCP Control, Pick-and-Place

**Major Feature:** Motion Recording and Playback

**Future Development:** Robotic Gripper, Camera/OpenCV, Reinforcement Learning, Autonomous Manipulation

---

# 👨‍💻 Project Direction

This project is being developed as more than a basic robotic-arm simulation.

The current 6-DOF arm and motion-recording system provide the foundation for progressively adding **perception, learning, grasping, and autonomous decision-making**.

The intended evolution is:

```text
Simulation
    ↓
Kinematics
    ↓
Motion Control
    ↓
Motion Recording
    ↓
Motion Dataset
    ↓
Computer Vision
    ↓
Gripper
    ↓
Reinforcement Learning
    ↓
Autonomous Manipulation
```

The ultimate objective is to build a robotic system capable of **seeing an object, understanding its location, planning a motion, grasping it, and placing it at a desired target using a combination of classical robotics and learning-based control**.

---

# 📄 License

This project is intended for educational, internship, research, and robotics-development purposes.
