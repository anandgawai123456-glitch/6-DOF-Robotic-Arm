# 🤖 6-DOF Robotic Arm Simulation

A complete **6-DOF robotic arm simulation and control project** designed in **Onshape**, integrated into **MuJoCo**, and controlled using **Python and NumPy**.

The project combines **3D CAD modeling, robotic kinematics, inverse kinematics, Cartesian/TCP control, motion recording and playback, and basic pick-and-place manipulation**.

The system is being developed as a foundation for an advanced autonomous manipulation platform with future integration of:

- 🦾 Robotic Gripper
- 📷 Camera
- 👁️ Computer Vision / OpenCV
- 🧠 Reinforcement Learning
- 🎯 Vision-Based Pick-and-Place
- 🚀 Autonomous Manipulation

---

# 📌 Project Overview

The robotic arm was designed and modeled as a 3D CAD assembly using **Onshape**.

The CAD geometry was exported as STL meshes and integrated into a **MuJoCo 3.12.0** simulation model.

The robot is controlled using Python and NumPy, with implementations for:

- Joint-space control
- Forward kinematics
- Jacobian calculation
- Numerical inverse kinematics
- Cartesian motion
- TCP/tool-space control
- Pick-and-place
- Motion recording
- Motion playback

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
⭐ Major Features
1. 6-DOF Robotic Arm

The project contains a complete six-degree-of-freedom robotic arm.

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

Joint names:

joint_1
joint_2
joint_3
joint_4
joint_5
joint_6

Total: 6 DOF

⭐ 2. CAD Design Using Onshape

The robotic arm was designed as a 3D mechanical assembly using Onshape.

The CAD design includes:

Base
Robotic links
Six rotational joints
End-effector/tool components
TCP/tool reference

The CAD geometry was exported and integrated into the simulation.

Onshape
   ↓
3D Assembly
   ↓
Individual Components
   ↓
STL Export
   ↓
MuJoCo / ROS2 Integration

Mesh files are stored inside:

src/robotic_arm_description/meshes/

Main components include:

base.stl
link1.stl
link2.stl
link3.stl
link4.stl
link5.stl
link6.stl
joint5_part.stl
tool.stl
tool2.stl
⭐ 3. MuJoCo Simulation

The primary robotic simulation is implemented using:

MuJoCo 3.12.0

Main model:

mujoco/robotic_arm.xml

The MuJoCo model contains:

Robot bodies
Revolute joints
Joint limits
Visual geometry
Collision geometry
TCP definition
Robot configuration
Simulation parameters

MuJoCo provides the physics and simulation environment used for testing and developing robotic motion.

⭐ 4. Forward Kinematics

Forward kinematics calculates the TCP position and orientation from the robot's joint configuration.

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

Testing script:

mujoco/test_fk.py
⭐ 5. Jacobian Calculation

The robot Jacobian is used to relate joint motion to Cartesian motion.

Joint Velocity
      │
      ▼
   Jacobian
      │
      ▼
Cartesian Velocity

Related scripts:

mujoco/test_jacobian.py
mujoco/verify_jacobian.py
⭐ 6. Numerical Inverse Kinematics

Numerical inverse kinematics is used to calculate the required joint configuration for a desired TCP position.

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

Main files:

mujoco/ik_solver.py
mujoco/ik_motion_demo.py
mujoco/ik_visualize.py

This provides the mathematical foundation required for autonomous target reaching and future vision-based manipulation.

⭐ 7. Cartesian / TCP Control

The robotic arm can be controlled using the Tool Center Point.

Instead of manually specifying every joint angle, a target can be defined in Cartesian space.

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

This provides the foundation for:

Target reaching
Object approach
Pick-and-place
Vision-guided positioning
Autonomous manipulation
🔥 8. Motion Recording and Playback
A Major Project Feature

One of the major features of this project is the ability to record robotic-arm motion and reproduce it through playback.

The motion recording system captures the robot's movement during simulation and stores the trajectory for later use.

Main implementation:

mujoco/motion_recorder.py

The recording pipeline is:

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

The saved trajectory can then be used for playback:

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
Why Motion Recording Matters

Motion recording is not only a debugging capability.

It provides a foundation for:

Repeating robotic movements
Saving successful trajectories
Motion analysis
Trajectory comparison
Demonstration-based robotics
Creating training datasets
Learning from demonstrations
Reinforcement-learning data generation
Trajectory optimization
Future sim-to-real development

The long-term concept is:

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

This creates a bridge between traditional programmed robotics and learning-based robotics.

⭐ 9. Pick-and-Place

A basic pick-and-place workflow is implemented as part of the current project.

Conceptually:

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

Run the demonstration:

python3 mujoco/pick_place_demo.py

The current implementation provides the foundation for future gripper-based and vision-based manipulation.

🦾 Future: Robotic Gripper

The next major development stage is integration of a dedicated robotic gripper.

Planned capabilities include:

Gripper modeling
Gripper joints
Open/close control
Object contact
Grasping
Object release
Collision-aware grasping
Pick-and-place with physical gripping
Gripper control through ROS2

Future architecture:

6-DOF Arm
    +
Robotic Gripper
    │
    ▼
Object Manipulation
📷 Future: Camera and Computer Vision

A simulated camera will be integrated into the robotic environment.

Computer vision will be developed using OpenCV.

Planned capabilities include:

Camera simulation
Image acquisition
Image processing
Object detection
Object localization
Object tracking
Object position estimation
Camera-to-robot coordinate transformation
Vision-based target generation
Visual servoing

Future vision pipeline:

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
🧠 Future: Reinforcement Learning

Reinforcement Learning will be introduced to move the project toward learning-based robotic control.

Potential applications include:

Robot reaching
Motion optimization
Target reaching
Pick-and-place learning
Grasping policies
Reward-based control
Trajectory optimization
Autonomous manipulation
Simulation-based training
Policy evaluation in MuJoCo
Future sim-to-real experimentation

Planned RL architecture:

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

Motion recording will also provide a potential source of trajectory data for future learning systems.

🚀 Long-Term Autonomous Manipulation

The long-term goal is to combine:

Camera + Computer Vision + IK + Motion Planning + Reinforcement Learning + Robotic Gripper

into a single autonomous manipulation pipeline.

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

The final objective is to progress from manually programmed robot motion toward perception-driven, learning-based autonomous manipulation.

🧪 Testing and Validation

The repository contains multiple scripts for testing different components of the robotic system.

MuJoCo Model
python3 mujoco/test_mujoco.py
Forward Kinematics
python3 mujoco/test_fk.py
Inverse Kinematics
python3 mujoco/test_ik.py
Jacobian
python3 mujoco/test_jacobian.py
Jacobian Verification
python3 mujoco/verify_jacobian.py
TCP Inspection
python3 mujoco/inspect_tcp.py
Individual Joint Testing
python3 mujoco/test_joint1.py
python3 mujoco/test_joint2.py
python3 mujoco/test_joint3.py
python3 mujoco/test_joint4.py
📂 Project Structure
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
🛠️ Software and Technologies
Technology	Purpose
Onshape	3D CAD design and robotic-arm modeling
MuJoCo 3.12.0	Physics and robotic simulation
Python 3	Controller and simulation programming
NumPy	Numerical calculations, kinematics and IK
XML	MuJoCo robot model configuration
ROS2 Humble	Robotics middleware and integration
Gazebo	Additional robotics simulation
OpenCV	Planned computer-vision integration
Reinforcement Learning	Planned learning-based robot control
Ubuntu Linux	Development environment
🖥️ Development Environment
Operating System : Ubuntu 22.04
ROS              : ROS2 Humble
Python           : Python 3
MuJoCo           : 3.12.0
Numerical Library: NumPy
CAD              : Onshape
⚙️ Installation

Clone the repository:

git clone <YOUR-GITHUB-REPOSITORY-URL>
cd robotic_arm_ws

Install Python dependencies:

python3 -m pip install -r requirements.txt

Or:

python3 -m pip install mujoco numpy
▶️ Running the Simulation

Navigate to the project:

cd robotic_arm_ws

Run the main MuJoCo visualization:

python3 mujoco/view_robotic_arm.py

Alternative viewer:

python3 mujoco/view_mujoco.py
🎯 Running Pick-and-Place

Run:

python3 mujoco/pick_place_demo.py

This demonstrates the current pick-and-place motion workflow.

🤖 ROS2 Support

The repository also contains ROS2 packages for future robotics-system integration.

src/
├── robotic_arm_description/
├── robotic_arm_controller/
└── arm_diagnostics/
Robotic Arm Description
src/robotic_arm_description/

Contains:

URDF
Xacro
STL meshes
Gazebo configuration
ROS2 control configuration
Launch files
Robotic Arm Controller
src/robotic_arm_controller/

Contains the ROS2 controller implementation.

Arm Diagnostics
src/arm_diagnostics/

Contains diagnostic and monitoring functionality.

The primary simulation is currently based on MuJoCo + Python, while the ROS2 components provide a foundation for further robotics integration.

📊 Current Results

The current implementation demonstrates:

✅ Complete 6-DOF robotic-arm model
✅ Onshape CAD integration
✅ STL mesh integration
✅ MuJoCo simulation
✅ Joint-space control
✅ Forward kinematics
✅ Jacobian calculation
✅ Numerical inverse kinematics
✅ Cartesian/TCP control
✅ Basic pick-and-place
✅ Motion recording
✅ Motion playback
✅ ROS2 robot description
✅ Gazebo integration support
🗺️ Development Roadmap
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
🔮 Future Scope

The project will progressively evolve toward a complete autonomous robotics platform.

Robotics
Robotic gripper
Improved end-effector
Collision-aware manipulation
Motion planning
MoveIt 2 integration
ROS2 control
Real robot hardware integration
Computer Vision
Simulated camera
OpenCV processing
Object detection
Object tracking
Object pose estimation
Vision-based target generation
Visual servoing
Artificial Intelligence
Reinforcement Learning
Learning from demonstrations
Trajectory datasets
Learned manipulation policies
Reward-based motion optimization
Autonomous task execution
Sim-to-real experimentation
Advanced Manipulation
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
📸 Project Screenshot

A simulation screenshot is included in the repository:

screenshot.png

Additional screenshots and demonstration videos can be added as the project develops.

🧩 Project Philosophy

This project is being developed as a progressive robotics platform rather than a single isolated simulation.

The development strategy is:

Build the Robot
      ↓
Understand the Kinematics
      ↓
Control the Motion
      ↓
Record the Motion
      ↓
Build Trajectory Data
      ↓
Add Perception
      ↓
Add Grasping
      ↓
Add Learning
      ↓
Achieve Autonomous Manipulation

The objective is to combine classical robotics engineering with modern AI-based robotics.

🚀 Future Vision

The final system is intended to move toward a robot that can:

SEE
 │
 ▼
UNDERSTAND
 │
 ▼
PLAN
 │
 ▼
MOVE
 │
 ▼
GRASP
 │
 ▼
MANIPULATE
 │
 ▼
LEARN

The long-term vision is a simulation-first autonomous robotic manipulation platform where perception, kinematics, motion planning, recorded demonstrations, reinforcement learning, and physical interaction work together.
