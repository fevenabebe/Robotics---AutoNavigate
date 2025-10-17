# Robotics---AutoNavigate

This repository contains the code and resources for an **autonomous lane-following robot** project.

## Repository Structure
```
Robotics---AutoNavigate/
│
├── build/ # Build files for the project
├── install/ # Installation scripts or outputs
├── log/ # Log files from the robot or simulation
├── src/lane_following_robot/
│ ├── pycache/ # Python cache files
│ ├── init.py
│ ├── camera_driver2.py
│ ├── camera_driver55.py
│ ├── controller._no_destination.py
│ ├── controller._stop.py
│ ├── controller.py
│ ├── controller_friction.py
│ ├── controller_rotate_backward.py
│ ├── controllergray.py
│ ├── image_processor.py
│ ├── image_processor_friction_issue.py
│ ├── image_processor_stop.py
│ ├── image_processor_with_no_destination.py
│ ├── spawn_robot55.py
│ ├── spawn_robotAfterRotate.py
│ ├── test_cv_display.py
│ ├── test_lane.jpg
│ ├── visualizer55.py
│ ├── launch/
│ │ └── lane_following.launch.py
│ └── models/
│ └── lane_bot/
│ ├── model.config
│ ├── model.urdf
│ ├── modelBackward.urdf
│ └── modelOld.urdf
├── gzserver_output.log # Output log from Gazebo server
```
markdown
Copy code

## Owner
- **Owner:** fevenabebe
- **Repository:** Public

## Features
- Lane-following robot implemented in ROS2
- Gazebo simulation environment
- Modular structure for easier development and testing

## How to Use

1. Clone the repository:

```bash
git clone https://github.com/fevenabebe/Robotics---AutoNavigate.git
