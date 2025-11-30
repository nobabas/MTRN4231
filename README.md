<!-- omit from toc -->
# SOIL BOT

- [Project Overview](#project-overview)
  - [Video Demonstration](#video-demonstration)
- [System Architecture](#system-architecture)
  - [ROS Packages and Node Descriptions](#ros-packages-and-node-descriptions)
  - [Closed-Loop System Behaviour](#closed-loop-system-behaviour)
- [Technical Components](#technical-components)
  - [Computer Vision](#computer-vision-hole-detection)
  - [Custom End-Effector](#custom-end-effector)
  - [System Visualisation](#system-visualisation)
  - [Closed-Loop Operation and Feedback](#closed-loop-operation-and-feedback)
- [Installation and Setup](#installation-and-setup)
  - [Prerequisites and Dependencies](#prerequisites-and-dependencies)
  - [Workspace Setup](#workspace-setup)
  - [Hardware Setup and Calibration](#hardware-setup-and-calibration)
- [Running the System](#running-the-system)
  - [Launching the Complete System](#launching-the-complete-system)
  - [Example Commands and Expected Output](#example-commands-and-expected-output)
  - [Testing Routines](#testing-routines)
- [Results and Demonstration](#results-and-demonstration)
  - [Performance Against Design Goals](#performance-against-design-goals)
  - [Quantitative Results (Accuracy, Repeatability)](#quantitative-results-accuracy-repeatability)
  - [Robustness, Adaptability, and Innovation](#robustness-adaptability-and-innovation)
- [Discussion and Future Work](#discussion-and-future-work)
  - [Engineering Challenges and Solutions](#engineering-challenges-and-solutions)
  - [Opportunities for Improvement](#opportunities-for-improvement)
- [Contributors and Roles](#contributors-and-roles)
- [Repository Structure](#repository-structure)
- [References and Acknowledgements](#references-and-acknowledgements)

[Minh]: https://www.linkedin.com/in/nickojbell/
[Samuel]: https://www.linkedin.com/in/jiawen-zhang-1aa622203/
[Brent]: https://www.linkedin.com/in/daryl-lee-7b022a201/
[David]: https://www.linkedin.com/in/davidnie0418/

[UR10e]: https://github.com/DaviddNie/UR10e_vision_based_fruit_harvesting
[Screw]: https://github.com/DaviddNie/ScrewDrivingBot1 

[ROS Packages and Node Descriptions]: #ros-packages-and-node-descriptions

## Project Overview

This project utilizes UR5e from Universal Robot for soil moisture testing detection to assist large farms.

This system utilizes the integration of camera detection, collaborating the UR5e, and a custom end effector, designed to detect moisture, to enable to robot to move and obtain the moisture from the soil and to be consistently repeated in multiple areas.

**Project Duration**: 6 weeks

**Authors**:
* Minh Thang Phan: [Minh's LinkedIn Profile][Minh]
* Samuel Gray: [Samuel's LinkedIn profile][Samuel]
* Brent Poon: [Brent's LinkedIn profile][Brent]

**Supervisor**:
* David Nie: [David's LinkedIn profile][David]

### Video Demonstration

Video link:
https://www.youtube.com/shorts/ufF1myLWomA

Most likely get from a test run

Add image of the robot with full setup
<div align="center">
  <img src="img/demo_img.png" alt="cad" width="100%">
</div>

Need an folder to store all the images


## System Architecture
### ROS Packages and Node Descriptions
- **brain**
  - brain
- **endeffector_description**
- **interfaces (custom messages and services)**
  - **srv**
    - BrainCmd (For testing individual Packages)
    - MoveRequest (Running Movement Request)
    - VisionCmd (Interface with Vision Module)
  - **msg**
    - Marker2D (For putting world coordinates in temporary)
    - Marker2DArray (For putting all the world coordinates in to be published)
    - MarkerData (For putting world coordinates of that specific id value)
- **moveit_planning_server**
    - moveit_server
- **take_image**
  - camera_run (Enables camera)
- **teensy_bridge**
  - teensy_bridge_node
- **transformation**
  - tf_main (Main transformation calculations)
  - tf_publisher (Publishes transformation coordinates)
  - tf_subscriber (Subscribes from vision_main)
  - tf_utils (Addition transformation calculations)
- **ur_moveit_config**
- **vision**
  - vision_main (Yolo detection and determines centroid)

>>--------------------------------------------------------------------------------------------->>
### Closed-Loop System Behaviour

#### Motion Plan Overview

#### System Flowchart

## Technical Components

<<--------------------------------------------------------------------------------------------<<>>
### Computer Vision
The computer vision module utilises a trained Yolov11n model with a pre-existing dataset, targetting specifically blue markers. An image feed is constantly fed into the vision in real time, within a reduced frequency, to update and annotate boxes around the blue markers. Using the boxes it then identifies the centroid position of each marker in pixels to then be published as /pixel_coords.

This vision effectiviely detects the starting position of each marker in pixels. This will be used to calculate the real world coordinates with the use of the transformation package.  

>>-------------------------------------------------------------------------------------------->>
### Custom End-Effector

<img width="1520" height="900" alt="endeffector v13" src="https://github.com/user-attachments/assets/2047fed1-d18d-4540-a036-8d602a249858" />

<img width="3309" height="2339" alt="endeffector Drawing v1-1" src="https://github.com/user-attachments/assets/791ba4ce-d30d-426f-b73b-57eca3f1bb17" />

provide photos/renders, assembly details, engineering drawings, 
control overview and integration details. 

### System Visualisation
 explain how your system is visualised (e.g. RViz) and what it 
demonstrates.

### Closed-Loop Operation and Feedback
 describe the feedback method and how it adapts system 
behaviour in real time.

## Installation and Setup
Step-by-step installation instructions for dependencies and workspace setup.

### Prerequisites and Dependencies


### Workspace Setup


### Hardware Setup and Calibration
Hardware Requirement
Intel Realsense D435 Depth Camera
- List of stuff relating to it

Calibration
- No Calibrations

## Running the System
Clear instructions for launching and running the complete system
Example commands (e.g. ros2 launch project_name bringup.launch.py).
- Expected behaviour and example outputs.
- Optional troubleshooting notes.
- The system should be launched by a single command (e.g. via a launch file, shell script
or Docker image), without manual sequencing.

### Testing Yolo Model
'python3 vision_basic.py'

### Launching System without End Effector

### Launching System with End Effector


### Example Commands and Expected Output

### Testing Routines



## Results and Demonstration

### Performance Against Design Goals
*(Discuss the success of the system based on the initial requirements.)*
- Inital Requirements
  - Reliability of system: 1 Failure per 500 Samples
  - Cycle time: Less than 5 minutes per sample
  - Sampling Accuracy: Within 2mm placement of sensor
  - Sensor Accuracy: Within 0.5 Analogue units
  - Repeatibility: Within 0.5mm of sensor position for same sampling point

Something about the success of the system

### Quantitative Results (Accuracy, Repeatability)
*(Include data or figures showing performance metrics.)*
Remember to put in photos

<<-----------------------------------------------------------------------------------------------------------------------------------<<

### Robustness, Adaptability, and Innovation
- Robutstness
  - Very Robust
- Adaptability
  - A majority of the computer vision aspect relies on the realsense camera hardware, as it most of the functions calls from the specs.
  - Can be changed based on training the data the Yolov11 files to aim for either different coloured markers or to adapt to the changes.
  - For UR5e related movement it is very adaptable.
  - Using this as it is in a different environement may yield different results.
- Innovation
  - Although there are some example similar to this project, there have been no example of pure automation of consistent soil detection.

## Discussion and Future Work
Our approach to this project has a simple objective to only detect the soil, which makes it effective and simple to complete the tasks without difficulty. It is creative as although there is a lot of soil detection related technology, not many of them are automated or utilises robot, as most uses are manual or requires manpower to run them in the real world. This idea is not novel to an extent but it provides an different approach to utilising this technology. 

### Engineering Challenges and Solutions
There were several engineering challenges that were faced during the development of this project. Such challenges faced were:
- Time
  - A tight time constraint,
- System Intergration
  - Multiple errors and fixes had to be made through the process of intergrating the entire system.
- Utilising Certain programs like Ros2 package, RVIS and Moveit
  - There were some difficultly in using those programs in terms of difficultly to understand and slow processing speed.

These were addressed by the team by hard focusing on the problems that exist on the code. If such problem continued, the team asked for assistance from other teams, the supervisor, and AI assistance whenever possible.

### Opportunities for Improvement
Due to the simipicty of the project, there are many improvements that can be made.

The following below are some things that can be improved on for "Version 2.0":
- End effector with different tools on it, where it can switch each tool in and out of use, with the capacity to dig and plant seed
- Another improvement is the making the path planning timing more efficient to reduce time delays.
- A greater dataset could have been made for YOLO11 to ensure it is consistent in detecting all the objects required.
- Instead of only using blue markers as it's main detection, it could also detect other items, such as plants or rocks in the soil.

## Contributors and Roles
- Minh: 
  - Primary Areas of Responsibility:
    - Moveit
    - Integration

- Samuel:
  - Primary Areas of Responsibility:
    - Endeffector
    - Brain
    - Integration

- Brent: 
  - Primary Areas of Responsibility:
    - Computer Vision
    - Transformations of coordinates
    - README


## Repository Structure
The below is the repository setup that is done in github -
- end_effector/arduino
  - Soil_moisture_reading
  - Soil_test_for_ur5e
- src
  - brain
  - endeffector_description
  - interfaces
  - moveit_planning server
  - take_image
  - teensy_bridge
  - transformation
  - ur5_moveit_config
  - vision
- yolo_dataset

Each folder does the following -
- End_effector:
  - Using Arduino to run the end effector with UR5e
- src:
  - The main folder which contains all the packages
  - These packages are explained in [ROS Packages][ROS Packages and Node Descriptions]
- yolo_dataset:
  - Only contains images that were used to train the Yolo model

## References and Acknowledgements
- MTRN4231 Labs Week 1-5
- David Nie's Github Repositories
  - [UR10e Vision Based Fruit Harvesting][UR10e]
  - [Screw Driving Bot][Screw]
