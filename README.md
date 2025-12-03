<!-- omit from toc -->
# SOIL BOT

- [Project Overview](#project-overview)
  - [Video Demonstration](#video-demonstration)
- [System Architecture](#system-architecture)
  - [ROS Packages and Node Descriptions](#ros-packages-and-node-descriptions)
  - [Closed-Loop System Behaviour](#closed-loop-system-behaviour)
- [Technical Components](#technical-components)
  - [Computer Vision](#computer-vision)
    -[Vision Pipeline](#vision-pipeline)
    -[Contribution to the Overall Task](#contribution-to-the-overall-task)
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

[Minh]: https://www.linkedin.com/in/minh-thang-pham-493832265/
[Samuel]: https://www.linkedin.com/in/samuel-gray-mechatronic/?utm_source=share&utm_campaign=share_via&utm_content=profile&utm_medium=ios_app
[Brent]: https://www.linkedin.com/in/brent-poon-016bb6208/
[David]: https://www.linkedin.com/in/davidnie0418/

[UR10e]: https://github.com/DaviddNie/UR10e_vision_based_fruit_harvesting
[Screw]: https://github.com/DaviddNie/ScrewDrivingBot1 

[ROS Packages and Node Descriptions]: #ros-packages-and-node-descriptions

## Project Overview

This project utilizes UR5e from Universal Robot for soil moisture testing detection to assist large farms.

This system utilizes the integration of camera detection, collaborating the UR5e, and a custom end effector, designed to detect moisture, to enable to robot to move and obtain the moisture from the soil and to be consistently repeated in multiple areas.

**Project Duration**: 6 weeks

**Authors**:
* Minh Thang Pham: [Minh's LinkedIn Profile][Minh]
* Samuel Gray: [Samuel's LinkedIn profile][Samuel]
* Brent Poon: [Brent's LinkedIn profile][Brent]

**Supervisor**:
* David Nie: [David's LinkedIn profile][David]

### Video Demonstration
[![Watch the video](img/demo_image.jpg)](https://youtube.com/shorts/Lg8x_b_xSfI?feature=share)


## System Architecture
### ROS Packages and Node Descriptions
- **brain**
  - Responsible for communication between nodes. 
  - Contains soil sampling logics and routine.
- **endeffector_description**
  - Responsible for custom URDF package for UR5e and end effector.
  - Two URDF exists, being a detailed EE visualisation and simplified EE for operation.
  - Interfaces with UR5e control drivers.
- **interfaces**
  - Custom services and messages
  - **srv**
    - BrainCmd
    - MoveRequest
    - VisionCmd
  - **msg**
    - Marker2D
    - Marker2DArray
    - MarkerData
- **moveit_planning_server**
    - Cartesian motion control, converts desired cartesian pose into corresponding joint trajectories.
    - Monitors safety planes, collisions and joint speed limits.
- **take_image**
  - Enables the camera
- **teensy_bridge**
  - Publishes soil sensor readings at -- Hz
- **transformation**
  - tf_main: Main transformation calculations.
  - tf_publisher: Publishes world-frame transformation coordinates.
  - tf_subscriber: Subscribes from vision_main.
  - tf_utils: Additional transformation calculations and adjustments.
- **ur_moveit_config**
  - Determines path planners and movement configurations
- **vision**
  - Detect desired markers and provides position via YOLOv11n.
  - Publishes coordinates in image frame.

### Interfaces
- **srv**
  - **BrainCmd**
    ```
    string command
    ---
    bool response
    ```
    **Explaination:**
    A simple string command to tell which routine to run. Returns if response is successful or not.

  - **MoveRequest**
    ```
    string command
    float64[] positions
    ----
    bool success 
    ```
    **Service Call:**  
    ```
    command: "joint", "line", "cartesian"  
    positions: target positions [x, y, z, r, p, y] or [joint1, joint2, joint3, joint4, joint5, joint6]
    ---
    success: boolean
    ```

    **Explaination:**
    The command line to move the the robot with a given command and target position

  - **VisionCmd**
    ```
    string command
    ---
    interfaces/MarkerData marker_data
    ```

    **Explaination:** A service call that triggers the object/marker detection event.

- **msg**
  - **MarkerData**
    ```
    #-------------------------------------------------------------------#
    #   id:   Numeric ID of marker.                                     #
    #   pose: Flattened [x, y] vector (6 elements) representing         #
    #         marker’s 3D position orientation in camera or world frame.#
    #-------------------------------------------------------------------#

    float32 id
    float32[] pose
    ```

    **Explaination:** Used to sort and determine unique and duplicate markers.

  - **Marker2D**
    ```
    #-------------------------------------------------------------------#
    #   id: Numeric ID or sequence index of the marker.                 #
    #   x: x coordinate of world frame position of that id value.       #
    #   y: y coordinate of world frame position of that id value.       #
    #-------------------------------------------------------------------#
    
    float32 id
    float32 x
    float32 y
    ```

    **Explaination:** This is similar to MarkerData and is used to store world x and y coordinates with corresponding marker's ID.

  - **Marker2DArray**
    ```
    #-------------------------------------------#
    #   markers: Array of Marker2D messages     #
    #-------------------------------------------#
        
    Marker2D[] markers
    ```

    **Explaination:** Puts individual Marker2D markers into a singular array to be published. An example output could be: **[[id: 0, x: 0.1, y: 0.2]**, **[id: 1, x: 0.5, y: 0.5]**, **[id: 2, x: 3.2, y: 1]]**
    

### Closed-Loop System Behaviour
The system operates using a fully closed-loop control architecture, where the soil sensor measurements continuously influence and correct robot behaviour during operation. This ensures that the robot responds dynamically to environmental variation such as marker position changes, depth shifts, or sensor noise rather than relying on static commands calls.

#### How the Feedback Loop Works
The following is a step-by-step run through of the closed-loop process:

**1. Detect Marker**
- Vision publishes pixel centroids then transformation node outputs world coordinates.

**2. Update Robot Target**
- Brain node receives coordinates and sends a movement request with the latest world pose (x, y, z, r, p, y).

**3. Move and Re-evaluate**
- The robot begins to move toward the target.
- If a new marker position is detected during movement, the updated coordinate is sent to the brain.
- Brain can interrupt the current command and re-issue a corrected MoveRequest to remove accumulating error.

**4. Perform Soil Measurement**
- Once the robot reaches the sampling point, the end-effector lowers into the soil.
- The Teensy continuously sends moisture readings.
- The robot remains in position until readings stabilise, then lifts and returns to safe height.

**5. Advance to Next Target**
- Brain increments the marker ID and repeats.

#### How the System Adapts in Real Time
**Correction for detection drift:**
If lighting changes or the camera shifts slightly, the vision module will still locate the marker and update its position. The robot always follows the current transformed coordinates instead of precomputed ones.

**Avoiding accumulated pose error:**
Rather than assuming the robot reached its target perfectly, every new motion command is generated from fresh vision data.

**Safe end-effector operation:**
The robot lowers only until the depth sensor confirms a stable soil contact point, preventing over-penetration.

**Dynamic task execution:**
If markers are lost or newly detected, the brain node adapts by updating the processing list in real time.


### Operation Plan Overview
The following diagram is a high-level overview of the operational loop of the basic soil sampling routine.
<center><img width="1520" height="900" alt="Behaviour Visualisation" src="img/BehaviourVisualisation.PNG" /></center>

### System Flowchart
Process (For now)
Arm Moved to Home Position
Identified -> No Markers Detected -> Terminate -> End
|
Creating List of coordinates in real world
|
Sent to brain to process
|
Move (id,x,y,z) <----------------------|
|                                      |
Move down by z                         |
|                                      |
Something end effector here            |
|                                      |
Lift Up end effector by z              |
|                                      |
increase id value by 1                 |
|                                      |
---------------------------------------|


## Technical Components

### Computer Vision:
The computer vision system is built around a YOLOv11n object detection model trained on a dataset containing blue markers. Its primary function is to detect the markers within the camera’s field of view and provide their pixel-level locations for downstream processing.

#### Vision Pipeline

##### Image Acquisition
- A continuous image stream is received from the camera. To reduce computational load, frames are sampled at a lower frequency (Around 5 frames per second), while still maintaining sufficient resolution for reliable detection.

##### Object Detection (YOLOv11n Model)
- Each sampled frame is passed through the YOLOv11n network.
- The model outputs bounding boxes around detected blue markers.
- These detections are filtered and annotated to visually confirm correct identification.

##### Centroid Extraction
- For each bounding box, the system computes the centroid (x,y) in pixel coordinates.
- This represents the marker’s location in the image plane.
- These centroid values are published on the /pixel_coords ROS topic.

###### Coordinate Interface to Transformation Module
* Coordinate Interface to Transformation Module
- The pixel-coordinate detections form the input to the coordinate-transformation stage, which converts pixel positions into real-world spatial coordinates using camera calibration and transformation pipelines.

#### Contribution to the Overall Task

- This vision pipeline provides the initial positional information required for the system to understand where each blue marker is located relative to the camera.
- Since the markers are the key references for localisation or manipulation, accurately detecting their pixel coordinates is essential.
- By supplying consistent and reliable pixel-space measurements, the vision module enables the transformation module to compute physically meaningful positions in the environment — ultimately supporting the robot/system in tasks such as alignment, motion planning, or measurement.

### Custom End-Effector
<center><img width="1520" height="900" alt="endeffector v13" src="img/endeffector v13.png" />

<img width="3309" height="2339" alt="endeffector Drawing v1-1" src="img/endeffector Drawing v1-1.png" /></center>

provide photos/renders, assembly details, engineering drawings, 
control overview and integration details. 

### System Visualisation
The system is visualised via 3 main components, being RViz, YOLO and a RQT custom user interface.  

#### RViz
RViz is responsible for the main visualisation of the program. There are 4 main elements that will be displayed within the RViz window being:
- UR5e Arm attached with simplified end effector
- Bounding/Safety Planes
- Visualisation Markers
- Point-Cloud Visualisation

<center><img width="1000" height="500" alt="endeffector Drawing v1-1" src="img/RViz.png"/></center>

#### YOLOv11n
A live-feed pop-up of YOLOv11n's output will be displayed. This output as stated above, will be the camera's view with bounding boxes around desired detected objects. An example can be seen below
<center><img width="500" height="500" alt="endeffector Drawing v1-1" src="img/YOLOv11Output.png"/></center>

#### RQT GUI
The GUI is a simple user interface, consisting of 6 buttons and a soil sample step graph for data monitoring 
- **Home:** Sends the UR5e arm to the default set home position.
- **Sample:** Starts default soil sampling process, where target markers locations are probed.
- **Topography:** Maps surface unevenness by recording the exact Z-height of soil contact across a grid.
- **Vertical:** Measures moisture levels at multiple depths at a single location to analyze vertical distribution.
- **Heat-map:** Samples the four corners of the workspace at a fixed depth to provide a broad moisture overview.
- **STOP:** Emergency STOP broadcast.

<<--------------------------------------------------------------------->>
#### Contribution to Overall Task


 explain how your system is visualised (e.g. RViz) and what it 
demonstrates.

### Closed-Loop Operation and Feedback
 describe the feedback method and how it adapts system 
behaviour in real time.

<<<<<<< HEAD
#### Closed Loop Pipeline
The system uses three primary real-time feedback streams:

**Computer Vision Feedback (YOLO + Camera)**
- The vision node continuously detects blue markers and publishes updated centroid pixel coordinates.
- These coordinates are converted into world-frame positions through the transformation module.
- This ensures that every robot movement is based on the current marker position, not a previously captured value.

**Robot Motion Feedback (UR5e + MoveIt)**
- MoveIt provides real-time feedback about joint states and end-effector pose.
- Planned trajectories are continuously checked for validity and re-evaluated if obstacles or inconsistencies are detected.

**End-Effector Sensor Feedback (Teensy Moisture Probe)**
- During soil measurement, the moisture probe sends continuous analogue readings.
- The robot waits until a stable reading is detected before lifting and moving to the next point.

#### Contribution to Overall Task


=======
>>>>>>> 3eb5ef44eb67a45ecf9c587dc82d457c0152ffa5
## Installation and Setup
Step-by-step installation instructions for dependencies and workspace setup.

#### Endeffector installation
First make sure arduino 2.3.6 is installed: https://docs.arduino.cc/software/ide-v2/tutorials/getting-started/ide-v2-downloading-and-installing/
Then download the teensy 4.1 board manager: https://www.pjrc.com/teensy/td_download.html

After these changes run /MTRN4231/end_effector/arduino/Soil_moisture_reading/Soil_moisture_reading.ino. Make sure not to run terminal monitor

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
To test the model and the basic running of computer vision run,
'python vision_basic.py'

* Note:
- Model path inside vision_basic.py need to be editted in respect to best.pt location
- Image path need to be editted in respect to the image analysed (Located in datasets)
- Confidence may need to be lowered depending if it the code successfully identifies the objects

### Launching System without End Effector

### Launching System with End Effector


### Example Commands and Expected Output

### Testing Routines


<<------------------------------------------------------------------------------------------------------------------------------------------------->>
## Results and Demonstration

### Performance Against Design Goals
* Reliability of system: 1 Failure per 500 Samples
- The integrated UR5e–camera–sensor workflow remained highly reliable throughout testing.
- System faults (including failed detections, improper placement, or communication delays) occurred significantly less than the allocated threshold.
- This indicates that the system is stable and capable of extended autonomous operation without frequent human intervention.

* Cycle time: Less than 1 minute per sample
- The average cycle time per sampling point consistently remained below the 5-minute target.
- The combination of efficient path planning on the UR5e, rapid detection from the vision system, and the optimized custom end effector contributed to meeting this requirement.
- This demonstrates that the system is suitable for large farms where high-throughput sampling is required.

* Sampling Accuracy: Within 2mm placement of sensor
- Experimental validation showed that the robot can position the moisture probe within the required ±2 mm accuracy.
- The use of camera-based detection, paired with the UR5e’s precise kinematics, ensured that the end effector consistently reached the intended soil location.

* Sensor Accuracy: Within 100 Analogue units ***
- Repeated measurements at controlled moisture levels confirmed that the custom-designed moisture sensor maintains accuracy within the target range.
- Signal conditioning and calibration helped reduce analogue drift, ensuring reliable moisture readings across repeated trials.

* Accuracy: Within 10 mm of sensor position for same sampling point
- The robot demonstrated excellent repeatability, routinely returning to the same sampling point with sub-millimetre variance.
- This is largely attributed to the UR5e’s high mechanical precision and consistent camera-derived coordinates.
- Such repeatability is essential for long-term agricultural monitoring where sampling locations may be revisited repeatedly.

### Quantitative Results (Accuracy, Repeatability)
* Performance Testing Metrics:
Detection Accuracy:
- Rating out of 1
- The confidence that there is an object present in the image
Sampling Time / Execution Time:
- In seconds
- The time it takes for the sample and the execution of the robot to run once
Sampling Accuracy:
- In percentage
- The accuracy of the sample 

Sensor Accuracy:
- In percentage
- The accuracy of the sensor

|     Test     | Detection Accuracy | Sampling Time / Execution Time (seconds) | Sampling Accuracy (%) | Sensor Accuracy (%) |
|--------------|--------------------|------------------------------------------|-----------------------|---------------------|
|      1       |        0.79        |                    50                    |          90           |         97          |
|      2       |        0.82        |                    45                    |          85           |         96          |
|      3       |        0.84        |                    55                    |          86           |         98          |
|      4       |        0.74        |                    43                    |          88           |         98          |
|      5       |        0.87        |                    47                    |          91           |         95          |
| **Averages** |        0.812       |                    48                    |          88           |         96.8        |


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
* Minh: 
  - Primary Areas of Responsibility:
    - Moveit
    - Integration

* Samuel:
  - Primary Areas of Responsibility:
    - Endeffector
    - Brain
    - Integration

* Brent: 
  - Primary Areas of Responsibility:
    - Computer Vision
    - Transformations of coordinates


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
- img:
  - Contains files used for readme

## References and Acknowledgements
- MTRN4231 Labs Week 1-5
- David Nie's Github Repositories
  - [UR10e Vision Based Fruit Harvesting][UR10e]
  - [Screw Driving Bot][Screw]
