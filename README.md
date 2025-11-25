<!-- omit from toc -->
# SCREWDRIVING BOT 1

- [Project Overview](#project-overview)
  - [System Functionality and Novelty](#system-functionality-and-novelty)
  - [Video Demonstration](#video-demonstration)
- [System Architecture](#system-architecture)
  - [ROS Packages and Node Descriptions](#ros-packages-and-node-descriptions)
  - [Interfaces (Custom Messages and Services)](#interfaces-custom-messages-and-services)
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
  - [Opportunities for Improvement (Version 2.0)](#opportunities-for-improvement-version-20)
  - [Novelty and Effectiveness Summary](#novelty-and-effectiveness-summary)
- [Contributors and Roles](#contributors-and-roles)
- [Repository Structure](#repository-structure)
- [References and Acknowledgements](#references-and-acknowledgements)

[Minh]: https://www.linkedin.com/in/nickojbell/
[Samuel]: https://www.linkedin.com/in/jiawen-zhang-1aa622203/
[Brent]: https://www.linkedin.com/in/daryl-lee-7b022a201/
[David]: https://www.linkedin.com/in/davidnie0418/

## 🚀 Project Overview

Something something project overview

**Project Duration**: 6 weeks

**Authors**:
* Minh Thang Phan: [Minh's LinkedIn Profile][Minh]
* Samuel Gray: [Samuel's LinkedIn profile][Samuel]
* Brent Poon: [Brent's LinkedIn profile][Brent]

**Supervisor**:
* David Nie: [David's LinkedIn profile][David]


### System Functionality and Novelty
*Summarise the robot's functionality.*

### Video Demonstration

Video link:
https://www.youtube.com/shorts/ufF1myLWomA

<div align="center">
  <img src="img/demo_img.png" alt="cad" width="100%">
</div>

## ⚙️ System Architecture

[cite_start]This section is for documenting your ROS2 architecture and integration[cite: 26].

### ROS Packages and Node Descriptions

### Interfaces (Custom Messages and Services)


### Closed-Loop System Behaviour

#### Motion Plan Overview

#### System Flowchart


## 🛠️ Technical Components

### Computer Vision

### Custom End-Effector


### System Visualisation

### Closed-Loop Operation and Feedback

* **Feedback Method:** The depth camera provides visual feedback (hole centroid) which is converted into the `OOI` frame.
* **Adaptation:** This real-time coordinate information is used by the `transformations` package to calculate a Cartesian movement command for the UR5e's `base_link`, allowing the robot to precisely approach the hole despite initial inaccuracies or shifts in the work piece (housing block).

---

## ⚙️ Installation and Setup


### Prerequisites and Dependencies


### Workspace Setup


### Hardware Setup and Calibration

-----------------------------------------------
## ▶️ Running the System


### Launching the Complete System
To run the full system with the end-effector connected:

**Step 1**: In one terminal, run the master launch file:
`ros2 launch brain system_launch.py`

### Example Commands and Expected Output

### Testing Routines



--------------------------------------

## 📊 Results and Demonstration

[cite_start]*Describe how your system performs against its design goals and include quantitative results (e.g., accuracy, repeatability)[cite: 70, 71].*

### Performance Against Design Goals
*(Discuss the success of the system based on the initial requirements.)*

### Quantitative Results (Accuracy, Repeatability)
*(Include data or figures showing performance metrics.)*

### Robustness, Adaptability, and Innovation

--------------------------------------------------------------
## 🗣️ Discussion and Future Work

### Engineering Challenges and Solutions

### Opportunities for Improvement (Version 2.0)

## 📁 Repository Structure
