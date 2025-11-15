To run brain.cpp type; 
ros2 launch brain system_launch.py

To start soil sampling: 
ros2 service call /brain_srv interfaces/srv/BrainCmd "{command: 'soil_sampling'}"
