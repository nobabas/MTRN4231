Run ur_with_endeffector launch file
Run server
try service calls commands
NOTE - zline constraint does work, just takes a lot of tries
line command is
ros2 service call /moveit_path_plan interfaces/srv/MoveRequest "{command: 'line', positions: [0.0, 0.0, 0.25, 0.0, 0.0, 0.0]}"
where z is the desired z height
