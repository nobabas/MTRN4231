// First run the vision node. The vision node send the /blue_markers_coords 
// This is then recieved by the moveit_planning_server. 
// Teensy is sending soil moisture as a topic /soil_moisture. 
// this is sent to brain, which shows whether it is in the soil. 

// Brain then sends command to movement as it goes more down until a threshold for soil value
// Waits 1 sec --> Then it keeps on going until soil value is reached 
// Then it loops again to the vision node to the next blue marker. If no new blue marker, it stops.

