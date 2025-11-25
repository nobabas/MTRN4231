# -----------------------------------------------------------
# Defines information about a detected visual marker.
#
# id:   Numeric ID or sequence index of the marker.
# pose: Flattened [x, y, z, roll, pitch, yaw] vector (6 elements)
#       representing the marker’s 3D position and orientation
#       in the camera or world frame.
# -----------------------------------------------------------

float32 id
float32[] pose