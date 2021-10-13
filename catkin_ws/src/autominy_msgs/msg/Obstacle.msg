# Detected or simulated object
################################################################################

# Header
# header.frame_id defines reference frame
Header header

# Object frame (analogous to child_frame_id in nav_msgs/Odometry)
string object_frame_id


# ID for tracking
uint16 object_id

# duration this object has been tracked for
duration age

# duration since last update/confirmation by measurement
# (set to 0 as soon as a measurement update is available)
duration prediction_age


# odometry of the object (position, orientation, linear and angular velocities)
# odom.header.frame_id is header.frame_id
# child_frame_id is object_frame_id
nav_msgs/Odometry odom

# maximal size of the object (x,y,z) or (depth, width, height) [m]
# relative to the object frame, i.e. the orientation of the object is taken into account
geometry_msgs/Vector3 size


# The contour points of the object [m]
geometry_msgs/Point[] contour_points


# definition of most likely class of this object:
#   0: unclassified
#   1: unknown small
#   2: unknown big
#   3: pedestrian
#   4: bike
#   5: car
#   6: truck
uint16 UNCLASSIFIED    = 0
uint16 UNKNOWN_SMALL   = 1
uint16 UNKNOWN_BIG     = 2
uint16 PEDESTRIAN      = 3
uint16 BIKE            = 4
uint16 CAR             = 5
uint16 TRUCK           = 6

# most likely class of this object
uint16 classification

# duration this object has been classified as current class
duration classification_age

# The higher this value the more reliable the assigned object class [0,1]
float64 classification_certainty

