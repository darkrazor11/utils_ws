# This represents obstacles' position with reference to polar coordinate frame and timestamp.
std_msgs/Header header
int8 tracking_id
float64 range  # meters
float64 bearing  # degrees
float64 range_extent  # meters
float64 bearing_extent # degrees
geometry_msgs/Vector3 velocity # m/s
float64 heading # radians wrt to x-axis