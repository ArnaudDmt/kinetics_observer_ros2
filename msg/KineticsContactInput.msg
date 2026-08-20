uint32 id
string name
bool active
bool has_wrench_sensor
bool flat_odometry

kinetics_observer_ros2/KineticsKinematics initial_world_kinematics
kinetics_observer_ros2/KineticsKinematics user_contact_kinematics
geometry_msgs/Wrench measured_wrench

float64[144] initial_covariance
float64[144] process_covariance
float64[36] wrench_covariance
float64[9] linear_stiffness
float64[9] linear_damping
float64[9] angular_stiffness
float64[9] angular_damping
