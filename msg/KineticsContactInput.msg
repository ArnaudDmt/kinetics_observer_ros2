uint32 id
bool active

kinetics_observer_ros2/KineticsKinematics initial_world_kinematics
kinetics_observer_ros2/KineticsKinematics user_contact_kinematics
geometry_msgs/Wrench measured_wrench
float64[36] wrench_covariance

# Row-major 6x6 transform carrying a wrench covariance from the force sensor frame to the
# contact frame. When set, the observer transports the *configured* contact_wrench through it
# instead of using wrench_covariance, so that covariance stays tunable at replay time.
float64[36] wrench_covariance_transform
