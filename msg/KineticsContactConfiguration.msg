uint32 id
string name
bool has_wrench_sensor
bool flat_odometry

float64[144] initial_covariance
float64[144] new_contact_initial_covariance
float64[144] process_covariance
float64[36] wrench_covariance
float64[9] linear_stiffness
float64[9] linear_damping
float64[9] angular_stiffness
float64[9] angular_damping

# Constant rotation (rotation vector, radians, contact frame) applied to the measured wrench
# before it reaches the observer, correcting a force-sensor calibration tilt. Identified from
# single support, where the whole ground reaction passes through one foot and must therefore be
# vertical: RHPS1 reads a tangential force of 0.29x the normal load in a direction fixed to each
# foot, which cannot be real, while HRP5-P reads 0.04x. Zero (the default) leaves the wrench alone.
float64[3] wrench_tilt_correction
