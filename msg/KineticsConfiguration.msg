std_msgs/Header header
uint32 max_contacts
uint32 max_imus
float64 sampling_time
float64 mass

bool with_unmodeled_wrench
bool with_gyro_bias
bool with_acceleration_estimation
bool with_damping_in_matrix_a
bool with_adaptative_contact_process_covariance
# Weights the mean that the adaptive contact process covariance pins, by contact load:
# w_i ~ f_i^exponent. 0 keeps the arithmetic mean (unchanged behaviour), 1 is load-proportional.
float64 contact_cov_load_weight_exponent

# Project the state update off the unobservable global-yaw direction. Default false keeps the

bool use_finite_difference_jacobians
float64 finite_difference_step

float64[9] state_position_initial_covariance
float64[9] state_orientation_initial_covariance
float64[9] state_linear_velocity_initial_covariance
float64[9] state_angular_velocity_initial_covariance
float64[9] gyro_bias_initial_covariance
float64[36] unmodeled_wrench_initial_covariance
float64[144] contact_initial_covariance

float64[9] state_position_process_covariance
float64[9] state_orientation_process_covariance
float64[9] state_linear_velocity_process_covariance
float64[9] state_angular_velocity_process_covariance
float64[9] gyro_bias_process_covariance
float64[36] unmodeled_wrench_process_covariance
float64[144] contact_process_covariance

kinetics_observer_ros2/KineticsImuConfiguration[] imus
kinetics_observer_ros2/KineticsContactConfiguration[] contacts

float64[] initial_state
