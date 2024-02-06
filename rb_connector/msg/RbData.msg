
std_msgs/Header header
int32           command_on_passing

float32         time			#[s]
float32[6]      joint_reference		#[deg]
float32[6]      joint_encoder		#[deg]
float32[6]      joint_current		#[mA]
float32[6]      tcp_reference
float32[6]      temperature		#[celcius]
int32[6]        joint_information


float32[4]      analog_in
float32[4]      analog_out
int32[16]       digital_in
int32[16]       digital_out
float32[2]      tfb_analog_in
int32[2]        tfb_digital_in
int32[2]        tfb_digital_out
float32	        tfb_voltage_out


float32	        default_speed		#overriding speed [0~1]

int32	        task_state
int32	        robot_state		#1:idle 2:paused/stopped by accident 3:moving
int32	        power_state
int32	        collision_detect	#0:off 1:on
int32	        freedrive_mode		#0:off 1:on
int32	        program_mode		#0:real 1:simulation


int32	        op_stat_collision_occur
int32           op_stat_sos_flag
int32           op_stat_self_collision
int32           op_stat_soft_estop_occur
int32           op_stat_ems_flag
	
float32[6]      joint_simulation		#[deg]
float32[6]      tcp_simulation		#[m]
