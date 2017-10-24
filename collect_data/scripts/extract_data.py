import numpy as np
import json
import math

#parameters
# imu = 50hz
# pressure = 50hz
# magnetometer = 500hz
# gps = 10hz
# true body postition = 30hz
# true world postition = 500hz
# body and world accelerarion = 500hz

'''
IMU stats
	mass_imu_sensor="0.015"
	gyroscope_noise_density="0.0003394"
	gyroscopoe_random_walk="0.000038785"
	gyroscope_bias_correlation_time="1000.0"
	gyroscope_turn_on_bias_sigma="0.0087"
	accelerometer_noise_density="0.004"
	accelerometer_random_walk="0.006"
	accelerometer_bias_correlation_time="300.0"
	accelerometer_turn_on_bias_sigma="0.1960"
	orientation_noise="0.01"
`'''

'''
Magnetomter stats
	mass="0.015"
	update_period="0.02"
	mag_intensity="65.0"
	noise_xy="1.0"
	noise_z="1.4"
	turn_on_bias="2.0">

'''

'''
GPS stats
	mass_gps_sensor="0.015"
    horizontal_pos_std_dev="0.0"
    vertical_pos_std_dev="0.0"
    horizontal_vel_std_dev="0.0"
    vertical_vel_std_dev="0.0">
'''

'''
Pressure stats
	mass="0.015"
    update_period="0.02"
   	range="30000"
   	stddev="3.0"
   	estimateDepth="false"
   	standardPressure="101.325"
   	kPaPerM="9.80638">
'''

def extract_data(filename):
	with open(filename) as data_file:
		raw_data = data_file.read()
		tweaked_data = raw_data.replace('""', '"<&>"')
		split_data = tweaked_data.split('<&>')
		parsed_data = []
		for bit_of_data in split_data:
			parsed_data.append(json.loads(json.loads(bit_of_data)))
	return parsed_data

imu_data       = extract_data("imu.txt")
mag_data       = extract_data("magnetometer.txt")
gps_data       = extract_data("gps.txt")
accel_b_data   = extract_data("accel_b.txt")
accel_w_data   = extract_data("accel_w.txt")
pose_gt_data   = extract_data("pose_gt.txt")
pressure_data  = extract_data("pressure.txt")
g_model_t_data = extract_data("g_model_states_twist.txt")
g_model_p_data = extract_data("g_model_states_position.txt")



#imu data
# (x , y, z) rad/s
measured_angular_velocity    = np.empty( [len(imu_data), 3], dtype=float)
# (x , y, z) m/s^2
measured_linear_accelaration = np.empty( [len(imu_data), 3], dtype=float)
# (x, y, z, w) quaternion
measured_orientation         = np.empty( [len(imu_data),4], dtype=float)

#magnetometer
#magentic field (x, y, z) Micro Tesla
measured_magnetic_field      = np.empty( [len(mag_data),3], dtype=float)

#gps
#gps_data (longitude , latitude, altitude)
measured_gps_position        = np.empty( [len(gps_data), 3], dtype=float)
pressure (I think there is offset of 100kpa in pressure sensor, it starts at 100)
fluid pressure kPa/m
measured_pressure 			 = np.empty(len(pressure_data), dtype=float)

# linear accelerarion body m/s^2
# true accelerarion ( x , y, z) quaternion
true_body_accelaration       = np.empty( [len(accel_b_data), 3], dtype=float)

# linear_acceleration world m/s^2
# (x , y, z) quaternion
true_world_accelaration      = np.empty( [len(accel_w_data), 3], dtype=float)

# body position, orientation, angular velocity
# (x , y, z) meters
true_body_postition          = np.empty( [len(pose_gt_data), 3], dtype=float)
# (x , y, z , w) quaternion
true_body_orientation        = np.empty( [len(pose_gt_data), 4], dtype=float)
# (x , y, z) rad/s
true_body_angular_velocity   = np.empty( [len(pose_gt_data), 3], dtype=float)

# world position, orientation , angular_velocity
# (x , y, z) meters
true_world_postition          = np.empty( [len(g_model_p_data), 3], dtype=float)
# (x , y, z, w) quaternion
true_world_orientation        = np.empty( [len(g_model_p_data), 4], dtype=float)
# (x , y, z) rad/s
true_world_angular_velocity   = np.empty( [len(g_model_t_data), 3], dtype=float)


####################################################################################################
for i in range(len(imu_data)):
	lin_accel = imu_data[i]["linear_acceleration"]
	orien     = imu_data[i]["orientation"]
	ang_vel   = imu_data[i]["angular_velocity"]
	measured_linear_accelaration[i] = ( lin_accel["x"], lin_accel["y"], lin_accel["z"]  )
	measured_angular_velocity[i]    = ( ang_vel["x"], ang_vel["y"], ang_vel["z"]  )
	measured_orientation[i]         = ( orien["x"], orien["y"], orien["z"] , orien["w"]  )

#####################################################################################################

for i in range(len(gps_data)):
	gps_d = gps_data[i]
	measured_gps_position[i] = (gps_d["longitude"], gps_d["latitude"], gps_d["altitude"])

#####################################################################################################
for i in range(len(mag_data)):
	mag_f = mag_data[i]["magnetic_field"]
	measured_magnetic_field[i] = (mag_f["x"], mag_f["y"], mag_f["z"])

#####################################################################################################
for i in range(len(true_body_accelaration)):
	lin_acc_b = accel_b_data[i]["linear"]
	lin_acc_w = accel_w_data[i]["linear"]
	true_body_accelaration[i]  = (lin_acc_b["x"], lin_acc_b["y"], lin_acc_b["z"])
	true_world_accelaration[i] = (lin_acc_w["x"], lin_acc_w["y"], lin_acc_w["z"])

#####################################################################################################
for i in range(len(pose_gt_data)):
	pos     = pose_gt_data[i]["pose"]["pose"]["position"]
	orien   = pose_gt_data[i]["pose"]["pose"]["orientation"]
	ang_vel = pose_gt_data[i]["twist"]["twist"]["angular"]

	true_body_postition[i]        = (pos["x"], pos["y"], pos["z"])
	true_body_orientation[i]      = (orien["x"], orien["y"], orien["z"], orien["w"])
	true_body_angular_velocity[i] = (ang_vel["x"], ang_vel["y"], ang_vel["z"])

#####################################################################################################
for i in range(len(g_model_p_data)):
	pos     = g_model_p_data[i]["position"]
	orien   = g_model_p_data[i]["orientation"]
	ang_vel = g_model_t_data[i]["angular"]

	true_world_postition[i]        = (pos["x"], pos["y"], pos["z"])
	true_world_orientation[i]      = (orien["x"], orien["y"], orien["z"], orien["w"])
	true_world_angular_velocity[i] = (ang_vel["x"], ang_vel["y"], ang_vel["z"])

#####################################################################################################
for i in range(len(pressure_data)):
	measured_pressure[i] = pressure_data[i]["fluid_pressure"]
	print measured_pressure[i]
#####################################################################################################
