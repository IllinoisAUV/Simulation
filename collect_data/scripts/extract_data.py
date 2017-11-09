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
mag_m_data     = extract_data("magnetometer_measured.txt")
mag_t_data     = extract_data("magnetometer_true.txt")
gps_data       = extract_data("gps.txt")
accel_b_data   = extract_data("accel_b.txt")
accel_w_data   = extract_data("accel_w.txt")
pose_gt_data   = extract_data("pose_gt.txt")
pressure_data  = extract_data("pressure.txt")
clock_data     = extract_data("clock.txt")
g_model_t_data = extract_data("g_model_states_twist.txt")
g_model_p_data = extract_data("g_model_states_position.txt")

#imu data
# (x , y, z) rad/s
measured_angular_velocity    = np.empty( [len(imu_data), 3], dtype=float)
# (x , y, z) m/s^2
measured_linear_accelaration = np.empty( [len(imu_data), 3], dtype=float)
# (x, y, z, w) quaternion
measured_orientation         = np.empty( [len(imu_data), 4], dtype=float)
# nano secs
imu_clock                    = np.empty( [len(imu_data), 1], dtype=float)


#magnetometer
#magentic field (x, y, z) Micro Tesla
measured_magnetic_field      = np.empty( [len(mag_m_data),3], dtype=float)
true_magnetic_field          = np.empty( [len(mag_t_data),3], dtype=float)
# nano secs
measure_mag_clock            = np.empty( [len(mag_t_data),1], dtype=float)
# nano secs
true_mag_clock               = np.empty( [len(mag_t_data),1], dtype=float)

#gps
#gps_data  (longitude , latitude, altitude) tuple
measured_gps_position        = np.empty( [len(gps_data), 3], dtype=float)
# nano secs
gps_clock                    = np.empty( [len(gps_data), 1], dtype=float)

#pressure I think there is offset of 100kpa in pressure sensor, it starts at 100
#fluid pressure kPa/m
measured_pressure 			 = np.empty([len(pressure_data),1], dtype=float)
# nano secs
pressure_clock               = np.empty([len(pressure_data),1], dtype=float)

# linear accelerarion body m/s^2
# true accelerarion ( x , y, z) quaternion
true_body_accelaration       = np.empty( [len(accel_b_data), 3], dtype=float)
# nano secs
true_body_accel_clock        = np.empty( [len(clock_data), 1], dtype=float)

# linear_acceleration world m/s^2
# (x , y, z) quaternion
true_world_accelaration      = np.empty( [len(accel_w_data), 3], dtype=float)
# nano secs
true_world_accel_clock       = np.empty( [len(clock_data), 1], dtype=float)

# body position, orientation, angular velocity
# (x , y, z) meters
true_body_postition          = np.empty( [len(pose_gt_data), 3], dtype=float)
# (x , y, z , w) quaternion
true_body_orientation        = np.empty( [len(pose_gt_data), 4], dtype=float)
# (x , y, z) rad/s
true_body_angular_velocity   = np.empty( [len(pose_gt_data), 3], dtype=float)
# nano secs, same clock for body position, orientation, angular velocity
true_body_poa_clock          = np.empty( [len(pose_gt_data), 1], dtype=float)


# world position, orientation , angular_velocity
# (x , y, z) meters
true_world_postition          = np.empty( [len(g_model_p_data), 3], dtype=float)
# (x , y, z, w) quaternion
true_world_orientation        = np.empty( [len(g_model_p_data), 4], dtype=float)
# (x , y, z) rad/s
true_world_angular_velocity   = np.empty( [len(g_model_t_data), 3], dtype=float)
# nano secs, same clock for world position, orientation, angular velocity
true_world_poa_clock          = np.empty( [len(clock_data),     1], dtype=float)


print "parsing data"

####################################################################################################
# IMU
for i in range(len(imu_data)):
	lin_accel = imu_data[i]["linear_acceleration"]
	orien     = imu_data[i]["orientation"]
	ang_vel   = imu_data[i]["angular_velocity"]

	measured_linear_accelaration[i] = ( lin_accel["x"], lin_accel["y"], lin_accel["z"]  )
	measured_angular_velocity[i]    = ( ang_vel["x"], ang_vel["y"], ang_vel["z"]  )
	measured_orientation[i]         = ( orien["x"], orien["y"], orien["z"] , orien["w"]  )
	imu_clock[i] 					= imu_data[i]["header"]["stamp"]["nsecs"]

#####################################################################################################
# GPS
for i in range(len(gps_data)):
	gps_d = gps_data[i]
	measured_gps_position[i] = (gps_d["longitude"], gps_d["latitude"], gps_d["altitude"])
	gps_clock[i] 			 = gps_data[i]["header"]["stamp"]["nsecs"]

#####################################################################################################
# Measured Magnetomter
for i in range(len(mag_m_data)):
	mag_m_f = mag_m_data[i]["magnetic_field"]
	measured_magnetic_field[i] = (mag_m_f["x"], mag_m_f["y"], mag_m_f["z"])
	measure_mag_clock[i] 	   = mag_m_data[i]["header"]["stamp"]["nsecs"]

# True Magnetomter
for i in range(len(mag_t_data)):
	mag_t_f = mag_t_data[i]["magnetic_field"]
	true_magnetic_field[i] = (mag_t_f["x"], mag_t_f["y"], mag_t_f["z"])
	true_mag_clock[i] 	   = mag_t_data[i]["header"]["stamp"]["nsecs"]

#####################################################################################################
# True Acceleration Body and World Frame
for i in range(len(true_body_accelaration)):
	lin_acc_b = accel_b_data[i]["linear"]
	true_body_accelaration[i]  = (lin_acc_b["x"], lin_acc_b["y"], lin_acc_b["z"])

for i in range(len(true_world_accelaration)):
	lin_acc_w = accel_w_data[i]["linear"]
	true_world_accelaration[i] = (lin_acc_w["x"], lin_acc_w["y"], lin_acc_w["z"])
#####################################################################################################
# True Body position, orientation and angular_velocity
for i in range(len(pose_gt_data)):
	pos     = pose_gt_data[i]["pose"]["pose"]["position"]
	orien   = pose_gt_data[i]["pose"]["pose"]["orientation"]
	ang_vel = pose_gt_data[i]["twist"]["twist"]["angular"]

	true_body_postition[i]        = (pos["x"], pos["y"], pos["z"])
	true_body_orientation[i]      = (orien["x"], orien["y"], orien["z"], orien["w"])
	true_body_angular_velocity[i] = (ang_vel["x"], ang_vel["y"], ang_vel["z"])
	true_body_poa_clock[i]        = pose_gt_data[i]["header"]["stamp"]["nsecs"]

#####################################################################################################
# True World position, orientation and angular_velocity
for i in range(len(g_model_p_data)):
	pos     = g_model_p_data[i]["position"]
	orien   = g_model_p_data[i]["orientation"]
	true_world_postition[i]        = (pos["x"], pos["y"], pos["z"])
	true_world_orientation[i]      = (orien["x"], orien["y"], orien["z"], orien["w"])

for i in range(len(g_model_t_data)):
	ang_vel = g_model_t_data[i]["angular"]
	true_world_angular_velocity[i] = (ang_vel["x"], ang_vel["y"], ang_vel["z"])
#####################################################################################################
#Pressure
for i in range(len(pressure_data)):
	measured_pressure[i] = pressure_data[i]["fluid_pressure"]
	pressure_clock[i] 	 = pressure_data[i]["header"]["stamp"]["nsecs"]
#####################################################################################################
#Clock
for i in range(len(clock_data)):
	true_body_accelaration[i] = clock_data[i]["clock"]["nsecs"]
	true_world_accel_clock[i] = clock_data[i]["clock"]["nsecs"]
	true_world_poa_clock[i] = clock_data[i]["clock"]["nsecs"]

#####################################################################################################

print "done"
