#!/usr/bin/python3
import rospy
import time

import numpy as np

from numpy import *
from tf.transformations import euler_from_quaternion

from model_2013 import Model, Variable_With_Delay
from kalman_quadro_no_xyz_dots import Kalman

from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3Stamped
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import QuaternionStamped

glob_imu_msg = Imu()
glob_rpy_msg = Vector3Stamped()
glob_control_msg = QuaternionStamped()
glob_pose_msg = PoseStamped()
glob_remote_msg = PointStamped()


def callbackRemote(msg):
	global glob_remote_msg
	glob_remote_msg = msg

imuUp = False
def callbackImu(msg):
	global glob_imu_msg, imuUp
	glob_imu_msg = msg
	if not imuUp:
		imuUp = True

def callbackRPY(msg):
	global glob_rpy_msg
	glob_rpy_msg = msg
	
controlUp = False
def callbackControl(msg):
	global glob_control_msg, controlUp
	glob_control_msg = msg
	if not controlUp:
		controlUp = True

cameraUp = False
def callbackPose(msg):
	global glob_pose_msg, cameraUp
	glob_pose_msg = msg
	if not cameraUp:
		cameraUp = True

camRawFile = open('CAM_raw.csv', 'w')
camStFile = open('CAM_stepped.csv', 'w')
camKalFile = open('CAM_kalman.csv', 'w')
cam2degFile = open('CAM_2deg.csv', 'w')
camRefFile = open('CAM_ref.csv', 'w')
camRPYRoughFile = open('CAM_rpy_rough.csv', 'w')

camRawData = []
camStData = []
camKalData = []
cam2degData = []
camRefData = []
camRPYRoughData = []

def dump_data():
	for line in camRawData:
		camRawFile.write(line)
	for line in camStData:
		camStFile.write(line)
	for line in camKalData:
		camKalFile.write(line)
	for line in cam2degData:
		cam2degFile.write(line)
	for line in camRefData:
		camRefFile.write(line)
	for line in camRPYRoughData:
		camRPYRoughFile.write(line)
	camRawFile.close()
	camStFile.close()
	camKalFile.close()
	cam2degFile.close()
	camRefFile.close()
	camRPYRoughFile.close()

def camera_handler():
	global glob_imu_msg, glob_pose_msg
	#global camRawFile

	rospy.init_node('camera_handler', anonymous=True)
	rospy.on_shutdown(dump_data)

	rospy.Subscriber("/remote", PointStamped, callbackRemote);
	rospy.Subscriber("/imu/_data_mod", Imu, callbackImu) # stepped / was data_mod
	rospy.Subscriber("/imu/rpy/_predicted", Vector3Stamped, callbackRPY) # stepped / was predicted
	rospy.Subscriber("/controls", QuaternionStamped, callbackControl)
	rospy.Subscriber("/mavros/vision_pose/pose", PoseStamped, callbackPose)


	pubRef = rospy.Publisher('/camera/ref', QuaternionStamped, queue_size=1)
	pubXYZ = rospy.Publisher('/camera/xyz/kalman', Vector3Stamped, queue_size=1)
	pubXYZ2 = rospy.Publisher('/camera/xyz/2degree', Vector3Stamped, queue_size=1)
	pubVel = rospy.Publisher('/camera/vel/kalman', Vector3Stamped, queue_size=1)
	pubVel2 = rospy.Publisher('/camera/vel/2degree', Vector3Stamped, queue_size=1)
	pubRPY = rospy.Publisher('/camera/rpy/kalman', Vector3Stamped, queue_size=1)

	pubVelRough = rospy.Publisher('/camera/vel/rough', Vector3Stamped, queue_size=1)
	pubRPYRough = rospy.Publisher('/camera/rpy/rough', Vector3Stamped, queue_size=1)

	rate = rospy.Rate(100) # 100hz

	#cam2degFile = open('CAM_2deg.csv', 'w')

	x0 = 0.
	y0 = 0.
	z0 = 0. # was 0!

	#while not imuUp and cameraUp:
	#	pass
	
	rospy.loginfo("camera_handler initialized")
	#while (x0 == 0):
	#	x0 = glob_pose_msg.pose.position.x
	#	y0 = glob_pose_msg.pose.position.y
	#	z0 = glob_pose_msg.pose.position.z

	model = Model(x=x0, y=y0, z=z0)
	model_ref = Model(x=x0, y=y0, z=z0)
	kalman = Kalman()
	x_estimated = array([x0, y0, z0, 0., 0., 0., 0., 0., 0., 0., 0., 0.], dtype = double)
	covariances = 0.5*eye(12)

	control = [0.,0.,0.,0.]

	delay1 = 10 # 12 # old model - 16
	u1_q = Variable_With_Delay(delay=delay1)
	u2_q = Variable_With_Delay(delay=delay1)
	u3_q = Variable_With_Delay(delay=delay1)
	u4_q = Variable_With_Delay(delay=delay1)

	
	x_cam2 = 0
	y_cam2 = 0
	z_cam2 = 0

	x_cam_dot2 = 0
	y_cam_dot2 = 0
	z_cam_dot2 = 0
	
	time_prev = rospy.get_time()
	time_nsteps = 0
	dt = 0.01
	
	x_prev=0
	y_prev=0
	z_prev=0

	torque_ref = 0
	phi_ref = 0
	teta_ref = 0
	psi_ref = 0

	count = 0
	
	timestart = rospy.get_time()

	while not rospy.is_shutdown():
		dt = rospy.get_time() - time_prev
		time_prev = rospy.get_time()
		time_nsteps += 1
		
		x_cam = glob_pose_msg.pose.position.x
		y_cam = glob_pose_msg.pose.position.y
		z_cam = glob_pose_msg.pose.position.z # + random.normal(0, 0.005, 1)[0]
		vx_cam = (x_cam-x_prev) / dt
		vy_cam = (y_cam-y_prev) / dt
		vz_cam = (z_cam-z_prev) / dt
		x_prev = x_cam
		y_prev = y_cam
		z_prev = z_cam

		camRawData.append(f'{rospy.get_time()}, {x_cam}, {y_cam}, {z_cam}, {vx_cam}, {vy_cam}, {vz_cam}\n')
		#camRawFile.write(f'{rospy.get_time()}, {x_cam}, {y_cam}, {z_cam}, {vx_cam}, {vy_cam}, {vz_cam}\n')

		quaternion = (glob_pose_msg.pose.orientation.x, glob_pose_msg.pose.orientation.y, glob_pose_msg.pose.orientation.z, glob_pose_msg.pose.orientation.w)
		euler = euler_from_quaternion(quaternion)
		phi = glob_rpy_msg.vector.x * 1 # from controller
		theta = glob_rpy_msg.vector.y * 1 # from controller
		psi = euler[2] * 1 # from camera
		#rospy.loginfo("psi %f", euler[2])
		
		camRPYRoughData.append(f'{rospy.get_time()}, {euler[0]}, {euler[1]}, {euler[2]}\n')

		phi_dot = glob_imu_msg.angular_velocity.x * 1
		theta_dot = glob_imu_msg.angular_velocity.y * 1
		psi_dot = glob_imu_msg.angular_velocity.z * 1
		#rospy.loginfo("CAMERA_HANDLER.py dot_psi=%f dot_phi=%f dot_teta=%f", psi_dot, phi_dot, theta_dot)

		# SECOND DEGREE
		d_dxy = 1.
		tau_dxy = 0.05 # 0.1
		k2_dxy = 1./(tau_dxy**2.)
		k_dxy = 1. * k2_dxy
		k1_dxy = 2.*d_dxy*tau_dxy*k2_dxy

		# 2 degree filter for estimating derivatives for x, y, z
		x_cam_dot2 += (-k1_dxy * x_cam_dot2 - k2_dxy * x_cam2 + k_dxy * x_cam) * dt
		x_cam2 += x_cam_dot2 * dt
		y_cam_dot2 += (-k1_dxy * y_cam_dot2 - k2_dxy * y_cam2 + k_dxy * y_cam) * dt
		y_cam2 += y_cam_dot2 * dt
		z_cam_dot2 += (-k1_dxy * z_cam_dot2 - k2_dxy * z_cam2 + k_dxy * z_cam) * dt
		z_cam2 += z_cam_dot2 * dt
		
		cam2degData.append(f'{rospy.get_time()}, {x_cam2}, {y_cam2}, {z_cam2}, {x_cam_dot2}, {y_cam_dot2}, {z_cam_dot2}\n')

		#rospy.loginfo("z_cam=%f z_cam_dot=%f", z_cam, vz_cam);
		#rospy.loginfo("u1=%f u2=%f u3=%f u4=%f", control[0], control[1], control[2], control[3])

		
		x_current = array([x_cam, y_cam, z_cam, psi, phi, theta, psi_dot, phi_dot, theta_dot], dtype = double)
		#x_current = array([x_cam, y_cam, z_cam, x_cam_dot2, y_cam_dot2, z_cam_dot2, psi, phi, theta, psi_dot, phi_dot, theta_dot], dtype = double)
		
		model_ref.x, model_ref.y, model_ref.z, model_ref.psi, model_ref.phi, model_ref.theta, model_ref.dot.psi, model_ref.dot.phi, model_ref.dot.theta = x_current
		model_ref.dot.x, model_ref.dot.y, model_ref.dot.z = model.dot.x, model.dot.y, model.dot.z
		#model_ref.x, model_ref.y, model_ref.z, model_ref.dot.x, model_ref.dot.y, model_ref.dot.z, model_ref.psi, model_ref.phi, model_ref.theta, model_ref.dot.psi, model_ref.dot.phi, model_ref.dot.theta = x_current
		
		n = 0
		while n < delay1: # delay additional steps
			model_ref.set_control([u1_q.get_index(delay1-n), u2_q.get_index(delay1-n), u3_q.get_index(delay1-n), u4_q.get_index(delay1-n)])
			model_ref.step(0.01)
			n += 1
		camStData.append(f'{rospy.get_time()}, {model_ref.x}, {model_ref.y}, {model_ref.z}, {model_ref.dot.x}, {model_ref.dot.y}, {model_ref.dot.z}\n')

		x_current_pred = array([model_ref.x, model_ref.y, model_ref.z, model_ref.psi, model_ref.phi, model_ref.theta, model_ref.dot.psi, model_ref.dot.phi, model_ref.dot.theta], dtype = double)
		#x_current_pred = array([model_ref.x, model_ref.y, model_ref.z, model_ref.dot.x, model_ref.dot.y, model_ref.dot.z, model_ref.psi, model_ref.phi, model_ref.theta, model_ref.dot.psi, model_ref.dot.phi, model_ref.dot.theta], dtype = double)
		
		x_prediction = array([model.x, model.y, model.z, model.dot.x, model.dot.y, model.dot.z, model.psi, model.phi, model.theta, model.dot.psi, model.dot.phi, model.dot.theta], dtype = double)

		x_estimated[:] = x_prediction[:]
		
		x_estimated, covariances = kalman.step(x_estimated, covariances, x_current_pred.reshape(9,1), x_prediction.reshape(12,1), control, [model.k1_psi, model.k1_phi, model.k1_theta, model.k2_psi, model.k2_phi, model.k2_theta, model.mass], dt)
		#x_estimated, covariances = kalman.step(x_estimated, covariances, x_current_pred.reshape(12,1), x_prediction.reshape(12,1), control, [model.k1_psi, model.k1_phi, model.k1_theta, model.k2_psi, model.k2_phi, model.k2_theta, model.mass], dt)
				 
		#rospy.loginfo("\nx=%f \ny=%f \nz=%f", x_estimated[0], x_estimated[1], x_estimated[2]);
		camKalData.append(f'{rospy.get_time()}, {x_estimated[0]}, {x_estimated[1]}, {x_estimated[2]}, {x_estimated[3]}, {x_estimated[4]}, {x_estimated[5]}\n')

		# Reference
		M = 0.43
		g = 9.81

		'''
		timelim = 20 # [s]
		if (rospy.get_time() - timestart) < timelim:
			x_ref = 0.
			y_ref = 0.
			z_ref = 1.2

		if (rospy.get_time() - timestart) > timelim:
			x_ref = 0.
			y_ref = 0.
			z_ref = 1.2
				
		if (rospy.get_time() - timestart) > 2*timelim:
			x_ref = 0.
			y_ref = 0.
			z_ref = 1.2
		'''
		x_ref = -0.5
		y_ref = 0.
		z_ref = 1.2
		
		
		a_x = k_x = a_y = k_y = a_z = k_z = 4. # 1.

		H_xx = - (a_x+k_x)*x_estimated[3] - a_x*k_x*(x_estimated[0]-x_ref) # ZERO VEL
		H_yy = - (a_y+k_y)*x_estimated[4] - a_y*k_y*(x_estimated[1]-y_ref)
		H_zz = - (a_z+k_z)*x_estimated[5] - a_z*k_z*(x_estimated[2]-z_ref) + g
		# From model
		#H_xx = - (a_x+k_x)*model_ref.dot.x - a_x*k_x*(model_ref.x-x_ref)
		#H_yy = - (a_y+k_y)*model_ref.dot.y - a_y*k_y*(model_ref.y-y_ref)
		#H_zz = - (a_z+k_z)*model_ref.dot.z - a_z*k_z*(model_ref.z-z_ref) + g
		
		#rospy.loginfo("x=%f y=%f z=%f dot_x=%f dot_y=%f dot_z=%f", x_estimated[0], x_estimated[1], x_estimated[2], x_estimated[3], x_estimated[4], x_estimated[5])

		if controlUp:
			torque_ref = M * sqrt(H_zz*H_zz)
			#torque_ref = M * sqrt(H_zz*H_zz + H_xx*H_xx)
			
			#torque_ref = M * g
			#torque_ref = M * sqrt(H_xx*H_xx + H_yy*H_yy + H_zz*H_zz)
			phi_ref = 0 * arctan(- H_yy / sqrt(H_xx*H_xx + H_zz*H_zz))
			teta_ref = 0 * arctan(H_xx / H_zz)
				
			psi_ref = 0
			#count += 1
			#if count == 10:
				#count = 0
				#rospy.loginfo("H_xx=%f H_yy=%f H_zz=%f", H_xx, H_yy, H_zz)
				#rospy.loginfo("teta_ref=%f\n", teta_ref)

		# Limits
		modelOn = False
		if not modelOn:
			if torque_ref > glob_remote_msg.point.x:
				torque_ref = glob_remote_msg.point.x
		
		if (phi_ref > 0.3):
			phi_ref = 0.3
		elif (phi_ref < -0.3):
			phi_ref = -0.3
			
		if (teta_ref > 0.3):
			teta_ref = 0.3
		elif (teta_ref < -0.3):
			teta_ref = -0.3
		
		if torque_ref > 8:
			torque_ref = 8
			
		camRefData.append(f'{rospy.get_time()}, {torque_ref}, {phi_ref}, {teta_ref}, {psi_ref}, {x_ref}, {y_ref}, {z_ref}\n')

		# Model step
		control[0] = torque_ref # glob_control_msg.quaternion.x
		control[1] = phi_ref # glob_control_msg.quaternion.y * 0 # now it's ref angle, not u!
		control[2] = teta_ref # glob_control_msg.quaternion.z * 0 # now it's ref angle, not u!
		control[3] = psi_ref # glob_control_msg.quaternion.w * 0 # now it's ref angle, not u!
		u1_q.value = control[0]
		u2_q.value = control[1]
		u3_q.value = control[2]
		u4_q.value = control[3]
		
		#rospy.loginfo("u1=%f", torque_ref)
		
		model.x, model.y, model.z, model.dot.x, model.dot.y, model.dot.z, model.psi, model.phi, model.theta, model.dot.psi, model.dot.phi, model.dot.theta = x_estimated
		model.set_control(control)
		model.step(dt)

		# Messages
		msg_ref_cam = QuaternionStamped()
		msg_ref_cam.header.stamp = rospy.get_rostime()
		msg_ref_cam.quaternion.x = torque_ref
		msg_ref_cam.quaternion.y = phi_ref
		msg_ref_cam.quaternion.z = teta_ref
		msg_ref_cam.quaternion.w = psi_ref

		msg_xyz_cam = Vector3Stamped()
		msg_xyz_cam.header.stamp = rospy.get_rostime()
		msg_xyz_cam.vector.x = x_estimated[0]
		msg_xyz_cam.vector.y = x_estimated[1]
		msg_xyz_cam.vector.z = x_estimated[2]
		
		
		msg_xyz_cam2 = Vector3Stamped()
		msg_xyz_cam2.header.stamp = rospy.get_rostime()
		msg_xyz_cam2.vector.x = x_cam2
		msg_xyz_cam2.vector.y = y_cam2
		msg_xyz_cam2.vector.z = z_cam2
		
		
		msg_vel_cam = Vector3Stamped()
		msg_vel_cam.header.stamp = rospy.get_rostime()
		msg_vel_cam.vector.x = x_estimated[3]
		msg_vel_cam.vector.y = x_estimated[4]
		msg_vel_cam.vector.z = x_estimated[5]
		
		msg_vel_cam2 = Vector3Stamped()
		msg_vel_cam2.header.stamp = rospy.get_rostime()
		msg_vel_cam2.vector.x = x_cam_dot2
		msg_vel_cam2.vector.y = y_cam_dot2
		msg_vel_cam2.vector.z = z_cam_dot2
		
		msg_velRough_cam = Vector3Stamped()
		msg_velRough_cam.header.stamp = rospy.get_rostime()
		msg_velRough_cam.vector.x = (x_cam-x_prev) / dt
		msg_velRough_cam.vector.y = (y_cam-y_prev) / dt
		msg_velRough_cam.vector.z = (z_cam-z_prev) / dt
		x_prev = x_cam
		y_prev = y_cam
		z_prev = z_cam
		
		msg_rpy_cam = Vector3Stamped() # Specific order
		msg_rpy_cam.header.stamp = rospy.get_rostime()
		msg_rpy_cam.vector.x = x_estimated[7]
		msg_rpy_cam.vector.y = x_estimated[8]
		msg_rpy_cam.vector.z = x_estimated[6]
		
		msg_rpy_rough = Vector3Stamped() # Specific order
		msg_rpy_rough.header.stamp = rospy.get_rostime()
		msg_rpy_rough.vector.x = euler[0]
		msg_rpy_rough.vector.y = euler[1]
		msg_rpy_rough.vector.z = euler[2]
		
		

		# Publishers
		pubRef.publish(msg_ref_cam)

		pubXYZ.publish(msg_xyz_cam)
		pubXYZ2.publish(msg_xyz_cam2)
		pubVel.publish(msg_vel_cam)
		pubVel2.publish(msg_vel_cam2)
		pubRPY.publish(msg_rpy_cam)
		
		pubVelRough.publish(msg_velRough_cam)
		pubRPYRough.publish(msg_rpy_rough)
		
		rate.sleep()

if __name__ == '__main__':
	try:
		camera_handler()
	except rospy.ROSInterruptException:
		pass
