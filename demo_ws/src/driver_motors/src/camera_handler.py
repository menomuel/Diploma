#!/usr/bin/python3
import rospy
import time

from numpy import *

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

def callbackImu(msg):
	global glob_imu_msg
	glob_imu_msg = msg
	#rospy.loginfo(rospy.get_caller_id() + "I heard!")
    #rospy.loginfo(rospy.get_caller_id() + "I heard pose=(%f, %f, %f)",
	#msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)
	
def callbackRPY(msg):
	global glob_rpy_msg
	glob_rpy_msg = msg
	
controlUp = False
def callbackControl(msg):
	global glob_control_msg, controlUp
	glob_control_msg = msg
	if not controlUp:
		controlUp = True

poseUp = False
def callbackPose(msg):
	global glob_pose_msg, poseUp
	glob_pose_msg = msg
	if not poseUp:
		poseUp = True


def camera_handler():
	global glob_imu_msg, glob_pose_msg
	rospy.init_node('camera_handler', anonymous=True)

	rospy.Subscriber("/imu/data_mod", Imu, callbackImu)
	rospy.Subscriber("/imu/rpy/predicted", Vector3Stamped, callbackRPY)
	rospy.Subscriber("/controls", QuaternionStamped, callbackControl)
	rospy.Subscriber("/mavros/vision_pose/pose", PoseStamped, callbackPose)

	
	pubXYZ = rospy.Publisher('/camera/xyz/kalman', Vector3Stamped, queue_size=1)
	pubXYZ2 = rospy.Publisher('/camera/xyz/2degree', Vector3Stamped, queue_size=1)
	pubVel = rospy.Publisher('/camera/vel/kalman', Vector3Stamped, queue_size=1)
	pubVel2 = rospy.Publisher('/camera/vel/2degree', Vector3Stamped, queue_size=1)
	pubRPY = rospy.Publisher('/camera/rpy/kalman', Vector3Stamped, queue_size=1)

	rate = rospy.Rate(100) # 100hz

	x0 = 0.
	y0 = 0.
	z0 = 0.
	while (x0 == 0):
		x0 = glob_pose_msg.pose.position.x
		y0 = glob_pose_msg.pose.position.y
		z0 = glob_pose_msg.pose.position.z

	model = Model(x=x0, y=y0, z=z0)
	model_ref = Model(x=x0, y=y0, z=z0)
	kalman = Kalman()
	x_estimated = array([x0, y0, z0, 0., 0., 0., 0., 0., 0., 0., 0., 0.], dtype = double)
	covariances = 0.5*eye(12)

	control = [0.,0.,0.,0.]

	delay1 = 1
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
	
	time_prev = 0
	time_nsteps = 0
	dt = 0.01

	while not rospy.is_shutdown():
		if time_nsteps > 0:
			dt = rospy.get_time() - time_prev
			time_prev = rospy.get_time()
			time_nsteps += 1
		
		x_cam = glob_pose_msg.pose.position.x
		y_cam = glob_pose_msg.pose.position.y
		z_cam = glob_pose_msg.pose.position.z

		phi = glob_rpy_msg.vector.x
		theta = glob_rpy_msg.vector.y
		psi = glob_rpy_msg.vector.z

		phi_dot = glob_imu_msg.angular_velocity.x
		theta_dot = glob_imu_msg.angular_velocity.y
		psi_dot = glob_imu_msg.angular_velocity.z

		control[0] = glob_control_msg.quaternion.x
		control[1] = glob_control_msg.quaternion.y
		control[2] = glob_control_msg.quaternion.z
		control[3] = glob_control_msg.quaternion.w
		
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

		#rospy.loginfo("x_cam=%f x_cam_dot=%f", x_cam, x_cam_dot);
		#rospy.loginfo("u1=%f u2=%f u3=%f u4=%f", control[0], control[1], control[2], control[3])

		
		#x_current = array([x_cam, y_cam, z_cam, psi, phi, theta, psi_dot, phi_dot, theta_dot], dtype = double)
		x_current = array([x_cam, y_cam, z_cam, x_cam_dot2, y_cam_dot2, z_cam_dot2, psi, phi, theta, psi_dot, phi_dot, theta_dot], dtype = double)
		#model_ref.x, model_ref.y, model_ref.z, model_ref.psi, model_ref.phi, model_ref.theta, model_ref.dot.psi, model_ref.dot.phi, model_ref.dot.theta = x_current
		model_ref.x, model_ref.y, model_ref.z, model_ref.dot.x, model_ref.dot.y, model_ref.dot.z, model_ref.psi, model_ref.phi, model_ref.theta, model_ref.dot.psi, model_ref.dot.phi, model_ref.dot.theta = x_current
		#model_ref.dot.x, model_ref.dot.y, model_ref.dot.z = model.dot.x, model.dot.y, model.dot.z
		
		n = 1
		while n < delay1:# delay additional steps
			model_ref.set_control([u1_q.get_index(delay1-n), u2_q.get_index(delay1-n), u3_q.get_index(delay1-n), u4_q.get_index(delay1-n)])
			model_ref.step(0.01)
			n += 1

		#x_current_pred = array([model_ref.x, model_ref.y, model_ref.z, model_ref.psi, model_ref.phi, model_ref.theta, model_ref.dot.psi, model_ref.dot.phi, model_ref.dot.theta], dtype = double)
		x_current_pred = array([model_ref.x, model_ref.y, model_ref.z, model_ref.dot.x, model_ref.dot.y, model_ref.dot.z, model_ref.psi, model_ref.phi, model_ref.theta, model_ref.dot.psi, model_ref.dot.phi, model_ref.dot.theta], dtype = double)
		
		x_prediction = array([model.x, model.y, model.z, model.dot.x, model.dot.y, model.dot.z, model.psi, model.phi, model.theta, model.dot.psi, model.dot.phi, model.dot.theta], dtype = double)

		x_estimated[:] = x_prediction[:]
		#x_estimated, covariances = kalman.step(x_estimated, covariances, x_current_pred.reshape(9,1), x_prediction.reshape(12,1), control, [model.k1_psi, model.k1_phi, model.k1_theta, model.k2_psi, model.k2_phi, model.k2_theta, model.mass], dt)
		x_estimated, covariances = kalman.step(x_estimated, covariances, x_current_pred.reshape(12,1), x_prediction.reshape(12,1), control, [model.k1_psi, model.k1_phi, model.k1_theta, model.k2_psi, model.k2_phi, model.k2_theta, model.mass], dt)
		
		u1_q.value = control[0]
		u2_q.value = control[1]
		u3_q.value = control[2]
		u4_q.value = control[3]
		 
		#rospy.loginfo("\nx=%f \ny=%f \nz=%f", x_estimated[0], x_estimated[1], x_estimated[2]);
		
		# Model step
		model.x, model.y, model.z, model.dot.x, model.dot.y, model.dot.z, model.psi, model.phi, model.theta, model.dot.psi, model.dot.phi, model.dot.theta = x_estimated
		model.set_control(control)
		model.step(dt)

		
		# Publishers
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
		
		
		msg_rpy_cam = Vector3Stamped() # Specific order
		msg_rpy_cam.header.stamp = rospy.get_rostime()
		msg_rpy_cam.vector.x = x_estimated[7]
		msg_rpy_cam.vector.y = x_estimated[8]
		msg_rpy_cam.vector.z = x_estimated[6]

		pubXYZ.publish(msg_xyz_cam)
		pubXYZ2.publish(msg_xyz_cam2)
		pubVel.publish(msg_vel_cam)
		pubVel2.publish(msg_vel_cam2)
		pubRPY.publish(msg_rpy_cam)
		
		rate.sleep()

if __name__ == '__main__':
	try:
		camera_handler()
	except rospy.ROSInterruptException:
		pass
