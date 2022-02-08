#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$
 
## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic
 
import rospy
import numpy as np
from std_msgs.msg import Float64
from std_msgs.msg import UInt16
import math
import matplotlib.pyplot as plt
 
x = []
y = []
 
dx_arr = []
dy_arr = []

phi_xy_arr = []
phi_xy_filtered_arr = []

phi_noise_arr = []
phi_noise_filtered_arr = []


phi_filtered = []
 
def init():
    x_k = 0
    y_k = 0
    fi_k = 0
    return x_k, y_k, fi_k
 
def step(x_k, y_k, fi_k, dt, v, omega):
    x_k = x_k + dt*v*math.sin(fi_k)
    y_k = y_k + dt*v*math.cos(fi_k)
    fi_k = fi_k + omega * dt
    return x_k, y_k, fi_k

def low_pass_filter(in_cur, out_prev, T, dt):
    out_cur = (T*out_prev + dt * in_cur)/(T + dt)
    return out_cur

def high_pass_filter(in_prev, in_cur, out_prev, T, dt):
    out_cur = T * (in_cur - in_prev + out_prev)/(T + dt)
    return out_cur
 
#def go_forward_one(velocity, x_k, y_k, fi_k):
#    x_k, y_k, fi_k = step(x_k, y_k, fi_k, dt, velocity, 0)
#    return x_k, y_k, fi_k
    
 
def go_to_point(x_k, y_k, fi_k, x_ref, y_ref, v, dt):
    global x,y
 
    coeff = 1
    
    dx = x_ref - x_k
    dy = y_ref - y_k
    fi_ref = math.atan2(dx, dy)
    omega = -(fi_k - fi_ref) * coeff
    
    x_k, y_k, fi_k = step(x_k, y_k, fi_k, dt, v, omega)
    
    x.append(x_k)
    y.append(y_k) 
    return x_k, y_k, fi_k, omega
    
 
def talker():
    global dx_arr, dy_arr, phi_xy_arr, phi_xy_filtered_arr, phi_noise_arr, phi_noise_filtered_arr, phi_filtered

    T = 0.1
    eps = 5e-2
    v = 1
    omega = math.pi/2
    wheel_r = 0.02
    wheel_dist = 0.11
    
    k1 = wheel_r/2.0
    k2 = wheel_r/wheel_dist

    x_ref = 3
    y_ref = 3

    pub = rospy.Publisher('omega_chatter', Float64, queue_size=10)
    
    left_wheel_pub = rospy.Publisher('vl', UInt16, queue_size=10)
    right_wheel_pub = rospy.Publisher('vr', UInt16, queue_size=10)
    
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(100) # 10hz
 
    x_k, y_k, phi_k = init()

    x_k_prev = x_k
    y_k_prev = y_k

    x_k_noise_prev = x_k
    y_k_noise_prev = y_k

    phi_xy_filtered_prev = phi_k
    phi_k_noise_prev = phi_k
    phi_k_noise_filtered_prev = phi_k

 
    start_time = rospy.get_time()
 
    while not rospy.is_shutdown():
        end_time = rospy.get_time()
        dt = end_time - start_time
        start_time = end_time
                
        x_k, y_k, phi_k, omega = go_to_point(x_k, y_k, phi_k, x_ref, y_ref, v, dt)
        if abs(x_k - x_ref) < eps and abs(y_k - y_ref) < eps:
            v = 0
        
        x_k_noise  = x_k + np.random.normal(0, 0.003)
        y_k_noise  = y_k + np.random.normal(0, 0.003)
        phi_k_noise = phi_k + np.random.normal(0, 1.5*math.pi/180.0)


        # Производные
        #x_dot = (x_k - x_k_prev)/dt
        #y_dot = (y_k - y_k_prev)/dt

        # или производные надо брать от зашумленных переменных?
        x_dot = (x_k_noise - x_k_noise_prev)/dt
        y_dot = (y_k_noise - y_k_noise_prev)/dt

        dx_arr.append(x_dot)
        dy_arr.append(y_dot)

        v_r = 0.5 * (v / k1 + omega / k2)
        v_l = 0.5 * (v / k1 - omega / k2)
        
        if abs(v_r) < eps:
                v_r = 0
        else:
                v_r = 1
        
        if abs(v_l) < eps:
                v_l = 0
        else:
                v_l = 1

        '''
        # Курсовой угол?
        phi_xy = math.atan2(x_dot, y_dot)
        phi_xy_filtered = low_pass_filter(phi_xy, phi_xy_filtered_prev, T, dt)

        phi_xy_arr.append(phi_xy)
        phi_xy_filtered_arr.append(phi_xy_filtered)


        # Фи с шумом:
        phi_k_noise_filtered = high_pass_filter(phi_k_noise_prev, phi_k_noise, phi_k_noise_filtered_prev, T, dt)
        phi_noise_arr.append(phi_k_noise)
        phi_noise_filtered_arr.append(phi_k_noise_filtered)


        # Результат - комплиментарный фильтр: 
        phi_kf =  (phi_xy_filtered + phi_k_noise_filtered)/2.
        phi_filtered.append(phi_kf)


        # previous states of variables
        x_k_prev = x_k
        y_k_prev = y_k
        x_k_noise_prev = x_k_noise
        y_k_noise_prev = y_k_noise
        phi_xy_filtered_prev = phi_xy_filtered
        phi_k_noise_prev = phi_k_noise
        phi_k_noise_filtered_prev = phi_k_noise_filtered
	'''
        publish_str1 = "\nNO NOISE: {:.5f}".format(x_k) + ' ' + "{:.5f}".format(y_k) + ' ' + "{:.5f}".format(phi_k)
        publish_str2 = "NOISE:    {:.5f}".format(x_k_noise) + ' ' + "{:.5f}".format(y_k_noise) + ' ' + "{:.5f}".format(phi_k_noise)
        publish_str = publish_str1 + '\n' + publish_str2
        rospy.loginfo(publish_str)
        pub.publish(publish_str)
        
        
        publish_wheels = "{:d}".format(v_l) + ' ' + "{:d}".format(v_r)
        left_wheel_pub.publish(v_l)
        right_wheel_pub.publish(v_r)
        rospy.loginfo(publish_wheels)

        rate.sleep()
 
 
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
    
    plt.plot(dx_arr,dy_arr)
    #plt.show()
    plt.savefig('der.png')
    plt.clf()
    
    plt.plot(x,y)
    plt.savefig('pos.png')
    plt.clf()
    
    plt.plot(phi_xy_arr)
    plt.savefig('fi_ref.png')
    plt.clf()

    plt.plot(phi_xy_filtered_arr)
    plt.savefig('fi_ref_filtered.png')
    plt.clf()

    plt.plot(phi_noise_arr)
    plt.savefig('fi_noise.png')
    plt.clf()

    plt.plot(phi_noise_filtered_arr)
    plt.savefig('fi_noise_filtered.png')
    plt.clf()

    plt.plot(phi_filtered)
    plt.savefig('fi_filtered_RESULT.png')
    plt.clf()
