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
from std_msgs.msg import UInt16
import math
import matplotlib.pyplot as plt
 
x = []
y = []

omega_arr = []
phi_ref_arr = []
phi_k_arr = []

v_l_arr = []
v_r_arr = []

 
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
 
def go_to_point(x_k, y_k, fi_k, x_ref, y_ref, v_const, omega_const, dt):
    global x, y, phi_k_arr, omega_arr, phi_ref_arr

    phi_eps = 0.005
    pos_eps = 0.005
    
    dx = x_ref - x_k
    dy = y_ref - y_k
    fi_ref = math.atan2(dx, dy)
    
    #if fi_ref < 0:
    #	fi_ref += 2 * math.pi


    if abs(fi_k - fi_ref) % (math.pi * 2) < phi_eps:
        omega = 0
        v = v_const
    else:
        if fi_ref > fi_k:
            omega = omega_const
        else:
            omega = -omega_const

        v = 0

    if abs(x_k - x_ref) < pos_eps and abs(y_k - y_ref) < pos_eps:
        v = 0

    #v = 0
    #publish_str = "\n v: {:.5f}".format(v) + ' omega: ' + "{:.5f}".format(omega)
    #rospy.loginfo(publish_str)

    x_k, y_k, fi_k = step(x_k, y_k, fi_k, dt, v, omega)
    
    x.append(x_k)
    y.append(y_k) 
    phi_k_arr.append(fi_k)
    phi_ref_arr.append(fi_ref)
    omega_arr.append(omega)

    return x_k, y_k, fi_k, v, omega
    
 
def talker():
    global v_l_arr, v_r_arr

    eps = 5e-3
    wheel_r = 0.02
    wheel_dist = 0.11
    
    k1 = wheel_r/2.0
    k2 = wheel_r/wheel_dist

    v_l_const = v_r_const = math.pi * 1000
    v_const = k1 * (v_l_const + v_r_const)
    omega_const = k2 * (v_l_const)

    curPoint = 0
    #points = [(3, 3), (4, -2), (3, -3)]
    #points = [(0, 1), (1, 1), (1, 0), (0, 0)]
    #points = [(0, -0.1), (-0.1, -0.1), (-0.1, 0), (0, 0)]
    points = [(0.2, 0)]
    x_ref = points[curPoint][0]
    y_ref = points[curPoint][1]
    
    left_wheel_pub = rospy.Publisher('vl', UInt16, queue_size=10)
    right_wheel_pub = rospy.Publisher('vr', UInt16, queue_size=10)
    
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(100) # 10hz
 
    x_k, y_k, phi_k = init()

    achieved_destination = False


    start_time = rospy.get_time()
 
    while not rospy.is_shutdown():
        end_time = rospy.get_time()
        dt = end_time - start_time
        start_time = end_time

        if achieved_destination: #конец
            left_wheel_pub.publish(0)
            right_wheel_pub.publish(0)
            break

        x_k, y_k, phi_k, v, omega = go_to_point(x_k, y_k, phi_k, x_ref, y_ref, v_const, omega_const, dt)

        #if curPoint < len(points):
        #publish_str = "\n X: {:.5f}".format(x_k) + ' Y: ' + "{:.5f}".format(y_k) + ' Phi:' + "{:.5f}".format(phi_k)
        #rospy.loginfo(publish_str)

        if abs(x_k - x_ref) < eps and abs(y_k - y_ref) < eps:
            publish_str = "\n X: {:.5f}".format(x_k) + ' Y: ' + "{:.5f}".format(y_k) + ' Phi:' + "{:.5f}".format(phi_k)
            rospy.loginfo(publish_str)

            curPoint += 1
            if curPoint == len(points):
                achieved_destination = True
            if not achieved_destination:
                x_ref = points[curPoint][0]
                y_ref = points[curPoint][1]

        v_r = 0.5 * (v / k1 + omega / k2)
        v_l = 0.5 * (v / k1 - omega / k2)

        # я понял в чём проблема с вращением! ведь мы должны вращаться с какой-то своей скоростью, а вращаемся с другой... 
        # как то по другому считать их?
        

        if v_r <= 0:
                v_r = 0
        else:
                v_r = 1
        
        if v_l <= 0:
                v_l = 0
        else:
                v_l = 1
                
        v_l_arr.append(v_l)
        v_r_arr.append(v_r)

        
        publish_wheels = "{:d}".format(v_l) + ' ' + "{:d}".format(v_r)
        left_wheel_pub.publish(v_l)
        right_wheel_pub.publish(v_r)
        #rospy.loginfo(publish_wheels)

        rate.sleep()
 
 
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
    
    plt.plot(x,y)
    plt.savefig('pos.png')
    plt.clf()

    plt.plot(omega_arr)
    plt.savefig('omega.png')
    plt.clf()
    
    plt.plot(phi_ref_arr)
    plt.savefig('fi_ref.png')
    plt.clf()

    plt.plot(phi_k_arr)
    plt.savefig('fi_k.png')
    plt.clf()

    plt.plot(v_l_arr)
    plt.plot(v_r_arr)
    plt.savefig('v_lr.png')
    plt.clf()

