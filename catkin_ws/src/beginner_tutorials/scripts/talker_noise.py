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
import math
import matplotlib.pyplot as plt

x = []
y = []

dx_arr = []
dy_arr = []
fi_ref_arr = []

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

def go_forward_one(velocity, x_k, y_k, fi_k):
    x_k, y_k, fi_k = step(x_k, y_k, fi_k, dt, velocity, 0)
    return x_k, y_k, fi_k
    

def go_to_point(x_k, y_k, fi_k, x_ref, y_ref, v, dt):
    global x,y
    global dx_arr, dy_arr, fi_ref_arr

    coeff = 1
    
    dx = x_ref - x_k
    dy = y_ref - y_k
    fi_ref = math.atan2(dx, dy)
    omega = -(fi_k - fi_ref) * coeff
    

    x_k_o = x_k
    y_k_o = y_k
    
    x_k, y_k, fi_k = step(x_k, y_k, fi_k, dt, v, omega)
    
    dxdt = (x_k - x_k_o)/dt
    dydt = (y_k - y_k_o)/dt
    
    dx_arr.append(dxdt)
    dy_arr.append(dydt)
    fi_ref_arr.append(math.atan2(dxdt, dydt))
    
    x.append(x_k)
    y.append(y_k) 
    return x_k, y_k, fi_k, omega
    

    

def talker():
    pub = rospy.Publisher('omega_chatter', Float64, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(100) # 10hz

    x_k, y_k, fi_k = init()

    v = 1
    omega = math.pi/2

    
    x_ref = 3
    y_ref = 3
    
   
    eps = 5e-2

    start_time = rospy.get_time()

    while not rospy.is_shutdown():
        end_time = rospy.get_time()
        dt = end_time - start_time
        start_time = end_time
        

        #x_k, y_k, fi_k = step(x_k, y_k, fi_k, dt, v, omega)
        
        x_k, y_k, fi_k, omega = go_to_point(x_k, y_k, fi_k, x_ref, y_ref, v, dt)
        if abs(x_k - x_ref) < eps and abs(y_k - y_ref) < eps:
            v = 0
        
        x_k_n  = x_k + np.random.normal(0, 0.003)
        y_k_n  = y_k + np.random.normal(0, 0.003)
        

        
        
        #x_k = x_k_n
        #y_k = y_k_n
        fi_k = fi_k + np.random.normal(0, 1.5*math.pi/180.0)
        
        #hello_str = "x_k = {} y_k = {} fi_k = {} dt = {} ".format(x_k, y_k, fi_k, dt)
        omega_str = "{:.5f}".format(omega) + ' ' + str(v)
        rospy.loginfo(str(omega))
        pub.publish(omega)
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
    
    plt.plot(fi_ref_arr)
    plt.savefig('fi.png')
