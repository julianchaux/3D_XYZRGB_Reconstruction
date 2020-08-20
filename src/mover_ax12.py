#!/usr/bin/env python

import rospy
from dynamixel_msgs.msg import JointState
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64
import numpy as np
import time
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from Tkinter import *

pos_actual = 0.0

def read_Dynamixel(data):
    pos_actual = data.current_pos

def mover_arriba():
    time.sleep(0.5)
    pos.data = pos_actual - 0.0051
    pub.publish(pos)

def mover_abajo():
    time.sleep(0.5)
    pos.data = pos_actual + 0.0051
    connections = pub.get_num_connections()
    rospy.loginfo('Conexiones: %d', connections)
    if connections > 0:
        pub.publish(pos)
        rospy.loginfo('Publicado')

if __name__ == '__main__':
    rospy.init_node("example")
    pub = rospy.Publisher('/motor_controller/command', Float64, queue_size=1)
    sub = rospy.Subscriber('/motor_controller/state', JointState, read_Dynamixel)
    pos = Float64()

    master = Tk()
    master.title("Configuracion Inicial")
    Button(master, text='Mover Arriba', command=mover_arriba).pack()
    Button(master, text='Mover Abajo', command=mover_abajo).pack()
    master.mainloop()
