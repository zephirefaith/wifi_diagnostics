#!/usr/bin/python

from wifi import Cell
import rospy
import tf
from tf.transformations import euler_from_quaternion
from wifi_diagnostics.msg import WifiDiagnostics

class WifiDiagnostics(object):
    def __init__(self):
        rospy.init_node('wifi_diagnostics')
        self.listener = tf.TranformListener()
        self.pub = rospy.Publisher('/wifi_diagnostics', WifiDiagnostics, queue_size=10)

    def filter(self, cell):
        if cell.ssid == 'UCSD-DEVICE':
            return True

    def collect_wifi_data(self):
        cell = Cell.where('wlan0', self.filter)[0]
        wifi_msg = WifiDiagnostics()
        wifi_msg.header.stamp = rospy.get_rostime()
        wifi_msg.header.frame_id = '/map'


