#!/usr/bin/python
#TODO Add a status field to the wifi msg
from wifi import Cell
import rospy
import tf
from wifi_diagnostics.msg import WifiDiagnostics

class WifiDiagnosticLogger(object):
    '''used for logging wifi diagnostic messages to measure signal strength and
    quality along with the location of the robot_frame'''
    def __init__(self):
        rospy.init_node('wifi_diagnostic_logger')
        self.listener = tf.TransformListener()
        self.pub = rospy.Publisher('/wifi_diagnostics', WifiDiagnostics, queue_size=10)
        self.robot_pose = None

    def ssid_filter(self, cell):
        '''filter for filtering all the wifi networks found'''
        if cell.ssid == 'UCSD-DEVICE':
            return True

    def get_robot_pose(self):
        """ listens to transform between map and base_link to get the current location """
        now = rospy.get_rostime()
        robot_frame = '/base_link'
        map_frame = '/map'
        self.listener.waitForTransform(map_frame, robot_frame, now, rospy.Duration(.15))
        return self.listener.lookupTransform(map_frame, robot_frame, now)

    def log_wifi_data(self):
        '''collects the robot pose and wifi signal strength + quality and publishes it'''
        wifi_msg = WifiDiagnostics()
        wifi_msg.header.stamp = rospy.get_rostime()
        wifi_msg.header.frame_id = '/map'
        try:
            (trans, rot) = self.get_robot_pose()
            self.robot_pose = (trans, rot)
        except Exception:
            if self.robot_pose is None:
                return
            (trans, rot) = self.robot_pose
        wifi_msg.position.x = trans[0]
        wifi_msg.position.y = trans[1]
        try:
            cell = Cell.where('wlan0', self.ssid_filter)[0]
        except IndexError:
            wifi_msg.signal.data = 0
            wifi_msg.qualityby70.data = 0
        else:
            wifi_msg.signal.data = cell.signal
            temp = cell.quality.encode("utf8")
            wifi_msg.qualityby70.data = int(temp.split('/')[0])
        self.pub.publish(wifi_msg)

if __name__ == '__main__':
    wifi_object = WifiDiagnosticLogger()
    node_rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        wifi_object.log_wifi_data()
        node_rate.sleep()
    rospy.spin()
