#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Temperature
import subprocess

class Vcgencmd():
    def __init__(self):
        rospy.init_node('temperature_sensor_publisher')

        self.pub = rospy.Publisher('temp', Temperature, queue_size=50)
        self.rate = 1.0

    def handle(self):

        temp = Temperature()
        temp.header.stamp = self.current_time
        temp.header.frame_id = 'temp_frame'

	output = subprocess.check_output(['vcgencmd','measure_temp']).decode('utf-8')
	v = float(output[output.find('=') + 1:].strip().rstrip('\'C'))
        temp.temperature = v
	rospy.loginfo(v)

        self.pub.publish(temp)

    def spin(self):
        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.current_time = rospy.Time.now()
            self.handle()
            r.sleep()

if __name__ == '__main__':
                                
    sensor = Vcgencmd()
    rospy.loginfo("=== run")
    sensor.spin()
    rospy.loginfo("=== end")
