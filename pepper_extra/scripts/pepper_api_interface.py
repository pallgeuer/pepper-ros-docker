#!/usr/bin/env python
# Author: Philipp Allgeuer <philipp.allgeuer@uni-hamburg.de>
# Node that provides an interface to custom NAOqi API calls via ROS

# Imports
import rospy
from naoqi_driver.naoqi_node import NaoqiNode

# Main function
def main():
	pepper_api_interface = PepperAPIInterface()
	pepper_api_interface.start()
	rospy.spin()

# Pepper API interface class
class PepperAPIInterface(NaoqiNode):

	ROS_RATE_HZ = 45.0

	def __init__(self, node_name='pepper_api_interface'):
		NaoqiNode.__init__(self, node_name)
		self.node_name = node_name

	def run(self):
		rospy.loginfo('Started PepperAPIInterface node as {0}'.format(self.node_name))
		rate = rospy.Rate(self.ROS_RATE_HZ)
		while self.is_looping():
			rate.sleep()

# Run main function
if __name__ == "__main__":
	main()
# EOF
