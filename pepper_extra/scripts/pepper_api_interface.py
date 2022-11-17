#!/usr/bin/env python
# Author: Philipp Allgeuer <philipp.allgeuer@uni-hamburg.de>
# Node that provides an interface to custom NAOqi API calls via ROS

# Imports
import Queue
import rospy
import pepper_extra.srv
from naoqi_driver.naoqi_node import NaoqiNode

# Main function
def main():
	pepper_api_interface = PepperAPIInterface()
	pepper_api_interface.start()
	rospy.spin()

# Pepper API interface class
class PepperAPIInterface(NaoqiNode):

	def __init__(self):

		NaoqiNode.__init__(self, 'pepper_api_interface')

		self.leds_proxy = self.get_proxy("ALLeds")
		self.tts_proxy = self.get_proxy("ALTextToSpeech")
		self.speech_proxy = self.get_proxy("ALAnimatedSpeech")

		self.srv_leds_set_rgb = rospy.Service("leds/set_rgb", pepper_extra.srv.LEDsSetRGB, self.handle_leds_set_rgb)
		self.srv_speech_say = rospy.Service("speech/say", pepper_extra.srv.SpeechSay, self.handle_speech_say)
		self.srv_speech_say_anim = rospy.Service("speech/say_anim", pepper_extra.srv.SpeechSay, self.handle_speech_say_anim)

		self.action_queue = Queue.Queue()

	def run(self):
		rospy.loginfo('Started PepperAPIInterface node as {0}'.format(rospy.get_name()))
		rate = rospy.Rate(rospy.get_param("~rate_hz", 45.0))
		while self.is_looping():
			self.perform_actions()
			rate.sleep()

	def perform_actions(self):
		pass  # TODO: Perform sequentially any actions available in the queue (.say() does not have _async)

	def handle_leds_set_rgb(self, msg):
		try:
			if msg.color:
				self.leds_proxy.fadeRGB(msg.name, msg.color, msg.duration, _async=not msg.wait)
			else:
				self.leds_proxy.fadeRGB(msg.name, msg.r, msg.g, msg.b, msg.duration, _async=not msg.wait)
			return pepper_extra.srv.LEDsSetRGBResponse()
		except RuntimeError as e:
			rospy.logerr("Exception caught:\n%s", e)
			return None

	def handle_speech_say(self, msg):
		try:
			if msg.wait:
				self.perform_speech_say(msg)
			else:
				raise NotImplementedError  # TODO: Use self.action_queue...
			return pepper_extra.srv.SpeechSayResponse()
		except RuntimeError as e:
			rospy.logerr("Exception caught:\n%s", e)
			return None

	def handle_speech_say_anim(self, msg):
		try:
			if msg.wait:
				self.perform_speech_say_anim(msg)
			else:
				raise NotImplementedError  # TODO: Use self.action_queue...
			return pepper_extra.srv.SpeechSayResponse()
		except RuntimeError as e:
			rospy.logerr("Exception caught:\n%s", e)
			return None

	def perform_speech_say(self, msg):
		self.tts_proxy.say(msg.text)

	def perform_speech_say_anim(self, msg):
		self.speech_proxy.say(msg.text)

# Run main function
if __name__ == "__main__":
	main()
# EOF
