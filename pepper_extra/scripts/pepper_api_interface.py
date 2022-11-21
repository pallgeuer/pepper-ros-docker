#!/usr/bin/env python
# Author: Philipp Allgeuer <philipp.allgeuer@uni-hamburg.de>
# Node that provides an interface to custom NAOqi API calls via ROS

# Imports
import Queue
import rospy
import std_srvs.srv
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
		self.anim_proxy = self.get_proxy("ALAnimationPlayer")
		self.posture_proxy = self.get_proxy("ALRobotPosture")

		self.srv_leds_set_rgb = rospy.Service("leds/set_rgb", pepper_extra.srv.LEDsSetRGB, self.handle_leds_set_rgb)
		self.srv_speech_say = rospy.Service("speech/say", pepper_extra.srv.SpeechSay, self.handle_speech_say)
		self.srv_speech_say_anim = rospy.Service("speech/say_anim", pepper_extra.srv.SpeechSay, self.handle_speech_say_anim)
		self.srv_anim_play = rospy.Service("anim/play", pepper_extra.srv.AnimPlay, self.handle_anim_play)
		self.srv_pose_set_posture = rospy.Service("pose/set_posture", pepper_extra.srv.SetPosture, self.handle_pose_set_posture)
		self.srv_pose_home = rospy.Service("pose/home", std_srvs.srv.Empty, self.handle_pose_home)

		self.action_queue = Queue.Queue()

	def run(self):
		rospy.loginfo('Started PepperAPIInterface node as {0}'.format(rospy.get_name()))
		rate = rospy.Rate(rospy.get_param("~rate_hz", 45.0))
		while self.is_looping():
			self.perform_actions()
			rate.sleep()

	def perform_actions(self):
		try:
			action = self.action_queue.get_nowait()
			action[0](*action[1:])
		except Queue.Empty:
			pass

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
				self.action_queue.put_nowait((self.perform_speech_say, msg))
			return pepper_extra.srv.SpeechSayResponse()
		except RuntimeError as e:
			rospy.logerr("Exception caught:\n%s", e)
			return None

	def perform_speech_say(self, msg):
		self.tts_proxy.say(msg.text)

	def handle_speech_say_anim(self, msg):
		try:
			if msg.wait:
				self.perform_speech_say_anim(msg)
			else:
				self.action_queue.put_nowait((self.perform_speech_say_anim, msg))
			return pepper_extra.srv.SpeechSayResponse()
		except RuntimeError as e:
			rospy.logerr("Exception caught:\n%s", e)
			return None

	def perform_speech_say_anim(self, msg):
		self.speech_proxy.say(msg.text)

	def handle_anim_play(self, msg):
		try:
			if msg.wait:
				self.perform_anim_play(msg)
			else:
				self.action_queue.put_nowait((self.perform_anim_play, msg))
			return pepper_extra.srv.AnimPlayResponse()
		except RuntimeError as e:
			rospy.logerr("Exception caught:\n%s", e)
			return None

	def perform_anim_play(self, msg):
		self.anim_proxy.run(msg.anim_path)

	def handle_pose_set_posture(self, msg):
		try:
			if msg.wait:
				self.perform_pose_set_posture(msg.posture, msg.speed)
			else:
				self.action_queue.put_nowait((self.perform_pose_set_posture, msg.posture, msg.speed))
			return pepper_extra.srv.SetPostureResponse()
		except RuntimeError as e:
			rospy.logerr("Exception caught:\n%s", e)
			return None

	def perform_pose_set_posture(self, posture, speed):
		self.posture_proxy.goToPosture(posture, speed)

	# noinspection PyUnusedLocal
	def handle_pose_home(self, msg):
		try:
			self.action_queue.put_nowait((self.perform_pose_set_posture, 'Stand', 0.3))
			return std_srvs.srv.EmptyResponse()
		except RuntimeError as e:
			rospy.logerr("Exception caught:\n%s", e)
			return None

# Run main function
if __name__ == "__main__":
	main()
# EOF
