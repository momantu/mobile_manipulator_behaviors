#!/usr/bin/env python
import rospy
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyServiceCaller


class DetectPlaneState(EventState):
	'''
	The state that the camera detect the plane to place the object.
	The detect part is done by Apriltag

	<= continue 			Given time has passed.
	<= failed 				Example for a failure outcome.

	'''

	def __init__(self):
		# Declare outcomes, input_keys, and output_keys by calling the super constructor with the corresponding arguments.
		super(DetectPlaneState, self).__init__(outcomes = ['continue', 'failed'])


		# The constructor is called when building the state machine, not when actually starting the behavior.
		# Thus, we cannot save the starting time now and will do so later.

		self._srv_topic = "/detect_plane_srv"
		self._srv = ProxyServiceCaller({self._srv_topic: Trigger})

		self._srv_result = None
		self._failed = False

	def execute(self, userdata):
		# This method is called periodically while the state is active.
		# Main purpose is to check state conditions and trigger a corresponding outcome.
		# If no outcome is returned, the state will stay active.

		if self._failed or self._srv_result.success is False:
			return 'failed'
		else:
			return "continue"
		

	def on_enter(self, userdata):
		# This method is called when the state becomes active, i.e. a transition from another state to this one is taken.
		# It is primarily used to start actions which are associated with this state.

		# The following code is just for illustrating how the behavior logger works.
		# Text logged by the behavior logger is sent to the operator and displayed in the GUI.
		try:
			self._srv_result = self._srv.call(self._srv_topic,
											  TriggerRequest())
		except Exception as e:
			Logger.logwarn('Failed to send service call:\n%s' % str(e))
			self._failed = True


	def on_exit(self, userdata):
		pass # Nothing to do in this service.


	def on_start(self):
		pass # Nothing to do in this service.


	def on_stop(self):
		pass # Nothing to do in this service.
		
