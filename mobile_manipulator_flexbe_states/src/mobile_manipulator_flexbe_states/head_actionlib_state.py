#!/usr/bin/env python
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient
import rospy

# example import of required action
from control_msgs.msg import PointHeadAction, PointHeadGoal


class HeadActionState(EventState):
    '''
    The head controller actionlib. Given a target point, which is based on the "base_link"

    ># point2see     geometry_msgs/Point      Target Point to look at.

    <= head_arrived		    Head move to see the point successfully
    <= command_error		Cannot send the action goal.

    '''

    def __init__(self):
        # See example_state.py for basic explanations.
        super(HeadActionState, self).__init__(outcomes = ['head_arrived', 'command_error'],
                                                 input_keys = ['point2see'])

        # Create the action client when building the behavior.
        # This will cause the behavior to wait for the client before starting execution
        # and will trigger a timeout error if it is not available.
        # Using the proxy client provides asynchronous access to the result and status
        # and makes sure only one client is used, no matter how often this state is used in a behavior.
        self._topic = '/head_controller/point_head'
        self._client = ProxyActionClient({self._topic: PointHeadAction}) # pass required clients as dict (topic: type)

        # It may happen that the action client fails to send the action goal.
        self._error = False


    def execute(self, userdata):
        # While this state is active, check if the action has been finished and evaluate the result.

        # Check if the client failed to send the goal.
        if self._error:
            return 'command_error'

        # Check if the action has been finished
        if self._client.has_result(self._topic):
            result = self._client.get_result(self._topic)
            return 'head_arrived'

    # If the action has not yet finished, no outcome will be returned and the state stays active.


    def on_enter(self, userdata):
        # When entering this state, we send the action goal once to let the robot start its work.

        # Create the goal.
        goal = PointHeadGoal()
        goal.target.header.stamp = rospy.Time.now()
        goal.target.header.frame_id = "base_link"
        goal.target.point = userdata.point2see

        # Send the goal.
        self._error = False # make sure to reset the error state since a previous state execution might have failed
        try:
            self._client.send_goal(self._topic, goal)
        except Exception as e:
            # Since a state failure not necessarily causes a behavior failure, it is recommended to only print warnings, not errors.
            # Using a linebreak before appending the error log enables the operator to collapse details in the GUI.
            Logger.logwarn('Failed to send the point2see command:\n%s' % str(e))
            self._error = True


    def on_exit(self, userdata):
        # Make sure that the action is not running when leaving this state.
        # A situation where the action would still be active is for example when the operator manually triggers an outcome.

        if not self._client.has_result(self._topic):
            self._client.cancel(self._topic)
            Logger.loginfo('Cancelled active action goal.')
