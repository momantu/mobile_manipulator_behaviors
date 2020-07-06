#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from flexbe_manipulation_states.moveit_to_joints_state import MoveitToJointsState
from mobile_manipulator_flexbe_states.detect_obj_srv_state import DetectObjState
from flexbe_navigation_states.move_base_state import MoveBaseState
from mobile_manipulator_flexbe_states.head_actionlib_state import HeadActionState
from mobile_manipulator_flexbe_states.pick_srv_state import PickState
from mobile_manipulator_flexbe_states.detect_plane_srv_state import DetectPlaneState
from mobile_manipulator_flexbe_states.place_srv_state import PlaceState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]
from geometry_msgs.msg import Pose2D, Point
# [/MANUAL_IMPORT]


'''
Created on Mon Jul 06 2020
@author: Yizheng Zhang
'''
class fetch_real_208to203SM(Behavior):
	'''
	The states set of mobile manipulator.
	'''


	def __init__(self):
		super(fetch_real_208to203SM, self).__init__()
		self.name = 'fetch_real_208to203'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:26 y:401, x:564 y:234
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.pick_waypoint = Pose2D(-1.057, 5.051, 1.826)
		_state_machine.userdata.place_waypoint = Pose2D(-10.65, -17.47, -1.468)
		_state_machine.userdata.picking_state = [0.385, 0.476, 1.266, -0.722, 1.954, -1.708, 2.0197, -1.415]
		_state_machine.userdata.prepare_state = [0.337, 1.334, 1.328, -0.145, 1.811, 0.0, 1.639, 0.042]
		_state_machine.userdata.start_point = Point(0.8, 0.0, 0.6)
		_state_machine.userdata.look_up_point = Point(2.0, 0.0, 1.1)
		_state_machine.userdata.see_apriltag = Point(0.8, 0.2, 0.7)
		_state_machine.userdata.init_joints = [0.3675, 1.5585, 1.421, -0.1457, 1.1609, -0.027, 1.966, -1.371]
		_state_machine.userdata.prepare_state2 = [1.334, 1.328, -0.145, 1.811, 0.0, 1.639, 0.042]

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:25 y:72
			OperatableStateMachine.add('init_arm_pose',
										MoveitToJointsState(move_group='arm_with_torso', joint_names=['torso_lift_joint', 'shoulder_pan_joint', 'shoulder_lift_joint', 'upperarm_roll_joint', 'elbow_flex_joint', 'forearm_roll_joint', 'wrist_flex_joint', 'wrist_roll_joint'], action_topic='/move_group'),
										transitions={'reached': 'look_down', 'planning_failed': 'failed', 'control_failed': 'failed'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off},
										remapping={'joint_config': 'prepare_state'})

			# x:1102 y:44
			OperatableStateMachine.add('detect_obj',
										DetectObjState(),
										transitions={'continue': 'pick_obj', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:820 y:46
			OperatableStateMachine.add('prepare_state',
										MoveitToJointsState(move_group='arm_with_torso', joint_names=['torso_lift_joint', 'shoulder_pan_joint', 'shoulder_lift_joint', 'upperarm_roll_joint', 'elbow_flex_joint', 'forearm_roll_joint', 'wrist_flex_joint', 'wrist_roll_joint'], action_topic='/move_group'),
										transitions={'reached': 'detect_obj', 'planning_failed': 'failed', 'control_failed': 'failed'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off},
										remapping={'joint_config': 'prepare_state'})

			# x:587 y:53
			OperatableStateMachine.add('nav_pick',
										MoveBaseState(),
										transitions={'arrived': 'prepare_state', 'failed': 'failed'},
										autonomy={'arrived': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'waypoint': 'pick_waypoint'})

			# x:1103 y:510
			OperatableStateMachine.add('look_up',
										HeadActionState(),
										transitions={'head_arrived': 'nav_place', 'command_error': 'failed'},
										autonomy={'head_arrived': Autonomy.Off, 'command_error': Autonomy.Off},
										remapping={'point2see': 'look_up_point'})

			# x:1095 y:214
			OperatableStateMachine.add('pick_obj',
										PickState(),
										transitions={'continue': 'pick_moving_state', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:292 y:61
			OperatableStateMachine.add('look_down',
										HeadActionState(),
										transitions={'head_arrived': 'nav_pick', 'command_error': 'failed'},
										autonomy={'head_arrived': Autonomy.Off, 'command_error': Autonomy.Off},
										remapping={'point2see': 'start_point'})

			# x:537 y:507
			OperatableStateMachine.add('find_apriltag',
										HeadActionState(),
										transitions={'head_arrived': 'detect_plane', 'command_error': 'failed'},
										autonomy={'head_arrived': Autonomy.Off, 'command_error': Autonomy.Off},
										remapping={'point2see': 'see_apriltag'})

			# x:271 y:511
			OperatableStateMachine.add('detect_plane',
										DetectPlaneState(),
										transitions={'continue': 'place_obj', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:178 y:396
			OperatableStateMachine.add('place_obj',
										PlaceState(),
										transitions={'continue': 'finished_state', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:1125 y:372
			OperatableStateMachine.add('pick_moving_state',
										MoveitToJointsState(move_group='arm_with_torso', joint_names=['torso_lift_joint', 'shoulder_pan_joint', 'shoulder_lift_joint', 'upperarm_roll_joint', 'elbow_flex_joint', 'forearm_roll_joint', 'wrist_flex_joint', 'wrist_roll_joint'], action_topic='/move_group'),
										transitions={'reached': 'look_up', 'planning_failed': 'failed', 'control_failed': 'failed'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off},
										remapping={'joint_config': 'picking_state'})

			# x:177 y:249
			OperatableStateMachine.add('finished_state',
										MoveitToJointsState(move_group='arm', joint_names=['shoulder_pan_joint', 'shoulder_lift_joint', 'upperarm_roll_joint', 'elbow_flex_joint', 'forearm_roll_joint', 'wrist_flex_joint', 'wrist_roll_joint'], action_topic='/move_group'),
										transitions={'reached': 'finished', 'planning_failed': 'failed', 'control_failed': 'failed'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off},
										remapping={'joint_config': 'prepare_state2'})

			# x:833 y:506
			OperatableStateMachine.add('nav_place',
										MoveBaseState(),
										transitions={'arrived': 'find_apriltag', 'failed': 'failed'},
										autonomy={'arrived': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'waypoint': 'place_waypoint'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
