#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState
from copy import deepcopy
import message_filters
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_input  as inputMsg

class FullState:
    def __init__(self):
        rospy.init_node('joint_state_republisher', anonymous=True)
        manipulator_ns = rospy.get_param("~manipulator_ns", "")
        gripper_ns = rospy.get_param("~gripper_ns", "gripper_controller")
        fts_input = rospy.get_param("~fts_input", "gripper_cmd/input")
        joint_states_sub = message_filters.Subscriber(manipulator_ns + '/' + 'joint_states', JointState)
        fts_sub = message_filters.Subscriber(gripper_ns + '/' + fts_input, inputMsg.Robotiq2FGripper_robot_input)
        self._ts = message_filters.ApproximateTimeSynchronizer([joint_states_sub, fts_sub], 10, 0.1, allow_headerless=True)
        self._ts.registerCallback(self._joint_states_callback)
        self._joint_states_pub = rospy.Publisher('joint_states', JointState, queue_size=1)
        rospy.spin()

    def _joint_states_callback(self, ur5_joints, gripper_joint):
        msg = JointState()
        msg.header = ur5_joints.header
        msg.name = list(ur5_joints.name)
        msg.name.append('gripper_finger_joint')
        msg.position = list(ur5_joints.position)
        msg.position.append(gripper_joint.gPO/255.0)
        msg.velocity = list(ur5_joints.velocity)
        msg.velocity.append(0.0)
        msg.effort = list(ur5_joints.effort)
        msg.effort.append(gripper_joint.gCU)
        # print msg
        self._joint_states_pub.publish(msg)

        
if __name__ == '__main__':
    FullState()
