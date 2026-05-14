#!/usr/bin/env python
# -*- coding: utf-8 -*-
import math

import rospy
import tf
from tf.transformations import euler_from_quaternion


def get_yaw_from_quaternion(orientation):
    _, _, yaw = euler_from_quaternion(orientation)
    return yaw


def get_current_pose(listener, time_limit=10.0, target='map', source='base_link'):
    try:
        listener.waitForTransform(
            target, source, rospy.Time(), rospy.Duration(time_limit))
        (trans, rot) = listener.lookupTransform(target, source, rospy.Time(0))
        yaw = get_yaw_from_quaternion(rot)
        return (trans[0], trans[1], yaw)
    except Exception as e:
        rospy.logwarn('Failed to get transform. Waiting...')
    return None


class SensorMessageGetter(object):
    def __init__(self, topic, msg_type, msg_wait=1.0):
        self.msg_wait = msg_wait
        self.topic = topic
        self.msg_type = msg_type

    def get_msg(self):
        message = None
        try:
            message = rospy.wait_for_message(
                self.topic, self.msg_type, self.msg_wait)
        except rospy.exceptions.ROSException as e:
            rospy.logdebug(e)
        return message


def main():
    rospy.init_node('get_odometry_pose')
    x = 0
    y = 0
    yaw = 0
    listener = tf.TransformListener()
    while True:
        ret = get_current_pose(listener, target='odom')
        if ret is not None:
            x, y, yaw = ret
            break
    rospy.loginfo('OK. got odometry pose (x, y, yaw)')
    rospy.loginfo('%f, %f, %f (rad)' % (x, y, yaw))
    rospy.loginfo('** For navigation **')
    rospy.loginfo(
        'initial_pose_x:=%f initial_pose_y:=%f initial_pose_a:=%f' % (x, y, yaw))
    rospy.loginfo('** Stage simulator **')
    rospy.loginfo(
        'pose [%f %f 0 %f]' % (x, y, math.degrees(yaw)))


if __name__ == '__main__':
    main()
