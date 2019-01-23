#!/usr/bin/env python

import rospy
import mavros
import math
import numpy as np

from geometry_msgs.msg import PoseStamped, Quaternion
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from tf.transformations import quaternion_from_euler
from std_msgs.msg import Header
from threading import Thread

class TryTakeOff(object):

    def __init__(self):
        self.pose = PoseStamped()
        self.radius = 0.1

        self.local_pos_pub = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=1)
        self.state_sub = rospy.Subscriber('mavros/state', State, self.state_cb)
        self.local_pos_sub = rospy.Subscriber('mavros/local_position/pose',
                                              PoseStamped,
                                              self.local_position_callback)
        service_timeout = 30
        rospy.loginfo("waiting for ROS services")
        try:
            rospy.wait_for_service('mavros/cmd/arming', service_timeout)
            rospy.wait_for_service('mavros/set_mode', service_timeout)
        except rospy.ROSInterruptException:
            self.fail("failed to connect to service")

        self.arming_client = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
        self.set_mode_client = rospy.ServiceProxy('mavros/set_mode', SetMode)

        # callback method for state sub
        self.current_state = State()
        self.offb_set_mode = SetMode

        #new threading for self.send_pos. and daemon with true means the threading while stoped with the main one.
        self.pos_thread = Thread(target=self.send_pos, args=())
        self.pos_thread.daemon = True
        self.pos_thread.start()

    def state_cb(self, state):
        self.current_state = state

    def local_position_callback(self, data):
        self.local_position = data

    def send_pos(self):
        rate = rospy.Rate(10)
        self.pose.header = Header()
        self.pose.header.frame_id = "base_footprint"

        while not rospy.is_shutdown():
            self.pose.header.stamp = rospy.Time.now()
            self.local_pos_pub.publish(self.pose)
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                pass

    def is_at_position(self, x, y, z, offset):
        rospy.logdebug("current position | x:{0:.2f}, y:{1:.2f}, z:{2:.2f}".format(
            self.local_position.pose.position.x, self.local_position.pose.position.y,
            self.local_position.pose.position.z
        ))
        desired = np.array((x,y,z))
        pos = np.array((self.local_position.pose.position.x,
                        self.local_position.pose.position.y,
                        self.local_position.pose.position.z
                        ))

        return np.linalg.norm(desired - pos) < offset

    def reach_position(self,x,y,z, timeout):
        #set a position setpoint
        self.pose.pose.position.x = x
        self.pose.pose.position.y = y
        self.pose.pose.position.z = z
        rospy.loginfo(
            "attempting to reach position | x: {0}, y: {1}, z: {2} | current position x: {3:.2f}, y: {4:.2f}, z: {5:.2f}".
            format(x, y, z, self.local_position.pose.position.x,
                   self.local_position.pose.position.y,
                   self.local_position.pose.position.z))

        # For demo purposes we will lock yaw/heading to north.
        yaw_degrees = 0  # North
        yaw = math.radians(yaw_degrees)
        quaternion = quaternion_from_euler(0, 0, yaw)
        self.pose.pose.orientation = Quaternion(*quaternion)

        # dose it reach the position in timeout seconds?
        loop_freq = 2  # Hz
        rate = rospy.Rate(loop_freq)
        reached = False
        for i in xrange(timeout * loop_freq):
            if self.is_at_position(self.pose.pose.position.x,
                                   self.pose.pose.position.y,
                                   self.pose.pose.position.z, self.radius):
                rospy.loginfo("position reached | seconnds: {0} of {1}".format(
                    i / loop_freq, timeout
                ))
                reached = True
                break
            try:
                rate.sleep()
            except rospy.ROSInterruptException as e:
                self.fail(e)



    def position_control(self):
 
        prev_state = self.current_state
        rate = rospy.Rate(20.0)  # MUST be more then 2Hz

        # send a few setpoints before starting
        for i in range(100):
            self.local_pos_pub.publish(self.pose)
            rate.sleep()

        # wait for FCU connection
        while not self.current_state.connected:
            rate.sleep()

        last_request = rospy.get_rostime()
        while not rospy.is_shutdown():
            now = rospy.get_rostime()
            if self.current_state.mode != "OFFBOARD" and (now - last_request > rospy.Duration(5.)):
                self.set_mode_client(base_mode=0, custom_mode="OFFBOARD")
                last_request = now
            else:
                if not self.current_state.armed and (now - last_request > rospy.Duration(5.)):
                    self.arming_client(True)
                    last_request = now

                    # older versions of PX4 always return success==True, so better to check Status instead
            if prev_state.armed != self.current_state.armed:
                rospy.loginfo("Vehicle armed: %r" % self.current_state.armed)
            if prev_state.mode != self.current_state.mode:
                rospy.loginfo("Current mode: %s" % self.current_state.mode)
            prev_state = self.current_state

            rospy.loginfo("run mission")

	    positions = ((0, 0, 2, 3),(0, 0, 2.5, 3),(0, 0.5, 2, 3), (0, 0, 1.5, 3),(0, -0.5, 2, 3),(0, 0, 2, 3))
	
            #positions = ((0,0,2,10),(4, 0, 5, 10))
	    #self.reach_position(0, 0, 2, 30)
            for i in xrange(len(positions)):
                self.reach_position(positions[i][0], positions[i][1],
                                 positions[i][2], positions[i][3])
            # Update timestamp and publish pose
            self.pose.header.stamp = rospy.Time.now()
            self.local_pos_pub.publish(self.pose)
            rate.sleep()


if __name__ == '__main__':
    rospy.init_node('offb_node', anonymous=True)
    try:
        TryTakeOff().position_control()
    except rospy.ROSInterruptException:
        pass

