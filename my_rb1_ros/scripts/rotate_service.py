#! /usr/bin/env python

import rospy
import time
from my_rb1_ros.srv import Rotate, RotateRequest, RotateResponse
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math


class RotateServerNode:

    def __init__(self,run_rate, yaw_tol=1, spd_reduce_ratio=0.4, slow_angle_ratio=0.8):
        # init variables
        self.current_yaw = 0  # in degrees
        self.target_yaw = 0   # in degrees
        self.angular_speed = 0  # in rad/s
        self.yaw_tolerance = yaw_tol  # in degrees
        self.speed_reduction_ratio = spd_reduce_ratio  # ratio used to reduce original speed at the end of angle travel
        self.slow_down_travel_ratio = slow_angle_ratio  # ratio of target angle at which speed is reduced for better target angle accuracy
        # init node
        rospy.init_node('rotate_node', log_level=rospy.DEBUG)
        # init rate
        self.rate = rospy.Rate(run_rate)
        self.timeout_counter_inc = 1/run_rate
        # create publisher of cmd_vel and subscriber to odom messages
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        # init speed msg
        self.rotate_msg = Twist()
        # init service object
        self.rotate_service = None
        
    
    def start_service(self):
        # start the service
        self.rotate_service = rospy.Service('/rotate_robot', Rotate, self.server_callback)
        # Report service readiness
        rospy.logdebug("Service Ready")
        # Keep it up until someone sends a request
        while not rospy.is_shutdown():
            self.rate.sleep()
        

    def odom_callback(self,msg):
        # get odom values
        q = msg.pose.pose.orientation
        # calculate yaw from orientation
        yaw_radians = 2 * math.atan2(q.z, q.w)
        # convert radians to degrees and normalize to 0 to 360 range
        self.current_yaw = math.degrees(yaw_radians) % 360
        #rospy.logdebug(f"Angle now is: {self.current_yaw}")
        

    def set_angular_speed(self, speed):
        self.angular_speed = speed
    
    def turn_angle(self, angle):
        # normalize angle
        normalized_angle = angle % 360
        # Get target angle and normalize it
        self.target_yaw = (self.current_yaw + normalized_angle) % 360 
        # calculate slow down angle (angle at which speed is reduced for better accuracy reaching target angle)
        slow_down_angle = (1 - self.slow_down_travel_ratio) * normalized_angle
        # initialize timeout counter
        timeout_counter = 0
        # make sure to send zero translational velocities in x and y to avoid any drift
        self.rotate_msg.linear.x = 0
        self.rotate_msg.linear.y = 0
        # start turning the robot
        # check first that the requested angle is greater than 10 degrees, if less, start moving with the reduced speed from the beginning
        if abs(angle) < 10:
            # always move with the reduced speed
            self.rotate_msg.angular.z = (angle/abs(angle)) * self.angular_speed * self.speed_reduction_ratio
            slow_down_angle = -1 # slow down angle is not needed anymore in this case
        else:
            # move with the normal speed, then reduce the speed later at the specified travelled angle ration
            self.rotate_msg.angular.z = (angle/abs(angle)) * self.angular_speed # this way we know needed direction of rotation 
        
        # send the requested speed through
        self.pub.publish(self.rotate_msg)

        # keep checking whether target angle is reached
        while not rospy.is_shutdown():
            # stop if the robot reaches target angle
            if abs(self.target_yaw - self.current_yaw) <= self.yaw_tolerance:  # check whether within tolerance
                # stop turning
                self.rotate_msg.angular.z = 0
                # make sure once again no translational drifts
                self.rotate_msg.linear.x = 0
                self.rotate_msg.linear.y = 0
                self.pub.publish(self.rotate_msg)
                return True
            # slow down speed to a predetermined ratio of the original speed at a set percentage of the angle travel, to avoid overshoots
            elif abs(self.target_yaw - self.current_yaw) <= slow_down_angle:
                self.rotate_msg.angular.z = (angle/abs(angle)) * self.angular_speed * self.speed_reduction_ratio
                self.pub.publish(self.rotate_msg)
            elif timeout_counter >= 120:  # timeout after 2 minute
                rospy.logerr("Failed: Turning around the corner took too long!")
                return False
            else:
                pass
            # increment counter
            timeout_counter += self.timeout_counter_inc
            # sleep until the next loop
            self.rate.sleep()


    def server_callback(self, request):
        # Report service requested
        rospy.logdebug("Service Requested")
        
        # prepare the response object
        response = RotateResponse()
        
        # start moving the robot according to the request
        action_result = self.turn_angle(request.degrees)

        # Return results
        if action_result == True:
            response.result = "Success completing rotation to target angle"
        else:
            response.result = "Failed to complete rotation to target angle"

        # Report service complete
        rospy.logdebug("Service Completed")

        # Return the final response
        return response # the service Response


# Main function
def main():
    # create node object and set rate to 50 Hz
    # Robot state is initially stopped
    rotate_node = RotateServerNode(50)

    # set angular speed
    rotate_node.set_angular_speed(0.2)  # set to 0.5 rad/s

    # start service
    rotate_node.start_service()


# Program entry point
if __name__ == "__main__":
    main()

