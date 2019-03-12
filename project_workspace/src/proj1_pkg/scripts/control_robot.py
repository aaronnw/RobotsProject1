#!/usr/bin/python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
import sensor_msgs.point_cloud2 as pc2
from kobuki_msgs.msg import BumperEvent
import threading
import time
import random
import Queue
from math import radians
import curses
import numpy as np
import os
import cv2
from cv_bridge import CvBridge, CvBridgeError

cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=1)
_turning_lock = False
_escape_lock = False
_escape_reset_lock = False
_key_input_lock = False
_threads = []
_shutdown_flag = False
_bridge = CvBridge()

def turn(angle, duration):
    """
    Turn the robot at the given radian speed and for <duration> tenths of a second
    """
    cmd_vel.publish(Twist())
    r = rospy.Rate(5)
    r.sleep()
    turn_cmd = Twist()
    turn_cmd.linear.x = 0
    turn_cmd.angular.z = radians(angle)
    # do this duration times
    for i in range(duration):
        # if a key is pressed while turning, stop turning
        if _key_input_lock:
            break
        cmd_vel.publish(turn_cmd)
        r.sleep()
    cmd_vel.publish(Twist())
    r.sleep()

def go_forward():
    """
    Drive the robot forward, making a random turn after every foot of movement
    """
    global _turning_lock
    global _shutdown_flag
    global _key_input_lock
    global _escape_lock
    global _escape_reset_lock
    distance = 0  # distance traveled since last random turn, or since last lock release
    max_straight_distance = 18  # maximum distance to travel before making a random turn
    key_input_lock_delay = 3  # number of seconds to pause after detecting a key press

    # keep the thread running until the program receives the shutdown signal
    while not _shutdown_flag:
        # if a key has been pressed, don't immediately attempt to drive again
        # instead, give the user time to react to the key press and then press another key
        if _key_input_lock:
            time.sleep(key_input_lock_delay)
            _key_input_lock = False
	    distance = 0
	
	# if we are currently escaping, reset the turning distance
	# additionally, set the lock that prevents multiple consecutive escape attempts
	elif _escape_lock:
	    distance = 0
	    _escape_reset_lock = True

        # if none of the higher-level tasks are active, drive forward
        elif not _turning_lock and not _escape_lock and not _key_input_lock:
            # publish the velocity
            move_cmd = Twist()
            move_cmd.linear.x = 0.15
            move_cmd.angular.z = 0
            cmd_vel.publish(move_cmd)
            # wait for 0.2 seconds (5 HZ) and publish again
            r = rospy.Rate(5)
            r.sleep()
	    _escape_reset_lock = False # after going forward once, we can allow an escape attempt

            # track the distance traveled
            distance += 1
            # if we've gone about a foot, then initiate the random turn protocol
            _turning_lock = distance >= max_straight_distance

        # turn randomly (uniformly sampled within +/- 15 degrees) after every 1ft of forward movement.
        elif _turning_lock:
	    print("\nTurning randomly after 1ft of forward movement")
            distance = 0  # reset the distance counter
            _turning_lock = True
            rng = random.randint(-45, 45) 
            turn(rng, 15)
            _turning_lock = False


def escape(data):
    """
    Callback function for image processing.
    Escapes from obstacles if necessary.
    """
    global _shutdown_flag
    global _key_input_lock
    global _turning_lock
    global _escape_lock
    global _escape_reset_lock
    global _bridge
    # don't escape while doing anything else, including escaping
    if not _key_input_lock and not _turning_lock and not _escape_lock and not _escape_reset_lock:
	# get the image, convert it to an array of depth values        
	cv_image = _bridge.imgmsg_to_cv2(data, "32FC1")

	# calculate the average depth at the midline of image
	# two averages, one for left half, one for right half
	# looking only at the midline works here due to all obstacles being large walls
	left_avg = 0
	right_avg = 0
	# Half of the point cloud width is 320
	half_width = 320
	# The midway height of the point cloud is 240
	half_height = 240

	for i in range(half_width):
	    left_avg += cv_image[half_height][i]
            right_avg += cv_image[half_height][half_width+i]
        left_avg /= half_width
        right_avg /= half_width

	# a second callback function may sneak in here before another one can lock it
	#	so we have to check just in case
	# if both averages are low enough, we need to 180
	# Incremental amount to turn when seeing an asymmetric object before re-checking
	turn_time = 5
	# Speed in rad/s for all turns
	turn_speed = 30
	if not _escape_lock and left_avg < .9 and right_avg < .9:
            print("\nEscaping symmetrical obstacles")
	    _escape_lock = True
	    turn(turn_speed, 55)	 # turn around ~180 degrees
	    time.sleep(1)	 # give it a little time to finish the turn
	    _escape_lock = False
	# if the left side is too close, turn right
	elif not _escape_lock and left_avg < .8:
            print("\nAvoiding asymmetrical obstacle, turning right")
            _escape_lock = True
	    turn(-turn_speed, turn_time)	 # turn right a small amount while the obstacle is close
	    time.sleep(1)	 # give it a little time to finish the turn
	    _escape_lock = False
	# if the right side is too close, turn left
	elif not _escape_lock and right_avg < .8:
            print("\nAvoiding asymmetrical obstacle, turning left")
	    _escape_lock = True
	    turn(turn_speed, turn_time)	 # turn left a small amount while the obstacle is close
	    time.sleep(1)	 # give it a little time to finish the turn
	    _escape_lock = False


def get_input():
    """
    Accept keyboard movement commands from a human user.
    """
    global _shutdown_flag
    global _key_input_lock

    # keep the thread running until the program receives the shutdown signal
    while not _shutdown_flag:
        # move based on keyboard input

        # setup move command
        turn_cmd = Twist()
        turn_cmd.linear.x = 0
        turn_cmd.angular.z = 0
        # get the curses screen window
        screen = curses.initscr()
        # turn off input echoing
        # curses.noecho()
        # respond to keys immediately (don't wait for enter)
        curses.cbreak()
        # map arrow keys to special values
        screen.keypad(True)

        char = ''
        # get input
        char = screen.getch()
        starttime = 0
	# if left turn left, if right turn right, if up go forward, if down go backward
	turn_speed = 60
	move_amount = 0.2
        if char == curses.KEY_RIGHT:
	    print("\nKey press RIGHT")
            _key_input_lock = True
            turn_cmd.angular.z = radians(-turn_speed)

        if char == curses.KEY_LEFT:
	    print("\nKey press LEFT")
            _key_input_lock = True
            turn_cmd.angular.z = radians(turn_speed)

        if char == curses.KEY_UP:
	    print("\nKey press FORWARD")
            _key_input_lock = True
            turn_cmd.linear.x = move_amount

        if char == curses.KEY_DOWN:
	    print("\nKey press BACKWARD")
            _key_input_lock = True
            turn_cmd.linear.x = -move_amount
	# if we have a velocity to publish, do it
        if _key_input_lock:
            cmd_vel.publish(turn_cmd)

    # shut down cleanly
    curses.nocbreak()
    screen.keypad(0)
    curses.echo()
    curses.endwin()


def collision(data):
    """
    Halt if collision detected by bumper(s).
    Wait for user to use keys to move away from the obstacle
    """
    global _key_input_lock
    if data.state == BumperEvent.PRESSED:
        _key_input_lock = True
	if _key_input_lock:
	    cmd_vel.publish(Twist())
            rospy.sleep(.2)
        print('\nCollision detected - use the arrow keys to move away')


def shutdown():
    """
    Stop all the processes and shutdown the turtlebot
    """
    global _shutdown_flag
    rospy.loginfo("Stopping TurtleBot")
    # a default Twist has linear.x of 0 and angular.z of 0
    cmd_vel.publish(Twist())
    # ensure TurtleBot receives the stop command prior to shutting down
    rospy.sleep(1)
    # join all _threads
    _shutdown_flag = True
    rospy.sleep(1)
    # join all _threads
    for t in _threads:
        print('Joining thread')
        t.join()
    # save the map
    os.system("rosrun map_server map_saver -f proj1_map_generated")
    print("Saving map files to current directory")


def main():
    # initialize
    rospy.init_node('Project_1_Robot_Control', anonymous=False)

    # Tell user how to stop TurtleBot
    rospy.loginfo("To stop TurtleBot press CTRL + C")

    # Create a publisher which can "talk" to TurtleBot and tell it to move
    cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=1)

    obstacle_sub = rospy.Subscriber('/camera/depth/image_raw', Image, escape)
    collision_sub = rospy.Subscriber('/mobile_base/events/bumper', BumperEvent, collision)

    # Function to call on ctrl + c
    rospy.on_shutdown(shutdown)

    # 10 HZ comms
    r = rospy.Rate(10);

    # Prep the move forward command
    move_cmd = Twist()
    move_cmd.linear.x = -0.2
    move_cmd.angular.z = 0

    tasks = [ get_input, go_forward]

    for t in tasks:
        th = threading.Thread(target=t)
        _threads.append(th)
        th.start()

    while not _shutdown_flag:
        # Do nothing, let the _threads execute
        pass

    print('Shutting down turtlebot')

if __name__ == '__main__':
    main()
