#!/usr/bin/python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, CompressedImage 
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

_cmd_vel = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size=1)
_turning_lock = False
_escape_lock = False
_key_input_lock = False
_threads = []
_shutdown_flag = False
_collision_lock = False
_bridge = CvBridge()
_hz = 5
_left_avg = 123456
_right_avg = 123456

def turn(angle, duration):
	"""
	Turn the robot at the given radian speed and for <duration> tenths of a second
	"""
	global _hz
	global _cmd_vel
	global _key_input_lock
	#_cmd_vel.publish(Twist())
	r = rospy.Rate(_hz)
	print('begin turn')
	turn_cmd = Twist()
	turn_cmd.linear.x = 0
	turn_cmd.angular.z = radians(angle)
	# do this duration times
	for i in range(duration):
		if _key_input_lock:
		   break
		print('actively turning')
		_cmd_vel.publish(turn_cmd)
		r.sleep()
	print('done turning')
	_cmd_vel.publish(Twist())
	r.sleep()

def go_forward():
	"""
	Drive the robot forward, making a random turn after every foot of movement
	"""
	global _turning_lock
	global _shutdown_flag
	global _key_input_lock
	global _escape_lock
	global _collision_lock
	global _hz
	global _cmd_vel
	distance = 0  # distance traveled since last random turn, or since last lock release
	max_straight_distance = 18  # maximum distance to travel before making a random turn
	key_input_lock_delay = 3  # number of seconds to pause after detecting a key press

	# keep the thread running until the program receives the shutdown signal
	while not _shutdown_flag:
		# if a key has been pressed, don't immediately attempt to drive again
		# instead, give the user time to react to the key press and then press another key
		if _key_input_lock and not _collision_lock:
			time.sleep(key_input_lock_delay)
			_key_input_lock = False
			distance = 0

		# if we are currently escaping, reset the turning distance
		elif _escape_lock:
			distance = 0

		# if none of the higher-level tasks are active, drive forward
		elif not _turning_lock and not _escape_lock and not _key_input_lock:
			# publish the velocity
			move_cmd = Twist()
			move_cmd.linear.x = 0.15
			move_cmd.angular.z = 0
			#print('moving???')
			_cmd_vel.publish(move_cmd)
			# wait for 0.5 seconds (2 HZ) and publish again
			r = rospy.Rate(10)
			r.sleep()

			# track the distance traveled
			distance += 1
			# if we've gone about a foot, then initiate the random turn protocol
			#_turning_lock = distance >= max_straight_distance

		# turn randomly (uniformly sampled within +/- 15 degrees) after every 1ft of forward movement.
		elif _turning_lock:
			print("\nTurning randomly after 1ft of forward movement")
			distance = 0  # reset the distance counter
			_turning_lock = True
			direction = random.choice([-1, 1]) 
			rng = random.randint(1, 12)
			turn(30*direction, rng)
			_turning_lock = False

def escape():
	global _key_input_lock
	global _turning_lock
	global _escape_lock
	global _left_avg
	global _right_avg

	turn_time = 25
	# Speed in rad/s for all turns
	turn_speed = 30
	while True:
		if _key_input_lock or _turning_lock:
			print("LOCKED BY KEYS")
			return
		elif _left_avg < 850 and _right_avg < 850:
			print(_left_avg, _right_avg)
			print("\nEscaping symmetrical obstacles")
			_escape_lock = True
			turn(turn_speed, 35)	 # turn around ~180 degrees
			time.sleep(1)	 # give it a little time to finish the turn
			_escape_lock = False
		# if the left side is too close, turn right
		elif _left_avg < 750:
			print(_left_avg, _right_avg)
			print("\nAvoiding asymmetrical obstacle, turning right")
			_escape_lock = True
			turn(-turn_speed, turn_time)	 # turn right a small amount while the obstacle is close
			time.sleep(1)	 # give it a little time to finish the turn
			_escape_lock = False
		# if the right side is too close, turn left
		elif _right_avg < 750:
			print(_left_avg, _right_avg)
			print("\nAvoiding asymmetrical obstacle, turning left")
			_escape_lock = True
			turn(turn_speed, turn_time)	 # turn left a small amount while the obstacle is close
			time.sleep(1)	 # give it a little time to finish the turn
			_escape_lock = False

def escape_calc(image):
	"""
	Callback function for image processing.
	"""   
	global _left_avg
	global _right_avg

	cv_image = _bridge.imgmsg_to_cv2(image, "32FC1")
	# print(cv_image)
	# calculate the average depth at the midline of image
	# two averages, one for left half, one for right half
	# looking only at the midline works here due to all obstacles being large walls
	running_left_avg = 0
	running_right_avg = 0
	# Half of the point cloud width is 320
	half_width = 320
	# The midway height of the point cloud is 240
	half_height = 240

	for i in range(half_width):
		running_left_avg += cv_image[half_height][i]
		running_right_avg += cv_image[half_height][half_width+i]
	_left_avg = running_left_avg / half_width
	_right_avg = running_right_avg / half_width

def get_input():
	"""
	Accept keyboard movement commands from a human user.
	"""
	global _shutdown_flag
	global _key_input_lock
	global _collision_lock
	global _cmd_vel

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
			_collision_lock = False
			_cmd_vel.publish(turn_cmd)

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
	global _collision_lock
	global _cmd_vel
	if data.state == BumperEvent.PRESSED:
		_key_input_lock = True
		_collision_lock = True
		if _key_input_lock:
			_cmd_vel.publish(Twist())
			print('\nCollision detected - use the arrow keys to move away')


def shutdown():
	"""
	Stop all the processes and shutdown the turtlebot
	"""
	global _shutdown_flag
	global _threads
	global _cmd_vel
	rospy.loginfo("Stopping TurtleBot")
	# a default Twist has linear.x of 0 and angular.z of 0
	_cmd_vel.publish(Twist())
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
	#os.system("rosrun map_server map_saver -f proj1_map_generated")
	print("Saving map files to current directory")


def main():
	global _threads
	global _cmd_vel
	# initialize
	rospy.init_node('Project_1_Robot_Control', anonymous=False)

	# Tell user how to stop TurtleBot
	rospy.loginfo("To stop TurtleBot press CTRL + C")

	# Create a publisher which can "talk" to TurtleBot and tell it to move
	_cmd_vel = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size=0)

	#Options: 
	# /camera/depth/image_raw
	# /camera/depth/image_raw/compressedDepth
	# /camera/depth/image/compressedDepth
	# /camera/depth/image_rect/compressedDepth
	#Change type to compressed image
	obstacle_sub = rospy.Subscriber('/camera/depth/image_raw', Image, escape_calc, queue_size=1)
	collision_sub = rospy.Subscriber('/mobile_base/events/bumper', BumperEvent, collision)

	# Function to call on ctrl + c
	rospy.on_shutdown(shutdown)

	# xxxxx HZ comms
	r = rospy.Rate(_hz);

	# Prep the move forward command
	#move_cmd = Twist()
	#move_cmd.linear.x = -0.2
	#move_cmd.angular.z = 0

	tasks = [ go_forward, escape, get_input ]
	#tasks = [escape, get_input ]

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
