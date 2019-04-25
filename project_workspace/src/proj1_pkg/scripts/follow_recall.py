#!/usr/bin/python

import rospy
import message_filters
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
import recall
from cv_bridge import CvBridge, CvBridgeError

_cmd_vel = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size=1)
_turning_lock = False
_escape_lock = False
_key_input_lock = False
_threads = []
_shutdown_flag = False
_collision_lock = False
_bridge = CvBridge()
_colbridge = CvBridge()
_hz = 5
_left_avg = 123456
_right_avg = 123456
_follow_turn_direction = 'fwd'
_follow_movement = 'stop'
_follow_flag = True
_recall_flag = False
_move_log = []

sample_log = [('right', 'stop', 573, 433.0), ('right', 'stop', 584, 594.0), ('right', 'stop', 583, 578.0), ('right', 'stop', 590, 798.0), 
('right', 'go', 537, 885.0), ('right', 'go', 537, 882.0), ('left', 'stop', 137, 2116.0), ('left', 'stop', 190, 2116.0), 
('left', 'stop', 108, 2129.0), ('left', 'stop', 190, 2116.0), ('left', 'stop', 190, 2116.0), ('left', 'stop', 190, 2116.0), 
('left', 'stop', 189, 2116.0), ('right', 'stop', 583, 577.0), ('fwd', 'go', 309, 935.0), ('right', 'go', 385, 987.0), 
('right', 'go', 388, 989.0), ('fwd', 'stop', 258, 754.0), ('left', 'stop', 184, 613.0), ('left', 'stop', 248, 589.0), 
('fwd', 'stop', 429, 4774.0), ('left', 'go', 239, 1219.0)]


def turn(angle, duration):
	"""
	Turn the robot at the given radian speed and for <duration> tenths of a second
	"""
	global _hz
	global _cmd_vel
	global _key_input_lock
	r = rospy.Rate(_hz)
	print('begin turn')
	turn_cmd = Twist()
	turn_cmd.linear.x = 0
	turn_cmd.angular.z = radians(angle)
	# do this duration times
	for i in range(duration):
		if _key_input_lock:
			break
		#print('actively turning')
		_cmd_vel.publish(turn_cmd)
		r.sleep()
	#print('done turning')
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
	max_straight_distance = 23  # maximum distance to travel before making a random turn
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
			_turning_lock = distance >= max_straight_distance

		# turn randomly (uniformly sampled within +/- 15 degrees) after every 1ft of forward movement.
		elif _turning_lock:
			print("\nTurning randomly after 1ft of forward movement")
			distance = 0  # reset the distance counter
			_turning_lock = True
			direction = random.choice([-1, 1]) 
			rng = random.randint(1, 8)
			turn_speed = 35
			turn(turn_speed*direction, rng)
			_turning_lock = False


def escape():
	global _key_input_lock
	global _turning_lock
	global _escape_lock
	global _left_avg
	global _right_avg

	turn_time = 7
	# Speed in rad/s for all turns
	turn_speed = 35
	while not _shutdown_flag:
		if _key_input_lock or _turning_lock:
			continue
		elif _left_avg < 885 and _right_avg < 885:
			print(_left_avg, _right_avg)
			print("\nEscaping symmetrical obstacles")
			_escape_lock = True
			turn(turn_speed, 33)	 # turn around ~180 degrees
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


def follow():
	global _follow_turn_direction
	global _follow_movement
	global _shutdown_flag
	global _follow_flag
	global _cmd_vel

	while not _shutdown_flag:
	 	if not _collision_lock and _follow_flag:
			#print(_follow_turn_direction, _follow_movement)
			move_cmd = Twist()
			
			turnspeed = 15
			if _follow_movement == 'go':
				move_cmd.linear.x = 0.1
			if _follow_turn_direction == 'left':
				move_cmd.angular.z = radians(turnspeed)
			elif _follow_turn_direction == 'right':
				move_cmd.angular.z = radians(-1 * turnspeed)
			_cmd_vel.publish(move_cmd)
			
		r = rospy.Rate(20)
		r.sleep()


def recall():
	while not _shutdown_flag:
		if not _collision_lock and _recall_flag:
			print('recall goes here')
		r = rospy.Rate(1)
		r.sleep()


def image_calc(depth, color):
	"""
	Callback function for image depth and color processing.
	"""
	global _follow_turn_direction
	global _follow_movement
	global _move_log
	global _follow_flag
	global _recall_flag

	cv_depth = _bridge.imgmsg_to_cv2(depth, "32FC1")
	cv_color = _bridge.imgmsg_to_cv2(color, "rgb8")

	# color calculations
	color = cv_color[240][320]
	if color[0] > 200 and color[1] < 150 and color[2] < 150:
		print('red')
		_recall_flag = False
		_follow_flag = True
	elif color[0] < 100 and color[1] > 150 and color[2] < 100:
		print('green')
		_follow_flag = False
		_recall_flag = True
	elif color[0] < 150 and color[1] < 150 and color[2] > 200:
		_move_log = []
		print('blue')

	# depth calculations
	closest = (0, 0)
	closest_depth = 100000000000
	for j in range(640):
		if cv_depth[240][j] > 0 and cv_depth[240][j] < closest_depth:
			closest_depth = cv_depth[240][j]
			closest = (240, j)

	if closest_depth < 900 and cv_depth[240][320] - closest_depth < 50:
		_follow_turn_direction = 'fwd'
	elif closest[1] < 270 - max((closest_depth - 800), 0) / 20:
		_follow_turn_direction = 'left'
	elif closest[1] > 370 + max((closest_depth - 800), 0) / 20:
		_follow_turn_direction = 'right'
	else:
		_follow_turn_direction = 'fwd'

	if closest_depth >= 800 and closest_depth < 1800:
		_follow_movement = 'go'
	else:
		_follow_movement = 'stop'

	if _follow_flag and not _collision_lock:
		_move_log.append((_follow_turn_direction, _follxow_movement, closest[1], closest_depth))
		print(_move_log)


def collision(data):
	"""
	Halt if collision detected by bumper(s).
	Wait for user to use keys to move away from the obstacle
	"""
	global _collision_lock
	global _cmd_vel
	print("Bumper Hit, stopping")
	if data.state == BumperEvent.PRESSED:
		_collision_lock = True
		_cmd_vel.publish(Twist())


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
	#print("Saving map files to current directory")


def main():
	recall.test()
	global _threads
	global _cmd_vel
	# initialize
	rospy.init_node('Project_1_Robot_Control', anonymous=False)

	# Tell user how to stop TurtleBot
	rospy.loginfo("To stop TurtleBot press CTRL + C")

	# Create a publisher which can "talk" to TurtleBot and tell it to move
	_cmd_vel = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size=0)

	# Subscribers to relevant topics and their associated callback functions
	depth_sub = message_filters.Subscriber('/camera/depth/image_raw', Image)
	color_sub = message_filters.Subscriber('/camera/rgb/image_raw', Image)
	ts = message_filters.ApproximateTimeSynchronizer([depth_sub, color_sub], 10, 1)
	ts.registerCallback(image_calc)
	collision_sub = rospy.Subscriber('/mobile_base/events/bumper', BumperEvent, collision)

	# Function to call on ctrl + c
	rospy.on_shutdown(shutdown)

	# xxxxx HZ comms
	r = rospy.Rate(_hz);

	tasks = [follow, recall]

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
