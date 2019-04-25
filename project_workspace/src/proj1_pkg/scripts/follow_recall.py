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
import recall_script
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


def follow():
	global _follow_turn_direction
	global _follow_movement
	global _shutdown_flag
	global _follow_flag
	global _cmd_vel

	while not _shutdown_flag and not _recall_flag:
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
	global _shutdown_flag
	global _collision_lock
	global _recall_flag

	while not _shutdown_flag:
		if not _collision_lock and _recall_flag:
			recall_script.return_to_start(_move_log)
			_recall_flag = False
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

	if not _recall_flag:
		cv_depth = _bridge.imgmsg_to_cv2(depth, "32FC1")
		cv_color = _bridge.imgmsg_to_cv2(color, "rgb8")

		# color calculations
		center_color = cv_color[240][320]
		red_val = center_color[0]
		green_val = center_color[1]
		blue_val = center_color[2]

		primary = max([red_val, green_val, blue_val])

		print(center_color)
		if primary == red_val and red_val > 200 and red_val-green_val > 80 and red_val - blue_val > 80:
			print('red')
			_recall_flag = False
			_follow_flag = True
		elif primary == green_val and green_val > 100 and green_val-red_val > 25 and green_val - blue_val > 25:
			print('green')
			_follow_flag = False
			_recall_flag = True
		elif primary == blue_val and blue_val > 100 and blue_val-red_val > 50 and blue_val - green_val > 50:
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
			_move_log.append((_follow_turn_direction, _follow_movement, closest[1], closest_depth))
			# print(_move_log)


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
