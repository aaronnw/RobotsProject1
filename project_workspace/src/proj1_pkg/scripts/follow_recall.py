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
import math
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
_colbridge = CvBridge()
_hz = 5
_left_avg = 123456
_right_avg = 123456
_follow_turn_direction = 'fwd'
_follow_movement = 'stop'
_follow_flag = False
_recall_flag = False
_recall_direct_flag = False
_move_log = []
_turn_speed = 25
_forward_speed = 0.15

sample_log = [('right', 'stop', 573, 433.0), ('right', 'stop', 584, 594.0), ('right', 'stop', 583, 578.0), ('right', 'stop', 590, 798.0), 
('right', 'go', 537, 885.0), ('right', 'go', 537, 882.0), ('left', 'stop', 137, 2116.0), ('left', 'stop', 190, 2116.0), 
('left', 'stop', 108, 2129.0), ('left', 'stop', 190, 2116.0), ('left', 'stop', 190, 2116.0), ('left', 'stop', 190, 2116.0), 
('left', 'stop', 189, 2116.0), ('right', 'stop', 583, 577.0), ('fwd', 'go', 309, 935.0), ('right', 'go', 385, 987.0), 
('right', 'go', 388, 989.0), ('fwd', 'stop', 258, 754.0), ('left', 'stop', 184, 613.0), ('left', 'stop', 248, 589.0), 
('fwd', 'stop', 429, 4774.0), ('left', 'go', 239, 1219.0)]


def turn(amount_rad):
	#Rad per 10th second
	"""
	Turn the robot at the given radian speed and for <duration> tenths of a second
	"""
	global _key_input_lock
	global _hz
	global _cmd_vel
	# The speed needed to turn 360 degrees in 10 seconds
	if amount_rad >= 0:
		turn_speed = math.pi/5
	else:
		turn_speed = -math.pi/5
	turn_time = abs(amount_rad/(turn_speed))
	if turn_time == 0:
		return
	print('begin turn of ', amount_rad)
	print('turn speed ', turn_speed)
	print('turn time ', turn_time)
	turn_cmd = Twist()
	turn_cmd.linear.x = 0
	turn_cmd.angular.z = turn_speed

	t0 = rospy.Time.now().to_sec()
	t1 = t0
	current_angle = 0
	while(t1-t0 < turn_time*1.42):  # 1.35
		_cmd_vel.publish(turn_cmd)
		t1 = rospy.Time.now().to_sec()
	print('done turn')
	_cmd_vel.publish(Twist())


def follow():
	global _follow_turn_direction
	global _follow_movement
	global _shutdown_flag
	global _follow_flag
	global _cmd_vel
	global _turn_speed
	global _forward_speed
	global _recall_flag

	while not _shutdown_flag:
		# ensure the follow flag is active
	 	if _follow_flag and not _collision_lock and not _recall_flag:
			#print(_follow_turn_direction, _follow_movement)
			move_cmd = Twist()
			# change the movement command based on the variables changed in the callback function
			if _follow_movement == 'go':
				move_cmd.linear.x = _forward_speed
			if _follow_turn_direction == 'left':
				move_cmd.angular.z = radians(_turn_speed)
			elif _follow_turn_direction == 'right':
				move_cmd.angular.z = radians(-1 * _turn_speed)
			_cmd_vel.publish(move_cmd)
			# log the move if it is a non-zero move
			if move_cmd.linear.x != 0 or move_cmd.angular.z != 0:
				_move_log.append(move_cmd)

		r = rospy.Rate(_hz)
		r.sleep()


def backtrack_all():
	global _shutdown_flag
	global _collision_lock
	global _move_log
	global _cmd_vel
	global _hz

	r = rospy.Rate(_hz)
	# loop through the movement log
	while len(_move_log) > 0 and not _shutdown_flag and not _collision_lock:
		# get the last move and invert it
		move_cmd = _move_log[-1]
		move_cmd.linear.x *= -1
		move_cmd.angular.z *= -1
		_cmd_vel.publish(move_cmd)
		# after use, delete it so we have a new last move
		del _move_log[-1]
		# sleep at the same rate(!!!) as we follow so the moves are for the appropriate amount of time
		r.sleep()
	_move_log = []
	print('Backtrack all finished')


def backtrack_direct():
	"""
		Perform the logged moves in reverse to return to the saved position
	"""
	global _shutdown_flag
	global _collision_lock
	global _move_log
	global _cmd_vel
	global _hz

	x = 0
	y = 0
	last_x = 0
	last_y = 0
	current_angle = 0 
	for movement in _move_log:
		r = movement.linear.x * 1/_hz
		angle = movement.angular.z * 1/_hz
		current_angle += angle
		x = last_x + r*math.cos(current_angle)
		y = last_y + r*math.sin(current_angle)
		print("current x: ", x)
		print("current y: ", y)
		last_x = x
		last_y = y
	_move_log = []
	# Turn 180
	# turn(math.pi)
	# Travel the vector_back
	distance = math.sqrt(x**2 + y **2)
	if y != 0:
		angle_to_origin = math.atan2(x, y)
	else:
		angle_to_origin = 0
	#turn_angle = 0
	#if y > 0: 
	#	turn_angle = -(math.pi - angle_to_origin) - current_angle
	#else:
	#	turn_angle = angle_to_origin - current_angle
	turn_angle = -current_angle
	if y > 0:
		turn_angle -= (math.pi/2 + angle_to_origin)
	else:
		turn_angle -= (math.pi/2 + angle_to_origin)

	print("x:", x)
	print("y:", y)
	print("to origin:", angle_to_origin)
	print("current", current_angle)
	if turn_angle > math.pi:
		turn_angle = math.pi*2 - turn_angle
	turn(turn_angle)

	# r = rospy.Rate(1/distance)
	return_cmd = Twist()
	return_cmd.linear.x = _forward_speed
	return_cmd.angular.z = 0

	t0 = rospy.Time.now().to_sec()
	t1 = t0
	current_angle = 0
	while(t1-t0 < distance/_forward_speed):
		_cmd_vel.publish(return_cmd)
		t1 = rospy.Time.now().to_sec()
	_cmd_vel.publish(Twist())
	print('Backtrack direct finished')


def recall():
	"""
		Determines when to recall the robot to the saved position
	"""
	global _shutdown_flag
	global _collision_lock
	global _recall_flag
	global _recall_direct_flag

	# constantly checking for when recall needs to occur
	while not _shutdown_flag:
		if not _collision_lock and _recall_flag:
			# call the appropriate recall function
			#return_to_start(_turn_speed, _forward_speed, _move_log)
			if _recall_direct_flag:
				print("Direct back")
				backtrack_direct()
			else:
				print("ALL back")
				backtrack_all()
			_recall_flag = False	# reset the flag
			_move_log = []

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
	global _recall_direct_flag
	global _collision_lock
	# print('image received')
	# no need to do these calculations if we're in a recall routine
	if not _recall_flag:
		# convert the images appropriately
		cv_depth = _bridge.imgmsg_to_cv2(depth, "32FC1")
		cv_color = _bridge.imgmsg_to_cv2(color, "rgb8")

		# color calculations
		center_color = cv_color[240][320]
		red_val = int(center_color[0])
		green_val = int(center_color[1])
		blue_val = int(center_color[2])
		# print(center_color)
		# which color is the most prevalent?
		primary = max([red_val, green_val, blue_val])
		# determine signaling for colors based on relative importance to the most prevalent color
		# If red, follow
		if primary == red_val and red_val-green_val > 80 and red_val - blue_val > 80:
			print('red')	 # begin following
			_recall_flag = False
			_follow_flag = True
			_collision_lock = False	 # note that this is the only way to override collision
		# some colors are more sensitive than others
		# If green, fire reverse action recall
		elif primary == green_val and green_val-red_val > 40 and green_val - blue_val > 40:
			print('green')	 # begin recall
			_follow_flag = False
			_recall_direct_flag = False
			_recall_flag = True
		# If yellow, fire direct recall
		elif primary == red_val and green_val-blue_val > 50 and red_val - green_val < 25:
			_follow_flag = False
			_recall_direct_flag = True	
			_recall_flag = True
			print('yellow')
		# If blue, save a location
		elif primary == blue_val and blue_val-red_val > 40 and blue_val - green_val > 40:
			# _move_log = []	 # save the position
			print('blue')

		# depth calculations
		closest = (0, 0)
		closest_depth = 100000000000
		for j in range(640):
			# check the midline for the closest point
			if cv_depth[240][j] > 0 and cv_depth[240][j] < closest_depth:
				closest_depth = cv_depth[240][j]
				closest = (240, j)
		# this is the "wall check" - when the center point is very close to the closest point, go forward
		# when facing a wall, this stops the robot from oscillating back and forth forever
		if closest_depth < 900 and cv_depth[240][320] - closest_depth < 50:
			_follow_turn_direction = 'fwd'
		# determining whether to turn is a function of depth as well
		# the further away an object is, the wider the range to go forward is
		elif closest[1] < 270 - max((closest_depth - 800), 0) / 20:
			_follow_turn_direction = 'left'
		elif closest[1] > 370 + max((closest_depth - 800), 0) / 20:
			_follow_turn_direction = 'right'
		else:
			_follow_turn_direction = 'fwd'

		# if we're not too close to the nearest object, go forward
		# additionally confirm the object is not too far away
		if closest_depth >= 700 and closest_depth < 1800:
			_follow_movement = 'go'
		else:
			_follow_movement = 'stop'


def collision(data):
	"""
	Halt if collision detected by bumper(s).
	"""
	global _collision_lock
	global _cmd_vel
	print("Bumper Hit, stopping")
	if data.state == BumperEvent.PRESSED:
		_collision_lock = True
		_follow_flag = False
		_recall_flag = False
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
	ts.registerCallback(image_calc)		# combine the depth and image callbacks
	collision_sub = rospy.Subscriber('/mobile_base/events/bumper', BumperEvent, collision)

	# Function to call on ctrl + c
	rospy.on_shutdown(shutdown)

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
