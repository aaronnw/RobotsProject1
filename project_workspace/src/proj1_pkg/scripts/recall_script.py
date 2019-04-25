def test():
	print("REcall script woo")


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


def return_to_start(log_of_movements):
	# Construct a rough graph of locations from the movement log
	# A* search the graph
	print(log_of_movements)
	test()