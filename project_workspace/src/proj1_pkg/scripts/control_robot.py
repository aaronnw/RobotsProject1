#!/usr/bin/python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import PointCloud2
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


cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=1)
turninglock = False
asymmetricescapelock = False
symmetricescapelock = False
escapelock = False
keyinputlock = False
threads = []
shutdown_flag = False

def go_forward():
    # Drive forward
    
    ### Get current position
    distance = 0
    max_straight_distance = 18
    global turninglock
    global shutdown_flag
    global keyinputlock
    while not shutdown_flag:
        if keyinputlock:
            time.sleep(3)
	    keyinputlock = False
        elif not turninglock and not asymmetricescapelock and not symmetricescapelock and not keyinputlock:
            # publish the velocity
            move_cmd = Twist()
            move_cmd.linear.x = 0.2
            move_cmd.angular.z = 0
            cmd_vel.publish(move_cmd)
	    # wait for 0.1 seconds (10 HZ) and publish again
            r = rospy.Rate(10)
            r.sleep()

            ####if we've gone approx a foot, then set some flag for turning to true
            distance += 1
            turninglock = distance >= max_straight_distance

        # Turn randomly (uniformly sampled within +/- 15 degrees) after every 1ft of forward movement.
        elif not keyinputlock and not symmetricescapelock and not asymmetricescapelock and turninglock:
	    distance = 0
            turninglock = True
            turn_cmd = Twist()
            turn_cmd.linear.x = 0
            rng = random.randint(-30, 30)
            turn_cmd.angular.z = radians(rng)
            #do this a couple times then turn off the need to turn
            for i in range(15):
                if keyinputlock:
                    break
                cmd_vel.publish(turn_cmd)
                r = rospy.Rate(10)
                r.sleep()

            turninglock = False
    
def escape(data):
    global shutdown_flag
    global keyinputlock
    global turninglock
    if not keyinputlock and not turninglock:
        #types = [(f.name, np.float32) for f in data.fields]
        #point_array = np.fromstring(data.data, types)
        #points = np.reshape(point_array, (data.height, data.width))
        points = pc2.read_points(data, skip_nans=True, field_names=('x','y','z'))
        xavg = 0.0
        yavg = 0.0
        zavg = 0.0
        pointcount = 0
        for p in points:
             pointcount += 1
             xavg += p[0]
             yavg += p[1]
             zavg += p[2]
        if pointcount != 0:
            xavg /= pointcount
            yavg /= pointcount        
            zavg /= pointcount

            print((xavg, yavg, zavg))
            #so it looks like the threshold for turning should be zavg < ~1.2, but NEED TO TEST THIS IN THE ROOM
            #as far as asymmetric escapes go

'''def escape_asymmetric():
    # Avoid asymmetric obstacles within 1ft in front of the robot.
    global shutdown_flag
    while not shutdown_flag:
        #actually no idea how this one works
        print('escape_asymmetric')
        asymmetricescapelock = False

def escape_symmetric():
    # Escape from (roughly) symmetric obstacles within 1ft in front of the robot.
    global shutdown_flag
    while not shutdown_flag:
        if not keyinputlock: #and there's an object  directly in front
            symmetricescapelock = True
            turn_cmd = Twist()
            turn_cmd.linear.x = 0
            rng = random.randint(-30, 30)
            turn_cmd.angular.z = radians(180+rng)
            #do this a couple times then turn off the need to turn
            for i in range(5):
                if keyinputlock:
                    break
                cmd_vel.publish(turn_cmd)
                r = rospy.Rate(10)
                r.sleep()
            symmetricescapelock = False
            '''

def get_input():
    # Accept keyboard movement commands from a human user.
    global shutdown_flag
    global keyinputlock
    while not shutdown_flag:
        #move based on keyboard input

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
        if char == curses.KEY_RIGHT:
            keyinputlock = True
            turn_cmd.angular.z = radians(-30)

        if char == curses.KEY_LEFT:
            keyinputlock = True
            turn_cmd.angular.z = radians(30)

        if char == curses.KEY_UP:
            keyinputlock = True
            turn_cmd.linear.x = 0.5

        if char == curses.KEY_DOWN:
            keyinputlock = True
            turn_cmd.linear.x = -0.5

        if keyinputlock:
            cmd_vel.publish(turn_cmd)

    # shut down cleanly
    curses.nocbreak()
    screen.keypad(0)
    curses.echo()
    curses.endwin()

def collision(data):
    global keyinputlock
    # Halt if collision detected by bumper(s).
    if data.state == BumperEvent.PRESSED:
	keyinputlock = True
	print('Collision detected - use keys to move away')

def shutdown():
    # stop turtlebot
    rospy.loginfo("Stop TurtleBot")
    # a default Twist has linear.x of 0 and angular.z of 0.  So it'll stop TurtleBot
    cmd_vel.publish(Twist())
    # sleep just makes sure TurtleBot receives the stop command prior to shutting down the script
    rospy.sleep(1)
    # join all threads
    global shutdown_flag
    shutdown_flag = True
    rospy.sleep(1)
    # join all threads
    for t in threads:
        print('joining thread')
        t.join()
    # save the map
    os.system("rosrun map_server map_saver -f proj1_map_generated")
    print("Saving map files to current directory")


def main():
    
    # initialize
    rospy.init_node('Proj1_Robot_Control', anonymous=False)

    # Tell user how to stop TurtleBot
    rospy.loginfo("To stop TurtleBot CTRL + C")

    # Create a publisher which can "talk" to TurtleBot and tell it to move
    cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=1)

    obstacle_sub = rospy.Subscriber('/camera/depth/points', PointCloud2, escape)
    collision_sub = rospy.Subscriber('/mobile_base/events/bumper', BumperEvent, collision)

    # Function to call on ctrl + c
    rospy.on_shutdown(shutdown)

    # 10 HZ comms
    r = rospy.Rate(10);

    # Prep the move forward command
    move_cmd = Twist()
    move_cmd.linear.x = -0.2
    move_cmd.angular.z = 0

    tasks = [#get_input,
             #escape_symmetric,
             #escape_asymmetric,
             go_forward]

    for t in tasks:
        th = threading.Thread(target=t)
        threads.append(th)
        th.start()

    while not shutdown_flag:
        # Do nothing, let the threads execute
        pass

    print('shutting down turtlebot')


if __name__ == '__main__':
    main()
