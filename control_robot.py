import rospy
from geometry_msgs.msg import Twist
import threading
import time
import random
import Queue
from math import radians
import curses


cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)
turninglock = False
asymmetricescapelock = False
symmetricescapelock = False
keyinputlock = False
collisionlock = False
threads = []
shutdown_flag = False

def go_forward():
    # Drive forward
    
    ### Get current position
    distance = 0
    max_straight_distance = 25
    global turninglock
    global shutdown_flag
    while not shutdown_flag:
        if keyinputlock:
            rospy.sleep(3)
        if not turninglock and not asymmetricescapelock and not symmetricescapelock and not keyinputlock and not collisionlock:
            print('moving forward | distance =', distance)
            # publish the velocity
            move_cmd = Twist()
            move_cmd.linear.x = 0.2
            move_cmd.angular.z = 0
            cmd_vel.publish(move_cmd)
	    # wait for 0.1 seconds (10 HZ) and publish again
            r = rospy.Rate(10)
            r.sleep()

            ####if we've gone a foot, then set some flag for turning to true
            distance += 1
            turninglock = distance >= max_straight_distance
        else:
            distance = 0

def turn_random():
    global turninglock
    global shutdown_flag
    while not shutdown_flag:
        # Turn randomly (uniformly sampled within +/- 15 degrees) after every 1ft of forward movement.
        if not collisionlock and not keyinputlock and not symmetricescapelock and not asymmetricescapelock and turninglock:
            print('starting random turning')
            turninglock = True
            turn_cmd = Twist()
            turn_cmd.linear.x = 0
            rng = random.randint(-30, 30)
            turn_cmd.angular.z = radians(rng)
            #do this a couple times then turn off the need to turn
            for i in range(20):
                if keyinputlock or collisionlock:
                    break
                cmd_vel.publish(turn_cmd)
                r = rospy.Rate(10)
                r.sleep()

            turninglock = False
    
def escape_asymmetric():
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
                if keyinputlock or collisionlock:
                    break
                cmd_vel.publish(turn_cmd)
                r = rospy.Rate(10)
                r.sleep()
            symmetricescapelock = False
            

def get_input():
    # Accept keyboard movement commands from a human user.
    global shutdown_flag
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
	print('char is' + str(char))
	starttime = 0
        if char == curses.KEY_RIGHT:
            keyinputlock = True
            print('right\n')
            turn_cmd.angular.z = radians(-15)

        if char == curses.KEY_LEFT:
            keyinputlock = True
            print('left\n')
            turn_cmd.angular.z = radians(15)

        if char == curses.KEY_UP:
            keyinputlock = True
            print('up\n')
            turn_cmd.linear.x = 0.2

        if char == curses.KEY_DOWN:
            keyinputlock = True
            print('down\n')
            turn_cmd.linear.x = -0.2

        if keyinputlock:
            cmd_vel.publish(turn_cmd)
            keyinputlock = False



    # shut down cleanly
    curses.nocbreak()
    screen.keypad(0)
    curses.echo()
    curses.endwin()

def collision():
    # Halt if collision(s) detected by bumper(s).
    global shutdown_flag
    while not shutdown_flag:
        if False: #there's a collision
            shutdown()

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


def main():
    
    # initialize
    rospy.init_node('Project 1 Robot Control', anonymous=False)

    # Tell user how to stop TurtleBot
    rospy.loginfo("To stop TurtleBot CTRL + C")

    # Create a publisher which can "talk" to TurtleBot and tell it to move
    cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=2)

    # Function to call on ctrl + c
    rospy.on_shutdown(shutdown)

    # 10 HZ comms
    r = rospy.Rate(10);

    # Prep the move forward command
    move_cmd = Twist()
    move_cmd.linear.x = -0.2
    move_cmd.angular.z = 0

    tasks = [
             #collision,
             get_input]
             #escape_symmetric,
             # escape_asymmetric,
             #turn_random,
             #go_forward]

    for t in tasks:
        th = threading.Thread(target=t)
        #th.daemon = True
        threads.append(th)
        th.start()

    while not shutdown_flag:
        # Do nothing, let the threads execute
        pass

    print('shutting down turtlebot')


if __name__ == '__main__':
    main()
