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

cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=1)
_turning_lock = False
_asymmetric_escape_lock = False
symmetric_escape_lock = False
_escape_lock = False
_key_input_lock = False
_threads = []
_shutdown_flag = False


def go_forward():
    """
    Drive the robot forward, making a random turn after every foot of movement
    """
    global _turning_lock
    global _shutdown_flag
    global _key_input_lock
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

        # if none of the higher-level tasks are active, drive forward
        elif not _turning_lock and not _asymmetric_escape_lock and not symmetric_escape_lock and not _key_input_lock:
            # publish the velocity
            move_cmd = Twist()
            move_cmd.linear.x = 0.2
            move_cmd.angular.z = 0
            cmd_vel.publish(move_cmd)
            # wait for 0.1 seconds (10 HZ) and publish again
            r = rospy.Rate(10)
            r.sleep()

            # track the distance traveled
            distance += 1
            # if we've gone about a foot, then initiate the random turn protocol
            _turning_lock = distance >= max_straight_distance

        # turn randomly (uniformly sampled within +/- 15 degrees) after every 1ft of forward movement.
        elif _turning_lock:
            distance = 0  # reset the distance counter
            _turning_lock = True
            turn_cmd = Twist()
            turn_cmd.linear.x = 0
            rng = random.randint(-30, 30)
            turn_cmd.angular.z = radians(rng)
            # do this a couple times then turn off the need to turn
            for i in range(15):
                # if a key is pressed while turning, stop turning
                if _key_input_lock:
                    break
                cmd_vel.publish(turn_cmd)
                r = rospy.Rate(10)
                r.sleep()

            _turning_lock = False


# TODO: add comments when this is done
def escape(data):
    global _shutdown_flag
    global _key_input_lock
    global _turning_lock
    if not _key_input_lock and not _turning_lock:
        # types = [(f.name, np.float32) for f in data.fields]
        # point_array = np.fromstring(data.data, types)
        # points = np.reshape(point_array, (data.height, data.width))
        points = pc2.read_points(data, skip_nans=True, field_names=('x', 'y', 'z'))
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
            # so it looks like the threshold for turning should be zavg < ~1.2, but NEED TO TEST THIS IN THE ROOM
            # as far as asymmetric escapes go


'''def escape_asymmetric():
    # Avoid asymmetric obstacles within 1ft in front of the robot.
    global _shutdown_flag
    while not _shutdown_flag:
        #actually no idea how this one works
        print('escape_asymmetric')
        _asymmetric_escape_lock = False

def escape_symmetric():
    # Escape from (roughly) symmetric obstacles within 1ft in front of the robot.
    global _shutdown_flag
    while not _shutdown_flag:
        if not _key_input_lock: #and there's an object  directly in front
            symmetric_escape_lock = True
            turn_cmd = Twist()
            turn_cmd.linear.x = 0
            rng = random.randint(-30, 30)
            turn_cmd.angular.z = radians(180+rng)
            #do this a couple times then turn off the need to turn
            for i in range(5):
                if _key_input_lock:
                    break
                cmd_vel.publish(turn_cmd)
                r = rospy.Rate(10)
                r.sleep()
            symmetric_escape_lock = False
            '''


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
        if char == curses.KEY_RIGHT:
            _key_input_lock = True
            turn_cmd.angular.z = radians(-30)

        if char == curses.KEY_LEFT:
            _key_input_lock = True
            turn_cmd.angular.z = radians(30)

        if char == curses.KEY_UP:
            _key_input_lock = True
            turn_cmd.linear.x = 0.5

        if char == curses.KEY_DOWN:
            _key_input_lock = True
            turn_cmd.linear.x = -0.5

        if _key_input_lock:
            cmd_vel.publish(turn_cmd)

    # shut down cleanly
    curses.nocbreak()
    screen.keypad(0)
    curses.echo()
    curses.endwin()


# TODO: do something intelligent instead of just shutting down
def collision(data):
    """
    Halt if collision detected by bumper(s).
    """
    global _key_input_lock
    if data.state == BumperEvent.PRESSED:
        _key_input_lock = True
        print('Collision detected - use keys to move away')


def shutdown():
    """
    Stop all the processes and shutdown the turtlebot
    """
    global _shutdown_flag
    rospy.loginfo("Stop TurtleBot")
    # a default Twist has linear.x of 0 and angular.z of 0
    cmd_vel.publish(Twist())
    # ensure TurtleBot receives the stop command prior to shutting down
    rospy.sleep(1)
    # join all _threads
    _shutdown_flag = True
    rospy.sleep(1)
    # join all _threads
    for t in _threads:
        print('joining thread')
        t.join()


def main():
    # initialize
    rospy.init_node('Project 1 Robot Control', anonymous=False)

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

    tasks = [  # get_input,
        # escape_symmetric,
        # escape_asymmetric,
        go_forward]

    for t in tasks:
        th = threading.Thread(target=t)
        _threads.append(th)
        th.start()

    while not _shutdown_flag:
        # Do nothing, let the _threads execute
        pass

    print('shutting down turtlebot')


if __name__ == '__main__':
    main()
