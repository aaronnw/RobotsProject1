import rospy
import threading
import time
import random
from queue import Queue

turninglock = False
asymmetricescapelock = False
symmetricescapelock = False
keyinputlock = False
collisionlock = False

def go_forward():
    # Drive forward
    
    ### Get current position
    
    while True:
        if not turninglock:
	        # publish the velocity
            self.cmd_vel.publish(move_cmd)
	        # wait for 0.1 seconds (10 HZ) and publish again
            r.sleep()
            
            ####if we've gone a foot, then set some flag for turning to true
            needtoturn = False

def turn_random():
    while True:
        # Turn randomly (uniformly sampled within ±15°) after every 1ft of forward movement.
        if not asymmetricescapelock and needtoturn:
            turninglock = True
            turn_cmd = Twist()
            turn_cmd.linear.x = 0
            rng = random.randint(-15, 15)
            turn_cmd.angular.z = radians(rng);
            self.cmd_vel.publish(turn_cmd)
            r.sleep()
            #do this a couple times then turn off the need to turn

def escape_asymmetric():
    # Avoid asymmetric obstacles within 1ft in front of the robot.
    while True:
        #actually no idea how this one works
        print('escape_asymmetric')

def escape_symmetric():
    # Escape from (roughly) symmetric obstacles within 1ft in front of the robot.
    while True:
        if not keyinputlock: #and there's an object  directly in front
            symmetricescapelock = True
            turn_cmd = Twist()
            turn_cmd.linear.x = 0
            rng = random.randint(-30, 30)
            turn_cmd.angular.z = radians(180+rng);
            self.cmd_vel.publish(turn_cmd)
            r.sleep()
            #do this a couple times then turn off the need to turn

def get_input():
    # Accept keyboard movement commands from a human user.
    while True :
        #moved based on keyboard input? not sure how this works

def collision():
    # Halt if collision(s) detected by bumper(s).
    while True :
        if False: #there's a collision
            self.shutdown()
            
def shutdown(self):
    # stop turtlebot
    rospy.loginfo("Stop TurtleBot")
	# a default Twist has linear.x of 0 and angular.z of 0.  So it'll stop TurtleBot
    self.cmd_vel.publish(Twist())
	# sleep just makes sure TurtleBot receives the stop command prior to shutting down the script
    rospy.sleep(1)
    

def main():
    
    # initialize
    rospy.init_node('Project 1 Robot Control', anonymous=False)

	# Tell user how to stop TurtleBot
	rospy.loginfo("To stop TurtleBot CTRL + C")

    # Function to call on ctrl + c    
    rospy.on_shutdown(self.shutdown)
        
	# Create a publisher which can "talk" to TurtleBot and tell it to move
    self.cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)
     
	# 10 HZ comms
    r = rospy.Rate(10);

    # Prep the move forward command
    move_cmd = Twist()
    move_cmd.linear.x = 0.2
	move_cmd.angular.z = 0
    
    tasks = [collision,
             get_input,
             escape_symmetric,
             escape_asymmetric,
             turn_random,
             go_forward]

    threads = []

    for t in tasks:
        th = threading.Thread(target=t)
        threads.append(th)
        th.start()
        
    while not rospy.is_shutdown():
        # Do nothing, let the threads execute
        a = 1

if __name__ == '__main__':
    main()
