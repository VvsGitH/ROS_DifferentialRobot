#!/usr/bin/env python

# PYTHON CODE TO CONTROL THE DIFFERENTIAL ROBOT MOVEMENTS IN GAZEBO WITH THE KEYBOARD
# SNIPPETS FROM: https://github.com/ros-teleop/teleop_twist_keyboard

import rospy
import curses
# The differential controller requires Twist messages
from geometry_msgs.msg import Twist


# GLOBAL: Starting message on the console
start_msg = """
Reading from keyboard and publishing to Twist
--------------------------------------------
* Commands: WASD
* To quit press Q or CTLR+C
"""

# GLOBAL KEYS' DICTIONARY: Associates a keyboard key to a vector (x,y,z,theta)
move_bindings = {
    'w': (1, 0, 0, 0),
    's': (-1, 0, 0, 0),
    'a': (0, 0, 0, -1),
    'd': (0, 0, 0, 1)
}


# FUNCTION start_curses(): Start a curses terminal app
def start_curses():
    app = curses.initscr()  # Create a terminal window
    curses.noecho()         # Makes input invisible
    app.addstr(start_msg)   # Print the start message
    return app              # Return the terminal window


# FUNCTION move(): MAIN FUNCTION, it creates a Twist msgs and publishes it
def move():
    # Defining the topic to publish on
    pub = rospy.Publisher(
        '/dr_diff_drive_controller/cmd_vel', Twist, queue_size=10)
    # Defining the name of the node represented by this script
    rospy.init_node('diff_robot_move', anonymous=True)
    # Defining the rate of publications
    rate = rospy.Rate(30)  # 30hz
    # Instatiating the object twist_msg that contains the Twist message
    twist_msg = Twist()

    # Starting and configuring Curses application
    app = start_curses()

    # MAIN WHILE LOOP
    while not rospy.is_shutdown():
        # Reading the pressed key from the curses app
        key = app.getkey()
        # Checking if the pressed key is in the dictionary
        if key in move_bindings.keys():
            # Associating all the element of the (x,y,z,theta) vector to ..
            # .. corresponding variables
            x = move_bindings[key][0]
            y = move_bindings[key][1]
            z = move_bindings[key][2]
            th = move_bindings[key][3]
        else:
            # Incorrect key => Robot doesn't move
            x = 0
            y = 0
            z = 0
            th = 0
            # q => exit from loop
            if (key == 'q'):
                curses.endwin()  # End Curses application
                break

        # Writing (x,y,z,theta) in the Twist message
        twist_msg.linear.x = x
        twist_msg.linear.y = y
        twist_msg.linear.z = z
        twist_msg.angular.x = 0.0
        twist_msg.angular.y = 0.0
        twist_msg.angular.z = th
        # Publishing the message
        pub.publish(twist_msg)
        # Pause the loop for a time based on the defined rate
        rate.sleep()
    # LOOP END


# SIMPLE SCRIPT MAIN WITH ERROR EXCEPTION
if __name__ == '__main__':
    try:
        move()
    except rospy.ROSInterruptException:
        pass
    except curses.error:
        curses.endwin()
