#!/usr/bin/env python

# PYTHON CODE TO MOVE THE ROBOT FORWARD

import rospy
# The differential controller requires Twist messages
from geometry_msgs.msg import Twist

# Starting message on the console
start_msg = "The robot will move forward"


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

    # MAIN WHILE LOOP
    while not rospy.is_shutdown():
        # Writing the Twist message
        twist_msg.linear.x = 1.0
        twist_msg.linear.y = 0.0
        twist_msg.linear.z = 0.0
        twist_msg.angular.x = 0.0
        twist_msg.angular.y = 0.0
        twist_msg.angular.z = 0.0
        # Publishing the message
        pub.publish(twist_msg)
        # Pause the loop for a time based on the defined rate
        rate.sleep()
    # LOOP END


# SIMPLE SCRIPT MAIN WITH ERROR EXCEPTION
if __name__ == '__main__':
    try:
        print(start_msg)
        move()
    except rospy.ROSInterruptException:
        pass
