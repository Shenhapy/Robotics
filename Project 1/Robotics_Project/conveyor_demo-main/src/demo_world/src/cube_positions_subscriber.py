#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64
from demo_world.cube_spawner import CubeSpawner  # Replace 'your_package_name' with the actual name of your package

def cube_position_callback(data):
    # You can perform any necessary processing with the received position data here if needed
    pass

def cube_spawner_subscriber():
    rospy.init_node('cube_spawner_subscriber', anonymous=True)
    rospy.Subscriber('/cube_position', Float64, cube_position_callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        cube_spawner_subscriber()
    except rospy.ROSInterruptException:
        pass

