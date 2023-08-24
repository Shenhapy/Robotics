import rospy
from gazebo_msgs.srv import GetModelState

def get_cube_position():
    try:
        # Assuming 'cube' is the name of the cube you want to get the position of
        get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        response = get_model_state('cube', 'world')
        if response.success:
            return response.pose.position.z
        else:
            rospy.logwarn("Failed to get cube position")
            return 0.0  # Return a default value or handle the error case
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)
        return 0.0  # Return a default value or handle the error case

