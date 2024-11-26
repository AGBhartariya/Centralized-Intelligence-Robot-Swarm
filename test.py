import rospy

def delete_ugv1_params():
    # Get a list of all parameters in the ROS parameter server
    params = rospy.get_param_names()

    # Iterate through the list of parameters
    for param in params:
        if 'ugv1' in param:
            rospy.delete_param(param)
            rospy.loginfo(f"Deleted parameter: {param}")

if __name__ == "__main__":
    rospy.init_node('delete_ugv1_params_node', anonymous=True)
    delete_ugv1_params()
