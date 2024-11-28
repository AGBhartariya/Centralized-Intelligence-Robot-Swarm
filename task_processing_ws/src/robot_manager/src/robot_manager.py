#!/usr/bin/env python3
import rospy
from robot_manager.srv import GetFreeRobots, GetTaskCost
from robot_manager.srv import GetState  # Add GetState service for each robot
import numpy as np

class RobotManager:
    def __init__(self):
        rospy.init_node('robot_manager')
        
        # Initialize robots by namespace (e.g., "robot_1", "robot_2", ...)
        self.numRobots = rospy.get_param('~num_robots', 3)  # Default to 3 robots
        self.robots = [f"ugv{i+1}" for i in range(self.numRobots)]
        
        self.free_robots = []
        self.tasks = {}

        # Initialize services
        rospy.Service('get_free_robots', GetFreeRobots, self.handle_get_free_robots)
        rospy.Service('get_task_cost', GetTaskCost, self.handle_get_task_cost)

        # Check the state of each robot at initialization
        self.update_free_robots()

    def update_free_robots(self):
        """ Check each robot's state using the getState service. """
        self.free_robots = []
        for robot in self.robots:
            try:
                # Call the getState service for each robot to check its state
                rospy.wait_for_service(f'/{robot}/get_state', timeout=5.0)
                get_state = rospy.ServiceProxy(f'/{robot}/get_state', GetState)
                response = get_state()
                if response.state != 4 and response.state != 3:  # Charging and Tasking => Not Free
                    self.free_robots.append(robot)
            except rospy.ServiceException as e:
                rospy.logwarn(f"Failed to call get_state for {robot}: {e}")
            except rospy.ROSException as e:
                rospy.logwarn(f"Service call timeout for {robot}: {e}")
        
        rospy.loginfo(f"Free robots: {self.free_robots}")
    
    def handle_get_free_robots(self, req):
        # This service will return the list of free robots and their count
        self.update_free_robots()  # Update free robots before returning response
        return {'free_robot_count': len(self.free_robots), 'robot_ids': self.free_robots}

    def computeCost(self, location, robot_id, task_desc):
        # TODO : implement cost function for all scenarios
        return None
    
    def handle_get_task_cost(self, req):
        task_desc = req.task_type  # Category of task i.e. x1
        robot_id = req.robot_id
        objectlocations = req.objectlocations  # Poses of all objects of type x2
        tasklocations = req.tasklocations  # Poses of where the task needs to be done (x3, x4)

        cost = np.inf
        obj_loc = None
        for location in objectlocations:
            temp = self.computeCost(location, tasklocations, robot_id, task_desc)
            if temp < cost:
                cost = temp
                obj_loc = location
        
        return {'cost': cost, 'object_location': obj_loc}  # Example cost function

    
if __name__ == '__main__':
    try:
        RobotManager()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
