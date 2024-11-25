#!/usr/bin/env python3
import rospy
from robot_manager.srv import GetFreeRobots, GetTaskCost
import numpy as np

class RobotManager:
    def __init__(self):
        rospy.init_node('robot_manager')
        
        self.free_robots = ["robot_1", "robot_2", "robot_3"]
        self.tasks = {}
        
        rospy.Service('get_free_robots', GetFreeRobots, self.handle_get_free_robots)
        rospy.Service('get_task_cost', GetTaskCost, self.handle_get_task_cost)

    def handle_get_free_robots(self, req):
        return {'free_robot_count': len(self.free_robots), 'robot_ids': self.free_robots}

    def computeCost(location, robot_id, task_desc):
        # TODO : implement cost function for all scenarios
        return None
    
    def handle_get_task_cost(self, req):
        task_desc = req.task_type #category of task i.e x1
        robot_id = req.robot_id
        objectlocations = req.objectlocations #poses of all objects of type x2
        tasklocations = req.tasklocations #poses of where the task to be done i.e x3 and x4 (if given)

        cost = np.inf
        obj_loc = None
        for location in objectlocations:
            temp = computeCost(location, tasklocations, robot_id, task_desc)
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
