#!/usr/bin/env python3
import rospy
from robot_manager.srv import GetFreeRobots, GetTaskCost

class RobotManager:
    def __init__(self):
        rospy.init_node('robot_manager')
        
        self.free_robots = ["robot_1", "robot_2", "robot_3"]
        self.tasks = {}
        
        rospy.Service('get_free_robots', GetFreeRobots, self.handle_get_free_robots)
        rospy.Service('get_task_cost', GetTaskCost, self.handle_get_task_cost)

    def handle_get_free_robots(self, req):
        return {'free_robot_count': len(self.free_robots), 'robot_ids': self.free_robots}

    def handle_get_task_cost(self, req):
        task_desc = req.task
        robot_id = req.robot_id
        return {'cost': len(task_desc) + len(robot_id)}  # Example cost function

if __name__ == '__main__':
    try:
        RobotManager()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
