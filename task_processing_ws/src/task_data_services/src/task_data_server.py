#!/usr/bin/env python

import rospy
import yaml
from task_data_services.srv import QueryTaskData, QueryTaskDataResponse

class TaskDataServer:
    def __init__(self):
        # Load task data from YAML file
        self.task_data = {}
        data_file = rospy.get_param("~data_file", "data/task_data.yaml")
        try:
            with open(data_file, 'r') as file:
                self.task_data = yaml.safe_load(file)['task_data']
                rospy.loginfo("Task data loaded successfully.")
        except Exception as e:
            rospy.logerr(f"Failed to load task data: {e}")
            rospy.signal_shutdown("Could not load task data.")
        
        # Initialize the service
        self.service = rospy.Service('QueryTaskData', QueryTaskData, self.handle_query)

    def handle_query(self, req):
        rospy.loginfo(f"Received query for task: {req.task}")
        metadata = self.task_data.get(req.task, "No data available")
        return QueryTaskDataResponse(metadata=metadata)

if __name__ == "__main__":
    rospy.init_node('task_data_server')
    TaskDataServer()
    rospy.spin()
