#!/usr/bin/env python3
import rospy
from robot_manager.msg import Task
import random

def task_publisher():
    pub = rospy.Publisher('tasks', Task, queue_size=10)
    rospy.init_node('task_publisher', anonymous=True)
    rate = rospy.Rate(1)  # Publish tasks every second
    
    priorities = [1, 2, 3, 4, 5]
    descriptions = ["Task A", "Task B", "Task C", "Task D", "Task E"]
    
    while not rospy.is_shutdown():
        task = Task()
        task.priority = random.choice(priorities)
        task.description = random.choice(descriptions)
        rospy.loginfo(f"Publishing task: {task}")
        pub.publish(task)
        rate.sleep()

if __name__ == '__main__':
    try:
        task_publisher()
    except rospy.ROSInterruptException:
        pass
