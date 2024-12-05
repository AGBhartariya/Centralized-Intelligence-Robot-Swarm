#!/usr/bin/env python3
import rospy
from robot_manager.msg import Task
from geometry_msgs.msg import PoseStamped
import heapq

class TaskPublisher:
    def __init__(self):
        self.pub = rospy.Publisher("tasks", Task, queue_size=10)
        rospy.init_node("task_publisher", anonymous=True)
        self.task_heap = []  # Min-heap to manage tasks by priority

    def get_pose(self, prompt):
        pose = PoseStamped()
        try:
            print(f"Enter {prompt} coordinates (x, y, z) or '0 0 0' if irrelevant:")
            coords = list(map(float, input().split()))
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = coords
            # Assign unit quaternion for orientation
            pose.pose.orientation.x = 0
            pose.pose.orientation.y = 0
            pose.pose.orientation.z = 0
            pose.pose.orientation.w = 1
        except ValueError:
            rospy.logerr("Invalid input for coordinates. Defaulting to (0, 0, 0).")
        return pose

    def create_task(self):
        task = Task()

        try:
            task.priority = int(input("Enter task priority (integer, lower value = higher priority): "))
            print("Select task type:")
            print("1: Bring Object\n2: Inspect/Interact\n3: Move Object\n4: Find Object\n5: Go to Location")
            x1 = int(input("Enter task type (1-5): "))

            if x1 not in [1, 2, 3, 4, 5]:
                rospy.logerr("Invalid task type. Defaulting to 1.")
                x1 = 1

            task_type = x1
            x2 = int(input("Enter task object/target (integer id): "))

            task.description = [x1, x2]
            x3 = PoseStamped()
            x4 = PoseStamped()

            if task_type in [2, 3]:
                x3 = self.get_pose("initial location")

            if task_type in [1, 3, 5]:
                x4 = self.get_pose("destination location")
            
            task.locations = [x3, x4]

            rospy.loginfo(f"Created task: {task}")

        except ValueError:
            rospy.logerr("Invalid input. Task creation failed.")

        return task

    def task_publisher(self):
        rate = rospy.Rate(1)  # Publish tasks every second
        while not rospy.is_shutdown():
            print("--- Task Creation Menu ---")
            print("1: Create a new task")
            print("2: Publish next task from the heap")
            print("3: Exit")

            choice = input("Enter your choice: ")
            if choice == "1":
                new_task = self.create_task()
                heapq.heappush(self.task_heap, (new_task.priority, new_task))

            elif choice == "2":
                if self.task_heap:
                    _, task_to_publish = heapq.heappop(self.task_heap)
                    rospy.loginfo(f"Publishing task: {task_to_publish}")
                    self.pub.publish(task_to_publish)
                else:
                    rospy.loginfo("No tasks in the heap to publish.")

            elif choice == "3":
                rospy.loginfo("Exiting task publisher.")
                break

            else:
                rospy.logwarn("Invalid choice. Please try again.")

            rate.sleep()

if __name__ == "__main__":
    try:
        task_pub = TaskPublisher()
        task_pub.task_publisher()
    except rospy.ROSInterruptException:
        pass
