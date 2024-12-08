#!/usr/bin/env python3
import rospy
import google.generativeai as genai
from robot_manager.msg import Task
from geometry_msgs.msg import PoseStamped
import ast

class TaskPublisher:
    def __init__(self):
        # ROS Setup
        self.pub = rospy.Publisher("task_topic", Task, queue_size=10)
        rospy.init_node("task_publisher", anonymous=True)
        
        # Gemini AI Setup
        genai.configure(api_key="AIzaSyBNty03zfPQ2hF1cfYe8-dEvk7fop3K37I")
        self.model = genai.GenerativeModel("gemini-1.5-flash")
        
        # Object type mapping
        self.object_types = {
            0: 'person', 1: 'bicycle', 2: 'car', 3: 'motorcycle', 4: 'airplane', 
            5: 'bus', 6: 'train', 7: 'truck', 8: 'boat', 9: 'traffic light', 
            10: 'fire hydrant', 11: 'stop sign', 12: 'parking meter', 13: 'bench', 
            14: 'bird', 15: 'cat', 16: 'dog', 17: 'horse', 18: 'sheep', 19: 'cow', 
            20: 'elephant', 21: 'bear', 22: 'zebra', 23: 'giraffe', 24: 'backpack', 
            25: 'umbrella', 26: 'handbag', 27: 'tie', 28: 'suitcase', 29: 'frisbee', 
            30: 'skis', 31: 'snowboard', 32: 'sports ball', 33: 'kite', 
            34: 'baseball bat', 35: 'baseball glove', 36: 'skateboard', 
            37: 'surfboard', 38: 'tennis racket', 39: 'bottle', 40: 'wine glass', 
            41: 'cup', 42: 'fork', 43: 'knife', 44: 'spoon', 45: 'bowl', 
            46: 'banana', 47: 'apple', 48: 'sandwich', 49: 'orange', 50: 'broccoli', 
            51: 'carrot', 52: 'hot dog', 53: 'pizza', 54: 'donut', 55: 'cake', 
            56: 'chair', 57: 'couch', 58: 'potted plant', 59: 'bed', 
            60: 'dining table', 61: 'toilet', 62: 'tv', 63: 'laptop', 64: 'mouse', 
            65: 'remote', 66: 'keyboard', 67: 'cell phone', 68: 'microwave', 
            69: 'oven', 70: 'toaster', 71: 'sink', 72: 'refrigerator', 73: 'book', 
            74: 'clock', 75: 'vase', 76: 'scissors', 77: 'teddy bear', 
            78: 'hair drier', 79: 'toothbrush'
        }

    def get_system_prompt(self, user_prompt):
        """Generate the system prompt for task conversion."""
        return f"""
This is my user prompt {user_prompt}. Convert this prompt into a Task message according to the following rules:

---
### Task Assignment Format:

Tasks will follow this structure:

[p, [x1, x2], [x3, x4]]

Where:

- **p**: Integer priority (lower priority value -> higher the priority). Managed by a **min-heap** implementation.

- **x1**: Task type (integer), as defined in the Task Types section below.

- **x2**: Task object or target id, such as an object type, category, or descriptor.

- **x3** and **x4**: Cartesian coordinate locations. Each coordinate includes x, y, and z components.

    - Use **0** value for any location that is irrelevant.

---

### Task Types:

1. **Bring Object:**  
   **x1 = 1**  
   **Description:** Bring an object of type **x2** to a specific location **x4** (specific location of the object is'nt given).

2. **Inspect/Interact:**  
   **x1 = 2**  
   **Description:** Interact with or inspect an object of type **x2** at location **x3**.

3. **Move Object:**  
   **x1 = 3**  
   **Description:** Move an object of type **x2** from one location **x3** to another **x4**.

4. **Find Object:**  
   **x1 = 4**  
   **Description:** Locate an object of type **x2**.  
   **Special Note:** This is a specific case of task type 2, but without a fixed location to bring the object (just locating).

5. **Go to Location:**
   **x1 = 5**
   **Description:** Move to a specific location **x4**.
   **Special Note:** This task type is used for navigation to a specified place and in invariant of object type
---

### Additional Notes:

- If **priority** is not mentioned in the input prompt, assume the default value of **1**.
- For Cartesian coordinates, include **all three dimensions (x, y, z)** explicitly, even if one or more dimensions are **0**.
- If **x3** or **x4** is irrelevant, explicitly set it to (0, 0, 0).
- Ensure that **x2** is derived directly from the user prompt (e.g., "package," "fire extinguisher") and then put in its encoded form/label.
  The objects of interest with their label are listed above.

---

ONLY OUTPUT THE TASK MESSAGE AND NOTHING ELSE.
"""

    def convert_nlp_to_task(self, user_prompt):
        """Convert natural language prompt to a task using Gemini AI."""
        system_prompt = self.get_system_prompt(user_prompt)
        try:
            response = self.model.generate_content(system_prompt)
            # Safely evaluate the response as a list
            task_list = ast.literal_eval(response.text)
            return task_list
        except Exception as e:
            rospy.logerr(f"Error converting NLP to task: {e}")
            return None

    def get_pose(self, prompt):
        """Get pose from user input."""
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

    def create_task_from_list(self, task_list):
        """Create a ROS Task message from the parsed task list."""
        if not task_list or len(task_list) < 3:
            rospy.logerr("Invalid task list format")
            return None

        task = Task()
        
        # Priority (first element)
        task.priority = task_list[0]
        
        # Task description (task type and object)
        task.description = task_list[1]
        
        # Locations
        x3 = PoseStamped()
        x4 = PoseStamped()
        
        # Parse locations if provided
        if len(task_list[2]) > 0:
            x3.pose.position.x = task_list[2][0][0]
            x3.pose.position.y = task_list[2][0][1]
            x3.pose.position.z = task_list[2][0][2]
        
        if len(task_list[2]) > 1:
            x4.pose.position.x = task_list[2][1][0]
            x4.pose.position.y = task_list[2][1][1]
            x4.pose.position.z = task_list[2][1][2]
        
        # Set orientation to unit quaternion
        x3.pose.orientation.w = 1
        x4.pose.orientation.w = 1
        
        task.locations = [x3, x4]
        
        return task

    def create_task_manually(self):
        """Manually create a task through user input."""
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
        """Main task publishing loop."""
        rate = rospy.Rate(1)  # Publish tasks every second
        while not rospy.is_shutdown():
            print("--- Task Creation Menu ---")
            print("1: Create task via NLP")
            print("2: Create task manually")
            print("3: Exit")

            choice = input("Enter your choice: ")
            
            if choice == "1":
                # NLP-based task creation
                user_prompt = input("Enter your task description in natural language: ")
                task_list = self.convert_nlp_to_task(user_prompt)
                
                if task_list:
                    # Convert parsed task list to ROS Task message
                    task = self.create_task_from_list(task_list)
                    
                    if task:
                        self.pub.publish(task)
                        print(f"Published task: {task}")
                    else:
                        rospy.logerr("Failed to create task from NLP input")
                else:
                    rospy.logerr("Failed to parse NLP input")

            elif choice == "2":
                # Manual task creation
                new_task = self.create_task_manually()
                self.pub.publish(new_task)

            elif choice == "3":
                rospy.loginfo("Exiting task publisher.")
                break

            else:
                rospy.logwarn("Invalid choice. Please try again.")

            rate.sleep()

if __name__ == "__main__":
    try:
        rospy.loginfo("Started Task assignment node")
        task_pub = TaskPublisher()
        task_pub.task_publisher()
    except rospy.ROSInterruptException:
        pass