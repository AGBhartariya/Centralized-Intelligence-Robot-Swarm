#!/usr/bin/env python3
import sys
import rospy
import google.generativeai as genai
import ast
from PyQt5.QtWidgets import (QApplication, QMainWindow, QVBoxLayout, QHBoxLayout, 
                             QWidget, QTabWidget, QLabel, QLineEdit, QComboBox, 
                             QTextEdit, QPushButton, QMessageBox, QGridLayout, 
                             QRadioButton, QButtonGroup)
from PyQt5.QtCore import Qt, QThread, pyqtSignal
from robot_manager.msg import Task
from geometry_msgs.msg import PoseStamped
import threading

class GeminiTaskConverter(QThread):
    """Background thread for converting NLP to task using Gemini AI"""
    task_converted = pyqtSignal(list)
    error_occurred = pyqtSignal(str)

    def __init__(self, user_prompt):
        super().__init__()
        self.user_prompt = user_prompt
        # Configure Gemini AI
        genai.configure(api_key="AIzaSyBNty03zfPQ2hF1cfYe8-dEvk7fop3K37I")
        self.model = genai.GenerativeModel("gemini-1.5-flash")

    def run(self):
        try:
            # System prompt for task conversion (same as previous implementation)
            system_prompt = f"""
This is my user prompt {self.user_prompt}. Convert this prompt into a Task message according to the following rules:

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
            response = self.model.generate_content(system_prompt)
            # Safely evaluate the response as a list
            task_list = ast.literal_eval(response.text)
            self.task_converted.emit(task_list)
        except Exception as e:
            self.error_occurred.emit(str(e))

class TaskPublisherGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        
        # ROS Setup
        rospy.init_node("task_publisher_gui", anonymous=True)
        self.pub = rospy.Publisher("task_topic", Task, queue_size=10)
        
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
        
        # Setup UI
        self.setWindowTitle("ROS Task Publisher")
        self.setGeometry(100, 100, 600, 500)
        
        # Main widget and layout
        main_widget = QWidget()
        main_layout = QVBoxLayout()
        
        # Tabs for different task creation methods
        self.tab_widget = QTabWidget()
        
        # NLP Task Creation Tab
        nlp_tab = QWidget()
        nlp_layout = QVBoxLayout()
        
        # NLP Input
        nlp_input_layout = QHBoxLayout()
        self.nlp_input = QLineEdit()
        self.nlp_input.setPlaceholderText("Enter task description in natural language")
        nlp_input_layout.addWidget(self.nlp_input)
        
        nlp_convert_btn = QPushButton("Convert & Publish")
        nlp_convert_btn.clicked.connect(self.convert_nlp_task)
        nlp_input_layout.addWidget(nlp_convert_btn)
        
        nlp_layout.addLayout(nlp_input_layout)
        
        # NLP Result Display
        self.nlp_result = QTextEdit()
        self.nlp_result.setReadOnly(True)
        nlp_layout.addWidget(self.nlp_result)
        
        nlp_tab.setLayout(nlp_layout)
        
        # Manual Task Creation Tab
        manual_tab = QWidget()
        manual_layout = QGridLayout()
        
        # Priority
        manual_layout.addWidget(QLabel("Priority:"), 0, 0)
        self.priority_input = QLineEdit()
        self.priority_input.setText("1")
        manual_layout.addWidget(self.priority_input, 0, 1)
        
        # Task Type
        manual_layout.addWidget(QLabel("Task Type:"), 1, 0)
        self.task_type_combo = QComboBox()
        task_types = [
            "Bring Object", 
            "Inspect/Interact", 
            "Move Object", 
            "Find Object", 
            "Go to Location"
        ]
        self.task_type_combo.addItems(task_types)
        manual_layout.addWidget(self.task_type_combo, 1, 1)
        
        # Object Selection
        manual_layout.addWidget(QLabel("Object:"), 2, 0)
        self.object_combo = QComboBox()
        self.object_combo.addItems([f"{k}: {v}" for k, v in self.object_types.items()])
        manual_layout.addWidget(self.object_combo, 2, 1)
        
        # Initial Location
        manual_layout.addWidget(QLabel("Initial Location (x, y, z):"), 3, 0)
        self.initial_loc_input = QLineEdit()
        self.initial_loc_input.setPlaceholderText("e.g., 0 0 0")
        manual_layout.addWidget(self.initial_loc_input, 3, 1)
        
        # Destination Location
        manual_layout.addWidget(QLabel("Destination Location (x, y, z):"), 4, 0)
        self.dest_loc_input = QLineEdit()
        self.dest_loc_input.setPlaceholderText("e.g., 10 20 0")
        manual_layout.addWidget(self.dest_loc_input, 4, 1)
        
        # Publish Manual Task Button
        manual_publish_btn = QPushButton("Publish Task")
        manual_publish_btn.clicked.connect(self.publish_manual_task)
        manual_layout.addWidget(manual_publish_btn, 5, 0, 1, 2)
        
        manual_tab.setLayout(manual_layout)
        
        # Add tabs
        self.tab_widget.addTab(nlp_tab, "NLP Task")
        self.tab_widget.addTab(manual_tab, "Manual Task")
        
        # Add tab widget to main layout
        main_layout.addWidget(self.tab_widget)
        
        # Task Log
        self.task_log = QTextEdit()
        self.task_log.setReadOnly(True)
        main_layout.addWidget(QLabel("Published Tasks Log:"))
        main_layout.addWidget(self.task_log)
        
        # Set main layout
        main_widget.setLayout(main_layout)
        self.setCentralWidget(main_widget)

    def convert_nlp_task(self):
        """Convert NLP to task using Gemini AI"""
        user_prompt = self.nlp_input.text().strip()
        if not user_prompt:
            QMessageBox.warning(self, "Input Error", "Please enter a task description.")
            return
        
        # Start background thread for task conversion
        self.nlp_thread = GeminiTaskConverter(user_prompt)
        self.nlp_thread.task_converted.connect(self.handle_nlp_task_conversion)
        self.nlp_thread.error_occurred.connect(self.handle_nlp_conversion_error)
        
        # Show loading message
        self.nlp_result.setText("Converting task... Please wait.")
        self.nlp_thread.start()

    def handle_nlp_task_conversion(self, task_list):
        """Handle successful NLP to task conversion"""
        try:
            # Create task from list
            task = self.create_task_from_list(task_list)
            
            if task:
                # Publish task
                self.pub.publish(task)
                
                # Update result display and log
                result_text = f"Converted Task:\nPriority: {task.priority}\n"
                result_text += f"Type: {task.description[0]}, Object: {task.description[1]}\n"
                result_text += f"Initial Loc: {task.locations[0].pose.position}\n"
                result_text += f"Destination Loc: {task.locations[1].pose.position}"
                
                self.nlp_result.setText(result_text)
                self.task_log.append(result_text + "\n---\n")
            else:
                self.nlp_result.setText("Failed to create task from conversion.")
        except Exception as e:
            self.nlp_result.setText(f"Error processing task: {str(e)}")

    def handle_nlp_conversion_error(self, error_msg):
        """Handle errors in NLP task conversion"""
        QMessageBox.critical(self, "Conversion Error", str(error_msg))
        self.nlp_result.setText(f"Error: {error_msg}")

    def create_task_from_list(self, task_list):
        """Convert parsed task list to ROS Task message"""
        if not task_list or len(task_list) < 3:
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

    def publish_manual_task(self):
        """Publish a manually created task"""
        try:
            # Create task message
            task = Task()
            
            # Priority
            task.priority = int(self.priority_input.text())
            
            # Task Type and Object
            task_type_index = self.task_type_combo.currentIndex() + 1
            object_id = int(self.object_combo.currentText().split(':')[0])
            task.description = [task_type_index, object_id]
            
            # Locations
            x3 = PoseStamped()
            x4 = PoseStamped()
            
            # Parse initial location
            init_loc = list(map(float, self.initial_loc_input.text().split()))
            if len(init_loc) == 3:
                x3.pose.position.x, x3.pose.position.y, x3.pose.position.z = init_loc
            
            # Parse destination location
            dest_loc = list(map(float, self.dest_loc_input.text().split()))
            if len(dest_loc) == 3:
                x4.pose.position.x, x4.pose.position.y, x4.pose.position.z = dest_loc
            
            # Set orientation to unit quaternion
            x3.pose.orientation.w = 1
            x4.pose.orientation.w = 1
            
            task.locations = [x3, x4]
            
            # Publish task
            self.pub.publish(task)
            
            # Log task
            log_text = f"Manual Task:\nPriority: {task.priority}\n"
            log_text += f"Type: {task_type_index}, Object: {object_id}\n"
            log_text += f"Initial Loc: {x3.pose.position}\n"
            log_text += f"Destination Loc: {x4.pose.position}"
            
            self.task_log.append(log_text + "\n---\n")
            
            # Show success message
            QMessageBox.information(self, "Task Published", "Task successfully published!")
        
        except ValueError as e:
            QMessageBox.critical(self, "Input Error", f"Invalid input: {str(e)}")
        except Exception as e:
            QMessageBox.critical(self, "Publish Error", f"Failed to publish task: {str(e)}")

def main():
    # Create ROS node
    rospy.init_node("task_publisher_gui", anonymous=True)
    
    # Create Qt Application
    app = QApplication(sys.argv)
    
    # Create and show the main window
    task_publisher = TaskPublisherGUI()
    task_publisher.show()
    
    # Run the Qt application
    # Spin ROS in a separate thread to allow GUI responsiveness
    def ros_spin():
        rospy.spin()
    
    # Create a thread for ROS spinning
    ros_thread = threading.Thread(target=ros_spin)
    ros_thread.daemon = True  # Ensure thread exits when main program exits
    ros_thread.start()
    
    # Execute the Qt application
    sys.exit(app.exec_())

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass