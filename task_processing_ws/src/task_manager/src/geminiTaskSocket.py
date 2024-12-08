#!/usr/bin/env python3
import socket
import json
import sys
import ast
import google.generativeai as genai

class GeminiTaskConverterServer:
    def __init__(self, host='localhost', port=65435):
        # Configure Gemini AI
        genai.configure(api_key="AIzaSyBNty03zfPQ2hF1cfYe8-dEvk7fop3K37I")
        self.model = genai.GenerativeModel("gemini-1.5-flash")
        
        # Socket setup
        self.host = host
        self.port = port
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.bind((self.host, self.port))
        self.server_socket.listen(1)
        
        print(f"Gemini Task Converter Server listening on {self.host}:{self.port}")
    
    def generate_system_prompt(self, user_prompt):
        """Generate system prompt for task conversion."""
        system_prompt = f"""
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
- If nothing is specified about the task object like in case of task type 5, take it as 0.


### Examples:

1. **Prompt:** "Find me a fire hydrant."  
   **Task Message:** [1, [4, 10], [[0,0,0], [0,0,0]]]

2. **Prompt:** "Inspect the laptop at (3, 4)."  
   **Task Message:** [1, [2, 63], [[3, 4, 0], [0, 0, 0]]]

3. **Prompt:** "Move the chair from (3, 4, 1) to (6, 7, 2)."  
   **Task Message:** [1, [3, 56], [[3, 4, 1], [6, 7, 2]]]

4. **Prompt:** "Bring a sandwich to (1, 2, 3)."  
   **Task Message:** [1, [1, 48], [[0, 0, 0], [1, 2, 3]]]

5. **Prompt:** "Check the snowboard at location (5, 6, 0)."  
   **Task Message:** [1, [2, 31], [[5, 6, 0], [0, 0, 0]]]

6. **Prompt:** "Go to 2, 3 "
   **Task Message:** [1, [5, 0], [[0, 0, 0], [2, 3, 0]]] 

---

ONLY OUTPUT THE TASK MESSAGE AND NOTHING ELSE.
"""
        return system_prompt
    
    def convert_task(self, user_prompt):
        """Convert user prompt to task using Gemini AI."""
        try:
            system_prompt = self.generate_system_prompt(user_prompt)
            response = self.model.generate_content(system_prompt)
            
            # Safely evaluate the response as a list
            task_list = ast.literal_eval(response.text)
            return task_list
        except Exception as e:
            return {"error": str(e)}
    
    def start(self):
        """Start the server and listen for connections."""
        try:
            while True:
                # Wait for a client connection
                client_socket, address = self.server_socket.accept()
                print(f"Connection from {address}")
                
                try:
                    # Receive data from the client
                    data = client_socket.recv(1024).decode('utf-8')
                    
                    # Convert the task
                    result = self.convert_task(data)
                    
                    # Send the result back to the client
                    client_socket.send(json.dumps(result).encode('utf-8'))
                
                except Exception as e:
                    print(f"Error processing request: {e}")
                
                finally:
                    # Close the client socket
                    client_socket.close()
        
        except KeyboardInterrupt:
            print("\nServer shutting down.")
        
        finally:
            # Close the server socket
            self.server_socket.close()

def main():
    converter_server = GeminiTaskConverterServer()
    converter_server.start()

if __name__ == "__main__":
    main()