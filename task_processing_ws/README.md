### Task Assignment Format:

Tasks will follow this structure:

[p, [x1, x2], [x3, x4]]

Where:

- **p**: Integer priority (lower priority value -> higher the priority). Managed by a **min-heap** implementation.

- **x1**: Task type (integer), as defined in the Task Types section below.

- **x2**: Task object or target, such as an object type, category, or descriptor.

- **x3** and **x4**: Cartesian coordinate locations. Each coordinate includes x, y, and z components.

    - Use **0** (the numeric zero) for any location that is irrelevant.

---

### Task Types:

1. **Bring Object:**  
   **x1 = 1**  
   **Description:** Bring an object of type **x2** to a specific location **x3**.

2. **Inspect/Interact:**  
   **x1 = 2**  
   **Description:** Interact with or inspect an object of type **x2** at location **x3**.

3. **Move Object:**  
   **x1 = 3**  
   **Description:** Move an object of type **x2** from one location **x3** to another **x4**.

4. **Find Object:**  
   **x1 = 4**  
   **Description:** Locate an object of type **x2**.  
   **Special Note:** This is a specific case of task type 2, but without a fixed location.

5. **Go to Location:**
   **x1 = 5**
   **Description:** Move to a specific location **x3**.
   **Special Note:** This task type is used for navigation to a specified place and in invariant of object type