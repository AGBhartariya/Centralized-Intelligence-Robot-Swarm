### Task Assignment

Tasks will be of the form:

[p, [x1, x2], [x3, x4]]

Where:

- **p**: Integer priority (lower the priority value -> higher the priority). This will be managed by a **min-heap** implementation.
  
- **x1** and **x2**: Task types and task objects.
  
- **x3** and **x4**: Locations associated with the task.

---

### Task Types:

- **x1 = 1**: **Bring object of type x2 to location x3**.
  
- **x1 = 2**: **Interact/Inspect object of type x2 at location x3**.
  
- **x1 = 3**: **Move object of type x2 from location x3 to x4** (special case).

- **x1 = 4**: **Find object of type x2** (special case of type 2).