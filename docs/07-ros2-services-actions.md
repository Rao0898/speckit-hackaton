---
sidebar_position: 7
---

# ROS 2 Services and Actions

While ROS 2 topics are perfect for continuous data streams, there are many situations where a request-response or long-running task paradigm is more appropriate. For these scenarios, ROS 2 provides two powerful communication patterns: **Services** and **Actions**.

## Services: Synchronous Request-Response

A **Service** is a synchronous communication pattern where one node (the client) sends a request to another node (the server) and waits for a response. This is analogous to calling a function in a traditional programming language.

- **Synchronous Nature**: The client blocks (waits) until the server has completed the request and sent back a response. This makes services ideal for quick, atomic tasks that must be completed before the client can proceed.

- **Service Definition**: Like messages, services have a defined type, specified in a `.srv` file. The file defines the structure of the request and the response, separated by `---`.

**Example `.srv` Definition (`SetBool.srv`):**
```
bool data      # The request part
---
bool success   # The response part
string message
```

### When to Use Services

- **Triggering an action**: Sending a command to a robot to switch to a specific mode (e.g., "enable safety stop").
- **Querying state**: Asking a node for a piece of information (e.g., "get current robot configuration").
- **Performing a quick calculation**: Requesting a node to perform a calculation, like an inverse kinematics solution, and return the result.

## Actions: Asynchronous, Long-Running Tasks with Feedback

An **Action** is used for long-running, asynchronous tasks that may take a significant amount of time to complete. Unlike services, actions provide feedback during their execution and are cancellable.

- **Asynchronous Nature**: When a client sends a goal to an action server, it does not block. The client can continue with other tasks while the action is executing.

- **Components of an Action**:
    1.  **Goal**: The client sends a goal to the action server (e.g., "navigate to coordinates X, Y").
    2.  **Feedback**: The server provides regular updates on its progress towards the goal (e.g., "current distance to goal is 5 meters").
    3.  **Result**: Once the task is complete, the server sends a final result (e.g., "succeeded in reaching goal").

- **Action Definition**: Actions are defined in `.action` files, which have three parts: the goal, the result, and the feedback, each separated by `---`.

**Example `.action` Definition (`Navigate.action`):**
```
# The goal
geometry_msgs/Point target_position
---
# The result
bool success
---
# The feedback
float32 distance_remaining
```

### When to Use Actions

Actions are perfect for any task that is not instantaneous and where monitoring progress is important.
- **Navigation**: Commanding a mobile robot to move to a new location.
- **Manipulation**: Instructing a robot arm to pick up and place an object.
- **Complex sequences**: Executing a multi-step assembly task.

## Topics vs. Services vs. Actions: A Summary

| Communication Type | Best For | Analogy |
|---|---|---|
| **Topics** | Continuous data streams (many-to-many) | Radio broadcast |
| **Services** | Quick, synchronous request-response (one-to-one) | Function call |
| **Actions** | Long-running, asynchronous tasks with feedback | Ordering food for delivery (you get updates) |

By mastering topics, services, and actions, you will have a complete toolbox for orchestrating the complex behaviors of a humanoid robot, from processing real-time sensor data to executing high-level, goal-oriented tasks.
