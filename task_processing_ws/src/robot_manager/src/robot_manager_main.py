#!/usr/bin/env python3
import rospy
import tf
from geometry_msgs.msg import Point
from robot_manager.srv import GetFreeRobots, GetTaskCost
from robot_manager.srv import GetState  # Add GetState service for each robot
import numpy as np
from trajectory_control_msgs.srv import GetCostPath, GetCostPathRequest

class RobotManager:
    def __init__(self):
        rospy.init_node("robot_manager")

        # Initialize robots by namespace (e.g., "robot_1", "robot_2", ...)
        self.numRobots = rospy.get_param("~num_robots", 3)  # Default to 3 robots
        self.robots = [f"ugv{i+1}" for i in range(self.numRobots)]

        self.free_robots = []
        self.tasks = {}

        # Initialize services
        rospy.Service("get_free_robots", GetFreeRobots, self.handle_get_free_robots)
        rospy.Service("get_task_cost", GetTaskCost, self.handle_get_task_cost)

        # Check the state of each robot at initialization
        self.update_free_robots()

    def update_free_robots(self):
        """Check each robot's state using the getState service."""
        self.free_robots = []
        for robot in self.robots:
            try:
                # Call the getState service for each robot to check its state
                rospy.wait_for_service(f"/{robot}/get_state", timeout=5.0)
                get_state = rospy.ServiceProxy(f"/{robot}/get_state", GetState)
                response = get_state()
                if (
                    response.state != 4 and response.state != 3
                ):  # Charging and Tasking => Not Free
                    self.free_robots.append(robot)
            except rospy.ServiceException as e:
                rospy.logwarn(f"Failed to call get_state for {robot}: {e}")
            except rospy.ROSException as e:
                rospy.logwarn(f"Service call timeout for {robot}: {e}")

        rospy.loginfo(f"Free robots: {self.free_robots}")

    def handle_get_free_robots(self, req):
        # This service will return the list of free robots and their count
        self.update_free_robots()  # Update free robots before returning response
        return {
            "free_robot_count": len(self.free_robots),
            "robot_ids": self.free_robots,
        }

    def getRobotPos(self,robot_id):
        listener = tf.TransformListener()
        try:
            # Wait for the transform to become available
            listener.waitForTransform("odom", f"{robot_id}/base_link", rospy.Time(0), rospy.Duration(5.0))
            (trans, rot) = listener.lookupTransform("odom", f"{robot_id}/base_link", rospy.Time(0))
            return Point(x=trans[0],y=trans[1],z=trans[2])
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr("Error getting transform: %s", e)
            return None
        
    
    def computeCost(self, objectlocation, task_location, robot_id, task_desc):
        #implement cost function for all scenarios
        rospy.wait_for_service('cost_path')
        try:
            cost_path = rospy.ServiceProxy('cost_path', GetCostPath)
            request = GetCostPathRequest()
            robot_pos = self.getRobotPos(robot_id)
            obj_point=objectlocation.pose.position
            task_point=[]
            for loc in task_location:
                task_point.append(loc.pose.position)   
            if task_desc==1:
                all_points=[robot_pos]+[obj_point]+[task_point[1]]
            elif task_desc==2:
                all_points=[robot_pos]+[task_point[0]]
            elif task_desc==3:
                all_points=[robot_pos]+task_point
            elif task_desc==4:
                all_points=[robot_pos]+[obj_point]
            elif task_desc==5:
                all_points=[robot_pos]+[task_point[1]]

            request.task.header.frame_id = 'odom' 
            request.task.waypoints = all_points
            request.task.segment_count=len(all_points) -1
            response = cost_path(request)
            total_cost=0
            for path in response.path_list:
                poses = path.poses
                for i in range(len(poses) - 1):
                    # Extract positions of successive waypoints
                    p1 = poses[i].pose.position
                    p2 = poses[i + 1].pose.position

                    distance = ((p2.x - p1.x)**2 + (p2.y - p1.y)**2 + (p2.z - p1.z)**2)**0.5
                    total_cost += distance
            return total_cost

        except rospy.ServiceException as e:
           print("Service call failed: %s"%e)
           return None
        

    def handle_get_task_cost(self, req):
        task_desc = req.task_type  # Category of task i.e. x1
        robot_id = req.robot_id
        objectlocations = req.objectlocations  # Poses of all objects of type x2
        tasklocations = req.tasklocations # Poses of where the task needs to be done (x3, x4)

        cost = np.inf
        obj_loc = None
        if (task_desc==1 or task_desc==4):
            for objectlocation in objectlocations:
                temp = self.computeCost(objectlocation, tasklocations, robot_id, task_desc)
                if temp < cost:
                    cost = temp
                    obj_loc = objectlocation
        else:
            cost=self.computeCost(objectlocation, tasklocations, robot_id, task_desc)
        return {"cost": cost, "object_location": obj_loc}  # Example cost function


if __name__ == "__main__":
    try:
        RobotManager()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
