Pose Database Node Configuration Guide
The Pose Database Node is a ROS node designed for managing object pose data within a MongoDB database. It ensures:

Real-time updates of object poses detected by multiple robots.
Efficient querying and clustering of object poses.
Integration with MongoDB Atlas for scalable and secure database management.
This guide explains how to configure, use, and extend the Pose Database Node for various robotics applications.

Key Features
Database Integration
Connects to a MongoDB Atlas cloud database.
Supports CRUD (Create, Read, Update, Delete) operations for object pose data.

ROS Services
update_data: Updates or adds object poses.
query_loc: Queries available object locations.

Multi-Robot Support
Dynamically subscribes to detection topics from multiple robots.
Clustering and Sampling
Utilizes KMeans clustering to group object poses.
Samples points from clusters for downstream tasks.
Spatial Queries
Leverages MongoDB’s geospatial indexing for efficient proximity-based queries.
Configuration and Initialization
MongoDB Atlas Connection
Mongo URI: Default:
mongodb+srv://all:simpledb@environment.wfwxr.mongodb.net/?retryWrites=true&w=majority&appName=Environment
Database: world
Collection: object
Set these parameters using ROS launch files or parameter server.

ROS Parameters
Parameter
Default Value

tolerance
0.1
epsilon
0.5

Threshold for confidence updates.

ROS Topic Subscriptions
The node subscribes to topics from robots, named:
/ugv<N>/detected_object
Each topic publishes DetectObject messages containing object ID, pose, and confidence.
Usage

Start the Node
Launch MongoDB Atlas:
Ensure your MongoDB Atlas cluster is accessible.

Run the ROS Node:
rosrun object_pose_database pose_database_node.py
Update Object Data
Call the update_data service to insert or update object poses. Example:
rosservice call /update_data '{task_object: "obj1", pose: {x: 1.0, y: 2.0, z: 0.5}, status: "free"}'
Query Free Objects
Retrieve free object locations using the query_loc service:
rosservice call /query_loc '{objectType: "obj1"}'
Clustering and Sampling
Request clustering and sampling of object poses for processing tasks.
Adding New Features
Adding a New Robot

Topic Subscription:
Add the new robot’s detection topic in the configuration.

Pose Integration:
Ensure the robot’s pose data aligns with the node’s spatial matching logic.

Custom Clustering Algorithms
Modify the clustering logic in the node.
Replace KMeans with the desired clustering algorithm (e.g., DBSCAN).

Troubleshooting
Common Issues
MongoDB Connection Errors
Verify the mongo_uri parameter.
Ensure proper network access and IP whitelisting.
Data Not Updating
Check for spatial index creation in the MongoDB collection.
Ensure incoming poses are within the specified tolerance range.
Clustering Errors
Verify sufficient data points exist for clustering.
Adjust clustering parameters for better results.

Future Enhancements
Advanced Clustering:
Support additional algorithms like DBSCAN or hierarchical clustering.
Real-Time Visualization:
Implement tools to visualize object poses and clusters in real-time.
Enhanced Logging:
Improve log details for better debugging and analytics.
