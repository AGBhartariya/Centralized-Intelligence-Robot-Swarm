from pymongo.mongo_client import MongoClient
from pymongo.server_api import ServerApi
from sklearn.cluster import KMeans
import rospy
from my_package.srv import ClusterObjects, ClusterObjectsResponse  # Replace with your package and service name

uri = "mongodb+srv://all:simpledb@environment.wfwxr.mongodb.net/?retryWrites=true&w=majority&appName=Environment"

# Create a new client and connect to the server
client = MongoClient(uri, server_api=ServerApi('1'))


# Send a ping to confirm a successful connection
try:
    client.admin.command('ping')
    print("Pinged your deployment. You successfully connected to MongoDB!")
except Exception as e:
    print(e)
    
    
db=client['world']
collection =db['object']


data = {'csv or json format':1}

#to insert data
insert_doc=collection.insert_one(data)

#doc id to print for confirmation 
print(f"inserted Document ID : {insert_doc.inserted_id}")

def fetch_object_locations():
    """Fetch object locations from MongoDB."""
    try:
        # Assuming each document has a `location` field containing [x, y] coordinates
        locations = [doc['location'] for doc in collection.find({}, {'_id': 0, 'location': 1})]
        return locations
    except Exception as e:
        print(f"Error fetching data: {e}")
        return []
    
def kmeans_clustering(locations, k):
    """Perform k-means clustering."""
    if not locations:
        return None
    
    kmeans = KMeans(n_clusters=k, random_state=42)
    kmeans.fit(locations)
    return kmeans.labels_, kmeans.cluster_centers_

def handle_clustering_request(req):
    """Service callback to handle clustering requests."""
    k = req.k  # Number of clusters from the user
    locations = fetch_object_locations()
    
    if not locations:
        return ClusterObjectsResponse(success=False, message="No data found in the database.", clusters=[])
    
    try:
        labels, centers = kmeans_clustering(locations, k)
        return ClusterObjectsResponse(success=True, 
                                      message="Clustering successful.",
                                      clusters=labels.tolist(),
                                      centers=[center.tolist() for center in centers])
    except Exception as e:
        return ClusterObjectsResponse(success=False, message=str(e), clusters=[])

def clustering_service_server():
    """Initialize the clustering ROS service."""
    rospy.init_node('clustering_service_server')
    service = rospy.Service('cluster_objects', ClusterObjects, handle_clustering_request)
    print("Clustering service is ready.")
    rospy.spin()  



client.close



#NEWLY ADDED CODE FOR TESTING


def backup_database():
    """Create a backup of the current database."""
    backup_data = list(collection.find({}))
    return backup_data



def restore_database(backup_data):
    """Restore the database from a backup."""
    collection.delete_many({})  # Clear the current database
    if backup_data:
        collection.insert_many(backup_data)
    print("Database restored to its original state.")
def update_for_new_environment(environment_id):
    """Update the database for a new environment."""
    try:
        # Simulate fetching new data for the environment (from a file, API, etc.)
        new_environment_data = fetch_environment_data(environment_id)
        collection.delete_many({})  # Clear current data
        collection.insert_many(new_environment_data)  # Insert new data
        print(f"Database updated for environment: {environment_id}")
    except Exception as e:
        print(f"Error updating database: {e}")

def fetch_environment_data(environment_id):
    """Simulate fetching environment data (replace with actual logic)."""
    # Example: Retrieve data for the new environment from a predefined source
    environments = {
        "env1": [{"_id": 1, "location": [0, 0], "type": "box"}],
        "env2": [{"_id": 2, "location": [1, 1], "type": "sphere"}]
    }
    return environments.get(environment_id, [])
def clustering_service_server():
    """Initialize the clustering ROS service."""
    rospy.init_node('clustering_service_server')

    # Start monitoring the environment in a separate thread
    from threading import Thread
    Thread(target=monitor_environment, daemon=True).start()

    service = rospy.Service('cluster_objects', ClusterObjects, handle_clustering_request)
    print("Clustering service is ready.")
    rospy.spin()