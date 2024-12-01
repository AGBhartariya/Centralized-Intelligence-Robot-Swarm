from pymongo.mongo_client import MongoClient
from pymongo.server_api import ServerApi

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
client.close