# This message is used to reload the world model from the database.
# This message appears when some process saves a model to the database and wants the other processes to load it.
# This requires a connection to the same database.

# The table of the worlds primary key which can be used to identify the saved world model
uint32 primary_key
semantic_world_msgs/MetaData meta_data