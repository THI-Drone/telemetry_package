# Overview

This node serves as a middle point between the ROS2 network and the telemetry-webapp of the drone. 
The functions are described in detail down below. 

## Functions

### subscription_callback(self, log_msg)
This function sends messages to a client socket in an encoded json format. The messages that the node sends are various log messages, which come from the `/rosout` topic.

### def __init__(self)
This function is the constructor of the node. In the constructor, following things are done: 
- Create the UNIX server socket
- Create a reference to the client socket
- Create the subscription to the `/rosout` topic: this ensures that the log-messages sent to the `/rosout` topic are sent to the webapp

# To-Dos: 
- A function is needed to parse the various log messages that get sent to teh client-webapp. Not sanitizing them properly results in an `extra line` error, since apparently, the json.loads(<received_data>.decode()) causes problems if the sent messages contain notation interfering with the JSON-format
- Implement a better way to remove the socket from path after shutting down the node, since it currently needs to be manually deleted from path before program execution.

