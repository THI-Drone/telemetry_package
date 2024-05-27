# Overview

This node serves as a middle point between the ROS2 network and the telemetry-webapp of the drone. 
The functions are described in detail down below. 

## Functions

### __rosout_callback(self, log_msg)
This function sends log messages captured by the `/rosout` topic to a client socket in an encoded json format. 

### __heartbeat_callback(self, hb_msg)
This function works similarly to `__rosout_callback()` with the difference that it sends the telemetry data instead of log messages. 

### def __init__(self)
This function is the constructor of the node. In the constructor, following things are done: 
- Set the appropriate QoS settings for the various subscripttions
- Create the UNIX server socket using either the DEFAULT_UNIX_SOCKET_PATH or, if provided, a path passed via CLI 
- Create a reference to the client socket
- Create the subscription to the following topics: 
    -`/rosout` 
    - `TopicNames.Control`
    - `TopicNames.Heartbeat`

# To-Dos: 
- Fix code formating
- Implement tests for the following aspects: 
	- Test performance 
	- Test robustness of cli functionality for socket path input
	- Others (work in progress)


## Suggestions are gladly accepted. 

