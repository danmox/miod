# command message disseminated to all nodes (task and network) in the network
# consisting of routing table updates and corresponding network node positions

routing_msgs/PRTableEntry[] routes

# mapping between agent id and pose command:
# if agent_ids[0] == 3 then agent3 should go to poses[0]
uint8[] agent_ids
geometry_msgs/PoseStamped[] poses