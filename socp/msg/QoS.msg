# communication requirement for a mobile autonomous network

float64 margin     # minimum rate margin available at source and destination nodes
float64 confidence # probability with which QoS should be guaranteed
uint64 src         # source node (there can only be one)
uint64[] dest      # destination node (there may be more than one)