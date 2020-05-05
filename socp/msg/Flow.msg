# communication requirements associated with a flow of data in a mobile
# autonomous network

uint8 OPPORTUNISTIC = 0
uint8 CONFIDENCE = 1

# required communication rate as a percentage of the maximum rate supported by
# the communication hardware
float64 rate        

uint64 src   # source node
uint64 dest  # destination node

uint8 method # routing method to use [options: OPPORTUNISTIC, CONFIDENCE]

# if method == OPPORTUNISTIC: qos represents the percentage of the optimal rate
# solution to retain when minimizing variance (i.e. optimal rates can be reduced
# by qos percent in order to minimize variance)
# if method == CONFIDENCE: qos is the probability with which the required rate
# must be satisfied (i.e. there is a qos percent chance that the network can
# satisfy the required communication rate)
float64 qos