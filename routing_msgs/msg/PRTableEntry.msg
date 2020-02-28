# a probabilistic routing table entry consisting of the possible gateways and
# their corresponding probability of use through which data originating at ~src~
# intended for ~dest~ should be routed at ~node~

string node # IP address of current node receiving data
string src  # IP address of node from which the data first originated
            # (NOTE: this may not be the same as the IP from which ~node~
            # received the data)
string dest # IP address of the destination node of the data

# IP, probability pairs of possible gateways
routing_msgs/ProbGateway[] gateways
