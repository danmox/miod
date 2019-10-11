#!/usr/bin/python
import rospy
from routing_msgs.msg import NetworkUpdate
#from sensor_msgs.msg import NavSatFix
#from std_msgs.msg import String

def callback(data):
    print_airline = []
    for i in data.routes:
        print_line = []
        print_line.append(i.node)
        print_line.append(i.src)
        print_line.append(i.dest)
        for j in i.gateways:
            print_line.append(j.IP)
            print_line.append(j.prob)
        print_airline.append(print_line)
    print(print_airline)



def listener():
    try:
        # In ROS, nodes are uniquely named. If two nodes with the same
        # node are launched, the previous one is kicked off. The
        # anonymous=True flag means that rospy will choose a unique
        # name for our 'listener' node so that multiple listeners can
        # run simultaneously.
        rospy.init_node('rt_sub', anonymous=True)

        rospy.Subscriber("rt_upd", NetworkUpdate, callback)

        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()
    except Exception as e:
        print(e)
        exit(0)


if __name__ == '__main__':
    try:
        listener()
    except:
        exit(0)