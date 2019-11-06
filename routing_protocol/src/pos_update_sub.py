#!/usr/bin/python
import rospy
from sensor_msgs.msg import NavSatFix
#from std_msgs.msg import String

def callback(data):
    print_line = []
    print_line.append(data.latitude)
    print_line.append(data.longitude)
    print_line.append(data.altitude)
    print(print_line)



def listener():
    try:
        # In ROS, nodes are uniquely named. If two nodes with the same
        # node are launched, the previous one is kicked off. The
        # anonymous=True flag means that rospy will choose a unique
        # name for our 'listener' node so that multiple listeners can
        # run simultaneously.
        rospy.init_node('pos_update_sub', anonymous=True)
        rospy.Subscriber("mavros/global_position/global", NavSatFix, callback)

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