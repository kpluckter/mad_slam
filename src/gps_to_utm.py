#!/usr/bin/env python
import rospy
from at_drone_interface.msg import GpsData
from nav_msgs.msg import Odometry

Odometry msg

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data)
    lat = data.latitude
    longi = data.longitude
    
    msg.pose.pose.position =
    
def gps_to_utm():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    pub = rospy.Publisher('/ground/odom', Odometry, queue_size=1)
    rospy.init_node('gps_to_utm', anonymous=True)
    rate = rospy.Rate(5)


    rospy.Subscriber("/at_drone_interface/gps_full_data", GpsData, callback)

    # spin() simply keeps python from exiting until this node is stopped
    while not rospy.is_shutdown():
        pub.publish(msg)
        rospy.spinOnce()


if __name__ == '__main__':
    gps_to_utm()