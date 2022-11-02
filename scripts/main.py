#!/usr/bin/env python3

import rospy

from nav_msgs.msg import Odometry

from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

from geometry_msgs.msg import PointStamped

def pointCloudSubscriber(last_pointcloud):
    print('pointcloud')
    # gen = pc2.read_points(ros_point_cloud, skip_nans=True)
    # print(list(gen))

def positionSubscriber(last_odom):
    print('odometry')
    print(last_odom)

def conePosition(cone_point):
    print('Cone position received')
    print('Header: ', cone_point.header.stamp)
    print('x: ', cone_point.point.x)
    print('y: ', cone_point.point.y)
    print('z: ', cone_point.point.z)
    

def main():
    rospy.init_node('main')

    print("Hello")

    # LiDAR Output Subscriber
    rospy.Subscriber('limovelo/full_pcl', PointCloud2, pointCloudSubscriberClbk, buff_size=10000)

    # LIMOVelo State Subscriber
    rospy.Subscriber('limovelo/state', Odometry, positionSubscriberClbk, buff_size=10000)

    # Ground Truth Cone Point
    rospy.Subscriber('/clicked_point', PointStamped, conePositionClbk, buff_size=10000)
    
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass