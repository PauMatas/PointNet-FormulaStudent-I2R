#!/usr/bin/env python3

import rospy

from nav_msgs.msg import Odometry

from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

def pointCloudSubscriber(ros_point_cloud):
    print('pointcloud')
    # gen = pc2.read_points(ros_point_cloud, skip_nans=True)
    # print(list(gen))

def positionSubscriber(ros_point_cloud):
    print('odometry')

def main():
    rospy.init_node('main')

    print("Hello")

    rospy.Subscriber('limovelo/full_pcl', PointCloud2, pointCloudSubscriber, buff_size=10000)
    rospy.Subscriber('limovelo/state', Odometry, positionSubscriber, buff_size=10000)
    
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass