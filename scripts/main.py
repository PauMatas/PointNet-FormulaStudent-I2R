#!/usr/bin/env python3

import rospy

from nav_msgs.msg import Odometry

from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

POINT_CLOUD_TABLE = []

def pointCloudSubscriber(ros_point_cloud):
    pointcloud_generator = pc2.read_points(ros_point_cloud, field_names=['x', 'y', 'z', 'timestamp'], skip_nans=True)

    global POINT_CLOUD_TABLE
    POINT_CLOUD_TABLE += [{'x': x, 'y': y, 'z': z, 'timestamp': timestamp} for x, y, z, timestamp in pointcloud_generator]
    print(POINT_CLOUD_TABLE)

def positionSubscriber(ros_point_cloud):
    pass

def main():
    rospy.init_node('main')
    print("Dataset construction node initialised.")

    rospy.Subscriber('limovelo/full_pcl', PointCloud2, pointCloudSubscriber, buff_size=10000)
    rospy.Subscriber('limovelo/state', Odometry, positionSubscriber, buff_size=10000)
    
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass