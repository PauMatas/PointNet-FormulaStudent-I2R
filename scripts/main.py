#!/usr/bin/env python3

import rospy
from datetime import datetime
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

import pointcloud_db as db


def pointcloud_subscriber(ros_point_cloud):
    """Callback function for the point cloud subscriber"""

    pointcloud_generator = pc2.read_points(
        ros_point_cloud,
        field_names=['x', 'y', 'z', 'timestamp'],
        skip_nans=True
    )

    pointcloud_table = db.PointCloudTable()
    for x, y, z, timestamp in pointcloud_generator:
        pointcloud_table.insert_data(
            x=x,
            y=y,
            z=z,
            datetime=datetime.fromtimestamp(timestamp)
        )


def odometry_subscriber(ros_point_cloud):
    """Callback function for the odometry subscriber"""
    pass


def main():
    """Main function"""

    rospy.init_node('main')
    print("Dataset construction node initialised.")

    rospy.Subscriber('limovelo/full_pcl', PointCloud2,
                     pointcloud_subscriber, buff_size=10000)
    rospy.Subscriber('limovelo/state', Odometry,
                     odometry_subscriber, buff_size=10000)

    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
