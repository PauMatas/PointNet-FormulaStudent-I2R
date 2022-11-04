#!/usr/bin/env python3

from datetime import datetime

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import PointStamped

import pointcloud_db as db


def pointcloud_subscriber_clbk(last_pointcloud):
    """Callback function for the point cloud subscriber"""

    pointcloud_generator = pc2.read_points(
        last_pointcloud,
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

odomList = []
def position_subscriber_clbk(last_odom):
    odom = {}
    
    odom['last_odom'] = last_odom.header.stamp

    odom['pos_x'] = last_odom.pose.pose.position.x
    odom['pos_y'] = last_odom.pose.pose.position.y
    odom['pos_z'] = last_odom.pose.pose.position.z

    odom['ori_x'] = last_odom.pose.pose.orientation.x
    odom['ori_y'] = last_odom.pose.pose.orientation.y
    odom['ori_z'] = last_odom.pose.pose.orientation.z
    odom['ori_w'] = last_odom.pose.pose.orientation.w
    

def cone_position_clbk(cone_point):
    print('Cone position received')
    print('[cone_point] Header: ', cone_point.header.stamp)
    print('x: ', cone_point.point.x)
    print('y: ', cone_point.point.y)
    print('z: ', cone_point.point.z)
    

def main():
    """Main function"""

    rospy.init_node('main')
    print("Dataset construction node initialised.")

    # LiDAR Output Subscriber
    rospy.Subscriber('limovelo/full_pcl', PointCloud2, pointcloud_subscriber_clbk, buff_size=10000)

    # LIMOVelo State Subscriber
    rospy.Subscriber('limovelo/state', Odometry, position_subscriber_clbk, buff_size=10000)

    # Ground Truth Cone Point
    rospy.Subscriber('/clicked_point', PointStamped, cone_position_clbk, buff_size=10000)
    
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
