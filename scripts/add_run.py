#!/usr/bin/env python3

from os import path
import rospy
import logging
from datetime import datetime
from argparse import ArgumentParser
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import PointStamped

from as_msgs.msg import ObservationArray
import pointcloud_db as db


START_TIME = None
# Upper confidence threshold for the cones retrieved by the pcl_filtering node in order to be tagged as no-cones
NO_CONE_THRESHOLD = 0.5


def _time_processed_print(timestamp, quiet: bool):
    """Prints the time processed by the node"""
    if not quiet:
        clbk_time = datetime.fromtimestamp(
            # Convert nanoseconds to seconds
            int(str(timestamp)) // 1000000000
        )

        global START_TIME
        if START_TIME is None:
            START_TIME = clbk_time

        print(
            f'Run time processed: {(clbk_time - START_TIME).total_seconds()} s      ', end='\r')


def pointcloud_subscriber_clbk(last_pointcloud: PointCloud2, kwargs):
    """Callback function for the point cloud subscriber"""
    run_name = kwargs['run_name']
    filename = kwargs['filename']
    quiet = kwargs['quiet']

    pointcloud_generator = pc2.read_points(
        last_pointcloud,
        field_names=['x', 'y', 'z', 'timestamp'],
        skip_nans=True
    )

    pointcloud_table = db.PointCloudTable()
    pointcloud_table.insert_rows(
        [
            (x, y, z, datetime.fromtimestamp(timestamp), run_name, filename)
            for x, y, z, timestamp
            in pointcloud_generator
        ])

    _time_processed_print(last_pointcloud.header.stamp, quiet)


def position_subscriber_clbk(last_odom, kwargs):
    """Callback function for the position subscriber"""
    run_name = kwargs['run_name']
    filename = kwargs['filename']
    quiet = kwargs['quiet']

    # Attribute Selection
    pos_x = last_odom.pose.pose.position.x
    pos_y = last_odom.pose.pose.position.y
    pos_z = last_odom.pose.pose.position.z

    ori_x = last_odom.pose.pose.orientation.x
    ori_y = last_odom.pose.pose.orientation.y
    ori_z = last_odom.pose.pose.orientation.z
    ori_w = last_odom.pose.pose.orientation.w

    timestamp = last_odom.header.stamp

    # Get time from stamp in seconds
    odom_time = datetime.fromtimestamp(int(str(timestamp)) / 1000000000)

    position_table = db.PoseTable()
    position_table.insert_rows(
        [
            (pos_x, pos_y, pos_z,
             ori_x, ori_y, ori_z, ori_w,
             odom_time, run_name, filename)
        ])

    _time_processed_print(timestamp, quiet)


def cone_position_clbk(cone_point, kwargs):
    """Callback function for the cone position subscriber"""
    run_name = kwargs['run_name']
    filename = kwargs['filename']
    quiet = kwargs['quiet']

    x = cone_point.point.x
    y = cone_point.point.y
    z = cone_point.point.z

    cone_table = db.ConePositionTable()
    cone_table.insert_row(x=x, y=y, z=z, run_name=run_name, filename=filename)

    _time_processed_print(cone_point.header.stamp, quiet)


def no_cone_position_clbk(no_cone_point, kwargs):
    """Callback function for the no cone position subscriber"""
    run_name = kwargs['run_name']
    filename = kwargs['filename']
    quiet = kwargs['quiet']

    no_cone_table = db.NoConePositionTable()
    no_cone_table.insert_rows(
        [
            (observation.centroid.x, observation.centroid.y, observation.centroid.z, run_name, filename)
            for observation
            in no_cone_point.observations
            if observation.confidence < NO_CONE_THRESHOLD
        ])

    _time_processed_print(no_cone_point.header.stamp, quiet)


def main():
    """Main function"""

    parser = ArgumentParser()
    parser.add_argument("-p", "--path", dest="path",
                        help="Path to the bag file", required=True, type=str)
    parser.add_argument("-r", "--run", dest="run_name",
                        help="Name of the run to be added to the database", required=True, type=str)
    parser.add_argument("-q", "--quiet", dest="quiet",
                        help="don't print status messages to stdout", default=False)
    args = parser.parse_args()

    filename = path.basename(args.path)
    db.RunTable().insert_row(name=args.run_name, filename=filename)

    rospy.init_node('add_run')

    if not args.quiet:
        print("Dataset construction node initialised.")

    # LiDAR Output Subscriber
    rospy.Subscriber('limovelo/full_pcl', PointCloud2,
                     pointcloud_subscriber_clbk, {'run_name': args.run_name, 'filename': filename, 'quiet': args.quiet}, buff_size=10000)

    # LIMOVelo State Subscriber
    rospy.Subscriber('limovelo/state', Odometry,
                     position_subscriber_clbk, {'run_name': args.run_name, 'filename': filename, 'quiet': args.quiet}, buff_size=10000)

    # Ground Truth Cone Point
    rospy.Subscriber('/clicked_point', PointStamped,
                     cone_position_clbk, {'run_name': args.run_name, 'filename': filename, 'quiet': args.quiet}, buff_size=10000)

    # Ground Truth No-Cone Point
    rospy.Subscriber('/AS/P/pcl_filtering/observations',
                     ObservationArray, no_cone_position_clbk, {'run_name': args.run_name, 'filename': filename, 'quiet': args.quiet}, buff_size=10000)

    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
