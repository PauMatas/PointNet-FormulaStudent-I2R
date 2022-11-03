#!/usr/bin/env python3

import rospy

from nav_msgs.msg import Odometry

from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

from geometry_msgs.msg import PointStamped

import csv

def writeCSVCones():
    with open('carPositions.csv', mode='w') as csv_file:
    fieldnames = ['timestamp', 'pos_x', 'pos_y', 'pos_z', 'ori_x', 'ori_y', 'ori_z', 'ori_w']
    writer = csv.DictWriter(csv_file, fieldnames=fieldnames)

    writer.writeheader()
    writer.writerow({'emp_name': 'John Smith', 'dept': 'Accounting', 'birth_month': 'November'})
    writer.writerow({'emp_name': 'Erica Meyers', 'dept': 'IT', 'birth_month': 'March'})


def pointCloudSubscriberClbk(last_pointcloud):
    print('pointcloud')
    print('[last_pointcloud] Header: ', last_pointcloud.header.stamp)
    gen = pc2.read_points(last_pointcloud, skip_nans=True)

    first = True
    for p in gen:
        if (first):
            print(type(p))
            x = p[0]
            y = p[1]
            z = p[2]
    
            print('PC: x', x)
            print('PC: y', y)
            print('PC: z', z)

            unk1 = p[3]
            unk2 = p[4]
            unk3 = p[5]

            print('PC: unk1', unk1)
            print('PC: unk2', unk2)
            print('PC: unk3', unk3)
            
            
            first = False

    # print(list(gen)[0])
    # print('type points: ', type(list(gen)[0])) )

odomList = []
def positionSubscriberClbk(last_odom):
    odom = {}
    
    odom['last_odom'] = last_odom.header.stamp

    odom['pos_x'] = last_odom.pose.pose.position.x
    odom['pos_y'] = last_odom.pose.pose.position.y
    odom['pos_z'] = last_odom.pose.pose.position.z

    odom['ori_x'] = last_odom.pose.pose.orientation.x
    odom['ori_y'] = last_odom.pose.pose.orientation.y
    odom['ori_z'] = last_odom.pose.pose.orientation.z
    odom['ori_w'] = last_odom.pose.pose.orientation.w
    

def conePositionClbk(cone_point):
    print('Cone position received')
    print('[cone_point] Header: ', cone_point.header.stamp)
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