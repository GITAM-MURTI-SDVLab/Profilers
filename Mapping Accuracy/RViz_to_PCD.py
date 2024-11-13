#!/usr/bin/env python3

import rospy
import open3d as o3d
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
import numpy as np

def pointcloud_callback(msg):
    # Convert ROS PointCloud2 message to a list of points
    points = []

    for point in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
        points.append([point[0], point[1], point[2]])

    # Convert list of points to a numpy array
    np_points = np.array(points, dtype=np.float32)

    # Create Open3D point cloud object and save to a .pcd file
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(np_points)
    o3d.io.write_point_cloud("/home/nagella/ISCLOAM.pcd", pcd)

    rospy.loginfo("3D map saved as 3d_map.pcd")

def main():
    rospy.init_node("pcd_saver", anonymous=True)

    # Replace '/map' with your 3D map topic
    rospy.Subscriber("/map", PointCloud2, pointcloud_callback)

    rospy.loginfo("Waiting for 3D map data...")
    rospy.spin()

if __name__ == "__main__":
    main()
