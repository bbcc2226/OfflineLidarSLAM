import argparse
import os
import numpy as np
from datetime import datetime

import rclpy
from rclpy.serialization import serialize_message

from rosbag2_py import SequentialWriter, StorageOptions, ConverterOptions, TopicMetadata

from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus, PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header


# ============================================================
# Time utilities
# ============================================================

def kitti_time_to_ns(t):
    t = t.strip()  # remove whitespace
    if "." in t:
        sec_str, nano_str = t.split(".")
        dt = datetime.strptime(sec_str, "%Y-%m-%d %H:%M:%S")
        ns = int(nano_str.ljust(9, "0"))  # pad to 9 digits if needed
        return int(dt.timestamp() * 1e9) + ns
    else:
        dt = datetime.strptime(t, "%Y-%m-%d %H:%M:%S")
        return int(dt.timestamp() * 1e9)



# ============================================================
# OXTS reader (IMU + GPS)
# ============================================================

def read_oxts(oxts_dir):
    data_dir = os.path.join(oxts_dir, "data")
    timestamps = open(os.path.join(oxts_dir, "timestamps.txt")).readlines()

    for i, ts in enumerate(timestamps):
        file = os.path.join(data_dir, f"{i:010d}.txt")
        vals = np.loadtxt(file)

        # ---------- IMU ----------
        imu = Imu()
        imu.header.frame_id = "imu_link"

        # Linear acceleration (m/s^2)
        imu.linear_acceleration.x = vals[11]
        imu.linear_acceleration.y = vals[12]
        imu.linear_acceleration.z = vals[13]

        # Angular velocity (rad/s)
        imu.angular_velocity.x = vals[17]
        imu.angular_velocity.y = vals[18]
        imu.angular_velocity.z = vals[19]

        # ---------- GPS ----------
        gps = NavSatFix()
        gps.header.frame_id = "gps_link"

        gps.latitude = vals[0]
        gps.longitude = vals[1]
        gps.altitude = vals[2]

        gps.status.status = NavSatStatus.STATUS_FIX
        gps.status.service = NavSatStatus.SERVICE_GPS
        gps.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN

        t_ns = kitti_time_to_ns(ts)
        print(ts,t_ns)
        ns = t_ns
 
        imu.header.stamp.sec = ns // 1_000_000_000
        imu.header.stamp.nanosec = ns % 1_000_000_000

        gps.header.stamp.sec = ns // 1_000_000_000
        gps.header.stamp.nanosec = ns % 1_000_000_000
        yield imu, gps, t_ns


# ============================================================
# Velodyne TXT reader
# ============================================================

def read_velodyne_txt(velo_dir):
    data_dir = os.path.join(velo_dir, "data")
    timestamps_file = os.path.join(velo_dir, "timestamps.txt")
    timestamps = open(timestamps_file).readlines()

    # Define PointCloud2 fields
    fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
    ]

    for i, ts in enumerate(timestamps):
        file_path = os.path.join(data_dir, f"{i:010d}.txt")
        # Load points: shape (N,4)
        points = np.loadtxt(file_path, dtype=np.float32)

        header = Header()
        header.frame_id = "velodyne"

        # Create PointCloud2 message
        cloud_msg = point_cloud2.create_cloud(
            header,
            fields,
            points
        )

        # Convert timestamp to nanoseconds
        t_ns = kitti_time_to_ns(ts)
        ns = t_ns

        cloud_msg.header.stamp.sec = ns // 1_000_000_000
        cloud_msg.header.stamp.nanosec = ns % 1_000_000_000
        yield cloud_msg, t_ns


# ============================================================
# Main
# ============================================================

def parse_args():
    parser = argparse.ArgumentParser(
        description="Convert KITTI OXTS and Velodyne text data to a ROS 2 bag."
    )
    parser.add_argument(
        "dataset_root",
        help="KITTI drive directory containing oxts/ and velodyne_points/.",
    )
    parser.add_argument("output_bag", help="Output ROS 2 bag directory.")
    parser.add_argument("--storage-id", default="sqlite3")
    parser.add_argument("--imu-topic", default="/kitti/imu")
    parser.add_argument("--gps-topic", default="/kitti/gps/fix")
    parser.add_argument("--lidar-topic", default="/kitti/velodyne_points")
    return parser.parse_args()


def main():
    args = parse_args()
    rclpy.init()

    writer = SequentialWriter()
    writer.open(
        StorageOptions(uri=args.output_bag, storage_id=args.storage_id),
        ConverterOptions("", "")
    )

    # ---------- Create topics ----------
    writer.create_topic(
        TopicMetadata(
            name=args.imu_topic,
            type="sensor_msgs/msg/Imu",
            serialization_format="cdr"
        )
    )

    writer.create_topic(
        TopicMetadata(
            name=args.gps_topic,
            type="sensor_msgs/msg/NavSatFix",
            serialization_format="cdr"
        )
    )

    writer.create_topic(
        TopicMetadata(
            name=args.lidar_topic,
            type="sensor_msgs/msg/PointCloud2",
            serialization_format="cdr"
        )
    )

    # ---------- Paths ----------
    oxts_dir = os.path.join(args.dataset_root, "oxts")
    velo_dir = os.path.join(args.dataset_root, "velodyne_points")

    # ---------- Write IMU + GPS ----------
    for imu, gps, t in read_oxts(oxts_dir):
        #print("imu gps {}".format(t))
        writer.write(args.imu_topic, serialize_message(imu), t)
        writer.write(args.gps_topic, serialize_message(gps), t)

    # ---------- Write LiDAR ----------
    for cloud, t in read_velodyne_txt(velo_dir):
        #print(t)
        writer.write(args.lidar_topic, serialize_message(cloud), t)

    print("✅ Unsynced KITTI ROS 2 bag created:", args.output_bag)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
