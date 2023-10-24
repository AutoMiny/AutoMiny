#!/usr/bin/python3

import rclpy
import roslibpy

from rclpy.node import Node
from rclpy.time import Time
from rclpy.executors import MultiThreadedExecutor

from nav_msgs.msg import Odometry

class GPSClient(Node):
    def __init__(self):
        super().__init__('gps_client')
        self.id = self.declare_parameter('id', 16)
        self.pub = self.create_publisher(Odometry, '/sensors/odometry/gps', 10)
        client = roslibpy.Ros(host='192.168.43.2', port=9090)
        client.run()
        listener = roslibpy.Topic(client, '/communication/gps/' + str(self.get_parameter('id').value), 'nav_msgs/Odometry')
        listener.subscribe(self.on_gps)

    def on_gps(self, message):
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg();
        odom.header.frame_id = message['header']['frame_id']
        odom.child_frame_id = message['child_frame_id']
        odom.pose.pose.position.x = message['pose']['pose']['position']['x']
        odom.pose.pose.position.y = message['pose']['pose']['position']['y']
        odom.pose.pose.position.z = message['pose']['pose']['position']['z']
        odom.pose.pose.orientation.x = message['pose']['pose']['orientation']['x']
        odom.pose.pose.orientation.y = message['pose']['pose']['orientation']['y']
        odom.pose.pose.orientation.z = message['pose']['pose']['orientation']['z']
        odom.pose.pose.orientation.w = message['pose']['pose']['orientation']['w']
        odom.twist.twist.linear.x = message['twist']['twist']['linear']['x']
        odom.twist.twist.linear.y = message['twist']['twist']['linear']['y']
        odom.twist.twist.linear.z = message['twist']['twist']['linear']['z']
        odom.twist.twist.angular.x = message['twist']['twist']['angular']['x']
        odom.twist.twist.angular.y = message['twist']['twist']['angular']['y']
        odom.twist.twist.angular.z = message['twist']['twist']['angular']['z']
        self.pub.publish(odom)

        print(message)

def main(args=None):
    rclpy.init(args=args)
    gps_client = GPSClient()
    executor = MultiThreadedExecutor()
    executor.add_node(gps_client)

    rclpy.spin(gps_client)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
