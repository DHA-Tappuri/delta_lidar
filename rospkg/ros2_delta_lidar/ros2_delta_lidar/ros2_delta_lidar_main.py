#!/usr/bin/env python3
# coding: utf-8

import rclpy
from rclpy.clock     import Clock
from rclpy.node      import Node

from sensor_msgs.msg import LaserScan
from std_msgs.msg    import Bool

from .delta_lidar    import delta_lidar



class ros2_delta_lidar(Node):

    def __init__(self):
        # initialize node
        super().__init__('ros2_delta_lidar')

        # declare parameter

        # moving object detection
        self.declare_parameter( 'port', '/dev/ttyUSB0')
        self.declare_parameter( 'baud', 115200)

        # read parameter
        self._port = self.get_parameter('port').value
        self._baud = int(self.get_parameter('baud').value)

        # initialize delta 2G
        self._lidar = delta_lidar( port=self._port, baud=self._baud, use_ctrl=True )
        self._lidar.start(self._callback_scan)

        # initialize topic
        self._pub_laserscan = self.create_publisher( LaserScan, 'scan',  10 )
        self._sub_activate  = self.create_subscription( Bool, 'active', self._callback_activate, 10 )


    # callback
    def _callback_scan( self, data ):
        msg = LaserScan()
        msg.header.stamp    = Clock().now().to_msg()
        msg.header.frame_id = 'laser'

        msg.angle_min       = -3.141592
        msg.angle_max       =  3.141592
        msg.angle_increment = -(msg.angle_max - msg.angle_min) / len(data._range)

        msg.range_min       = 0.0
        msg.range_max       = 8.0
        msg.ranges          = data._range
        msg.intensities     = data._rssi

        self._pub_laserscan.publish(msg)
        
        
    def _callback_activate(self, msg):
        self._lidar.ctrl_lidar(bool(msg.data))
        return



# main function
def main(args=None):
    # initialize ROS2
    rclpy.init(args=args)

    # initialize node
    node = ros2_delta_lidar()
    rclpy.spin(node)

    # Destroy the node explicitly
    node.destroy_node()
    rclpy.shutdown()




# main function
if(__name__ == '__main__'):
    main()
