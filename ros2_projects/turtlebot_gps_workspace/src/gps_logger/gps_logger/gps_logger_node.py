#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
import json
import os
from datetime import datetime

class GPSLogger(Node):
    def __init__(self):
        super().__init__('gps_logger')
        self.subscription = self.create_subscription(
            NavSatFix,
            '/gps/fix',
            self.gps_callback,
            10)
        
        # Create GPS log file
        self.log_file = os.path.expanduser('~/turtlebot_gps_workspace/gps_data.json')
        self.gps_data = []
        
        self.get_logger().info('GPS Logger started - saving to: ' + self.log_file)

    def gps_callback(self, msg):
        timestamp = datetime.now().isoformat()
        gps_point = {
            'timestamp': timestamp,
            'latitude': msg.latitude,
            'longitude': msg.longitude,
            'altitude': msg.altitude
        }
        
        self.gps_data.append(gps_point)
        
        # Save to file
        with open(self.log_file, 'w') as f:
            json.dump(self.gps_data, f, indent=2)
        
        self.get_logger().info(f'GPS: Lat={msg.latitude:.6f}, Lon={msg.longitude:.6f}')

def main(args=None):
    rclpy.init(args=args)
    gps_logger = GPSLogger()
    
    try:
        rclpy.spin(gps_logger)
    except KeyboardInterrupt:
        pass
        
    gps_logger.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
