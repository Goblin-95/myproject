#!/usr/bin/env python3
"""
Auto-Updating TurtleBot GPS Logger
- Logs GPS data from /gps/fix topic
- Automatically copies to shared folder for Windows
- Creates new file for each simulation run
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
import json
import os
from datetime import datetime
import shutil

class AutoGPSLogger(Node):
    def __init__(self):
        super().__init__('auto_gps_logger')
        
        # File paths
        self.workspace_dir = os.path.expanduser('~/turtlebot_gps_workspace')
        self.shared_folder = '/media/sf_TurtleBotGPS'
        
        # Create timestamped filename for this run
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.local_file = os.path.join(self.workspace_dir, f'gps_data_{timestamp}.json')
        self.current_file = os.path.join(self.workspace_dir, 'gps_data.json')  # Current file
        self.shared_file = os.path.join(self.shared_folder, 'gps_data.json')
        
        # GPS data storage
        self.gps_data = []
        self.last_copy_time = 0
        
        # Subscribe to GPS topic
        self.subscription = self.create_subscription(
            NavSatFix,
            '/gps/fix',
            self.gps_callback,
            10
        )
        
        # Timer to periodically copy to shared folder (every 2 seconds)
        self.copy_timer = self.create_timer(2.0, self.copy_to_shared_folder)
        
        self.get_logger().info(f'🚀 Auto GPS Logger started!')
        self.get_logger().info(f'📝 Local file: {self.local_file}')
        self.get_logger().info(f'📋 Current file: {self.current_file}')
        self.get_logger().info(f'📤 Shared file: {self.shared_file}')
        self.get_logger().info(f'🎯 Listening to /gps/fix topic...')
        
    def gps_callback(self, msg):
        """Handle incoming GPS data"""
        # Create GPS data entry
        gps_entry = {
            "timestamp": datetime.now().isoformat(),
            "latitude": msg.latitude,
            "longitude": msg.longitude,
            "altitude": msg.altitude
        }
        
        # Add to our data list
        self.gps_data.append(gps_entry)
        
        # Log every 10th point to avoid spam
        if len(self.gps_data) % 10 == 0:
            self.get_logger().info(f'📍 GPS Point {len(self.gps_data)}: '
                                 f'Lat={msg.latitude:.2e}, Lon={msg.longitude:.2e}')
        
        # Save to local files
        self.save_gps_data()
        
    def save_gps_data(self):
        """Save GPS data to local files"""
        try:
            # Save to timestamped file (permanent record)
            with open(self.local_file, 'w') as f:
                json.dump(self.gps_data, f, indent=2)
            
            # Save to current file (for compatibility)
            with open(self.current_file, 'w') as f:
                json.dump(self.gps_data, f, indent=2)
                
        except Exception as e:
            self.get_logger().error(f'❌ Error saving GPS data: {e}')
    
    def copy_to_shared_folder(self):
        """Copy GPS data to shared folder for Windows access"""
        try:
            # Check if we have new data
            if len(self.gps_data) == 0:
                return
                
            # Check if shared folder exists
            if not os.path.exists(self.shared_folder):
                self.get_logger().warn(f'⚠️  Shared folder not found: {self.shared_folder}')
                return
            
            # Copy current GPS data to shared folder
            if os.path.exists(self.current_file):
                shutil.copy2(self.current_file, self.shared_file)
                
                # Log copy success (but not too often)
                current_time = len(self.gps_data)
                if current_time - self.last_copy_time >= 20:  # Every 20 points
                    self.get_logger().info(f'📤 Copied {len(self.gps_data)} GPS points to shared folder')
                    self.last_copy_time = current_time
                    
        except Exception as e:
            self.get_logger().error(f'❌ Error copying to shared folder: {e}')
    
    def shutdown(self):
        """Clean shutdown"""
        self.get_logger().info(f'🏁 GPS Logger shutting down...')
        self.get_logger().info(f'📊 Total GPS points collected: {len(self.gps_data)}')
        
        # Final save and copy
        if len(self.gps_data) > 0:
            self.save_gps_data()
            self.copy_to_shared_folder()
            self.get_logger().info(f'✅ Final GPS data saved!')

def main(args=None):
    print("🐢 Auto GPS Logger for TurtleBot")
    print("=" * 40)
    
    rclpy.init(args=args)
    
    try:
        gps_logger = AutoGPSLogger()
        
        print("🎯 GPS logging started! Move your robot to generate data.")
        print("📤 Data will auto-sync to Windows every 2 seconds.")
        print("⏹️  Press Ctrl+C to stop logging.")
        print()
        
        rclpy.spin(gps_logger)
        
    except KeyboardInterrupt:
        print("\n🛑 Stopping GPS logger...")
        
    finally:
        if 'gps_logger' in locals():
            gps_logger.shutdown()
        rclpy.shutdown()
        print("👋 GPS Logger stopped!")

if __name__ == '__main__':
    main()
