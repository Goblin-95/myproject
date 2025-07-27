#!/usr/bin/env python3
"""
TurtleBot GPS Bridge - Connects TurtleBot GPS data to your existing GUI
This script reads GPS data from the TurtleBot and sends it to your tkinter GUI
"""

import socket
import time
import json
import os
import threading
from datetime import datetime

class TurtleBotGPSBridge:
    def __init__(self):
        # GUI connection settings (same as your animating_sender)
        self.gui_host = '127.0.0.1'
        self.gui_port = 65432
        
        # GPS data file path
        self.gps_file = os.path.expanduser('~/turtlebot_gps_workspace/gps_data.json')
        
        # GPS to GUI coordinate conversion settings
        # These will be set based on the GPS map bounds in your GUI
        self.center_lat = None
        self.center_lon = None
        self.lat_scale = 1.0  # meters per degree latitude
        self.lon_scale = 1.0  # meters per degree longitude
        
        # Last processed GPS point
        self.last_processed_count = 0
        
        print("[GPS Bridge] Initialized TurtleBot GPS Bridge")
    
    def setup_coordinate_conversion(self, center_lat, center_lon):
        """
        Set up the coordinate conversion from GPS to GUI coordinates
        center_lat, center_lon: The center point of your GPS map in the GUI
        """
        self.center_lat = center_lat
        self.center_lon = center_lon
        
        # Rough conversion: 1 degree latitude ≈ 111,000 meters
        # 1 degree longitude ≈ 111,000 * cos(latitude) meters
        import math
        self.lat_scale = 111000  # meters per degree
        self.lon_scale = 111000 * math.cos(math.radians(center_lat))
        
        print(f"[GPS Bridge] Coordinate conversion set up for center: {center_lat}, {center_lon}")
    
    def gps_to_gui_coordinates(self, lat, lon):
        """
        Convert GPS coordinates to GUI coordinates (0-100 range)
        """
        if self.center_lat is None or self.center_lon is None:
            print("[GPS Bridge] Warning: Coordinate conversion not set up!")
            return 50, 50  # Default to center
        
        # Calculate offset in meters from center
        lat_offset_meters = (lat - self.center_lat) * self.lat_scale
        lon_offset_meters = (lon - self.center_lon) * self.lon_scale
        
        # Convert to GUI coordinates (assuming GUI shows ~500m x 500m area)
        # Adjust these scale factors based on your GPS map zoom level
        gui_scale = 500.0  # meters that the 100x100 GUI represents
        
        gui_x = 50 + (lon_offset_meters / gui_scale) * 100
        gui_y = 50 + (lat_offset_meters / gui_scale) * 100
        
        # Clamp to GUI bounds
        gui_x = max(0, min(100, gui_x))
        gui_y = max(0, min(100, gui_y))
        
        return gui_x, gui_y
    
    def read_gps_data(self):
        """
        Read new GPS points from the JSON file
        """
        try:
            if not os.path.exists(self.gps_file):
                return []
            
            with open(self.gps_file, 'r') as f:
                all_data = json.load(f)
            
            # Return only new points since last read
            new_points = all_data[self.last_processed_count:]
            self.last_processed_count = len(all_data)
            
            return new_points
        
        except Exception as e:
            print(f"[GPS Bridge] Error reading GPS data: {e}")
            return []
    
    def connect_to_gui(self):
        """
        Connect to your existing GUI (like animating_sender does)
        """
        print("[GPS Bridge] Connecting to GUI...")
        
        while True:
            try:
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.connect((self.gui_host, self.gui_port))
                print("[GPS Bridge] Connected to GUI successfully!")
                return sock
            except ConnectionRefusedError:
                print("[GPS Bridge] GUI not ready, retrying in 2s...")
                time.sleep(2)
            except Exception as e:
                print(f"[GPS Bridge] Connection error: {e}")
                time.sleep(2)
    
    def send_gps_position(self, sock, lat, lon):
        """
        Send GPS position to GUI
        """
        try:
            gui_x, gui_y = self.gps_to_gui_coordinates(lat, lon)
            message = f"{gui_x},{gui_y},slow"  # Using slow speed for smooth GPS tracking
            sock.sendall(message.encode())
            print(f"[GPS Bridge] Sent GPS: {lat:.6f},{lon:.6f} -> GUI: {gui_x:.1f},{gui_y:.1f}")
            return True
        except Exception as e:
            print(f"[GPS Bridge] Error sending position: {e}")
            return False
    
    def run(self, center_lat=40.7128, center_lon=-74.0060):
        """
        Main loop - monitor GPS data and send to GUI
        """
        # Set up coordinate conversion
        self.setup_coordinate_conversion(center_lat, center_lon)
        
        # Connect to GUI
        sock = self.connect_to_gui()
        
        print("[GPS Bridge] Starting GPS monitoring...")
        print("Make sure your TurtleBot simulation and GPS logger are running!")
        
        try:
            while True:
                # Check for new GPS data
                new_points = self.read_gps_data()
                
                for point in new_points:
                    lat = point['latitude']
                    lon = point['longitude']
                    timestamp = point['timestamp']
                    
                    print(f"[GPS Bridge] Processing GPS point: {timestamp}")
                    
                    # Send to GUI
                    if not self.send_gps_position(sock, lat, lon):
                        # Connection lost, try to reconnect
                        sock.close()
                        sock = self.connect_to_gui()
                    
                    # Small delay between points for smooth movement
                    time.sleep(0.1)
                
                # Check for new data every 0.5 seconds
                time.sleep(0.5)
                
        except KeyboardInterrupt:
            print("\n[GPS Bridge] Shutting down...")
        except Exception as e:
            print(f"[GPS Bridge] Unexpected error: {e}")
        finally:
            sock.close()

def main():
    """
    Main function - you can customize the GPS center coordinates here
    """
    bridge = TurtleBotGPSBridge()
    
    # Set your GPS center coordinates here (these should match your GPS map in GUI)
    # Default is New York City coordinates (same as TurtleBot's initial GPS position)
    center_lat = 40.7128
    center_lon = -74.0060
    
    print("=== TurtleBot GPS Bridge ===")
    print(f"Center coordinates: {center_lat}, {center_lon}")
    print("Make sure to:")
    print("1. Start your GUI (gui_receiver.py)")
    print("2. Start TurtleBot simulation with GPS")
    print("3. Start GPS logger")
    print("4. Set GPS map in GUI to the same center coordinates")
    print("Press Ctrl+C to stop")
    print("===============================")
    
    bridge.run(center_lat, center_lon)

if __name__ == "__main__":
    main()
