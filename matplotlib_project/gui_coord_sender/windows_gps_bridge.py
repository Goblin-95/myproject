#!/usr/bin/env python3
"""
Improved GPS Bridge with Dynamic Scaling
"""

import socket
import time
import json
import os

# === Configuration ===
GPS_DATA_PATH = r"C:\Users\georg\OneDrive\Desktop\TurtleBotGPS\gps_data.json"
HOST = '127.0.0.1'
PORT = 65432

print("🐢 IMPROVED GPS Bridge Starting...")
print(f"📁 GPS file: {GPS_DATA_PATH}")

class GPSBridge:
    def __init__(self):
        self.min_lat = None
        self.max_lat = None
        self.min_lon = None
        self.max_lon = None
        self.center_lat = None
        self.center_lon = None
        
    def analyze_gps_range(self, gps_data):
        """Analyze GPS data to find min/max for better scaling"""
        if not gps_data:
            return
            
        lats = [reading.get('latitude', 0) for reading in gps_data]
        lons = [reading.get('longitude', 0) for reading in gps_data]
        
        self.min_lat = min(lats)
        self.max_lat = max(lats)
        self.min_lon = min(lons)
        self.max_lon = max(lons)
        
        self.center_lat = (self.min_lat + self.max_lat) / 2
        self.center_lon = (self.min_lon + self.max_lon) / 2
        
        lat_range = self.max_lat - self.min_lat
        lon_range = self.max_lon - self.min_lon
        
        print(f"📊 GPS Analysis:")
        print(f"   Latitude range: {self.min_lat:.2e} to {self.max_lat:.2e} (span: {lat_range:.2e})")
        print(f"   Longitude range: {self.min_lon:.2e} to {self.max_lon:.2e} (span: {lon_range:.2e})")
        print(f"   Center: ({self.center_lat:.2e}, {self.center_lon:.2e})")

    def convert_gps_to_gui(self, lat, lon):
        """Convert GPS to GUI coordinates with dynamic scaling"""
        if self.center_lat is None or self.center_lon is None:
            # Fallback to old method
            scale = 1000000000
            gui_x = 50 + (lat * scale)
            gui_y = 50 + (lon * scale)
        else:
            # Use dynamic scaling based on actual GPS range
            # Normalize to -1 to 1, then scale to GUI coordinates
            lat_normalized = (lat - self.center_lat) 
            lon_normalized = (lon - self.center_lon)
            
            # Scale to use most of the GUI space (80% of 100x100 area)
            scale_factor = 20000000  # Adjust this if movement is too small/large
            
            gui_x = 50 + (lat_normalized * scale_factor)
            gui_y = 50 + (lon_normalized * scale_factor)
        
        # Keep within bounds
        gui_x = max(5, min(95, gui_x))
        gui_y = max(5, min(95, gui_y))
        
        return round(gui_x, 2), round(gui_y, 2)

def read_gps_file():
    """Read GPS JSON file"""
    try:
        if not os.path.exists(GPS_DATA_PATH):
            print(f"❌ GPS file not found: {GPS_DATA_PATH}")
            return None
            
        with open(GPS_DATA_PATH, 'r') as file:
            content = file.read().strip()
            
        if content.startswith('['):
            gps_data = json.loads(content)
        else:
            lines = [line.strip().rstrip(',') for line in content.split('\n') if line.strip()]
            gps_data = []
            for line in lines:
                try:
                    if line:
                        gps_data.append(json.loads(line))
                except json.JSONDecodeError:
                    continue
        
        return gps_data
        
    except Exception as e:
        print(f"❌ Error reading GPS file: {e}")
        return None

def connect_to_gui():
    """Connect to GUI"""
    print("🔗 Connecting to GUI...")
    
    attempts = 0
    while attempts < 10:
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.connect((HOST, PORT))
            print("✅ Connected to GUI!")
            return sock
        except ConnectionRefusedError:
            attempts += 1
            print(f"⏳ GUI not ready (attempt {attempts}/10)...")
            time.sleep(1)
        except Exception as e:
            print(f"❌ Connection error: {e}")
            return None
    
    print("❌ Failed to connect to GUI!")
    return None

def send_position(sock, x, y):
    """Send position to GUI"""
    try:
        message = f"{x},{y}"
        sock.sendall(message.encode())
        return True
    except Exception as e:
        print(f"❌ Error sending: {e}")
        return False

def monitor_live_gps(sock):
    """Monitor GPS file for changes with improved scaling"""
    print("\n📡 LIVE GPS MONITORING with Dynamic Scaling")
    print("=" * 50)
    
    bridge = GPSBridge()
    last_size = 0
    last_position = None
    analysis_done = False
    
    print("🔄 Monitoring GPS file for changes...")
    print("🤖 Move your TurtleBot with teleop to see movement!")
    print("⏹️  Press Ctrl+C to stop")
    print()
    
    while True:
        try:
            if os.path.exists(GPS_DATA_PATH):
                current_size = os.path.getsize(GPS_DATA_PATH)
                
                if current_size > last_size:
                    gps_data = read_gps_file()
                    
                    if gps_data and len(gps_data) > 10:  # Need some data for analysis
                        
                        # Analyze GPS range once we have enough data
                        if not analysis_done and len(gps_data) > 50:
                            bridge.analyze_gps_range(gps_data)
                            analysis_done = True
                        
                        # Get latest reading
                        latest = gps_data[-1]
                        lat = latest.get('latitude', 0)
                        lon = latest.get('longitude', 0)
                        
                        # Check if position changed significantly
                        current_position = (lat, lon)
                        if (last_position is None or 
                            abs(lat - last_position[0]) > 1e-12 or 
                            abs(lon - last_position[1]) > 1e-12):
                            
                            gui_x, gui_y = bridge.convert_gps_to_gui(lat, lon)
                            
                            if send_position(sock, gui_x, gui_y):
                                print(f"📍 Position: GPS({lat:.2e}, {lon:.2e}) → GUI({gui_x}, {gui_y}) [Points: {len(gps_data)}]")
                                last_position = current_position
                            else:
                                print("💔 Connection lost!")
                                break
                    
                    last_size = current_size
            
            time.sleep(0.2)  # Check more frequently
            
        except KeyboardInterrupt:
            print("\n⏹️  Monitoring stopped")
            break
        except Exception as e:
            print(f"❌ Error: {e}")
            time.sleep(1)

def main():
    print("🚀 IMPROVED GPS Bridge")
    print("=" * 30)
    
    # Connect to GUI
    sock = connect_to_gui()
    if not sock:
        return
    
    print("\n🎮 Make sure your GUI is in GPS Map mode!")
    print("🤖 Start moving your TurtleBot with teleop!")
    
    try:
        monitor_live_gps(sock)
    except Exception as e:
        print(f"❌ Error: {e}")
    finally:
        if sock:
            sock.close()
        print("👋 Done!")

if __name__ == "__main__":
    main()