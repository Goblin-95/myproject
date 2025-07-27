#!/usr/bin/env python3
"""
Quick test to verify GPS file path
"""
import os

# Test the exact path
gps_path = r"C:\Users\georg\OneDrive\Desktop\TurtleBotGPS\gps_data.json"

print("=== GPS File Path Test ===")
print(f"Looking for: {gps_path}")
print()

# Check if file exists
if os.path.exists(gps_path):
    print("✅ FILE FOUND!")
    
    # Get file info
    file_size = os.path.getsize(gps_path)
    print(f"📏 File size: {file_size} bytes")
    
    # Try to read first few lines
    try:
        with open(gps_path, 'r') as f:
            content = f.read(500)  # Read first 500 characters
            print("📖 First part of file:")
            print("-" * 40)
            print(content)
            print("-" * 40)
    except Exception as e:
        print(f"❌ Error reading file: {e}")
        
else:
    print("❌ FILE NOT FOUND!")
    print()
    print("Let's check what IS in your Desktop folder:")
    
    desktop_path = r"C:\Users\georg\OneDrive\Desktop"
    if os.path.exists(desktop_path):
        print(f"📁 Contents of {desktop_path}:")
        for item in os.listdir(desktop_path):
            item_path = os.path.join(desktop_path, item)
            if os.path.isdir(item_path):
                print(f"  📁 {item}/ (folder)")
            else:
                print(f"  📄 {item} (file)")
    else:
        print("❌ Desktop folder not found either!")

print("\n=== Test Complete ===")