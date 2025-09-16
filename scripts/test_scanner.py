#!/usr/bin/env python3
"""
Test script for the updated device scanner
"""

import rclpy
import sys
import threading
import time
from device_scanner import DeviceScanner, MAVLinkDevice


def test_device_scanner():
    """Test the device scanner functionality"""
    rclpy.init()

    devices_found = []
    devices_lost = []

    def on_device_found(device: MAVLinkDevice):
        print(f"[FOUND] {device.device_type.upper()} ID {device.device_id} - Status: {device.status}")
        devices_found.append(device)

    def on_device_lost(device: MAVLinkDevice):
        print(f"[LOST] {device.device_type.upper()} ID {device.device_id}")
        devices_lost.append(device)

    # Create scanner
    scanner = DeviceScanner(on_device_found, on_device_lost)

    # Start scanning
    success = scanner.start_scan()
    print(f"Scanner started: {success}")

    if not success:
        print("Failed to start scanner")
        scanner.destroy_node()
        rclpy.shutdown()
        return

    # Run for 10 seconds in a separate thread
    def spin_scanner():
        rclpy.spin(scanner)

    spinner_thread = threading.Thread(target=spin_scanner, daemon=True)
    spinner_thread.start()

    print("Monitoring for devices for 10 seconds...")
    print("Make sure stm32_mavlink_interface is running and publishing device states!")

    # Wait for some time to collect devices
    time.sleep(10)

    # Stop scanning
    scanner.stop_scan()

    # Get current devices
    current_devices = scanner.get_discovered_devices()

    print("\n=== SCAN RESULTS ===")
    print(f"Total devices found during scan: {len(devices_found)}")
    print(f"Currently active devices: {len(current_devices)}")

    if current_devices:
        print("\nActive devices:")
        for device in current_devices:
            active = scanner.is_device_active(device.device_type, device.device_id)
            print(f"  - {device.device_type.upper()} ID {device.device_id}: {device.status} (Active: {active})")
    else:
        print("\nNo active devices found.")
        print("This is expected if stm32_mavlink_interface is not running.")
        print("To test with real devices:")
        print("  1. Start stm32_mavlink_interface: ros2 run stm32_mavlink_interface mavlink_serial_node")
        print("  2. Ensure devices are connected and publishing state messages")

    # Cleanup
    scanner.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        test_device_scanner()
    except KeyboardInterrupt:
        print("\nTest interrupted by user")
    except Exception as e:
        print(f"Test failed with error: {e}")
        sys.exit(1)