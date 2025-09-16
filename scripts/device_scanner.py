#!/usr/bin/env python3

import time
from typing import List, Dict, Callable, Optional
from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from stm32_mavlink_interface.msg import ServoState, EncoderState, RobomasterMotorState


@dataclass
class MAVLinkDevice:
    """Represents a discovered MAVLink device"""
    device_id: int
    device_type: str
    last_seen: float = 0.0
    status: str = "Unknown"


class DeviceScanner(Node):
    """Scanner for discovering MAVLink devices through ROS2 topics"""

    def __init__(self, on_device_found: Optional[Callable] = None, on_device_lost: Optional[Callable] = None):
        super().__init__('mavlink_device_scanner')

        self.on_device_found = on_device_found
        self.on_device_lost = on_device_lost

        self.discovered_devices: Dict[str, MAVLinkDevice] = {}  # Key: "device_type_id"

        # Device timeout (if no messages received)
        self.device_timeout = 5.0  # seconds
        self.heartbeat_timer = self.create_timer(1.0, self.check_device_timeouts)

        # QoS profile for reliable communication (compatible with publisher settings)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )

        # Subscribers to device state topics
        self.servo_state_sub = self.create_subscription(
            ServoState,
            '/servo/states',
            self.servo_state_callback,
            qos_profile
        )

        self.encoder_state_sub = self.create_subscription(
            EncoderState,
            '/encoder/states',
            self.encoder_state_callback,
            qos_profile
        )

        self.motor_state_sub = self.create_subscription(
            RobomasterMotorState,
            '/robomaster/motor_state',
            self.motor_state_callback,
            qos_profile
        )

        self.get_logger().info('Device Scanner initialized - monitoring ROS2 topics')

    def start_scan(self, port: str = None, baud_rate: int = None) -> bool:
        """Start scanning for devices - now just enables monitoring"""
        self.get_logger().info('Device scanning started - monitoring ROS2 topics for active devices')
        return True

    def stop_scan(self):
        """Stop device scanning - clears discovered devices"""
        self.discovered_devices.clear()
        self.get_logger().info('Device scanning stopped - cleared device list')

    def servo_state_callback(self, msg: ServoState):
        """Handle servo state messages to discover servo devices"""
        device_key = f"servo_{msg.servo_id}"
        current_time = time.time()

        status_map = {
            0: "OK",
            1: "NOT_INITIALIZED",
            2: "TIMER_ERROR",
            3: "OUT_OF_RANGE",
            4: "TIMEOUT",
            5: "CONFIG_ERROR"
        }

        status = status_map.get(msg.status, "UNKNOWN")

        if device_key not in self.discovered_devices:
            # New device discovered
            device = MAVLinkDevice(
                device_id=msg.servo_id,
                device_type="servo",
                last_seen=current_time,
                status=status
            )
            self.discovered_devices[device_key] = device

            if self.on_device_found:
                self.on_device_found(device)

            self.get_logger().info(f'Discovered servo device with ID {msg.servo_id}')
        else:
            # Update existing device
            device = self.discovered_devices[device_key]
            device.last_seen = current_time
            device.status = status

    def encoder_state_callback(self, msg: EncoderState):
        """Handle encoder state messages to discover encoder devices"""
        device_key = f"encoder_{msg.encoder_id}"
        current_time = time.time()

        # Simple status mapping based on encoder status field
        status = "OK" if msg.status == 0 else "ERROR"

        if device_key not in self.discovered_devices:
            # New device discovered
            device = MAVLinkDevice(
                device_id=msg.encoder_id,
                device_type="encoder",
                last_seen=current_time,
                status=status
            )
            self.discovered_devices[device_key] = device

            if self.on_device_found:
                self.on_device_found(device)

            self.get_logger().info(f'Discovered encoder device with ID {msg.encoder_id}')
        else:
            # Update existing device
            device = self.discovered_devices[device_key]
            device.last_seen = current_time
            device.status = status

    def motor_state_callback(self, msg: RobomasterMotorState):
        """Handle motor state messages to discover motor devices"""
        device_key = f"motor_{msg.motor_id}"
        current_time = time.time()

        status_map = {
            0: "OK",
            1: "NOT_INITIALIZED",
            2: "CAN_ERROR",
            3: "OUT_OF_RANGE",
            4: "TIMEOUT",
            5: "CONFIG_ERROR",
            6: "OVERHEAT",
            7: "OVERCURRENT"
        }

        status = status_map.get(msg.status, "UNKNOWN")

        if device_key not in self.discovered_devices:
            # New device discovered
            device = MAVLinkDevice(
                device_id=msg.motor_id,
                device_type="motor",
                last_seen=current_time,
                status=status
            )
            self.discovered_devices[device_key] = device

            if self.on_device_found:
                self.on_device_found(device)

            self.get_logger().info(f'Discovered motor device with ID {msg.motor_id}')
        else:
            # Update existing device
            device = self.discovered_devices[device_key]
            device.last_seen = current_time
            device.status = status

    def check_device_timeouts(self):
        """Check for device timeouts and remove inactive devices"""
        current_time = time.time()
        devices_to_remove = []

        for device_key, device in self.discovered_devices.items():
            if current_time - device.last_seen > self.device_timeout:
                devices_to_remove.append(device_key)
                device.status = "Timeout"

                if self.on_device_lost:
                    self.on_device_lost(device)

        for device_key in devices_to_remove:
            device = self.discovered_devices[device_key]
            del self.discovered_devices[device_key]
            self.get_logger().info(f'Device {device.device_type} ID {device.device_id} timed out and removed')

    def get_discovered_devices(self) -> List[MAVLinkDevice]:
        """Get list of currently discovered devices"""
        return list(self.discovered_devices.values())

    def get_device(self, device_type: str, device_id: int) -> Optional[MAVLinkDevice]:
        """Get a specific device by type and ID"""
        device_key = f"{device_type}_{device_id}"
        return self.discovered_devices.get(device_key)

    def is_device_active(self, device_type: str, device_id: int) -> bool:
        """Check if a device is currently active (receiving messages)"""
        device = self.get_device(device_type, device_id)
        if device is None:
            return False

        current_time = time.time()
        return (current_time - device.last_seen) < self.device_timeout


def main():
    """Test function for the device scanner"""
    rclpy.init()

    def on_device_found(device):
        print(f"Found device: {device.device_type} ID {device.device_id} - Status: {device.status}")

    def on_device_lost(device):
        print(f"Lost device: {device.device_type} ID {device.device_id}")

    scanner = DeviceScanner(on_device_found, on_device_lost)

    try:
        # Start monitoring ROS2 topics for devices
        scanner.start_scan()
        print("Device scanner started - monitoring ROS2 topics for active devices...")
        rclpy.spin(scanner)
    except KeyboardInterrupt:
        print("\nStopping device scanner...")
    finally:
        scanner.stop_scan()
        scanner.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()