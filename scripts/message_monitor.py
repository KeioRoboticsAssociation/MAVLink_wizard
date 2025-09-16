#!/usr/bin/env python3

import time
import threading
from typing import Dict, List, Any, Optional, Callable
from dataclasses import dataclass, field
from collections import deque
from enum import Enum

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from stm32_mavlink_interface.msg import ServoState, EncoderState, RobomasterMotorState


class MessageType(Enum):
    """Types of messages that can be monitored"""
    SERVO_STATE = "servo_state"
    ENCODER_STATE = "encoder_state"
    MOTOR_STATE = "motor_state"
    HEARTBEAT = "heartbeat"
    PARAMETER = "parameter"
    COMMAND = "command"


@dataclass
class MessageData:
    """Container for message data with timestamp"""
    timestamp: float
    message_type: MessageType
    device_id: int
    data: Dict[str, Any]
    raw_message: Any = None


@dataclass
class DeviceStatistics:
    """Statistics for a monitored device"""
    device_id: int
    device_type: str
    message_count: int = 0
    last_message_time: float = 0.0
    average_frequency: float = 0.0
    min_frequency: float = float('inf')
    max_frequency: float = 0.0
    error_count: int = 0
    status: str = "Unknown"
    uptime: float = 0.0
    message_history: deque = field(default_factory=lambda: deque(maxlen=1000))


class MessageMonitor(Node):
    """Real-time monitor for MAVLink messages and device states"""

    def __init__(self, max_history_size: int = 10000):
        super().__init__('mavlink_message_monitor')

        self.max_history_size = max_history_size
        self.message_history: deque = deque(maxlen=max_history_size)
        self.device_statistics: Dict[int, DeviceStatistics] = {}

        # Monitoring control
        self.monitoring_enabled = False
        self.recording_enabled = False
        self.recording_file = None

        # Callbacks for real-time updates
        self.on_message_received: Optional[Callable] = None
        self.on_device_status_changed: Optional[Callable] = None
        self.on_statistics_updated: Optional[Callable] = None

        # Message filters
        self.filtered_message_types: List[MessageType] = []
        self.filtered_device_ids: List[int] = []

        # QoS profile for reliable monitoring
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )

        # Subscribers for different message types
        self.servo_state_sub = self.create_subscription(
            ServoState, '/servo/states',
            lambda msg: self._handle_servo_state(msg),
            qos_profile
        )

        self.encoder_state_sub = self.create_subscription(
            EncoderState, '/encoder/states',
            lambda msg: self._handle_encoder_state(msg),
            qos_profile
        )

        self.motor_state_sub = self.create_subscription(
            RobomasterMotorState, '/robomaster/motor_state',
            lambda msg: self._handle_motor_state(msg),
            qos_profile
        )

        # Statistics update timer
        self.stats_timer = self.create_timer(1.0, self._update_statistics)

        # Cleanup timer for old data
        self.cleanup_timer = self.create_timer(10.0, self._cleanup_old_data)

        self.get_logger().info('Message Monitor initialized')

    def start_monitoring(self):
        """Start message monitoring"""
        self.monitoring_enabled = True
        self.get_logger().info('Message monitoring started')

    def stop_monitoring(self):
        """Stop message monitoring"""
        self.monitoring_enabled = False
        if self.recording_enabled:
            self.stop_recording()
        self.get_logger().info('Message monitoring stopped')

    def start_recording(self, filename: str):
        """Start recording messages to file"""
        try:
            self.recording_file = open(filename, 'w')
            self.recording_enabled = True
            self.recording_file.write("# MAVLink Message Recording\n")
            self.recording_file.write("# Timestamp, MessageType, DeviceID, Data\n")
            self.get_logger().info(f'Started recording to {filename}')
        except Exception as e:
            self.get_logger().error(f'Failed to start recording: {str(e)}')

    def stop_recording(self):
        """Stop recording messages"""
        if self.recording_file:
            self.recording_file.close()
            self.recording_file = None
        self.recording_enabled = False
        self.get_logger().info('Stopped recording')

    def clear_history(self):
        """Clear message history and statistics"""
        self.message_history.clear()
        for stats in self.device_statistics.values():
            stats.message_history.clear()
            stats.message_count = 0
            stats.error_count = 0
        self.get_logger().info('Cleared message history and statistics')

    def set_message_filter(self, message_types: List[MessageType]):
        """Set filter for message types to monitor"""
        self.filtered_message_types = message_types
        self.get_logger().info(f'Set message filter: {[t.value for t in message_types]}')

    def set_device_filter(self, device_ids: List[int]):
        """Set filter for device IDs to monitor"""
        self.filtered_device_ids = device_ids
        self.get_logger().info(f'Set device filter: {device_ids}')

    def get_device_statistics(self, device_id: int) -> Optional[DeviceStatistics]:
        """Get statistics for a specific device"""
        return self.device_statistics.get(device_id)

    def get_all_statistics(self) -> Dict[int, DeviceStatistics]:
        """Get statistics for all monitored devices"""
        return self.device_statistics.copy()

    def get_recent_messages(self, count: int = 100) -> List[MessageData]:
        """Get the most recent messages"""
        return list(self.message_history)[-count:]

    def get_messages_by_device(self, device_id: int, count: int = 100) -> List[MessageData]:
        """Get recent messages for a specific device"""
        if device_id in self.device_statistics:
            return list(self.device_statistics[device_id].message_history)[-count:]
        return []

    def get_messages_by_type(self, message_type: MessageType, count: int = 100) -> List[MessageData]:
        """Get recent messages of a specific type"""
        filtered_messages = [
            msg for msg in self.message_history
            if msg.message_type == message_type
        ]
        return filtered_messages[-count:]

    def _handle_servo_state(self, msg):
        """Handle servo state message"""
        if not self.monitoring_enabled:
            return

        device_id = msg.servo_id
        message_type = MessageType.SERVO_STATE

        if self._should_filter_message(message_type, device_id):
            return

        data = {
            'position': msg.present_position,
            'velocity': msg.present_velocity,
            'current': msg.present_current,
            'voltage': msg.present_voltage,
            'temperature': msg.present_temperature,
            'moving': msg.moving,
            'error': msg.hardware_error_status
        }

        self._process_message(message_type, device_id, data, msg)

    def _handle_encoder_state(self, msg):
        """Handle encoder state message"""
        if not self.monitoring_enabled:
            return

        device_id = msg.encoder_id
        message_type = MessageType.ENCODER_STATE

        if self._should_filter_message(message_type, device_id):
            return

        data = {
            'position': msg.position,
            'angle_rad': msg.angle_rad,
            'angle_deg': msg.angle_deg,
            'revolutions': msg.revolutions,
            'z_detected': msg.z_detected,
            'status': msg.status,
            'error_count': msg.error_count
        }

        self._process_message(message_type, device_id, data, msg)

    def _handle_motor_state(self, msg):
        """Handle motor state message"""
        if not self.monitoring_enabled:
            return

        device_id = msg.motor_id
        message_type = MessageType.MOTOR_STATE

        if self._should_filter_message(message_type, device_id):
            return

        data = {
            'angle': msg.angle,
            'velocity': msg.velocity,
            'current': msg.current,
            'temperature': msg.temperature,
            'error_flags': msg.error_flags
        }

        self._process_message(message_type, device_id, data, msg)

    def _should_filter_message(self, message_type: MessageType, device_id: int) -> bool:
        """Check if message should be filtered out"""
        if self.filtered_message_types and message_type not in self.filtered_message_types:
            return True

        if self.filtered_device_ids and device_id not in self.filtered_device_ids:
            return True

        return False

    def _process_message(self, message_type: MessageType, device_id: int, data: Dict[str, Any], raw_message: Any):
        """Process a received message"""
        timestamp = time.time()

        # Create message data object
        message_data = MessageData(
            timestamp=timestamp,
            message_type=message_type,
            device_id=device_id,
            data=data,
            raw_message=raw_message
        )

        # Add to global history
        self.message_history.append(message_data)

        # Update device statistics
        self._update_device_statistics(device_id, message_type, timestamp, data)

        # Record to file if enabled
        if self.recording_enabled and self.recording_file:
            self._record_message(message_data)

        # Notify callbacks
        if self.on_message_received:
            self.on_message_received(message_data)

    def _update_device_statistics(self, device_id: int, message_type: MessageType, timestamp: float, data: Dict[str, Any]):
        """Update statistics for a device"""
        if device_id not in self.device_statistics:
            device_type = self._determine_device_type(message_type)
            self.device_statistics[device_id] = DeviceStatistics(
                device_id=device_id,
                device_type=device_type
            )

        stats = self.device_statistics[device_id]

        # Update message count and timing
        stats.message_count += 1
        time_diff = timestamp - stats.last_message_time if stats.last_message_time > 0 else 0
        stats.last_message_time = timestamp

        # Calculate frequency
        if time_diff > 0:
            frequency = 1.0 / time_diff
            stats.max_frequency = max(stats.max_frequency, frequency)
            stats.min_frequency = min(stats.min_frequency, frequency)

            # Running average frequency
            alpha = 0.1  # Smoothing factor
            if stats.average_frequency == 0:
                stats.average_frequency = frequency
            else:
                stats.average_frequency = alpha * frequency + (1 - alpha) * stats.average_frequency

        # Check for errors
        if 'error' in data and data['error']:
            stats.error_count += 1

        # Update status based on data
        stats.status = self._determine_device_status(data)

        # Add message to device history
        message_data = MessageData(
            timestamp=timestamp,
            message_type=message_type,
            device_id=device_id,
            data=data
        )
        stats.message_history.append(message_data)

    def _determine_device_type(self, message_type: MessageType) -> str:
        """Determine device type from message type"""
        if message_type == MessageType.SERVO_STATE:
            return "servo"
        elif message_type == MessageType.ENCODER_STATE:
            return "encoder"
        elif message_type == MessageType.MOTOR_STATE:
            return "motor"
        else:
            return "unknown"

    def _determine_device_status(self, data: Dict[str, Any]) -> str:
        """Determine device status from message data"""
        if 'error' in data and data['error']:
            return "Error"
        elif 'moving' in data and data['moving']:
            return "Moving"
        elif 'z_detected' in data and data['z_detected']:
            return "Index Detected"
        else:
            return "OK"

    def _record_message(self, message_data: MessageData):
        """Record a message to file"""
        try:
            record_line = f"{message_data.timestamp},{message_data.message_type.value},{message_data.device_id},{message_data.data}\n"
            self.recording_file.write(record_line)
            self.recording_file.flush()
        except Exception as e:
            self.get_logger().error(f'Failed to record message: {str(e)}')

    def _update_statistics(self):
        """Periodic statistics update"""
        current_time = time.time()

        for device_id, stats in self.device_statistics.items():
            # Update uptime
            if stats.last_message_time > 0:
                stats.uptime = current_time - stats.last_message_time

            # Check for timeout (no messages for 5 seconds)
            if current_time - stats.last_message_time > 5.0:
                if stats.status != "Timeout":
                    stats.status = "Timeout"
                    if self.on_device_status_changed:
                        self.on_device_status_changed(device_id, "Timeout")

        # Notify statistics update
        if self.on_statistics_updated:
            self.on_statistics_updated(self.device_statistics.copy())

    def _cleanup_old_data(self):
        """Clean up old data to prevent memory issues"""
        current_time = time.time()
        cleanup_threshold = 300.0  # 5 minutes

        # Clean up global history
        while (self.message_history and
               current_time - self.message_history[0].timestamp > cleanup_threshold):
            self.message_history.popleft()

        # Clean up device histories
        for stats in self.device_statistics.values():
            while (stats.message_history and
                   current_time - stats.message_history[0].timestamp > cleanup_threshold):
                stats.message_history.popleft()

    def export_statistics(self, filename: str) -> bool:
        """Export statistics to a JSON file"""
        try:
            import json

            export_data = {
                'timestamp': time.time(),
                'total_messages': len(self.message_history),
                'devices': {}
            }

            for device_id, stats in self.device_statistics.items():
                export_data['devices'][str(device_id)] = {
                    'device_type': stats.device_type,
                    'message_count': stats.message_count,
                    'last_message_time': stats.last_message_time,
                    'average_frequency': stats.average_frequency,
                    'min_frequency': stats.min_frequency if stats.min_frequency != float('inf') else 0,
                    'max_frequency': stats.max_frequency,
                    'error_count': stats.error_count,
                    'status': stats.status,
                    'uptime': stats.uptime
                }

            with open(filename, 'w') as f:
                json.dump(export_data, f, indent=2)

            self.get_logger().info(f'Exported statistics to {filename}')
            return True

        except Exception as e:
            self.get_logger().error(f'Failed to export statistics: {str(e)}')
            return False

    def get_frequency_statistics(self, device_id: int, window_seconds: int = 60) -> Dict[str, float]:
        """Get frequency statistics for a device over a time window"""
        if device_id not in self.device_statistics:
            return {}

        stats = self.device_statistics[device_id]
        current_time = time.time()
        window_start = current_time - window_seconds

        # Get messages in the time window
        window_messages = [
            msg for msg in stats.message_history
            if msg.timestamp >= window_start
        ]

        if len(window_messages) < 2:
            return {'frequency': 0.0, 'message_count': len(window_messages)}

        # Calculate frequency
        time_span = window_messages[-1].timestamp - window_messages[0].timestamp
        frequency = (len(window_messages) - 1) / time_span if time_span > 0 else 0

        return {
            'frequency': frequency,
            'message_count': len(window_messages),
            'time_span': time_span
        }

    def get_message_rate_history(self, device_id: int, bins: int = 60) -> List[float]:
        """Get message rate history for plotting"""
        if device_id not in self.device_statistics:
            return []

        stats = self.device_statistics[device_id]
        current_time = time.time()
        bin_duration = 60.0 / bins  # 1 minute divided into bins

        rates = []
        for i in range(bins):
            bin_start = current_time - (bins - i) * bin_duration
            bin_end = bin_start + bin_duration

            bin_messages = [
                msg for msg in stats.message_history
                if bin_start <= msg.timestamp < bin_end
            ]

            rate = len(bin_messages) / bin_duration
            rates.append(rate)

        return rates


def main():
    """Test function for the message monitor"""
    rclpy.init()

    def on_message(message_data):
        print(f"Received {message_data.message_type.value} from device {message_data.device_id}")

    def on_status_change(device_id, status):
        print(f"Device {device_id} status changed to {status}")

    monitor = MessageMonitor()
    monitor.on_message_received = on_message
    monitor.on_device_status_changed = on_status_change

    try:
        monitor.start_monitoring()
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        pass
    finally:
        monitor.stop_monitoring()
        monitor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()