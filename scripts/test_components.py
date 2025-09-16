#!/usr/bin/env python3

"""
Test script for MAVLink Wizard components
This script tests the basic functionality of all modules without requiring hardware
"""

import sys
import os
import time
import threading
from unittest.mock import Mock, patch

# Add the scripts directory to the path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

try:
    import rclpy
    from rclpy.node import Node
except ImportError:
    print("ROS2 not available, running in standalone mode")
    rclpy = None

def test_parameter_manager():
    """Test parameter manager functionality"""
    print("Testing Parameter Manager...")

    if rclpy:
        rclpy.init()

    try:
        from parameter_manager import ParameterManager, ParameterType, Parameter

        # Create parameter manager
        if rclpy:
            param_manager = ParameterManager()
        else:
            # Mock the Node class for testing without ROS2
            with patch('parameter_manager.Node'):
                param_manager = ParameterManager()

        # Test device creation
        device_params = param_manager.create_device_parameters(1, 'servo')
        assert device_params is not None, "Failed to create device parameters"

        # Test parameter update
        result = param_manager.update_parameter(1, 'SPEED', 200)
        assert result, "Failed to update parameter"

        # Test parameter validation
        result = param_manager.validate_parameter_value('servo', 'SPEED', 500)
        assert result, "Parameter validation failed"

        # Test invalid parameter
        result = param_manager.validate_parameter_value('servo', 'SPEED', 2000)
        assert not result, "Invalid parameter should not validate"

        print("✓ Parameter Manager tests passed")

    except Exception as e:
        print(f"✗ Parameter Manager test failed: {e}")
        return False

    finally:
        if rclpy and rclpy.ok():
            rclpy.shutdown()

    return True


def test_device_scanner():
    """Test device scanner functionality"""
    print("Testing Device Scanner...")

    if rclpy:
        rclpy.init()

    try:
        from device_scanner import DeviceScanner, MAVLinkDevice

        devices_found = []
        devices_lost = []

        def on_device_found(device):
            devices_found.append(device)

        def on_device_lost(device):
            devices_lost.append(device)

        # Create scanner
        if rclpy:
            scanner = DeviceScanner(on_device_found, on_device_lost)
        else:
            with patch('device_scanner.Node'):
                scanner = DeviceScanner(on_device_found, on_device_lost)

        # Test MAVLink device creation
        device = MAVLinkDevice(
            device_id=1,
            device_type="servo",
            system_id=1,
            component_id=140
        )

        assert device.device_id == 1, "Device ID mismatch"
        assert device.device_type == "servo", "Device type mismatch"

        # Test checksum calculation
        if hasattr(scanner, '_calculate_checksum'):
            checksum = scanner._calculate_checksum(0, b'\x00\x01\x02')
            assert isinstance(checksum, bytes), "Checksum should be bytes"

        print("✓ Device Scanner tests passed")

    except Exception as e:
        print(f"✗ Device Scanner test failed: {e}")
        return False

    finally:
        if rclpy and rclpy.ok():
            rclpy.shutdown()

    return True


def test_message_monitor():
    """Test message monitor functionality"""
    print("Testing Message Monitor...")

    if rclpy:
        rclpy.init()

    try:
        from message_monitor import MessageMonitor, MessageType, MessageData, DeviceStatistics

        messages_received = []

        def on_message(message_data):
            messages_received.append(message_data)

        # Create monitor
        if rclpy:
            monitor = MessageMonitor()
        else:
            with patch('message_monitor.Node'):
                monitor = MessageMonitor()

        monitor.on_message_received = on_message

        # Test message data creation
        message_data = MessageData(
            timestamp=time.time(),
            message_type=MessageType.SERVO_STATE,
            device_id=1,
            data={'position': 45.0, 'velocity': 10.0}
        )

        assert message_data.device_id == 1, "Message device ID mismatch"
        assert message_data.message_type == MessageType.SERVO_STATE, "Message type mismatch"

        # Test device statistics
        stats = DeviceStatistics(device_id=1, device_type="servo")
        assert stats.device_id == 1, "Statistics device ID mismatch"

        # Test filtering
        monitor.set_message_filter([MessageType.SERVO_STATE])
        monitor.set_device_filter([1, 2])

        print("✓ Message Monitor tests passed")

    except Exception as e:
        print(f"✗ Message Monitor test failed: {e}")
        return False

    finally:
        if rclpy and rclpy.ok():
            rclpy.shutdown()

    return True


def test_gui_components():
    """Test GUI components (without actually showing windows)"""
    print("Testing GUI Components...")

    try:
        # Test PyQt5 import
        from PyQt5.QtWidgets import QApplication
        from PyQt5.QtCore import QTimer

        # Create minimal application
        app = QApplication([])

        # Test widget imports
        import mavlink_wizard_gui
        from mavlink_wizard_gui import DeviceTreeWidget, ParameterConfigWidget, MonitorWidget

        # Create widgets without showing them
        device_tree = DeviceTreeWidget()
        config_widget = ParameterConfigWidget()
        monitor_widget = MonitorWidget()

        # Test basic functionality
        device_tree.add_device('servo', 1, 'Connected')
        config_widget.set_device('servo', 1)

        # Cleanup
        app.quit()

        print("✓ GUI Components tests passed")

    except ImportError as e:
        print(f"✗ GUI Components test skipped (PyQt5 not available): {e}")
        return True  # Don't fail if GUI libraries aren't available
    except Exception as e:
        print(f"✗ GUI Components test failed: {e}")
        return False

    return True


def run_all_tests():
    """Run all component tests"""
    print("MAVLink Wizard Component Tests")
    print("=" * 40)

    tests = [
        test_parameter_manager,
        test_device_scanner,
        test_message_monitor,
        test_gui_components
    ]

    results = []
    for test in tests:
        try:
            result = test()
            results.append(result)
        except Exception as e:
            print(f"✗ Test {test.__name__} crashed: {e}")
            results.append(False)
        print()

    # Summary
    passed = sum(results)
    total = len(results)

    print("=" * 40)
    print(f"Test Results: {passed}/{total} passed")

    if passed == total:
        print("✓ All tests passed!")
        return 0
    else:
        print("✗ Some tests failed!")
        return 1


if __name__ == '__main__':
    exit_code = run_all_tests()
    sys.exit(exit_code)