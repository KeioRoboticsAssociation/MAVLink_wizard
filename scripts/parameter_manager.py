#!/usr/bin/env python3

import json
import time
import logging
from typing import Dict, List, Any, Optional, Callable, Protocol
from dataclasses import dataclass, asdict
from enum import Enum
from abc import ABC, abstractmethod

import rclpy
from rclpy.node import Node

# Setup logging
logger = logging.getLogger(__name__)
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from stm32_mavlink_interface.msg import RobomasterMotorConfig
from stm32_mavlink_interface.srv import (
    SetServoConfig, SetEncoderConfig,
    SetRobomasterMotorConfig, GetRobomasterMotorConfig,
    SetDCMotorConfig, GetDCMotorConfig
)


class ParameterType(Enum):
    """MAVLink parameter types"""
    UINT8 = 1
    INT8 = 2
    UINT16 = 3
    INT16 = 4
    UINT32 = 5
    INT32 = 6
    UINT64 = 7
    INT64 = 8
    REAL32 = 9
    REAL64 = 10


@dataclass
class Parameter:
    """Represents a device parameter"""
    name: str
    value: Any
    param_type: ParameterType
    description: str = ""
    min_value: Optional[float] = None
    max_value: Optional[float] = None
    default_value: Optional[Any] = None
    read_only: bool = False
    unit: str = ""


@dataclass
class DeviceParameters:
    """Collection of parameters for a device"""
    device_id: int
    device_type: str
    parameters: Dict[str, Parameter]
    last_updated: float
    version: str = "1.0"


class ParameterManager(Node):
    """Manages parameters for MAVLink devices via ROS2 services"""

    def __init__(self):
        super().__init__('mavlink_parameter_manager')

        self.device_parameters: Dict[str, DeviceParameters] = {}  # Key: "device_type_id"

        # Callbacks for parameter events
        self.on_parameter_updated: Optional[Callable] = None
        self.on_parameter_read_complete: Optional[Callable] = None

        # ROS2 service clients for configuration
        self.servo_config_client = self.create_client(SetServoConfig, '/servo/set_config')
        self.encoder_config_client = self.create_client(SetEncoderConfig, '/encoder/set_config')
        self.motor_config_set_client = self.create_client(SetRobomasterMotorConfig, '/robomaster/set_motor_config')
        self.motor_config_get_client = self.create_client(GetRobomasterMotorConfig, '/robomaster/get_motor_config')

        # Parameter definitions for different device types
        self.parameter_definitions = self._load_parameter_definitions()

        self.get_logger().info('Parameter Manager initialized with ROS2 service clients')

    def _load_parameter_definitions(self) -> Dict[str, Dict[str, Parameter]]:
        """Load parameter definitions for different device types"""
        definitions = {
            'servo': {
                'ID': Parameter(
                    name='ID',
                    value=1,
                    param_type=ParameterType.UINT8,
                    description='Device ID',
                    min_value=1,
                    max_value=254,
                    default_value=1
                ),
                'MIN_ANGLE': Parameter(
                    name='MIN_ANGLE',
                    value=-180.0,
                    param_type=ParameterType.REAL32,
                    description='Minimum angle in degrees',
                    min_value=-180.0,
                    max_value=180.0,
                    default_value=-180.0,
                    unit='degrees'
                ),
                'MAX_ANGLE': Parameter(
                    name='MAX_ANGLE',
                    value=180.0,
                    param_type=ParameterType.REAL32,
                    description='Maximum angle in degrees',
                    min_value=-180.0,
                    max_value=180.0,
                    default_value=180.0,
                    unit='degrees'
                ),
                'SPEED': Parameter(
                    name='SPEED',
                    value=100,
                    param_type=ParameterType.UINT16,
                    description='Movement speed',
                    min_value=1,
                    max_value=1000,
                    default_value=100,
                    unit='units/sec'
                ),
                'TORQUE_LIMIT': Parameter(
                    name='TORQUE_LIMIT',
                    value=50,
                    param_type=ParameterType.UINT8,
                    description='Torque limit percentage',
                    min_value=0,
                    max_value=100,
                    default_value=50,
                    unit='%'
                ),
                'RETURN_DELAY': Parameter(
                    name='RETURN_DELAY',
                    value=2,
                    param_type=ParameterType.UINT8,
                    description='Return delay time',
                    min_value=0,
                    max_value=254,
                    default_value=2,
                    unit='x2 microseconds'
                )
            },
            'encoder': {
                'ID': Parameter(
                    name='ID',
                    value=1,
                    param_type=ParameterType.UINT8,
                    description='Device ID',
                    min_value=1,
                    max_value=254,
                    default_value=1
                ),
                'CPR': Parameter(
                    name='CPR',
                    value=1024,
                    param_type=ParameterType.UINT16,
                    description='Counts per revolution',
                    min_value=1,
                    max_value=10000,
                    default_value=1024,
                    unit='counts'
                ),
                'INVERT_A': Parameter(
                    name='INVERT_A',
                    value=0,
                    param_type=ParameterType.UINT8,
                    description='Invert channel A',
                    min_value=0,
                    max_value=1,
                    default_value=0
                ),
                'INVERT_B': Parameter(
                    name='INVERT_B',
                    value=0,
                    param_type=ParameterType.UINT8,
                    description='Invert channel B',
                    min_value=0,
                    max_value=1,
                    default_value=0
                ),
                'USE_INDEX': Parameter(
                    name='USE_INDEX',
                    value=0,
                    param_type=ParameterType.UINT8,
                    description='Use index channel for zero position',
                    min_value=0,
                    max_value=1,
                    default_value=0
                ),
                'FILTER_TIME': Parameter(
                    name='FILTER_TIME',
                    value=10,
                    param_type=ParameterType.UINT8,
                    description='Digital filter time constant',
                    min_value=0,
                    max_value=255,
                    default_value=10,
                    unit='ms'
                )
            },
            'motor': {
                'ID': Parameter(
                    name='ID',
                    value=1,
                    param_type=ParameterType.UINT8,
                    description='Device ID',
                    min_value=1,
                    max_value=8,
                    default_value=1
                ),
                'MAX_VELOCITY_RPS': Parameter(
                    name='MAX_VELOCITY_RPS',
                    value=10.0,
                    param_type=ParameterType.REAL32,
                    description='Maximum velocity in rotations per second',
                    min_value=0.1,
                    max_value=100.0,
                    default_value=10.0,
                    unit='RPS'
                ),
                'MAX_CURRENT_MA': Parameter(
                    name='MAX_CURRENT_MA',
                    value=5000,
                    param_type=ParameterType.INT16,
                    description='Maximum current in milliamps',
                    min_value=100,
                    max_value=25000,
                    default_value=5000,
                    unit='mA'
                ),
                'POSITION_KP': Parameter(
                    name='POSITION_KP',
                    value=10.0,
                    param_type=ParameterType.REAL32,
                    description='Position P gain',
                    min_value=0.0,
                    max_value=1000.0,
                    default_value=10.0
                ),
                'POSITION_KI': Parameter(
                    name='POSITION_KI',
                    value=0.0,
                    param_type=ParameterType.REAL32,
                    description='Position I gain',
                    min_value=0.0,
                    max_value=1000.0,
                    default_value=0.0
                ),
                'POSITION_KD': Parameter(
                    name='POSITION_KD',
                    value=0.0,
                    param_type=ParameterType.REAL32,
                    description='Position D gain',
                    min_value=0.0,
                    max_value=1000.0,
                    default_value=0.0
                ),
                'VELOCITY_KP': Parameter(
                    name='VELOCITY_KP',
                    value=2000.0,
                    param_type=ParameterType.REAL32,
                    description='Velocity P gain',
                    min_value=0.0,
                    max_value=10000.0,
                    default_value=2000.0
                ),
                'VELOCITY_KI': Parameter(
                    name='VELOCITY_KI',
                    value=0.0,
                    param_type=ParameterType.REAL32,
                    description='Velocity I gain',
                    min_value=0.0,
                    max_value=10000.0,
                    default_value=0.0
                ),
                'VELOCITY_KD': Parameter(
                    name='VELOCITY_KD',
                    value=0.0,
                    param_type=ParameterType.REAL32,
                    description='Velocity D gain',
                    min_value=0.0,
                    max_value=10000.0,
                    default_value=0.0
                ),
                'MAX_TEMPERATURE': Parameter(
                    name='MAX_TEMPERATURE',
                    value=80,
                    param_type=ParameterType.UINT8,
                    description='Maximum temperature in Celsius',
                    min_value=40,
                    max_value=100,
                    default_value=80,
                    unit='°C'
                ),
                'WATCHDOG_TIMEOUT': Parameter(
                    name='WATCHDOG_TIMEOUT',
                    value=3000,
                    param_type=ParameterType.UINT32,
                    description='Watchdog timeout in milliseconds',
                    min_value=100,
                    max_value=10000,
                    default_value=3000,
                    unit='ms'
                )
            },
            'dcmotor': {
                'ID': Parameter(
                    name='ID',
                    value=10,
                    param_type=ParameterType.UINT8,
                    description='DC Motor ID',
                    min_value=1,
                    max_value=20,
                    default_value=10
                ),
                'CONTROL_MODE': Parameter(
                    name='CONTROL_MODE',
                    value=0,
                    param_type=ParameterType.UINT8,
                    description='Control mode (0: position, 1: velocity, 2: current)',
                    min_value=0,
                    max_value=2,
                    default_value=0
                ),
                'SPEED_KP': Parameter(
                    name='SPEED_KP',
                    value=0.8,
                    param_type=ParameterType.REAL32,
                    description='Speed control P gain',
                    min_value=0.0,
                    max_value=100.0,
                    default_value=0.8
                ),
                'SPEED_KI': Parameter(
                    name='SPEED_KI',
                    value=0.0,
                    param_type=ParameterType.REAL32,
                    description='Speed control I gain',
                    min_value=0.0,
                    max_value=100.0,
                    default_value=0.0
                ),
                'SPEED_KD': Parameter(
                    name='SPEED_KD',
                    value=0.0,
                    param_type=ParameterType.REAL32,
                    description='Speed control D gain',
                    min_value=0.0,
                    max_value=100.0,
                    default_value=0.0
                ),
                'POSITION_KP': Parameter(
                    name='POSITION_KP',
                    value=3.0,
                    param_type=ParameterType.REAL32,
                    description='Position control P gain',
                    min_value=0.0,
                    max_value=100.0,
                    default_value=3.0
                ),
                'POSITION_KI': Parameter(
                    name='POSITION_KI',
                    value=0.0,
                    param_type=ParameterType.REAL32,
                    description='Position control I gain',
                    min_value=0.0,
                    max_value=100.0,
                    default_value=0.0
                ),
                'POSITION_KD': Parameter(
                    name='POSITION_KD',
                    value=0.0,
                    param_type=ParameterType.REAL32,
                    description='Position control D gain',
                    min_value=0.0,
                    max_value=100.0,
                    default_value=0.0
                ),
                'MAX_SPEED': Parameter(
                    name='MAX_SPEED',
                    value=15.0,
                    param_type=ParameterType.REAL32,
                    description='Maximum speed in rad/s',
                    min_value=0.1,
                    max_value=100.0,
                    default_value=15.0,
                    unit='rad/s'
                ),
                'MAX_ACCELERATION': Parameter(
                    name='MAX_ACCELERATION',
                    value=50.0,
                    param_type=ParameterType.REAL32,
                    description='Maximum acceleration in rad/s²',
                    min_value=1.0,
                    max_value=500.0,
                    default_value=50.0,
                    unit='rad/s²'
                ),
                'POSITION_LIMITS': Parameter(
                    name='POSITION_LIMITS',
                    value=0,
                    param_type=ParameterType.UINT8,
                    description='Use position limits (0: disabled, 1: enabled)',
                    min_value=0,
                    max_value=1,
                    default_value=0
                ),
                'MIN_POSITION': Parameter(
                    name='MIN_POSITION',
                    value=-3.14159,
                    param_type=ParameterType.REAL32,
                    description='Minimum position limit in radians',
                    min_value=-6.28318,
                    max_value=6.28318,
                    default_value=-3.14159,
                    unit='rad'
                ),
                'MAX_POSITION': Parameter(
                    name='MAX_POSITION',
                    value=3.14159,
                    param_type=ParameterType.REAL32,
                    description='Maximum position limit in radians',
                    min_value=-6.28318,
                    max_value=6.28318,
                    default_value=3.14159,
                    unit='rad'
                ),
                'WATCHDOG_TIMEOUT': Parameter(
                    name='WATCHDOG_TIMEOUT',
                    value=1000,
                    param_type=ParameterType.UINT32,
                    description='Watchdog timeout in milliseconds',
                    min_value=100,
                    max_value=10000,
                    default_value=1000,
                    unit='ms'
                )
            }
        }

        return definitions

    def get_device_parameters(self, device_type: str, device_id: int) -> Optional[DeviceParameters]:
        """Get parameters for a specific device"""
        device_key = f"{device_type}_{device_id}"
        return self.device_parameters.get(device_key)

    def create_device_parameters(self, device_type: str, device_id: int) -> DeviceParameters:
        """Create parameter set for a new device"""
        if device_type not in self.parameter_definitions:
            self.get_logger().warning(f'Unknown device type: {device_type}')
            return None

        device_key = f"{device_type}_{device_id}"

        # Copy parameter definitions for this device type
        parameters = {}
        for name, param_def in self.parameter_definitions[device_type].items():
            # Create a copy of the parameter with default values
            param_copy = Parameter(
                name=param_def.name,
                value=param_def.default_value,
                param_type=param_def.param_type,
                description=param_def.description,
                min_value=param_def.min_value,
                max_value=param_def.max_value,
                default_value=param_def.default_value,
                read_only=param_def.read_only,
                unit=param_def.unit
            )
            parameters[name] = param_copy

        device_params = DeviceParameters(
            device_id=device_id,
            device_type=device_type,
            parameters=parameters,
            last_updated=time.time()
        )

        self.device_parameters[device_key] = device_params
        self.get_logger().info(f'Created parameter set for {device_type} device {device_id}')

        return device_params

    def update_parameter(self, device_type: str, device_id: int, param_name: str, value: Any) -> bool:
        """Update a parameter value using ROS2 services"""
        device_key = f"{device_type}_{device_id}"

        if device_key not in self.device_parameters:
            self.get_logger().error(f'Device {device_type} {device_id} not found')
            return False

        device_params = self.device_parameters[device_key]

        if param_name not in device_params.parameters:
            self.get_logger().error(f'Parameter {param_name} not found for device {device_type} {device_id}')
            return False

        parameter = device_params.parameters[param_name]

        if parameter.read_only:
            self.get_logger().error(f'Parameter {param_name} is read-only')
            return False

        # Validate value range
        if parameter.min_value is not None and value < parameter.min_value:
            self.get_logger().error(f'Value {value} below minimum {parameter.min_value} for {param_name}')
            return False

        if parameter.max_value is not None and value > parameter.max_value:
            self.get_logger().error(f'Value {value} above maximum {parameter.max_value} for {param_name}')
            return False

        # Update the parameter locally
        old_value = parameter.value
        parameter.value = value
        device_params.last_updated = time.time()

        self.get_logger().info(f'Updated {param_name} from {old_value} to {value} for {device_type} {device_id}')

        # Send parameter to device via appropriate service
        success = self._send_parameter_to_device(device_type, device_id, param_name, value)

        if not success:
            # Revert the change if sending failed
            parameter.value = old_value
            return False

        # Notify callbacks
        if self.on_parameter_updated:
            self.on_parameter_updated(device_id, param_name, value, old_value)

        return True

    def _send_parameter_to_device(self, device_type: str, device_id: int, param_name: str, value: Any) -> bool:
        """Send parameter to device via appropriate ROS2 service"""
        try:
            if device_type == 'motor':
                # For motors, we need to get current config, update it, and send it back
                return self._update_motor_config(device_id, param_name, value)
            elif device_type == 'dcmotor':
                # For DC motors, we need to get current config, update it, and send it back
                return self._update_dcmotor_config(device_id, param_name, value)
            elif device_type == 'servo':
                # For servos, call the servo config service
                self.get_logger().info(f'Servo parameter update not yet implemented for {param_name}')
                return True  # Placeholder
            elif device_type == 'encoder':
                # For encoders, call the encoder config service
                self.get_logger().info(f'Encoder parameter update not yet implemented for {param_name}')
                return True  # Placeholder
            else:
                self.get_logger().error(f'Unknown device type: {device_type}')
                return False
        except Exception as e:
            self.get_logger().error(f'Failed to send parameter to device: {str(e)}')
            return False

    def _update_motor_config(self, device_id: int, param_name: str, value: Any) -> bool:
        """Update motor configuration via ROS2 service"""
        try:
            # First get current configuration
            get_request = GetRobomasterMotorConfig.Request()
            get_request.motor_id = device_id

            if not self.motor_config_get_client.wait_for_service(timeout_sec=2.0):
                self.get_logger().error('Motor config get service not available')
                return False

            get_future = self.motor_config_get_client.call_async(get_request)
            rclpy.spin_until_future_complete(self, get_future, timeout_sec=5.0)

            if not get_future.result().success:
                self.get_logger().error(f'Failed to get motor config: {get_future.result().message}')
                return False

            # Update the specific parameter
            config = get_future.result().config
            self._apply_motor_parameter(config, param_name, value)

            # Send updated configuration
            set_request = SetRobomasterMotorConfig.Request()
            set_request.motor_id = device_id
            set_request.config = config

            if not self.motor_config_set_client.wait_for_service(timeout_sec=2.0):
                self.get_logger().error('Motor config set service not available')
                return False

            set_future = self.motor_config_set_client.call_async(set_request)
            rclpy.spin_until_future_complete(self, set_future, timeout_sec=5.0)

            if set_future.result().success:
                self.get_logger().info(f'Successfully updated motor {device_id} parameter {param_name} to {value}')
                return True
            else:
                self.get_logger().error(f'Failed to set motor config: {set_future.result().message}')
                return False

        except Exception as e:
            self.get_logger().error(f'Error updating motor config: {str(e)}')
            return False

    def _apply_motor_parameter(self, config: RobomasterMotorConfig, param_name: str, value: Any):
        """Apply parameter value to motor config message"""
        param_mapping = {
            'MAX_VELOCITY_RPS': 'max_velocity_rps',
            'MAX_CURRENT_MA': 'max_current_ma',
            'POSITION_KP': 'position_kp',
            'POSITION_KI': 'position_ki',
            'POSITION_KD': 'position_kd',
            'VELOCITY_KP': 'velocity_kp',
            'VELOCITY_KI': 'velocity_ki',
            'VELOCITY_KD': 'velocity_kd',
            'MAX_TEMPERATURE': 'max_temperature_celsius',
            'WATCHDOG_TIMEOUT': 'watchdog_timeout_ms'
        }

        if param_name in param_mapping:
            setattr(config, param_mapping[param_name], value)
        else:
            self.get_logger().warning(f'Unknown motor parameter: {param_name}')

    def read_all_parameters(self, device_type: str, device_id: int) -> bool:
        """Request all parameters from a device via ROS2 services"""
        device_key = f"{device_type}_{device_id}"

        if device_key not in self.device_parameters:
            self.get_logger().error(f'Device {device_type} {device_id} not found')
            return False

        try:
            if device_type == 'servo':
                return self._read_servo_parameters(device_id)
            elif device_type == 'motor':
                return self._read_motor_parameters(device_id)
            elif device_type == 'dcmotor':
                return self._read_dcmotor_parameters(device_id)
            else:
                self.get_logger().info(f'Parameter reading not yet implemented for {device_type}')
                return True  # Placeholder
        except Exception as e:
            self.get_logger().error(f'Failed to read parameters: {str(e)}')
            return False

    def _read_motor_parameters(self, device_id: int) -> bool:
        """Read motor parameters via ROS2 service"""
        try:
            get_request = GetRobomasterMotorConfig.Request()
            get_request.motor_id = device_id

            if not self.motor_config_get_client.wait_for_service(timeout_sec=2.0):
                self.get_logger().error('Motor config get service not available')
                return False

            get_future = self.motor_config_get_client.call_async(get_request)
            rclpy.spin_until_future_complete(self, get_future, timeout_sec=5.0)

            if not get_future.result().success:
                self.get_logger().error(f'Failed to get motor config: {get_future.result().message}')
                return False

            # Update local parameters with values from device
            config = get_future.result().config
            device_key = f"motor_{device_id}"
            device_params = self.device_parameters[device_key]

            param_mapping = {
                'MAX_VELOCITY_RPS': config.max_velocity_rps,
                'MAX_CURRENT_MA': config.max_current_ma,
                'POSITION_KP': config.position_kp,
                'POSITION_KI': config.position_ki,
                'POSITION_KD': config.position_kd,
                'VELOCITY_KP': config.velocity_kp,
                'VELOCITY_KI': config.velocity_ki,
                'VELOCITY_KD': config.velocity_kd,
                'MAX_TEMPERATURE': config.max_temperature_celsius,
                'WATCHDOG_TIMEOUT': config.watchdog_timeout_ms
            }

            for param_name, param_value in param_mapping.items():
                if param_name in device_params.parameters:
                    device_params.parameters[param_name].value = param_value

            device_params.last_updated = time.time()

            if self.on_parameter_read_complete:
                self.on_parameter_read_complete(device_id)

            self.get_logger().info(f'Successfully read parameters from motor {device_id}')
            return True

        except Exception as e:
            self.get_logger().error(f'Error reading motor parameters: {str(e)}')
            return False

    def reset_to_defaults(self, device_type: str, device_id: int) -> bool:
        """Reset all parameters to default values"""
        device_key = f"{device_type}_{device_id}"

        if device_key not in self.device_parameters:
            self.get_logger().error(f'Device {device_type} {device_id} not found')
            return False

        device_params = self.device_parameters[device_key]
        success = True

        for param_name, parameter in device_params.parameters.items():
            if not parameter.read_only and parameter.default_value is not None:
                old_value = parameter.value

                # Update parameter and send to device
                if self.update_parameter(device_type, device_id, param_name, parameter.default_value):
                    self.get_logger().info(f'Reset {param_name} from {old_value} to {parameter.default_value}')
                else:
                    self.get_logger().error(f'Failed to reset {param_name} to default value')
                    success = False

        return success

    def save_parameters_to_file(self, device_type: str, device_id: int, filename: str) -> bool:
        """Save device parameters to a JSON file"""
        device_key = f"{device_type}_{device_id}"

        if device_key not in self.device_parameters:
            self.get_logger().error(f'Device {device_type} {device_id} not found')
            return False

        try:
            device_params = self.device_parameters[device_key]

            # Convert parameters to serializable format
            params_dict = {
                'device_id': device_params.device_id,
                'device_type': device_params.device_type,
                'version': device_params.version,
                'last_updated': device_params.last_updated,
                'parameters': {}
            }

            for param_name, parameter in device_params.parameters.items():
                params_dict['parameters'][param_name] = {
                    'name': parameter.name,
                    'value': parameter.value,
                    'param_type': parameter.param_type.value,
                    'description': parameter.description,
                    'min_value': parameter.min_value,
                    'max_value': parameter.max_value,
                    'default_value': parameter.default_value,
                    'read_only': parameter.read_only,
                    'unit': parameter.unit
                }

            with open(filename, 'w') as f:
                json.dump(params_dict, f, indent=2)

            self.get_logger().info(f'Saved parameters for device {device_type} {device_id} to {filename}')
            return True

        except Exception as e:
            self.get_logger().error(f'Failed to save parameters: {str(e)}')
            return False

    def load_parameters_from_file(self, filename: str) -> bool:
        """Load device parameters from a JSON file"""
        try:
            with open(filename, 'r') as f:
                params_dict = json.load(f)

            device_id = params_dict['device_id']
            device_type = params_dict['device_type']
            device_key = f"{device_type}_{device_id}"

            # Create device parameters object
            parameters = {}
            for param_name, param_data in params_dict['parameters'].items():
                parameter = Parameter(
                    name=param_data['name'],
                    value=param_data['value'],
                    param_type=ParameterType(param_data['param_type']),
                    description=param_data['description'],
                    min_value=param_data['min_value'],
                    max_value=param_data['max_value'],
                    default_value=param_data['default_value'],
                    read_only=param_data['read_only'],
                    unit=param_data['unit']
                )
                parameters[param_name] = parameter

            device_params = DeviceParameters(
                device_id=device_id,
                device_type=device_type,
                parameters=parameters,
                last_updated=params_dict['last_updated'],
                version=params_dict['version']
            )

            self.device_parameters[device_key] = device_params

            self.get_logger().info(f'Loaded parameters for device {device_id} from {filename}')
            return True

        except Exception as e:
            self.get_logger().error(f'Failed to load parameters: {str(e)}')
            return False

    def get_parameter_definitions(self, device_type: str) -> Optional[Dict[str, Parameter]]:
        """Get parameter definitions for a device type"""
        return self.parameter_definitions.get(device_type)

    def validate_parameter_value(self, device_type: str, param_name: str, value: Any) -> bool:
        """Validate a parameter value against its definition"""
        if device_type not in self.parameter_definitions:
            return False

        if param_name not in self.parameter_definitions[device_type]:
            return False

        param_def = self.parameter_definitions[device_type][param_name]

        # Check range
        if param_def.min_value is not None and value < param_def.min_value:
            return False

        if param_def.max_value is not None and value > param_def.max_value:
            return False

        return True

    def _update_dcmotor_config(self, device_id: int, param_name: str, value: Any) -> bool:
        """Update DC motor configuration parameter"""
        try:
            # Get current config first
            get_client = self.create_client(GetDCMotorConfig, '/dcmotor/get_config')
            if not get_client.wait_for_service(timeout_sec=3.0):
                self.get_logger().error('DC motor get config service not available')
                return False

            get_request = GetDCMotorConfig.Request()
            get_request.motor_id = device_id

            future = get_client.call_async(get_request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

            if not future.result() or not future.result().success:
                self.get_logger().error('Failed to get current DC motor config')
                return False

            config = future.result().config

            # Update the specific parameter
            param_mapping = {
                'dc_speed_kp': 'speed_kp',
                'dc_speed_ki': 'speed_ki',
                'dc_speed_kd': 'speed_kd',
                'dc_position_kp': 'position_kp',
                'dc_position_ki': 'position_ki',
                'dc_position_kd': 'position_kd',
                'dc_max_speed_rad_s': 'max_speed_rad_s',
                'dc_max_acceleration_rad_s2': 'max_acceleration_rad_s2',
                'dc_use_position_limits': 'use_position_limits',
                'dc_position_limit_min_rad': 'position_limit_min_rad',
                'dc_position_limit_max_rad': 'position_limit_max_rad',
                'dc_watchdog_timeout_ms': 'watchdog_timeout_ms',
                'dc_control_mode': 'mode'
            }

            if param_name in param_mapping:
                setattr(config, param_mapping[param_name], value)
            else:
                self.get_logger().warning(f'Unknown DC motor parameter: {param_name}')
                return False

            # Send updated config
            set_client = self.create_client(SetDCMotorConfig, '/dcmotor/set_config')
            if not set_client.wait_for_service(timeout_sec=3.0):
                self.get_logger().error('DC motor set config service not available')
                return False

            set_request = SetDCMotorConfig.Request()
            set_request.config = config

            future = set_client.call_async(set_request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

            result = future.result()
            if result and result.success:
                self.get_logger().info(f'Updated DC motor {device_id} parameter {param_name} = {value}')
                return True
            else:
                error_msg = result.message if result else "Unknown error"
                self.get_logger().error(f'Failed to update DC motor config: {error_msg}')
                return False

        except Exception as e:
            self.get_logger().error(f'Error updating DC motor config: {str(e)}')
            return False

    def _read_servo_parameters(self, device_id: int) -> bool:
        """Read servo parameters via ROS2 service"""
        try:
            # For now, servo parameter reading is not fully implemented in the interface
            # Return True to avoid crashes, but log that it's not implemented
            self.get_logger().info(f'Servo parameter reading for servo {device_id} - using default values')
            return True
        except Exception as e:
            self.get_logger().error(f'Error reading servo parameters: {str(e)}')
            return False

    def _read_dcmotor_parameters(self, device_id: int) -> bool:
        """Read DC motor parameters via ROS2 service"""
        try:
            client = self.create_client(GetDCMotorConfig, '/dcmotor/get_config')
            if not client.wait_for_service(timeout_sec=3.0):
                self.get_logger().error('DC motor config service not available')
                return False

            request = GetDCMotorConfig.Request()
            request.motor_id = device_id

            future = client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

            result = future.result()
            if not result or not result.success:
                error_msg = result.message if result else "Unknown error"
                self.get_logger().error(f'Failed to read DC motor config: {error_msg}')
                return False

            config = result.config

            # Update local parameters with received config
            device_key = f"dcmotor_{device_id}"
            if device_key not in self.device_parameters:
                self.get_logger().warning(f'DC motor {device_id} not found in local parameters')
                return False

            device_params = self.device_parameters[device_key]

            # Map config fields to parameter names
            param_mapping = {
                'speed_kp': 'dc_speed_kp',
                'speed_ki': 'dc_speed_ki',
                'speed_kd': 'dc_speed_kd',
                'position_kp': 'dc_position_kp',
                'position_ki': 'dc_position_ki',
                'position_kd': 'dc_position_kd',
                'max_speed_rad_s': 'dc_max_speed_rad_s',
                'max_acceleration_rad_s2': 'dc_max_acceleration_rad_s2',
                'use_position_limits': 'dc_use_position_limits',
                'position_limit_min_rad': 'dc_position_limit_min_rad',
                'position_limit_max_rad': 'dc_position_limit_max_rad',
                'watchdog_timeout_ms': 'dc_watchdog_timeout_ms',
                'mode': 'dc_control_mode'
            }

            for config_field, param_name in param_mapping.items():
                if hasattr(config, config_field) and param_name in device_params.parameters:
                    device_params.parameters[param_name].value = getattr(config, config_field)

            device_params.last_updated = time.time()
            self.get_logger().info(f'Read DC motor {device_id} parameters successfully')
            return True

        except Exception as e:
            self.get_logger().error(f'Error reading DC motor parameters: {str(e)}')
            return False


def main():
    """Test function for the parameter manager"""
    rclpy.init()

    param_manager = ParameterManager()

    try:
        # Create some test devices
        param_manager.create_device_parameters('servo', 1)
        param_manager.create_device_parameters('encoder', 2)
        param_manager.create_device_parameters('motor', 5)

        # Update some parameters
        param_manager.update_parameter('servo', 1, 'SPEED', 200)
        param_manager.update_parameter('encoder', 2, 'CPR', 2048)
        param_manager.update_parameter('motor', 5, 'MAX_VELOCITY_RPS', 15.0)

        # Save parameters
        param_manager.save_parameters_to_file('servo', 1, '/tmp/servo_config.json')

        rclpy.spin(param_manager)

    except KeyboardInterrupt:
        pass
    finally:
        param_manager.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()