#!/usr/bin/env python3

import sys
import os
import json
import logging
import traceback
from typing import Dict, List, Any, Optional
from functools import wraps

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *

from std_msgs.msg import String
from sensor_msgs.msg import JointState
from stm32_mavlink_interface.msg import ServoState, EncoderState, RobomasterMotorState, RobomasterMotorCommand, DCMotorState, DCMotorCommand, ServoCommand
from stm32_mavlink_interface.srv import SetServoConfig, SetEncoderConfig, SetDCMotorConfig, GetDCMotorConfig

# Import the updated device scanner and parameter manager
from device_scanner import DeviceScanner
from parameter_manager import ParameterManager

# Setup logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

def error_handler(func):
    """Decorator for handling exceptions with user feedback"""
    @wraps(func)
    def wrapper(self, *args, **kwargs):
        try:
            return func(self, *args, **kwargs)
        except Exception as e:
            error_msg = f"Error in {func.__name__}: {str(e)}"
            logger.error(error_msg)
            logger.debug(traceback.format_exc())

            if hasattr(self, 'show_error_message'):
                self.show_error_message("Operation Failed", error_msg)
            else:
                # Fallback for functions without access to GUI
                print(f"ERROR: {error_msg}")
            return None
    return wrapper

class ConfigurationManager:
    """Manages application configuration with validation"""

    def __init__(self, config_path: str):
        self.config_path = config_path
        self.config = self._load_default_config()
        self._load_config()

    def _load_default_config(self) -> Dict[str, Any]:
        """Load default configuration"""
        return {
            'gui': {
                'window_size': [1200, 800],
                'update_rate': 10.0,
                'max_history_entries': 1000,
                'auto_scan_on_startup': True
            },
            'communication': {
                'service_timeout': 5.0,
                'connection_retry_count': 3,
                'heartbeat_timeout': 10.0
            },
            'devices': {
                'servo_ids': list(range(1, 17)),
                'encoder_ids': list(range(1, 17)),
                'motor_ids': list(range(1, 9))
            },
            'logging': {
                'level': 'INFO',
                'max_log_entries': 1000
            }
        }

    def _load_config(self):
        """Load configuration from file"""
        try:
            if os.path.exists(self.config_path):
                with open(self.config_path, 'r') as f:
                    loaded_config = json.load(f)
                    self._merge_config(loaded_config)
                    logger.info(f"Configuration loaded from {self.config_path}")
        except Exception as e:
            logger.warning(f"Failed to load config: {e}, using defaults")

    def _merge_config(self, loaded_config: Dict[str, Any]):
        """Merge loaded config with defaults"""
        for section, values in loaded_config.items():
            if section in self.config:
                self.config[section].update(values)

    def save_config(self):
        """Save current configuration to file"""
        try:
            os.makedirs(os.path.dirname(self.config_path), exist_ok=True)
            with open(self.config_path, 'w') as f:
                json.dump(self.config, f, indent=2)
            logger.info(f"Configuration saved to {self.config_path}")
        except Exception as e:
            logger.error(f"Failed to save config: {e}")

    def get(self, section: str, key: str, default=None):
        """Get configuration value"""
        return self.config.get(section, {}).get(key, default)

    def set(self, section: str, key: str, value):
        """Set configuration value"""
        if section not in self.config:
            self.config[section] = {}
        self.config[section][key] = value

class SignalBridge(QObject):
    """Bridge class for Qt signals from ROS2 callbacks"""
    servo_state_signal = pyqtSignal(object)
    encoder_state_signal = pyqtSignal(object)
    motor_state_signal = pyqtSignal(object)
    dcmotor_state_signal = pyqtSignal(object)

class MAVLinkWizardNode(Node):
    """ROS2 node for MAVLink Wizard communication"""

    def __init__(self, signal_bridge):
        super().__init__('mavlink_wizard_node')

        # Store reference to signal bridge
        self.signal_bridge = signal_bridge

        # QoS profile for reliable communication (compatible with publisher settings)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )

        # Subscribers for device states
        self.servo_state_sub = self.create_subscription(
            ServoState, '/servo/states', self.servo_state_callback, qos_profile)
        self.encoder_state_sub = self.create_subscription(
            EncoderState, '/encoder/states', self.encoder_state_callback, qos_profile)
        self.motor_state_sub = self.create_subscription(
            RobomasterMotorState, '/robomaster/motor_state', self.motor_state_callback, qos_profile)
        self.dcmotor_state_sub = self.create_subscription(
            DCMotorState, '/dcmotor/state', self.dcmotor_state_callback, qos_profile)

        # Service clients for configuration
        self.servo_config_client = self.create_client(SetServoConfig, '/servo/set_config')
        self.encoder_config_client = self.create_client(SetEncoderConfig, '/encoder/set_config')

        # Publisher for motor commands
        self.motor_command_pub = self.create_publisher(RobomasterMotorCommand, '/robomaster/motor_command', 10)
        self.dcmotor_command_pub = self.create_publisher(DCMotorCommand, '/dcmotor/command', 10)
        self.servo_command_pub = self.create_publisher(ServoCommand, '/servo/command', 10)

        self.get_logger().info('MAVLink Wizard Node initialized')

    def servo_state_callback(self, msg):
        self.signal_bridge.servo_state_signal.emit(msg)

    def encoder_state_callback(self, msg):
        self.signal_bridge.encoder_state_signal.emit(msg)

    def motor_state_callback(self, msg):
        self.signal_bridge.motor_state_signal.emit(msg)

    def dcmotor_state_callback(self, msg):
        self.signal_bridge.dcmotor_state_signal.emit(msg)

class DeviceTreeWidget(QTreeWidget):
    """Tree widget for displaying discovered devices"""

    device_selected = pyqtSignal(str, int)  # device_type, device_id

    def __init__(self):
        super().__init__()
        self.setHeaderLabels(['Device', 'ID', 'Status', 'Last Update'])
        self.itemClicked.connect(self.on_item_clicked)

        # Create top-level categories
        self.servo_category = QTreeWidgetItem(self, ['Servos', '', '', ''])
        self.encoder_category = QTreeWidgetItem(self, ['Encoders', '', '', ''])
        self.motor_category = QTreeWidgetItem(self, ['Motors', '', '', ''])
        self.dcmotor_category = QTreeWidgetItem(self, ['DC Motors', '', '', ''])

        self.expandAll()

    def clear_devices(self):
        """Clear all devices from the tree"""
        self.servo_category.takeChildren()
        self.encoder_category.takeChildren()
        self.motor_category.takeChildren()
        self.dcmotor_category.takeChildren()

    def add_device(self, device_type: str, device_id: int, status: str = 'Unknown'):
        """Add a device to the tree"""
        if device_type == 'servo':
            parent = self.servo_category
        elif device_type == 'encoder':
            parent = self.encoder_category
        elif device_type == 'motor':
            parent = self.motor_category
        elif device_type == 'dcmotor':
            parent = self.dcmotor_category
        else:
            return

        # Check if device already exists
        for i in range(parent.childCount()):
            child = parent.child(i)
            if child.text(1) == str(device_id):
                child.setText(2, status)
                child.setText(3, QTime.currentTime().toString())
                return

        # Add new device
        device_item = QTreeWidgetItem(parent, [
            f"{device_type.title()} {device_id}",
            str(device_id),
            status,
            QTime.currentTime().toString()
        ])
        self.expandAll()

    def on_item_clicked(self, item, column):
        """Handle device selection"""
        if item.parent() is not None:  # Only emit for actual devices, not categories
            device_type = item.parent().text(0).lower().rstrip('s')  # Remove 's' from plural
            device_id = int(item.text(1))
            self.device_selected.emit(device_type, device_id)

class ParameterConfigWidget(QWidget):
    """Widget for configuring device parameters"""

    def __init__(self, parameter_manager):
        super().__init__()
        self.current_device_type = None
        self.current_device_id = None
        self.parameter_manager = parameter_manager
        self.setup_ui()

    def setup_ui(self):
        layout = QVBoxLayout()

        # Device info
        self.device_info_label = QLabel("No device selected")
        self.device_info_label.setFont(QFont("Arial", 12, QFont.Bold))
        layout.addWidget(self.device_info_label)

        # Parameter form
        self.parameter_form = QFormLayout()
        self.parameter_widgets = {}

        # Common parameters for all devices
        self.id_spinbox = QSpinBox()
        self.id_spinbox.setRange(1, 254)
        self.parameter_form.addRow("Device ID:", self.id_spinbox)
        self.parameter_widgets['id'] = self.id_spinbox

        # Servo-specific parameters
        self.servo_group = QGroupBox("Servo Parameters")
        servo_layout = QFormLayout()

        self.min_angle_spinbox = QDoubleSpinBox()
        self.min_angle_spinbox.setRange(-180, 180)
        self.min_angle_spinbox.setSuffix("°")
        servo_layout.addRow("Min Angle:", self.min_angle_spinbox)
        self.parameter_widgets['min_angle'] = self.min_angle_spinbox

        self.max_angle_spinbox = QDoubleSpinBox()
        self.max_angle_spinbox.setRange(-180, 180)
        self.max_angle_spinbox.setSuffix("°")
        servo_layout.addRow("Max Angle:", self.max_angle_spinbox)
        self.parameter_widgets['max_angle'] = self.max_angle_spinbox

        self.servo_speed_spinbox = QSpinBox()
        self.servo_speed_spinbox.setRange(1, 1000)
        servo_layout.addRow("Speed:", self.servo_speed_spinbox)
        self.parameter_widgets['speed'] = self.servo_speed_spinbox

        self.servo_group.setLayout(servo_layout)

        # Encoder-specific parameters
        self.encoder_group = QGroupBox("Encoder Parameters")
        encoder_layout = QFormLayout()

        self.cpr_spinbox = QSpinBox()
        self.cpr_spinbox.setRange(1, 10000)
        encoder_layout.addRow("Counts Per Revolution:", self.cpr_spinbox)
        self.parameter_widgets['cpr'] = self.cpr_spinbox

        self.invert_a_checkbox = QCheckBox()
        encoder_layout.addRow("Invert Channel A:", self.invert_a_checkbox)
        self.parameter_widgets['invert_a'] = self.invert_a_checkbox

        self.invert_b_checkbox = QCheckBox()
        encoder_layout.addRow("Invert Channel B:", self.invert_b_checkbox)
        self.parameter_widgets['invert_b'] = self.invert_b_checkbox

        self.use_index_checkbox = QCheckBox()
        encoder_layout.addRow("Use Index Channel:", self.use_index_checkbox)
        self.parameter_widgets['use_index'] = self.use_index_checkbox

        self.encoder_group.setLayout(encoder_layout)

        # Motor-specific parameters
        self.motor_group = QGroupBox("Motor Parameters")
        motor_layout = QFormLayout()

        self.max_velocity_spinbox = QDoubleSpinBox()
        self.max_velocity_spinbox.setRange(0.1, 100.0)
        self.max_velocity_spinbox.setSuffix(" RPS")
        motor_layout.addRow("Max Velocity:", self.max_velocity_spinbox)
        self.parameter_widgets['max_velocity_rps'] = self.max_velocity_spinbox

        self.max_current_spinbox = QSpinBox()
        self.max_current_spinbox.setRange(100, 25000)
        self.max_current_spinbox.setSuffix(" mA")
        motor_layout.addRow("Max Current:", self.max_current_spinbox)
        self.parameter_widgets['max_current_ma'] = self.max_current_spinbox

        # Position PID parameters
        self.position_kp_spinbox = QDoubleSpinBox()
        self.position_kp_spinbox.setRange(0.0, 1000.0)
        motor_layout.addRow("Position KP:", self.position_kp_spinbox)
        self.parameter_widgets['position_kp'] = self.position_kp_spinbox

        self.position_ki_spinbox = QDoubleSpinBox()
        self.position_ki_spinbox.setRange(0.0, 1000.0)
        motor_layout.addRow("Position KI:", self.position_ki_spinbox)
        self.parameter_widgets['position_ki'] = self.position_ki_spinbox

        self.position_kd_spinbox = QDoubleSpinBox()
        self.position_kd_spinbox.setRange(0.0, 1000.0)
        motor_layout.addRow("Position KD:", self.position_kd_spinbox)
        self.parameter_widgets['position_kd'] = self.position_kd_spinbox

        # Velocity PID parameters
        self.velocity_kp_spinbox = QDoubleSpinBox()
        self.velocity_kp_spinbox.setRange(0.0, 10000.0)
        motor_layout.addRow("Velocity KP:", self.velocity_kp_spinbox)
        self.parameter_widgets['velocity_kp'] = self.velocity_kp_spinbox

        self.velocity_ki_spinbox = QDoubleSpinBox()
        self.velocity_ki_spinbox.setRange(0.0, 10000.0)
        motor_layout.addRow("Velocity KI:", self.velocity_ki_spinbox)
        self.parameter_widgets['velocity_ki'] = self.velocity_ki_spinbox

        self.velocity_kd_spinbox = QDoubleSpinBox()
        self.velocity_kd_spinbox.setRange(0.0, 10000.0)
        motor_layout.addRow("Velocity KD:", self.velocity_kd_spinbox)
        self.parameter_widgets['velocity_kd'] = self.velocity_kd_spinbox

        self.max_temp_spinbox = QSpinBox()
        self.max_temp_spinbox.setRange(40, 100)
        self.max_temp_spinbox.setSuffix("°C")
        motor_layout.addRow("Max Temperature:", self.max_temp_spinbox)
        self.parameter_widgets['max_temperature'] = self.max_temp_spinbox

        self.motor_group.setLayout(motor_layout)

        # DC Motor-specific parameters
        self.dcmotor_group = QGroupBox("DC Motor Parameters")
        dcmotor_layout = QFormLayout()

        # Control mode selection
        self.dcmotor_mode_combo = QComboBox()
        self.dcmotor_mode_combo.addItems(["Position Control", "Velocity Control", "Current Control"])
        dcmotor_layout.addRow("Control Mode:", self.dcmotor_mode_combo)
        self.parameter_widgets['dc_control_mode'] = self.dcmotor_mode_combo

        # Speed PID parameters
        self.dc_speed_kp_spinbox = QDoubleSpinBox()
        self.dc_speed_kp_spinbox.setRange(0.0, 100.0)
        self.dc_speed_kp_spinbox.setDecimals(3)
        dcmotor_layout.addRow("Speed KP:", self.dc_speed_kp_spinbox)
        self.parameter_widgets['dc_speed_kp'] = self.dc_speed_kp_spinbox

        self.dc_speed_ki_spinbox = QDoubleSpinBox()
        self.dc_speed_ki_spinbox.setRange(0.0, 100.0)
        self.dc_speed_ki_spinbox.setDecimals(3)
        dcmotor_layout.addRow("Speed KI:", self.dc_speed_ki_spinbox)
        self.parameter_widgets['dc_speed_ki'] = self.dc_speed_ki_spinbox

        self.dc_speed_kd_spinbox = QDoubleSpinBox()
        self.dc_speed_kd_spinbox.setRange(0.0, 100.0)
        self.dc_speed_kd_spinbox.setDecimals(3)
        dcmotor_layout.addRow("Speed KD:", self.dc_speed_kd_spinbox)
        self.parameter_widgets['dc_speed_kd'] = self.dc_speed_kd_spinbox

        # Position PID parameters
        self.dc_position_kp_spinbox = QDoubleSpinBox()
        self.dc_position_kp_spinbox.setRange(0.0, 100.0)
        self.dc_position_kp_spinbox.setDecimals(3)
        dcmotor_layout.addRow("Position KP:", self.dc_position_kp_spinbox)
        self.parameter_widgets['dc_position_kp'] = self.dc_position_kp_spinbox

        self.dc_position_ki_spinbox = QDoubleSpinBox()
        self.dc_position_ki_spinbox.setRange(0.0, 100.0)
        self.dc_position_ki_spinbox.setDecimals(3)
        dcmotor_layout.addRow("Position KI:", self.dc_position_ki_spinbox)
        self.parameter_widgets['dc_position_ki'] = self.dc_position_ki_spinbox

        self.dc_position_kd_spinbox = QDoubleSpinBox()
        self.dc_position_kd_spinbox.setRange(0.0, 100.0)
        self.dc_position_kd_spinbox.setDecimals(3)
        dcmotor_layout.addRow("Position KD:", self.dc_position_kd_spinbox)
        self.parameter_widgets['dc_position_kd'] = self.dc_position_kd_spinbox

        # Physical limits
        self.dc_max_speed_spinbox = QDoubleSpinBox()
        self.dc_max_speed_spinbox.setRange(1.0, 50.0)
        self.dc_max_speed_spinbox.setSuffix(" rad/s")
        dcmotor_layout.addRow("Max Speed:", self.dc_max_speed_spinbox)
        self.parameter_widgets['dc_max_speed_rad_s'] = self.dc_max_speed_spinbox

        self.dc_max_accel_spinbox = QDoubleSpinBox()
        self.dc_max_accel_spinbox.setRange(1.0, 200.0)
        self.dc_max_accel_spinbox.setSuffix(" rad/s²")
        dcmotor_layout.addRow("Max Acceleration:", self.dc_max_accel_spinbox)
        self.parameter_widgets['dc_max_acceleration_rad_s2'] = self.dc_max_accel_spinbox

        # Position limits
        self.dc_position_limits_checkbox = QCheckBox()
        dcmotor_layout.addRow("Use Position Limits:", self.dc_position_limits_checkbox)
        self.parameter_widgets['dc_use_position_limits'] = self.dc_position_limits_checkbox

        self.dc_min_position_spinbox = QDoubleSpinBox()
        self.dc_min_position_spinbox.setRange(-1000.0, 1000.0)
        self.dc_min_position_spinbox.setSuffix(" rad")
        dcmotor_layout.addRow("Min Position:", self.dc_min_position_spinbox)
        self.parameter_widgets['dc_position_limit_min_rad'] = self.dc_min_position_spinbox

        self.dc_max_position_spinbox = QDoubleSpinBox()
        self.dc_max_position_spinbox.setRange(-1000.0, 1000.0)
        self.dc_max_position_spinbox.setSuffix(" rad")
        dcmotor_layout.addRow("Max Position:", self.dc_max_position_spinbox)
        self.parameter_widgets['dc_position_limit_max_rad'] = self.dc_max_position_spinbox

        # Safety settings
        self.dc_watchdog_timeout_spinbox = QSpinBox()
        self.dc_watchdog_timeout_spinbox.setRange(100, 10000)
        self.dc_watchdog_timeout_spinbox.setSuffix(" ms")
        dcmotor_layout.addRow("Watchdog Timeout:", self.dc_watchdog_timeout_spinbox)
        self.parameter_widgets['dc_watchdog_timeout_ms'] = self.dc_watchdog_timeout_spinbox

        self.dcmotor_group.setLayout(dcmotor_layout)

        # Add all groups to main layout
        layout.addLayout(self.parameter_form)
        layout.addWidget(self.servo_group)
        layout.addWidget(self.encoder_group)
        layout.addWidget(self.motor_group)
        layout.addWidget(self.dcmotor_group)

        # Control buttons
        button_layout = QHBoxLayout()
        self.read_button = QPushButton("Read Parameters")
        self.write_button = QPushButton("Write Parameters")
        self.reset_button = QPushButton("Factory Reset")

        # Connect button signals
        self.read_button.clicked.connect(self.read_parameters)
        self.write_button.clicked.connect(self.write_parameters)
        self.reset_button.clicked.connect(self.reset_parameters)

        button_layout.addWidget(self.read_button)
        button_layout.addWidget(self.write_button)
        button_layout.addWidget(self.reset_button)

        layout.addLayout(button_layout)
        layout.addStretch()

        self.setLayout(layout)
        self.update_visibility()

    def set_device(self, device_type: str, device_id: int):
        """Set the current device for configuration"""
        self.current_device_type = device_type
        self.current_device_id = device_id
        self.device_info_label.setText(f"{device_type.title()} ID: {device_id}")
        self.id_spinbox.setValue(device_id)

        # Create device parameters if not exist
        if self.parameter_manager:
            if not self.parameter_manager.get_device_parameters(device_type, device_id):
                self.parameter_manager.create_device_parameters(device_type, device_id)

            # Load current parameter values
            self.load_current_parameters()

        self.update_visibility()

    def load_current_parameters(self):
        """Load current parameter values from parameter manager"""
        if not self.parameter_manager or not self.current_device_type or not self.current_device_id:
            return

        device_params = self.parameter_manager.get_device_parameters(self.current_device_type, self.current_device_id)
        if not device_params:
            return

        # Update GUI elements with current parameter values
        for param_name, parameter in device_params.parameters.items():
            widget_key = param_name.lower()
            if widget_key in self.parameter_widgets:
                widget = self.parameter_widgets[widget_key]
                if isinstance(widget, QSpinBox):
                    widget.setValue(int(parameter.value))
                elif isinstance(widget, QDoubleSpinBox):
                    widget.setValue(float(parameter.value))
                elif isinstance(widget, QCheckBox):
                    widget.setChecked(bool(parameter.value))

    def read_parameters(self):
        """Read parameters from device"""
        if not self.parameter_manager or not self.current_device_type or not self.current_device_id:
            QMessageBox.warning(self, "Warning", "No device selected")
            return

        success = self.parameter_manager.read_all_parameters(self.current_device_type, self.current_device_id)
        if success:
            self.load_current_parameters()
            QMessageBox.information(self, "Success", "Parameters read successfully")
        else:
            QMessageBox.critical(self, "Error", "Failed to read parameters")

    def write_parameters(self):
        """Write current parameter values to device"""
        if not self.parameter_manager or not self.current_device_type or not self.current_device_id:
            QMessageBox.warning(self, "Warning", "No device selected")
            return

        # Update parameter manager with current GUI values
        success = True
        param_mapping = {
            'speed': 'SPEED',
            'min_angle': 'MIN_ANGLE',
            'max_angle': 'MAX_ANGLE',
            'cpr': 'CPR',
            'invert_a': 'INVERT_A',
            'invert_b': 'INVERT_B',
            'use_index': 'USE_INDEX',
            'max_velocity_rps': 'MAX_VELOCITY_RPS',
            'max_current_ma': 'MAX_CURRENT_MA',
            'position_kp': 'POSITION_KP',
            'position_ki': 'POSITION_KI',
            'position_kd': 'POSITION_KD',
            'velocity_kp': 'VELOCITY_KP',
            'velocity_ki': 'VELOCITY_KI',
            'velocity_kd': 'VELOCITY_KD',
            'max_temperature': 'MAX_TEMPERATURE'
        }

        for widget_key, param_name in param_mapping.items():
            if widget_key in self.parameter_widgets:
                widget = self.parameter_widgets[widget_key]

                value = None
                if isinstance(widget, QSpinBox):
                    value = widget.value()
                elif isinstance(widget, QDoubleSpinBox):
                    value = widget.value()
                elif isinstance(widget, QCheckBox):
                    value = 1 if widget.isChecked() else 0

                if value is not None:
                    param_success = self.parameter_manager.update_parameter(
                        self.current_device_type, self.current_device_id,
                        param_name, value
                    )
                    if not param_success:
                        success = False

        if success:
            QMessageBox.information(self, "Success", "Parameters written successfully")
        else:
            QMessageBox.critical(self, "Error", "Some parameters failed to write")

    def reset_parameters(self):
        """Reset parameters to default values"""
        if not self.parameter_manager or not self.current_device_type or not self.current_device_id:
            QMessageBox.warning(self, "Warning", "No device selected")
            return

        reply = QMessageBox.question(self, "Confirm Reset",
                                   "Are you sure you want to reset all parameters to default values?",
                                   QMessageBox.Yes | QMessageBox.No)

        if reply == QMessageBox.Yes:
            success = self.parameter_manager.reset_to_defaults(self.current_device_type, self.current_device_id)
            if success:
                self.load_current_parameters()
                QMessageBox.information(self, "Success", "Parameters reset to defaults")
            else:
                QMessageBox.critical(self, "Error", "Failed to reset parameters")

    def update_visibility(self):
        """Update widget visibility based on device type"""
        self.servo_group.setVisible(self.current_device_type == 'servo')
        self.encoder_group.setVisible(self.current_device_type == 'encoder')
        self.motor_group.setVisible(self.current_device_type == 'motor')
        self.dcmotor_group.setVisible(self.current_device_type == 'dcmotor')

        # Update button availability
        has_device = self.current_device_type is not None and self.current_device_id is not None
        self.read_button.setEnabled(has_device)
        self.write_button.setEnabled(has_device)
        self.reset_button.setEnabled(has_device)

class MotorControlWidget(QWidget):
    """Widget for controlling RoboMaster motors"""

    def __init__(self, motor_command_publisher=None):
        super().__init__()
        self.motor_command_publisher = motor_command_publisher
        self.current_motor_id = None
        self.setup_ui()

    def setup_ui(self):
        layout = QVBoxLayout()

        # Motor selection
        motor_layout = QHBoxLayout()
        motor_layout.addWidget(QLabel("Motor ID:"))
        self.motor_id_spinbox = QSpinBox()
        self.motor_id_spinbox.setRange(1, 8)
        self.motor_id_spinbox.setValue(5)  # Default to motor 5
        self.motor_id_spinbox.valueChanged.connect(self.on_motor_changed)
        motor_layout.addWidget(self.motor_id_spinbox)
        motor_layout.addStretch()
        layout.addLayout(motor_layout)

        # Control mode selection
        mode_group = QGroupBox("Control Mode")
        mode_layout = QVBoxLayout()

        self.mode_group = QButtonGroup()
        self.current_mode_radio = QRadioButton("Current Control")
        self.velocity_mode_radio = QRadioButton("Velocity Control")
        self.position_mode_radio = QRadioButton("Position Control")

        self.mode_group.addButton(self.current_mode_radio, 0)
        self.mode_group.addButton(self.velocity_mode_radio, 1)
        self.mode_group.addButton(self.position_mode_radio, 2)

        self.velocity_mode_radio.setChecked(True)  # Default mode

        mode_layout.addWidget(self.current_mode_radio)
        mode_layout.addWidget(self.velocity_mode_radio)
        mode_layout.addWidget(self.position_mode_radio)
        mode_group.setLayout(mode_layout)
        layout.addWidget(mode_group)

        # Control inputs
        control_group = QGroupBox("Control Values")
        control_layout = QFormLayout()

        # Current control
        self.current_spinbox = QSpinBox()
        self.current_spinbox.setRange(-16000, 16000)
        self.current_spinbox.setSuffix(" mA")
        control_layout.addRow("Target Current:", self.current_spinbox)

        # Velocity control
        self.velocity_spinbox = QDoubleSpinBox()
        self.velocity_spinbox.setRange(-50.0, 50.0)
        self.velocity_spinbox.setSuffix(" RPS")
        self.velocity_spinbox.setDecimals(2)
        control_layout.addRow("Target Velocity:", self.velocity_spinbox)

        # Position control
        self.position_spinbox = QDoubleSpinBox()
        self.position_spinbox.setRange(-50.0, 50.0)
        self.position_spinbox.setSuffix(" rad")
        self.position_spinbox.setDecimals(3)
        control_layout.addRow("Target Position:", self.position_spinbox)

        control_group.setLayout(control_layout)
        layout.addWidget(control_group)

        # Control buttons
        button_layout = QHBoxLayout()

        self.enable_button = QPushButton("Enable Motor")
        self.disable_button = QPushButton("Disable Motor")
        self.stop_button = QPushButton("Emergency Stop")

        self.enable_button.clicked.connect(self.enable_motor)
        self.disable_button.clicked.connect(self.disable_motor)
        self.stop_button.clicked.connect(self.emergency_stop)

        button_layout.addWidget(self.enable_button)
        button_layout.addWidget(self.disable_button)
        button_layout.addWidget(self.stop_button)

        layout.addLayout(button_layout)

        # Send command button
        self.send_command_button = QPushButton("Send Command")
        self.send_command_button.clicked.connect(self.send_command)
        layout.addWidget(self.send_command_button)

        layout.addStretch()
        self.setLayout(layout)

        # Update initial state
        self.on_motor_changed()

    def set_motor_command_publisher(self, publisher):
        """Set the motor command publisher"""
        self.motor_command_publisher = publisher

    def on_motor_changed(self):
        self.current_motor_id = self.motor_id_spinbox.value()

    def send_command(self):
        if not self.motor_command_publisher or not self.current_motor_id:
            return

        cmd = RobomasterMotorCommand()
        cmd.motor_id = self.current_motor_id
        cmd.control_mode = self.mode_group.checkedId()
        cmd.enabled = True

        if cmd.control_mode == 0:  # Current control
            cmd.target_current_ma = self.current_spinbox.value()
        elif cmd.control_mode == 1:  # Velocity control
            cmd.target_velocity_rps = self.velocity_spinbox.value()
        elif cmd.control_mode == 2:  # Position control
            cmd.target_position_rad = self.position_spinbox.value()

        self.motor_command_publisher.publish(cmd)

    def enable_motor(self):
        if not self.motor_command_publisher or not self.current_motor_id:
            return

        cmd = RobomasterMotorCommand()
        cmd.motor_id = self.current_motor_id
        cmd.control_mode = self.mode_group.checkedId()
        cmd.enabled = True
        cmd.target_current_ma = 0
        cmd.target_velocity_rps = 0.0
        cmd.target_position_rad = 0.0

        self.motor_command_publisher.publish(cmd)

    def disable_motor(self):
        if not self.motor_command_publisher or not self.current_motor_id:
            return

        cmd = RobomasterMotorCommand()
        cmd.motor_id = self.current_motor_id
        cmd.control_mode = self.mode_group.checkedId()
        cmd.enabled = False
        cmd.target_current_ma = 0
        cmd.target_velocity_rps = 0.0
        cmd.target_position_rad = 0.0

        self.motor_command_publisher.publish(cmd)

    def emergency_stop(self):
        if not self.motor_command_publisher or not self.current_motor_id:
            return

        cmd = RobomasterMotorCommand()
        cmd.motor_id = self.current_motor_id
        cmd.control_mode = 0  # Current control
        cmd.enabled = False
        cmd.emergency_stop = True
        cmd.target_current_ma = 0
        cmd.target_velocity_rps = 0.0
        cmd.target_position_rad = 0.0

        self.motor_command_publisher.publish(cmd)


class DCMotorControlWidget(QWidget):
    """Widget for controlling DC motors"""

    def __init__(self, dcmotor_command_publisher=None):
        super().__init__()
        self.dcmotor_command_publisher = dcmotor_command_publisher
        self.current_motor_id = 10  # Default DC motor ID
        self.setup_ui()

    def setup_ui(self):
        layout = QVBoxLayout()

        # Motor info
        info_layout = QHBoxLayout()
        info_layout.addWidget(QLabel("DC Motor ID: 10"))
        info_layout.addStretch()
        layout.addLayout(info_layout)

        # Control mode selection
        mode_group = QGroupBox("Control Mode")
        mode_layout = QVBoxLayout()

        self.mode_group = QButtonGroup()
        self.position_mode_radio = QRadioButton("Position Control")
        self.velocity_mode_radio = QRadioButton("Velocity Control")
        self.current_mode_radio = QRadioButton("Current Control")

        self.mode_group.addButton(self.position_mode_radio, 0)
        self.mode_group.addButton(self.velocity_mode_radio, 1)
        self.mode_group.addButton(self.current_mode_radio, 2)

        self.position_mode_radio.setChecked(True)  # Default mode

        mode_layout.addWidget(self.position_mode_radio)
        mode_layout.addWidget(self.velocity_mode_radio)
        mode_layout.addWidget(self.current_mode_radio)
        mode_group.setLayout(mode_layout)
        layout.addWidget(mode_group)

        # Control inputs
        control_group = QGroupBox("Control Values")
        control_layout = QFormLayout()

        # Position control
        self.position_spinbox = QDoubleSpinBox()
        self.position_spinbox.setRange(-100.0, 100.0)
        self.position_spinbox.setSuffix(" rad")
        self.position_spinbox.setDecimals(3)
        control_layout.addRow("Target Position:", self.position_spinbox)

        # Velocity control
        self.velocity_spinbox = QDoubleSpinBox()
        self.velocity_spinbox.setRange(-15.0, 15.0)
        self.velocity_spinbox.setSuffix(" rad/s")
        self.velocity_spinbox.setDecimals(3)
        control_layout.addRow("Target Velocity:", self.velocity_spinbox)

        # Current control
        self.current_spinbox = QDoubleSpinBox()
        self.current_spinbox.setRange(-5.0, 5.0)
        self.current_spinbox.setSuffix(" A")
        self.current_spinbox.setDecimals(3)
        control_layout.addRow("Target Current:", self.current_spinbox)

        control_group.setLayout(control_layout)
        layout.addWidget(control_group)

        # Enable/Disable control
        enable_layout = QHBoxLayout()
        self.enable_checkbox = QCheckBox("Motor Enabled")
        self.enable_checkbox.setChecked(True)
        enable_layout.addWidget(self.enable_checkbox)
        enable_layout.addStretch()
        layout.addLayout(enable_layout)

        # Control buttons
        button_layout = QHBoxLayout()

        self.send_command_button = QPushButton("Send Command")
        self.send_command_button.clicked.connect(self.send_command)
        button_layout.addWidget(self.send_command_button)

        self.stop_button = QPushButton("STOP")
        self.stop_button.setStyleSheet("background-color: red; color: white; font-weight: bold;")
        self.stop_button.clicked.connect(self.emergency_stop)
        button_layout.addWidget(self.stop_button)

        button_layout.addStretch()
        layout.addLayout(button_layout)

        # Status display
        status_group = QGroupBox("Motor Status")
        status_layout = QFormLayout()

        self.position_label = QLabel("0.000 rad")
        self.velocity_label = QLabel("0.000 rad/s")
        self.current_label = QLabel("0.000 A")
        self.temperature_label = QLabel("0.0 °C")
        self.status_label = QLabel("Unknown")

        status_layout.addRow("Position:", self.position_label)
        status_layout.addRow("Velocity:", self.velocity_label)
        status_layout.addRow("Current:", self.current_label)
        status_layout.addRow("Temperature:", self.temperature_label)
        status_layout.addRow("Status:", self.status_label)

        status_group.setLayout(status_layout)
        layout.addWidget(status_group)

        layout.addStretch()
        self.setLayout(layout)

    def send_command(self):
        """Send motor control command"""
        if not self.dcmotor_command_publisher:
            logger.warning("DC motor command publisher not available")
            return

        cmd = DCMotorCommand()
        cmd.motor_id = self.current_motor_id
        cmd.control_mode = self.mode_group.checkedId()
        cmd.enabled = self.enable_checkbox.isChecked()

        # Set target value based on control mode
        if cmd.control_mode == 0:  # Position control
            cmd.target_value = self.position_spinbox.value()
        elif cmd.control_mode == 1:  # Velocity control
            cmd.target_value = self.velocity_spinbox.value()
        elif cmd.control_mode == 2:  # Current control
            cmd.target_value = self.current_spinbox.value()

        logger.info(f"Sending DC motor command: mode={cmd.control_mode}, value={cmd.target_value}, enabled={cmd.enabled}")
        self.dcmotor_command_publisher.publish(cmd)

    def emergency_stop(self):
        """Send emergency stop command"""
        if not self.dcmotor_command_publisher:
            logger.warning("DC motor command publisher not available")
            return

        cmd = DCMotorCommand()
        cmd.motor_id = self.current_motor_id
        cmd.control_mode = 1  # Velocity control
        cmd.target_value = 0.0
        cmd.enabled = False

        logger.info("Sending DC motor emergency stop")
        self.dcmotor_command_publisher.publish(cmd)

    def update_status(self, state_msg):
        """Update status display with received state message"""
        if state_msg.motor_id != self.current_motor_id:
            return

        self.position_label.setText(f"{state_msg.position_rad:.3f} rad")
        self.velocity_label.setText(f"{state_msg.velocity_rad_s:.3f} rad/s")
        self.current_label.setText(f"{state_msg.current_a:.3f} A")
        self.temperature_label.setText(f"{state_msg.temperature_c:.1f} °C")

        status_map = {
            0: "OK",
            1: "NOT_INITIALIZED",
            2: "ERROR",
            3: "OVERHEAT",
            4: "OVERCURRENT",
            5: "TIMEOUT"
        }
        status_text = status_map.get(state_msg.status, "UNKNOWN")
        self.status_label.setText(status_text)

        # Update enable checkbox based on motor state
        self.enable_checkbox.setChecked(state_msg.enabled)


class ServoControlWidget(QWidget):
    """Widget for controlling servo motors"""

    def __init__(self, servo_command_publisher=None):
        super().__init__()
        self.servo_command_publisher = servo_command_publisher
        self.current_servo_id = 1  # Default servo ID
        self.setup_ui()

    def setup_ui(self):
        layout = QVBoxLayout()

        # Servo selection
        servo_layout = QHBoxLayout()
        servo_layout.addWidget(QLabel("Servo ID:"))
        self.servo_id_spinbox = QSpinBox()
        self.servo_id_spinbox.setRange(1, 16)
        self.servo_id_spinbox.setValue(1)  # Default to servo 1
        self.servo_id_spinbox.valueChanged.connect(self.on_servo_changed)
        servo_layout.addWidget(self.servo_id_spinbox)
        servo_layout.addStretch()
        layout.addLayout(servo_layout)

        # Control mode selection
        mode_group = QGroupBox("Control Mode")
        mode_layout = QVBoxLayout()

        self.mode_group = QButtonGroup()
        self.angle_mode_radio = QRadioButton("Angle Control")
        self.pwm_mode_radio = QRadioButton("PWM Control")

        self.mode_group.addButton(self.angle_mode_radio, 0)
        self.mode_group.addButton(self.pwm_mode_radio, 1)

        self.angle_mode_radio.setChecked(True)  # Default mode

        mode_layout.addWidget(self.angle_mode_radio)
        mode_layout.addWidget(self.pwm_mode_radio)
        mode_group.setLayout(mode_layout)
        layout.addWidget(mode_group)

        # Control inputs
        control_group = QGroupBox("Control Values")
        control_layout = QFormLayout()

        # Angle control
        self.angle_spinbox = QDoubleSpinBox()
        self.angle_spinbox.setRange(-180.0, 180.0)
        self.angle_spinbox.setSuffix("°")
        self.angle_spinbox.setDecimals(1)
        self.angle_spinbox.setValue(0.0)
        control_layout.addRow("Target Angle:", self.angle_spinbox)

        # PWM control
        self.pwm_spinbox = QSpinBox()
        self.pwm_spinbox.setRange(0, 2000)
        self.pwm_spinbox.setSuffix(" μs")
        self.pwm_spinbox.setValue(1000)
        control_layout.addRow("PWM Pulse Width:", self.pwm_spinbox)

        control_group.setLayout(control_layout)
        layout.addWidget(control_group)

        # Enable/Disable control
        enable_layout = QHBoxLayout()
        self.enable_checkbox = QCheckBox("Servo Enabled")
        self.enable_checkbox.setChecked(True)
        enable_layout.addWidget(self.enable_checkbox)
        enable_layout.addStretch()
        layout.addLayout(enable_layout)

        # Control buttons
        button_layout = QHBoxLayout()

        self.send_command_button = QPushButton("Send Command")
        self.send_command_button.clicked.connect(self.send_command)
        button_layout.addWidget(self.send_command_button)

        self.center_button = QPushButton("Center (0°)")
        self.center_button.clicked.connect(self.move_to_center)
        button_layout.addWidget(self.center_button)

        self.disable_button = QPushButton("Disable")
        self.disable_button.clicked.connect(self.disable_servo)
        button_layout.addWidget(self.disable_button)

        button_layout.addStretch()
        layout.addLayout(button_layout)

        # Quick position buttons
        quick_pos_group = QGroupBox("Quick Positions")
        quick_pos_layout = QGridLayout()

        positions = [
            ("−180°", -180), ("−90°", -90), ("0°", 0), ("+90°", 90), ("+180°", 180)
        ]

        for i, (label, angle) in enumerate(positions):
            btn = QPushButton(label)
            btn.clicked.connect(lambda checked, a=angle: self.move_to_angle(a))
            quick_pos_layout.addWidget(btn, 0, i)

        quick_pos_group.setLayout(quick_pos_layout)
        layout.addWidget(quick_pos_group)

        # Status display
        status_group = QGroupBox("Servo Status")
        status_layout = QFormLayout()

        self.current_angle_label = QLabel("0.0°")
        self.target_angle_label = QLabel("0.0°")
        self.pwm_value_label = QLabel("1000 μs")
        self.status_label = QLabel("Unknown")

        status_layout.addRow("Current Angle:", self.current_angle_label)
        status_layout.addRow("Target Angle:", self.target_angle_label)
        status_layout.addRow("PWM Value:", self.pwm_value_label)
        status_layout.addRow("Status:", self.status_label)

        status_group.setLayout(status_layout)
        layout.addWidget(status_group)

        layout.addStretch()
        self.setLayout(layout)

        # Update initial state
        self.on_servo_changed()

    def set_servo_command_publisher(self, publisher):
        """Set the servo command publisher"""
        self.servo_command_publisher = publisher

    def on_servo_changed(self):
        self.current_servo_id = self.servo_id_spinbox.value()

    def send_command(self):
        """Send servo control command"""
        if not self.servo_command_publisher:
            logger.warning("Servo command publisher not available")
            return

        cmd = ServoCommand()
        cmd.servo_id = self.current_servo_id
        cmd.enable = self.enable_checkbox.isChecked()

        if self.mode_group.checkedId() == 0:  # Angle control
            cmd.angle_deg = self.angle_spinbox.value()
            cmd.pulse_us = 0  # Let the system calculate PWM
            logger.info(f"Sending servo command: ID={cmd.servo_id}, angle={cmd.angle_deg}°, enabled={cmd.enable}")
        else:  # PWM control
            cmd.pulse_us = self.pwm_spinbox.value()
            cmd.angle_deg = 0.0  # PWM mode, angle is ignored
            logger.info(f"Sending servo command: ID={cmd.servo_id}, PWM={cmd.pulse_us}μs, enabled={cmd.enable}")

        self.servo_command_publisher.publish(cmd)

    def move_to_center(self):
        """Move servo to center position (0°)"""
        self.angle_spinbox.setValue(0.0)
        self.angle_mode_radio.setChecked(True)
        self.send_command()

    def move_to_angle(self, angle):
        """Move servo to specific angle"""
        self.angle_spinbox.setValue(angle)
        self.angle_mode_radio.setChecked(True)
        self.send_command()

    def disable_servo(self):
        """Disable servo"""
        self.enable_checkbox.setChecked(False)
        self.send_command()

    def update_status(self, state_msg):
        """Update status display with received state message"""
        if state_msg.servo_id != self.current_servo_id:
            return

        self.current_angle_label.setText(f"{state_msg.current_angle_deg:.1f}°")
        self.target_angle_label.setText(f"{state_msg.target_angle_deg:.1f}°")
        self.pwm_value_label.setText(f"{state_msg.pulse_us} μs")

        status_map = {
            0: "OK",
            1: "NOT_INITIALIZED",
            2: "TIMER_ERROR",
            3: "OUT_OF_RANGE",
            4: "TIMEOUT",
            5: "CONFIG_ERROR"
        }
        status_text = status_map.get(state_msg.status, "UNKNOWN")
        self.status_label.setText(status_text)

        # Update enable checkbox based on servo state
        self.enable_checkbox.setChecked(state_msg.enabled)


class MonitorWidget(QWidget):
    """Widget for real-time monitoring of device status"""

    def __init__(self):
        super().__init__()
        self.setup_ui()
        self.monitoring_data = {}

    def setup_ui(self):
        layout = QVBoxLayout()

        # Control panel
        control_layout = QHBoxLayout()
        self.start_monitoring_btn = QPushButton("Start Monitoring")
        self.stop_monitoring_btn = QPushButton("Stop Monitoring")
        self.clear_data_btn = QPushButton("Clear Data")

        control_layout.addWidget(self.start_monitoring_btn)
        control_layout.addWidget(self.stop_monitoring_btn)
        control_layout.addWidget(self.clear_data_btn)
        control_layout.addStretch()

        layout.addLayout(control_layout)

        # Data display table
        self.data_table = QTableWidget()
        self.data_table.setColumnCount(7)
        self.data_table.setHorizontalHeaderLabels([
            'Device Type', 'ID', 'Position/Angle', 'Velocity', 'Current', 'Temperature', 'Status'
        ])
        self.data_table.horizontalHeader().setStretchLastSection(True)

        layout.addWidget(self.data_table)

        self.setLayout(layout)

    def update_device_data(self, device_type: str, device_id: int, data: Dict[str, Any]):
        """Update monitoring data for a device"""
        key = f"{device_type}_{device_id}"
        self.monitoring_data[key] = data
        self.refresh_table()

    def refresh_table(self):
        """Refresh the monitoring table with current data"""
        self.data_table.setRowCount(len(self.monitoring_data))

        for row, (key, data) in enumerate(self.monitoring_data.items()):
            device_type, device_id = key.split('_')

            self.data_table.setItem(row, 0, QTableWidgetItem(device_type.title()))
            self.data_table.setItem(row, 1, QTableWidgetItem(device_id))
            self.data_table.setItem(row, 2, QTableWidgetItem(str(data.get('position', 'N/A'))))
            self.data_table.setItem(row, 3, QTableWidgetItem(str(data.get('velocity', 'N/A'))))
            self.data_table.setItem(row, 4, QTableWidgetItem(str(data.get('current', 'N/A'))))
            self.data_table.setItem(row, 5, QTableWidgetItem(str(data.get('temperature', 'N/A'))))
            self.data_table.setItem(row, 6, QTableWidgetItem(data.get('status', 'Unknown')))

class MAVLinkWizardGUI(QMainWindow):
    """Main GUI application for MAVLink Wizard"""

    def __init__(self):
        super().__init__()
        self.ros_node = None
        self.parameter_manager = None
        self.executor = None
        self.ros_thread = None

        # Configuration management
        config_path = os.path.expanduser("~/.config/mavlink_wizard/config.json")
        self.config_manager = ConfigurationManager(config_path)

        # Progress tracking
        self.progress_dialog = None
        self.operation_status = {}

        self.setup_ui()
        self.setup_ros()

    def setup_ui(self):
        self.setWindowTitle("MAVLink Wizard - Device Configuration Tool")
        self.setGeometry(100, 100, 1200, 800)

        # Central widget with splitter
        central_widget = QWidget()
        self.setCentralWidget(central_widget)

        main_layout = QHBoxLayout()
        splitter = QSplitter(Qt.Horizontal)

        # Left panel - Device tree and scanning
        left_panel = QWidget()
        left_layout = QVBoxLayout()

        # Scan control
        scan_group = QGroupBox("Device Scanner")
        scan_layout = QVBoxLayout()

        # Information label
        info_label = QLabel("Device scanner monitors ROS2 topics for active devices.\n"
                           "Make sure the stm32_mavlink_interface node is running.")
        info_label.setWordWrap(True)
        info_label.setStyleSheet("color: #888888; font-style: italic;")
        scan_layout.addWidget(info_label)

        # Scan button
        self.scan_button = QPushButton("Start Monitoring")
        self.scan_button.clicked.connect(self.scan_devices)
        scan_layout.addWidget(self.scan_button)

        # Stop button
        self.stop_scan_button = QPushButton("Stop Monitoring")
        self.stop_scan_button.clicked.connect(self.stop_scan_devices)
        self.stop_scan_button.setEnabled(False)
        scan_layout.addWidget(self.stop_scan_button)

        scan_group.setLayout(scan_layout)
        left_layout.addWidget(scan_group)

        # Device tree
        self.device_tree = DeviceTreeWidget()
        self.device_tree.device_selected.connect(self.on_device_selected)
        left_layout.addWidget(self.device_tree)

        left_panel.setLayout(left_layout)
        splitter.addWidget(left_panel)

        # Right panel - Tabs for different functions
        self.tab_widget = QTabWidget()

        # Configuration tab - will be initialized after ROS setup
        self.config_widget = None

        # Servo Control tab - will be initialized after ROS setup
        self.servo_control_widget = ServoControlWidget()
        self.tab_widget.addTab(self.servo_control_widget, "Servo Control")

        # Motor Control tab - will be initialized after ROS setup
        self.motor_control_widget = MotorControlWidget()
        self.tab_widget.addTab(self.motor_control_widget, "Motor Control")

        # DC Motor Control tab - will be initialized after ROS setup
        self.dcmotor_control_widget = DCMotorControlWidget()
        self.tab_widget.addTab(self.dcmotor_control_widget, "DC Motor Control")

        # Monitoring tab
        self.monitor_widget = MonitorWidget()
        self.tab_widget.addTab(self.monitor_widget, "Real-time Monitor")

        # Firmware tab (placeholder)
        firmware_widget = QLabel("Firmware update functionality will be implemented here")
        firmware_widget.setAlignment(Qt.AlignCenter)
        self.tab_widget.addTab(firmware_widget, "Firmware")

        splitter.addWidget(self.tab_widget)
        splitter.setSizes([300, 900])

        main_layout.addWidget(splitter)
        central_widget.setLayout(main_layout)

        # Status bar
        self.status_bar = self.statusBar()
        self.status_bar.showMessage("Ready - No devices connected")

        # Menu bar
        self.create_menu_bar()

    def create_menu_bar(self):
        menubar = self.menuBar()

        # File menu
        file_menu = menubar.addMenu('File')

        save_config_action = QAction('Save Configuration', self)
        save_config_action.triggered.connect(self.save_configuration)
        file_menu.addAction(save_config_action)

        load_config_action = QAction('Load Configuration', self)
        load_config_action.triggered.connect(self.load_configuration)
        file_menu.addAction(load_config_action)

        file_menu.addSeparator()

        exit_action = QAction('Exit', self)
        exit_action.triggered.connect(self.close)
        file_menu.addAction(exit_action)

        # Tools menu
        tools_menu = menubar.addMenu('Tools')

        rescan_action = QAction('Rescan Devices', self)
        rescan_action.triggered.connect(self.scan_devices)
        tools_menu.addAction(rescan_action)

        # Help menu
        help_menu = menubar.addMenu('Help')

        about_action = QAction('About', self)
        about_action.triggered.connect(self.show_about)
        help_menu.addAction(about_action)

    def setup_ros(self):
        """Initialize ROS2 node and communication"""
        try:
            if not rclpy.ok():
                rclpy.init()

            # Create signal bridge for Qt communication
            self.signal_bridge = SignalBridge()
            self.ros_node = MAVLinkWizardNode(self.signal_bridge)

            # Initialize parameter manager
            self.parameter_manager = ParameterManager()

            # Initialize configuration widget now that parameter manager is ready
            self.config_widget = ParameterConfigWidget(self.parameter_manager)
            self.tab_widget.insertTab(0, self.config_widget, "Configuration")

            # Initialize device scanner with callbacks
            self.device_scanner = DeviceScanner(
                on_device_found=self.on_device_found,
                on_device_lost=self.on_device_lost
            )

            self.executor = MultiThreadedExecutor()
            self.executor.add_node(self.ros_node)
            self.executor.add_node(self.device_scanner)
            self.executor.add_node(self.parameter_manager)

            # Start ROS spinning in separate thread
            self.ros_thread = QThread()
            self.ros_thread.run = lambda: self.executor.spin()
            self.ros_thread.start()

            # Set servo command publisher for servo control widget
            if hasattr(self.ros_node, 'servo_command_pub'):
                self.servo_control_widget.set_servo_command_publisher(self.ros_node.servo_command_pub)

            # Set motor command publisher for motor control widget
            if hasattr(self.ros_node, 'motor_command_pub'):
                self.motor_control_widget.set_motor_command_publisher(self.ros_node.motor_command_pub)

            # Set DC motor command publisher for DC motor control widget
            if hasattr(self.ros_node, 'dcmotor_command_pub'):
                self.dcmotor_control_widget.dcmotor_command_publisher = self.ros_node.dcmotor_command_pub

            # Connect signal bridge for real-time monitoring
            self.signal_bridge.servo_state_signal.connect(self.on_servo_state_received)
            self.signal_bridge.motor_state_signal.connect(self.on_motor_state_received)
            self.signal_bridge.dcmotor_state_signal.connect(self.on_dcmotor_state_received)

            self.status_bar.showMessage("ROS2 nodes initialized successfully")

        except Exception as e:
            QMessageBox.critical(self, "ROS2 Error", f"Failed to initialize ROS2: {str(e)}")

    def scan_devices(self):
        """Start monitoring for connected MAVLink devices"""
        if hasattr(self, 'device_scanner') and self.device_scanner:
            self.device_scanner.start_scan()
            self.scan_button.setEnabled(False)
            self.stop_scan_button.setEnabled(True)
            self.status_bar.showMessage("Monitoring ROS2 topics for active devices...")
        else:
            self.status_bar.showMessage("Device scanner not available - check ROS2 connection")

    def stop_scan_devices(self):
        """Stop monitoring for devices"""
        if hasattr(self, 'device_scanner') and self.device_scanner:
            self.device_scanner.stop_scan()
            self.scan_button.setEnabled(True)
            self.stop_scan_button.setEnabled(False)
            self.status_bar.showMessage("Device monitoring stopped")

            # Clear the device tree
            self.device_tree.clear_devices()

    def on_device_found(self, device):
        """Callback when a new device is discovered"""
        self.device_tree.add_device(device.device_type, device.device_id, device.status)
        device_count = len(self.device_scanner.get_discovered_devices())
        self.status_bar.showMessage(f"Found {device.device_type} ID {device.device_id} - Total: {device_count} devices")

        # Create parameter set for new device if parameter manager is available
        if hasattr(self, 'parameter_manager') and self.parameter_manager:
            if not self.parameter_manager.get_device_parameters(device.device_type, device.device_id):
                self.parameter_manager.create_device_parameters(device.device_type, device.device_id)

    def on_device_lost(self, device):
        """Callback when a device is lost/disconnected"""
        # For now, we'll let the timeout mechanism handle device removal
        # The GUI will be updated through the regular refresh cycle
        device_count = len(self.device_scanner.get_discovered_devices())
        self.status_bar.showMessage(f"Lost {device.device_type} ID {device.device_id} - Total: {device_count} devices")

    def on_device_selected(self, device_type: str, device_id: int):
        """Handle device selection from tree"""
        if self.config_widget:
            self.config_widget.set_device(device_type, device_id)

        # Update servo control widget if servo is selected
        if device_type == 'servo':
            self.servo_control_widget.servo_id_spinbox.setValue(device_id)
            self.servo_control_widget.on_servo_changed()

        self.status_bar.showMessage(f"Selected {device_type} ID {device_id}")

    def on_servo_state_received(self, msg):
        """Handle servo state messages for real-time monitoring"""
        data = {
            'position': f"{msg.current_angle_deg:.1f}°",
            'velocity': 'N/A',  # Servos don't have velocity feedback
            'current': 'N/A',   # Most servos don't report current
            'temperature': 'N/A',  # Most servos don't have temperature
            'status': 'OK' if msg.status == 0 else 'ERROR'
        }
        self.monitor_widget.update_device_data('servo', msg.servo_id, data)

        # Update servo control widget status
        self.servo_control_widget.update_status(msg)

    def on_motor_state_received(self, msg):
        """Handle motor state messages for real-time monitoring"""
        data = {
            'position': f"{msg.current_position_rad:.3f} rad",
            'velocity': f"{msg.current_velocity_rps:.2f} RPS",
            'current': f"{msg.current_milliamps} mA",
            'temperature': f"{msg.temperature_celsius}°C",
            'status': 'OK' if msg.status == 0 else 'ERROR'
        }
        self.monitor_widget.update_device_data('motor', msg.motor_id, data)

    def on_dcmotor_state_received(self, msg):
        """Handle DC motor state messages for real-time monitoring"""
        status_map = {
            0: "OK",
            1: "NOT_INITIALIZED",
            2: "ERROR",
            3: "OVERHEAT",
            4: "OVERCURRENT",
            5: "TIMEOUT"
        }

        data = {
            'position': f"{msg.position_rad:.3f} rad",
            'velocity': f"{msg.velocity_rad_s:.2f} rad/s",
            'current': f"{msg.current_a:.2f} A",
            'temperature': f"{msg.temperature_c:.1f}°C",
            'status': status_map.get(msg.status, "UNKNOWN"),
            'enabled': 'Yes' if msg.enabled else 'No'
        }
        self.monitor_widget.update_device_data('dcmotor', msg.motor_id, data)

        # Update DC motor control widget status
        self.dcmotor_control_widget.update_status(msg)

    def save_configuration(self):
        """Save current device configuration to file"""
        filename, _ = QFileDialog.getSaveFileName(
            self, "Save Configuration", "", "JSON Files (*.json)"
        )
        if filename:
            # Implementation for saving configuration
            QMessageBox.information(self, "Save", "Configuration saved successfully")

    def load_configuration(self):
        """Load device configuration from file"""
        filename, _ = QFileDialog.getOpenFileName(
            self, "Load Configuration", "", "JSON Files (*.json)"
        )
        if filename:
            # Implementation for loading configuration
            QMessageBox.information(self, "Load", "Configuration loaded successfully")

    def show_about(self):
        """Show about dialog"""
        QMessageBox.about(self, "About MAVLink Wizard",
                         "MAVLink Wizard v1.0\n\n"
                         "A GUI tool for configuring and monitoring MAVLink devices,\n"
                         "similar to Dynamixel Wizard.\n\n"
                         "Built with ROS2 and PyQt5")

    @error_handler
    def show_error_message(self, title: str, message: str):
        """Show error message dialog"""
        QMessageBox.critical(self, title, message)

    @error_handler
    def show_info_message(self, title: str, message: str):
        """Show information message dialog"""
        QMessageBox.information(self, title, message)

    @error_handler
    def show_warning_message(self, title: str, message: str):
        """Show warning message dialog"""
        QMessageBox.warning(self, title, message)

    def show_progress_dialog(self, title: str, message: str, max_value: int = 0):
        """Show progress dialog for long operations"""
        if self.progress_dialog:
            self.progress_dialog.close()

        self.progress_dialog = QProgressDialog(message, "Cancel", 0, max_value, self)
        self.progress_dialog.setWindowTitle(title)
        self.progress_dialog.setModal(True)
        self.progress_dialog.setAutoClose(True)
        self.progress_dialog.show()

        return self.progress_dialog

    def update_progress(self, value: int, message: str = None):
        """Update progress dialog"""
        if self.progress_dialog:
            self.progress_dialog.setValue(value)
            if message:
                self.progress_dialog.setLabelText(message)

    def hide_progress_dialog(self):
        """Hide progress dialog"""
        if self.progress_dialog:
            self.progress_dialog.close()
            self.progress_dialog = None

    @error_handler
    def validate_parameter_input(self, param_name: str, value: str, param_type: str, min_val: float = None, max_val: float = None) -> bool:
        """Validate parameter input with real-time feedback"""
        try:
            if param_type in ['INT8', 'INT16', 'INT32', 'UINT8', 'UINT16', 'UINT32']:
                val = int(value)
            elif param_type in ['REAL32', 'REAL64']:
                val = float(value)
            else:
                return True  # String types don't need numerical validation

            if min_val is not None and val < min_val:
                self.show_warning_message("Validation Error", f"{param_name}: Value {val} is below minimum {min_val}")
                return False

            if max_val is not None and val > max_val:
                self.show_warning_message("Validation Error", f"{param_name}: Value {val} is above maximum {max_val}")
                return False

            return True

        except ValueError:
            self.show_warning_message("Validation Error", f"{param_name}: Invalid {param_type} value: {value}")
            return False

    @error_handler
    def batch_update_parameters(self, device_type: str, device_ids: List[int], parameters: Dict[str, Any]):
        """Update parameters for multiple devices"""
        progress_dialog = self.show_progress_dialog(
            "Batch Update", f"Updating parameters for {len(device_ids)} {device_type}s...", len(device_ids)
        )

        success_count = 0
        for i, device_id in enumerate(device_ids):
            if progress_dialog.wasCanceled():
                break

            self.update_progress(i, f"Updating {device_type} {device_id}...")

            try:
                if self.parameter_manager:
                    for param_name, value in parameters.items():
                        self.parameter_manager.set_parameter_value(device_type, device_id, param_name, value)
                success_count += 1
            except Exception as e:
                logger.error(f"Failed to update {device_type} {device_id}: {e}")

        self.hide_progress_dialog()
        self.show_info_message("Batch Update Complete", f"Successfully updated {success_count}/{len(device_ids)} devices")

    def get_connection_health(self, device_type: str, device_id: int) -> Dict[str, Any]:
        """Get connection health information for a device"""
        if not hasattr(self, 'device_scanner') or not self.device_scanner:
            return {'status': 'unknown', 'latency': 0, 'packet_loss': 0}

        devices = self.device_scanner.get_discovered_devices()
        for device in devices:
            if device.device_type == device_type and device.device_id == device_id:
                # Calculate basic health metrics
                current_time = QTime.currentTime()
                last_seen = device.last_seen if hasattr(device, 'last_seen') else current_time

                latency = current_time.msecsTo(last_seen) if hasattr(QTime, 'msecsTo') else 0
                health_status = 'good' if abs(latency) < 1000 else 'poor'

                return {
                    'status': health_status,
                    'latency': abs(latency),
                    'packet_loss': 0,  # Placeholder - would need packet counting
                    'last_seen': last_seen.toString()
                }

        return {'status': 'disconnected', 'latency': 9999, 'packet_loss': 100}

    def closeEvent(self, event):
        """Clean up when closing the application"""
        if hasattr(self, 'device_scanner') and self.device_scanner:
            self.device_scanner.stop_scan()

        if self.ros_thread and self.ros_thread.isRunning():
            self.executor.shutdown()
            self.ros_thread.quit()
            self.ros_thread.wait()

        # Clean up parameter manager
        if hasattr(self, 'parameter_manager') and self.parameter_manager:
            self.parameter_manager.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()

        event.accept()

def main():
    app = QApplication(sys.argv)
    app.setApplicationName("MAVLink Wizard")
    app.setOrganizationName("ROS2 MAVLink Tools")

    # Set application style
    app.setStyle('Fusion')

    # Apply dark theme
    palette = QPalette()
    palette.setColor(QPalette.Window, QColor(53, 53, 53))
    palette.setColor(QPalette.WindowText, QColor(255, 255, 255))
    palette.setColor(QPalette.Base, QColor(25, 25, 25))
    palette.setColor(QPalette.AlternateBase, QColor(53, 53, 53))
    palette.setColor(QPalette.ToolTipBase, QColor(0, 0, 0))
    palette.setColor(QPalette.ToolTipText, QColor(255, 255, 255))
    palette.setColor(QPalette.Text, QColor(255, 255, 255))
    palette.setColor(QPalette.Button, QColor(53, 53, 53))
    palette.setColor(QPalette.ButtonText, QColor(255, 255, 255))
    palette.setColor(QPalette.BrightText, QColor(255, 0, 0))
    palette.setColor(QPalette.Link, QColor(42, 130, 218))
    palette.setColor(QPalette.Highlight, QColor(42, 130, 218))
    palette.setColor(QPalette.HighlightedText, QColor(0, 0, 0))
    app.setPalette(palette)

    window = MAVLinkWizardGUI()
    window.show()

    sys.exit(app.exec_())

if __name__ == '__main__':
    main()