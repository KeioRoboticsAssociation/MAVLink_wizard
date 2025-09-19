#!/usr/bin/env python3

import logging
import math
from typing import Dict, List, Optional, Tuple, Callable
from dataclasses import dataclass
from enum import Enum
import time

from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *

import rclpy
from rclpy.node import Node

logger = logging.getLogger(__name__)

class CalibrationStep(Enum):
    """Steps in the calibration process"""
    PREPARE = "prepare"
    CENTER_POSITION = "center_position"
    MIN_POSITION = "min_position"
    MAX_POSITION = "max_position"
    VERIFY = "verify"
    COMPLETE = "complete"

@dataclass
class CalibrationData:
    """Data collected during calibration"""
    device_type: str
    device_id: int
    center_position: Optional[float] = None
    min_position: Optional[float] = None
    max_position: Optional[float] = None
    range_degrees: Optional[float] = None
    zero_offset: Optional[float] = None
    completed: bool = False

class ServoCalibrationWizard(QWizard):
    """Wizard for calibrating servo motors"""

    def __init__(self, device_id: int, parameter_manager=None, parent=None):
        super().__init__(parent)
        self.device_id = device_id
        self.parameter_manager = parameter_manager
        self.calibration_data = CalibrationData("servo", device_id)

        self.setWindowTitle(f"Servo Calibration Wizard - ID {device_id}")
        self.setFixedSize(600, 500)

        # Add pages
        self.addPage(self.create_intro_page())
        self.addPage(self.create_center_calibration_page())
        self.addPage(self.create_range_calibration_page())
        self.addPage(self.create_verification_page())
        self.addPage(self.create_completion_page())

    def create_intro_page(self) -> QWizardPage:
        """Create introduction page"""
        page = QWizardPage()
        page.setTitle("Servo Calibration - Introduction")
        page.setSubTitle("This wizard will help you calibrate your servo motor for accurate positioning.")

        layout = QVBoxLayout()

        # Instructions
        instructions = """
        <h3>Calibration Process Overview:</h3>
        <ol>
            <li><b>Center Position:</b> Set the servo to its mechanical center position</li>
            <li><b>Range Calibration:</b> Determine the minimum and maximum positions</li>
            <li><b>Verification:</b> Test the calibrated settings</li>
            <li><b>Save Settings:</b> Apply the calibration to the servo</li>
        </ol>

        <h3>Before Starting:</h3>
        <ul>
            <li>Ensure the servo is properly connected and powered</li>
            <li>Remove any mechanical loads if possible</li>
            <li>Have the servo horn or attachment ready for visual reference</li>
        </ul>

        <h3>Safety Notes:</h3>
        <ul>
            <li>The servo will move during calibration</li>
            <li>Be ready to power off if something goes wrong</li>
            <li>Do not force the servo beyond its mechanical limits</li>
        </ul>
        """

        info_label = QLabel(instructions)
        info_label.setWordWrap(True)
        layout.addWidget(info_label)

        layout.addStretch()
        page.setLayout(layout)

        return page

    def create_center_calibration_page(self) -> QWizardPage:
        """Create center position calibration page"""
        page = QWizardPage()
        page.setTitle("Center Position Calibration")
        page.setSubTitle("Set the servo to its mechanical center position")

        layout = QVBoxLayout()

        # Instructions
        instructions = QLabel(
            "Move the servo to its center position using the controls below. "
            "The center position is typically where the servo horn is perpendicular to the servo body."
        )
        instructions.setWordWrap(True)
        layout.addWidget(instructions)

        # Control group
        control_group = QGroupBox("Servo Control")
        control_layout = QVBoxLayout()

        # Position control
        position_layout = QHBoxLayout()
        position_layout.addWidget(QLabel("Position:"))

        self.center_position_slider = QSlider(Qt.Horizontal)
        self.center_position_slider.setRange(-1800, 1800)  # -180 to 180 degrees * 10
        self.center_position_slider.setValue(0)
        self.center_position_slider.valueChanged.connect(self.update_center_position)

        self.center_position_spinbox = QDoubleSpinBox()
        self.center_position_spinbox.setRange(-180.0, 180.0)
        self.center_position_spinbox.setSingleStep(0.1)
        self.center_position_spinbox.setSuffix("°")
        self.center_position_spinbox.valueChanged.connect(self.center_spinbox_changed)

        position_layout.addWidget(self.center_position_slider)
        position_layout.addWidget(self.center_position_spinbox)

        control_layout.addLayout(position_layout)

        # Quick buttons
        quick_layout = QHBoxLayout()
        for angle in [-180, -90, 0, 90, 180]:
            btn = QPushButton(f"{angle}°")
            btn.clicked.connect(lambda checked, a=angle: self.set_center_position(a))
            quick_layout.addWidget(btn)

        control_layout.addLayout(quick_layout)

        # Enable/disable
        self.center_enable_checkbox = QCheckBox("Enable Servo")
        self.center_enable_checkbox.setChecked(True)
        self.center_enable_checkbox.stateChanged.connect(self.toggle_servo_enable)
        control_layout.addWidget(self.center_enable_checkbox)

        control_group.setLayout(control_layout)
        layout.addWidget(control_group)

        # Confirmation
        confirm_group = QGroupBox("Confirmation")
        confirm_layout = QVBoxLayout()

        confirm_layout.addWidget(QLabel("When the servo is at its mechanical center position:"))

        self.center_confirm_checkbox = QCheckBox("The servo is at the center position")
        confirm_layout.addWidget(self.center_confirm_checkbox)

        confirm_group.setLayout(confirm_layout)
        layout.addWidget(confirm_group)

        layout.addStretch()
        page.setLayout(layout)

        # Set validation
        page.registerField("center_confirmed", self.center_confirm_checkbox)

        return page

    def create_range_calibration_page(self) -> QWizardPage:
        """Create range calibration page"""
        page = QWizardPage()
        page.setTitle("Range Calibration")
        page.setSubTitle("Determine the minimum and maximum positions")

        layout = QVBoxLayout()

        # Instructions
        instructions = QLabel(
            "Find the minimum and maximum positions where the servo can move without binding. "
            "Do not force the servo beyond its mechanical limits."
        )
        instructions.setWordWrap(True)
        layout.addWidget(instructions)

        # Range controls
        range_group = QGroupBox("Range Detection")
        range_layout = QVBoxLayout()

        # Minimum position
        min_layout = QHBoxLayout()
        min_layout.addWidget(QLabel("Min Position:"))

        self.min_position_slider = QSlider(Qt.Horizontal)
        self.min_position_slider.setRange(-1800, 0)
        self.min_position_slider.setValue(-1800)  # -180 degrees default
        self.min_position_slider.valueChanged.connect(self.update_min_position)

        self.min_position_spinbox = QDoubleSpinBox()
        self.min_position_spinbox.setRange(-180.0, 180.0)
        self.min_position_spinbox.setSingleStep(0.1)
        self.min_position_spinbox.setSuffix("°")
        self.min_position_spinbox.setValue(-180.0)
        self.min_position_spinbox.valueChanged.connect(self.min_spinbox_changed)

        min_test_btn = QPushButton("Test Min")
        min_test_btn.clicked.connect(self.test_min_position)

        min_layout.addWidget(self.min_position_slider)
        min_layout.addWidget(self.min_position_spinbox)
        min_layout.addWidget(min_test_btn)

        range_layout.addLayout(min_layout)

        # Maximum position
        max_layout = QHBoxLayout()
        max_layout.addWidget(QLabel("Max Position:"))

        self.max_position_slider = QSlider(Qt.Horizontal)
        self.max_position_slider.setRange(0, 1800)
        self.max_position_slider.setValue(1800)  # 180 degrees default
        self.max_position_slider.valueChanged.connect(self.update_max_position)

        self.max_position_spinbox = QDoubleSpinBox()
        self.max_position_spinbox.setRange(-180.0, 180.0)
        self.max_position_spinbox.setSingleStep(0.1)
        self.max_position_spinbox.setSuffix("°")
        self.max_position_spinbox.setValue(180.0)
        self.max_position_spinbox.valueChanged.connect(self.max_spinbox_changed)

        max_test_btn = QPushButton("Test Max")
        max_test_btn.clicked.connect(self.test_max_position)

        max_layout.addWidget(self.max_position_slider)
        max_layout.addWidget(self.max_position_spinbox)
        max_layout.addWidget(max_test_btn)

        range_layout.addLayout(max_layout)

        # Range sweep
        sweep_layout = QHBoxLayout()
        sweep_btn = QPushButton("Sweep Range")
        sweep_btn.clicked.connect(self.sweep_range)

        self.range_display = QLabel("Range: 360.0°")
        self.range_display.setStyleSheet("font-weight: bold; color: blue;")

        sweep_layout.addWidget(sweep_btn)
        sweep_layout.addStretch()
        sweep_layout.addWidget(self.range_display)

        range_layout.addLayout(sweep_layout)

        range_group.setLayout(range_layout)
        layout.addWidget(range_group)

        # Confirmation
        confirm_group = QGroupBox("Confirmation")
        confirm_layout = QVBoxLayout()

        self.range_confirm_checkbox = QCheckBox("The range limits are correct and safe")
        confirm_layout.addWidget(self.range_confirm_checkbox)

        confirm_group.setLayout(confirm_layout)
        layout.addWidget(confirm_group)

        layout.addStretch()
        page.setLayout(layout)

        # Set validation
        page.registerField("range_confirmed", self.range_confirm_checkbox)

        return page

    def create_verification_page(self) -> QWizardPage:
        """Create verification page"""
        page = QWizardPage()
        page.setTitle("Verification")
        page.setSubTitle("Test the calibrated settings")

        layout = QVBoxLayout()

        # Summary
        self.summary_label = QLabel()
        self.update_summary()
        layout.addWidget(self.summary_label)

        # Test controls
        test_group = QGroupBox("Test Controls")
        test_layout = QVBoxLayout()

        # Test buttons
        test_button_layout = QHBoxLayout()

        test_center_btn = QPushButton("Test Center")
        test_center_btn.clicked.connect(lambda: self.test_position(0))

        test_min_btn = QPushButton("Test Min")
        test_min_btn.clicked.connect(self.test_calibrated_min)

        test_max_btn = QPushButton("Test Max")
        test_max_btn.clicked.connect(self.test_calibrated_max)

        test_sequence_btn = QPushButton("Test Sequence")
        test_sequence_btn.clicked.connect(self.test_sequence)

        test_button_layout.addWidget(test_center_btn)
        test_button_layout.addWidget(test_min_btn)
        test_button_layout.addWidget(test_max_btn)
        test_button_layout.addWidget(test_sequence_btn)

        test_layout.addLayout(test_button_layout)

        # Test result
        self.test_result_label = QLabel("Click test buttons to verify calibration")
        test_layout.addWidget(self.test_result_label)

        test_group.setLayout(test_layout)
        layout.addWidget(test_group)

        # Final confirmation
        confirm_group = QGroupBox("Final Confirmation")
        confirm_layout = QVBoxLayout()

        self.verification_confirm_checkbox = QCheckBox("Calibration is correct and ready to save")
        confirm_layout.addWidget(self.verification_confirm_checkbox)

        confirm_group.setLayout(confirm_layout)
        layout.addWidget(confirm_group)

        layout.addStretch()
        page.setLayout(layout)

        # Set validation
        page.registerField("verification_confirmed", self.verification_confirm_checkbox)

        return page

    def create_completion_page(self) -> QWizardPage:
        """Create completion page"""
        page = QWizardPage()
        page.setTitle("Calibration Complete")
        page.setSubTitle("Calibration has been completed and saved")

        layout = QVBoxLayout()

        # Success message
        success_label = QLabel("✅ Servo calibration completed successfully!")
        success_label.setFont(QFont("Arial", 14, QFont.Bold))
        success_label.setStyleSheet("color: green;")
        layout.addWidget(success_label)

        # Final summary
        self.final_summary_label = QLabel()
        layout.addWidget(self.final_summary_label)

        # Next steps
        next_steps = QLabel("""
        <h3>Next Steps:</h3>
        <ul>
            <li>The calibration settings have been saved to the servo</li>
            <li>You can now use precise angle positioning</li>
            <li>The servo will remember these settings until recalibrated</li>
        </ul>
        """)
        next_steps.setWordWrap(True)
        layout.addWidget(next_steps)

        layout.addStretch()
        page.setLayout(layout)

        return page

    def update_center_position(self, value):
        """Update center position from slider"""
        angle = value / 10.0  # Convert from tenths of degrees
        self.center_position_spinbox.setValue(angle)
        self.send_servo_command(angle)

    def center_spinbox_changed(self, value):
        """Update center position from spinbox"""
        self.center_position_slider.setValue(int(value * 10))
        self.send_servo_command(value)

    def set_center_position(self, angle):
        """Set center position to specific angle"""
        self.center_position_spinbox.setValue(angle)
        self.center_position_slider.setValue(int(angle * 10))
        self.send_servo_command(angle)

    def update_min_position(self, value):
        """Update minimum position from slider"""
        angle = value / 10.0
        self.min_position_spinbox.setValue(angle)
        self.update_range_display()

    def min_spinbox_changed(self, value):
        """Update minimum position from spinbox"""
        self.min_position_slider.setValue(int(value * 10))
        self.update_range_display()

    def update_max_position(self, value):
        """Update maximum position from slider"""
        angle = value / 10.0
        self.max_position_spinbox.setValue(angle)
        self.update_range_display()

    def max_spinbox_changed(self, value):
        """Update maximum position from spinbox"""
        self.max_position_slider.setValue(int(value * 10))
        self.update_range_display()

    def update_range_display(self):
        """Update range display label"""
        min_pos = self.min_position_spinbox.value()
        max_pos = self.max_position_spinbox.value()
        range_degrees = max_pos - min_pos
        self.range_display.setText(f"Range: {range_degrees:.1f}°")

    def test_min_position(self):
        """Test minimum position"""
        angle = self.min_position_spinbox.value()
        self.send_servo_command(angle)

    def test_max_position(self):
        """Test maximum position"""
        angle = self.max_position_spinbox.value()
        self.send_servo_command(angle)

    def sweep_range(self):
        """Sweep through the full range"""
        min_angle = self.min_position_spinbox.value()
        max_angle = self.max_position_spinbox.value()

        # Create sweep sequence
        steps = 20
        angles = [min_angle + (max_angle - min_angle) * i / (steps - 1) for i in range(steps)]
        angles.extend(reversed(angles[:-1]))  # Add return sweep

        self.sweep_timer = QTimer()
        self.sweep_angles = angles
        self.sweep_index = 0

        def sweep_step():
            if self.sweep_index < len(self.sweep_angles):
                angle = self.sweep_angles[self.sweep_index]
                self.send_servo_command(angle)
                self.sweep_index += 1
            else:
                self.sweep_timer.stop()
                # Return to center
                self.send_servo_command(0)

        self.sweep_timer.timeout.connect(sweep_step)
        self.sweep_timer.start(200)  # 200ms between steps

    def test_position(self, angle):
        """Test a specific position"""
        self.send_servo_command(angle)

    def test_calibrated_min(self):
        """Test calibrated minimum position"""
        self.test_position(self.min_position_spinbox.value())

    def test_calibrated_max(self):
        """Test calibrated maximum position"""
        self.test_position(self.max_position_spinbox.value())

    def test_sequence(self):
        """Run a test sequence"""
        positions = [
            0,  # Center
            self.min_position_spinbox.value(),  # Min
            0,  # Center
            self.max_position_spinbox.value(),  # Max
            0,  # Center
        ]

        self.test_timer = QTimer()
        self.test_positions = positions
        self.test_index = 0

        def test_step():
            if self.test_index < len(self.test_positions):
                angle = self.test_positions[self.test_index]
                self.send_servo_command(angle)
                self.test_result_label.setText(f"Testing position {self.test_index + 1}/{len(self.test_positions)}: {angle:.1f}°")
                self.test_index += 1
            else:
                self.test_timer.stop()
                self.test_result_label.setText("✅ Test sequence completed successfully")

        self.test_timer.timeout.connect(test_step)
        self.test_timer.start(1000)  # 1 second between positions

    def toggle_servo_enable(self, state):
        """Toggle servo enable state"""
        enabled = state == Qt.Checked
        # Send enable/disable command to servo
        logger.info(f"Servo {self.device_id} enable: {enabled}")

    def send_servo_command(self, angle_degrees):
        """Send servo command"""
        if self.parameter_manager:
            # In a real implementation, this would send the command via ROS
            logger.info(f"Moving servo {self.device_id} to {angle_degrees:.1f}°")

    def update_summary(self):
        """Update verification summary"""
        center = self.center_position_spinbox.value() if hasattr(self, 'center_position_spinbox') else 0
        min_pos = self.min_position_spinbox.value() if hasattr(self, 'min_position_spinbox') else -180
        max_pos = self.max_position_spinbox.value() if hasattr(self, 'max_position_spinbox') else 180
        range_deg = max_pos - min_pos

        summary_text = f"""
        <h3>Calibration Summary:</h3>
        <table>
        <tr><td><b>Device:</b></td><td>Servo {self.device_id}</td></tr>
        <tr><td><b>Center Position:</b></td><td>{center:.1f}°</td></tr>
        <tr><td><b>Minimum Position:</b></td><td>{min_pos:.1f}°</td></tr>
        <tr><td><b>Maximum Position:</b></td><td>{max_pos:.1f}°</td></tr>
        <tr><td><b>Total Range:</b></td><td>{range_deg:.1f}°</td></tr>
        </table>
        """

        if hasattr(self, 'summary_label'):
            self.summary_label.setText(summary_text)

        if hasattr(self, 'final_summary_label'):
            self.final_summary_label.setText(summary_text)

    def accept(self):
        """Save calibration and close wizard"""
        # Save calibration data
        self.calibration_data.center_position = self.center_position_spinbox.value()
        self.calibration_data.min_position = self.min_position_spinbox.value()
        self.calibration_data.max_position = self.max_position_spinbox.value()
        self.calibration_data.range_degrees = self.calibration_data.max_position - self.calibration_data.min_position
        self.calibration_data.zero_offset = -self.calibration_data.center_position  # Offset to make center = 0
        self.calibration_data.completed = True

        # Apply calibration to parameter manager
        if self.parameter_manager:
            self.apply_calibration()

        super().accept()

    def apply_calibration(self):
        """Apply calibration to the servo parameters"""
        try:
            # Set calibration parameters
            if hasattr(self.parameter_manager, 'set_parameter_value'):
                self.parameter_manager.set_parameter_value(
                    'servo', self.device_id, 'ANGLE_MIN', self.calibration_data.min_position
                )
                self.parameter_manager.set_parameter_value(
                    'servo', self.device_id, 'ANGLE_MAX', self.calibration_data.max_position
                )
                self.parameter_manager.set_parameter_value(
                    'servo', self.device_id, 'ZERO_OFFSET', self.calibration_data.zero_offset
                )

                logger.info(f"Applied calibration to servo {self.device_id}")

        except Exception as e:
            logger.error(f"Failed to apply calibration: {e}")
            QMessageBox.warning(self, "Calibration Error", f"Failed to save calibration: {str(e)}")

class EncoderCalibrationWizard(QWizard):
    """Wizard for calibrating encoder zero position"""

    def __init__(self, device_id: int, parameter_manager=None, parent=None):
        super().__init__(parent)
        self.device_id = device_id
        self.parameter_manager = parameter_manager
        self.calibration_data = CalibrationData("encoder", device_id)

        self.setWindowTitle(f"Encoder Calibration Wizard - ID {device_id}")
        self.setFixedSize(500, 400)

        # Add pages
        self.addPage(self.create_intro_page())
        self.addPage(self.create_zero_calibration_page())
        self.addPage(self.create_verification_page())
        self.addPage(self.create_completion_page())

    def create_intro_page(self) -> QWizardPage:
        """Create introduction page"""
        page = QWizardPage()
        page.setTitle("Encoder Calibration - Introduction")
        page.setSubTitle("This wizard will help you set the zero position for your encoder.")

        layout = QVBoxLayout()

        instructions = """
        <h3>Calibration Process:</h3>
        <ol>
            <li><b>Position Setup:</b> Manually position the encoder to the desired zero position</li>
            <li><b>Zero Setting:</b> Set the current position as the new zero reference</li>
            <li><b>Verification:</b> Test the zero position setting</li>
        </ol>

        <h3>Before Starting:</h3>
        <ul>
            <li>Ensure the encoder is properly connected</li>
            <li>Position the encoder shaft to your desired zero position</li>
            <li>This could be a mechanical reference point or alignment mark</li>
        </ul>
        """

        info_label = QLabel(instructions)
        info_label.setWordWrap(True)
        layout.addWidget(info_label)

        layout.addStretch()
        page.setLayout(layout)

        return page

    def create_zero_calibration_page(self) -> QWizardPage:
        """Create zero position calibration page"""
        page = QWizardPage()
        page.setTitle("Zero Position Setting")
        page.setSubTitle("Set the current encoder position as zero reference")

        layout = QVBoxLayout()

        # Current reading
        current_group = QGroupBox("Current Encoder Reading")
        current_layout = QVBoxLayout()

        self.current_position_label = QLabel("Position: --- counts")
        self.current_angle_label = QLabel("Angle: --- degrees")

        current_layout.addWidget(self.current_position_label)
        current_layout.addWidget(self.current_angle_label)

        # Refresh button
        refresh_btn = QPushButton("Refresh Reading")
        refresh_btn.clicked.connect(self.refresh_encoder_reading)
        current_layout.addWidget(refresh_btn)

        current_group.setLayout(current_layout)
        layout.addWidget(current_group)

        # Instructions
        instructions = QLabel(
            "Position the encoder to your desired zero position, then click 'Set Zero Position' below."
        )
        instructions.setWordWrap(True)
        layout.addWidget(instructions)

        # Set zero button
        self.set_zero_btn = QPushButton("Set Zero Position")
        self.set_zero_btn.clicked.connect(self.set_zero_position)
        layout.addWidget(self.set_zero_btn)

        # Confirmation
        self.zero_set_label = QLabel("")
        layout.addWidget(self.zero_set_label)

        layout.addStretch()
        page.setLayout(layout)

        # Auto-refresh timer
        self.refresh_timer = QTimer()
        self.refresh_timer.timeout.connect(self.refresh_encoder_reading)
        self.refresh_timer.start(500)  # Refresh every 500ms

        return page

    def create_verification_page(self) -> QWizardPage:
        """Create verification page for encoder"""
        page = QWizardPage()
        page.setTitle("Verification")
        page.setSubTitle("Verify the zero position setting")

        layout = QVBoxLayout()

        # Verification instructions
        instructions = QLabel(
            "Move the encoder away from zero and back to verify that it reads close to zero "
            "at your reference position."
        )
        instructions.setWordWrap(True)
        layout.addWidget(instructions)

        # Current reading (post-calibration)
        reading_group = QGroupBox("Current Reading (After Calibration)")
        reading_layout = QVBoxLayout()

        self.verification_position_label = QLabel("Position: 0 counts")
        self.verification_angle_label = QLabel("Angle: 0.0 degrees")

        reading_layout.addWidget(self.verification_position_label)
        reading_layout.addWidget(self.verification_angle_label)

        reading_group.setLayout(reading_layout)
        layout.addWidget(reading_group)

        # Verification result
        self.verification_result_label = QLabel("Move encoder to test zero position")
        layout.addWidget(self.verification_result_label)

        # Confirmation checkbox
        self.encoder_confirm_checkbox = QCheckBox("Zero position is correct")
        layout.addWidget(self.encoder_confirm_checkbox)

        layout.addStretch()
        page.setLayout(layout)

        # Set validation
        page.registerField("encoder_confirmed", self.encoder_confirm_checkbox)

        return page

    def create_completion_page(self) -> QWizardPage:
        """Create completion page for encoder"""
        page = QWizardPage()
        page.setTitle("Calibration Complete")
        page.setSubTitle("Encoder calibration has been completed")

        layout = QVBoxLayout()

        success_label = QLabel("✅ Encoder calibration completed successfully!")
        success_label.setFont(QFont("Arial", 14, QFont.Bold))
        success_label.setStyleSheet("color: green;")
        layout.addWidget(success_label)

        summary_label = QLabel(f"""
        <h3>Summary:</h3>
        <p><b>Device:</b> Encoder {self.device_id}</p>
        <p><b>Zero offset applied:</b> The encoder will now read zero at your reference position</p>
        <p><b>Settings saved:</b> The calibration is stored in the encoder memory</p>
        """)
        layout.addWidget(summary_label)

        layout.addStretch()
        page.setLayout(layout)

        return page

    def refresh_encoder_reading(self):
        """Refresh current encoder reading"""
        # In a real implementation, this would read from the actual encoder
        # For now, simulate reading
        current_counts = 1234  # Placeholder
        current_angle = current_counts * 0.36  # Assuming 1000 CPR encoder

        self.current_position_label.setText(f"Position: {current_counts} counts")
        self.current_angle_label.setText(f"Angle: {current_angle:.1f} degrees")

        if hasattr(self, 'verification_position_label'):
            # Update verification labels too (these would show calibrated values)
            calibrated_counts = current_counts - (self.calibration_data.zero_offset or 0)
            calibrated_angle = calibrated_counts * 0.36

            self.verification_position_label.setText(f"Position: {calibrated_counts} counts")
            self.verification_angle_label.setText(f"Angle: {calibrated_angle:.1f} degrees")

            # Update verification result
            if abs(calibrated_counts) < 10:  # Within 10 counts of zero
                self.verification_result_label.setText("✅ Zero position is accurate")
                self.verification_result_label.setStyleSheet("color: green;")
            else:
                self.verification_result_label.setText(f"Position offset: {calibrated_counts} counts")
                self.verification_result_label.setStyleSheet("color: orange;")

    def set_zero_position(self):
        """Set current position as zero"""
        # Get current encoder reading
        current_counts = 1234  # Placeholder - would read from actual encoder

        # Set zero offset
        self.calibration_data.zero_offset = current_counts

        # Update UI
        self.zero_set_label.setText(f"✅ Zero position set (offset: {current_counts} counts)")
        self.zero_set_label.setStyleSheet("color: green; font-weight: bold;")

        logger.info(f"Set zero position for encoder {self.device_id}, offset: {current_counts}")

    def accept(self):
        """Save calibration and close wizard"""
        self.calibration_data.completed = True

        # Apply calibration to parameter manager
        if self.parameter_manager and self.calibration_data.zero_offset is not None:
            self.apply_calibration()

        super().accept()

    def apply_calibration(self):
        """Apply calibration to the encoder parameters"""
        try:
            if hasattr(self.parameter_manager, 'set_parameter_value'):
                self.parameter_manager.set_parameter_value(
                    'encoder', self.device_id, 'ZERO_OFFSET', self.calibration_data.zero_offset
                )
                logger.info(f"Applied calibration to encoder {self.device_id}")

        except Exception as e:
            logger.error(f"Failed to apply calibration: {e}")
            QMessageBox.warning(self, "Calibration Error", f"Failed to save calibration: {str(e)}")

class CalibrationManagerWidget(QWidget):
    """Widget for managing device calibration"""

    def __init__(self, parameter_manager=None, parent=None):
        super().__init__(parent)
        self.parameter_manager = parameter_manager
        self.init_ui()

    def init_ui(self):
        """Initialize calibration manager UI"""
        layout = QVBoxLayout()

        # Header
        header_label = QLabel("Device Calibration")
        header_label.setFont(QFont("Arial", 16, QFont.Bold))
        layout.addWidget(header_label)

        # Device selection
        selection_group = QGroupBox("Device Selection")
        selection_layout = QHBoxLayout()

        selection_layout.addWidget(QLabel("Device Type:"))
        self.device_type_combo = QComboBox()
        self.device_type_combo.addItems(["servo", "encoder"])
        selection_layout.addWidget(self.device_type_combo)

        selection_layout.addWidget(QLabel("Device ID:"))
        self.device_id_spinbox = QSpinBox()
        self.device_id_spinbox.setRange(1, 16)
        selection_layout.addWidget(self.device_id_spinbox)

        self.calibrate_button = QPushButton("Start Calibration")
        self.calibrate_button.clicked.connect(self.start_calibration)
        selection_layout.addWidget(self.calibrate_button)

        selection_group.setLayout(selection_layout)
        layout.addWidget(selection_group)

        # Calibration history
        history_group = QGroupBox("Calibration History")
        history_layout = QVBoxLayout()

        self.history_table = QTableWidget()
        self.history_table.setColumnCount(5)
        self.history_table.setHorizontalHeaderLabels([
            'Device', 'Type', 'Date', 'Status', 'Actions'
        ])

        history_layout.addWidget(self.history_table)
        history_group.setLayout(history_layout)
        layout.addWidget(history_group)

        self.setLayout(layout)

    def start_calibration(self):
        """Start calibration wizard for selected device"""
        device_type = self.device_type_combo.currentText()
        device_id = self.device_id_spinbox.value()

        if device_type == "servo":
            wizard = ServoCalibrationWizard(device_id, self.parameter_manager, self)
        elif device_type == "encoder":
            wizard = EncoderCalibrationWizard(device_id, self.parameter_manager, self)
        else:
            QMessageBox.warning(self, "Error", "Unsupported device type")
            return

        result = wizard.exec_()

        if result == QWizard.Accepted:
            self.add_calibration_record(device_type, device_id, "Completed")
            QMessageBox.information(self, "Success", f"{device_type.title()} {device_id} calibration completed!")
        else:
            QMessageBox.information(self, "Cancelled", "Calibration was cancelled")

    def add_calibration_record(self, device_type: str, device_id: int, status: str):
        """Add calibration record to history table"""
        row = self.history_table.rowCount()
        self.history_table.insertRow(row)

        self.history_table.setItem(row, 0, QTableWidgetItem(f"{device_type.title()} {device_id}"))
        self.history_table.setItem(row, 1, QTableWidgetItem(device_type.title()))
        self.history_table.setItem(row, 2, QTableWidgetItem(time.strftime("%Y-%m-%d %H:%M:%S")))
        self.history_table.setItem(row, 3, QTableWidgetItem(status))

        # Add action button
        action_btn = QPushButton("Recalibrate")
        action_btn.clicked.connect(lambda: self.recalibrate_device(device_type, device_id))
        self.history_table.setCellWidget(row, 4, action_btn)

    def recalibrate_device(self, device_type: str, device_id: int):
        """Recalibrate a device"""
        self.device_type_combo.setCurrentText(device_type)
        self.device_id_spinbox.setValue(device_id)
        self.start_calibration()