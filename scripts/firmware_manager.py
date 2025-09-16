#!/usr/bin/env python3

import logging
import os
import hashlib
import json
from typing import Dict, List, Optional, Tuple, Callable
from dataclasses import dataclass, asdict
from enum import Enum
import time

from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *

import rclpy
from rclpy.node import Node

logger = logging.getLogger(__name__)

class FirmwareType(Enum):
    """Supported firmware types"""
    STM32_MAVLINK = "stm32_mavlink"
    BOOTLOADER = "bootloader"
    APPLICATION = "application"

@dataclass
class FirmwareInfo:
    """Information about a firmware file"""
    filename: str
    filepath: str
    version: str
    device_type: str
    firmware_type: FirmwareType
    checksum: str
    size_bytes: int
    upload_date: Optional[str] = None
    description: str = ""

class FirmwareValidator:
    """Validates firmware files before upload"""

    @staticmethod
    def calculate_checksum(filepath: str) -> str:
        """Calculate MD5 checksum of firmware file"""
        try:
            hash_md5 = hashlib.md5()
            with open(filepath, "rb") as f:
                for chunk in iter(lambda: f.read(4096), b""):
                    hash_md5.update(chunk)
            return hash_md5.hexdigest()
        except Exception as e:
            logger.error(f"Failed to calculate checksum for {filepath}: {e}")
            return ""

    @staticmethod
    def validate_firmware_file(filepath: str) -> Tuple[bool, str]:
        """Validate firmware file format and content"""
        if not os.path.exists(filepath):
            return False, "File does not exist"

        file_size = os.path.getsize(filepath)
        if file_size == 0:
            return False, "File is empty"

        if file_size > 1024 * 1024:  # 1MB limit
            return False, "File too large (>1MB)"

        # Check file extension
        _, ext = os.path.splitext(filepath)
        if ext.lower() not in ['.bin', '.hex', '.elf']:
            return False, "Unsupported file format. Use .bin, .hex, or .elf"

        # Basic binary file validation
        try:
            with open(filepath, 'rb') as f:
                header = f.read(64)  # Read first 64 bytes

            # For .bin files, check if it looks like ARM Cortex-M binary
            if ext.lower() == '.bin':
                if len(header) >= 8:
                    # Check if first 4 bytes look like a valid stack pointer (ARM Cortex-M)
                    stack_ptr = int.from_bytes(header[0:4], 'little')
                    reset_vector = int.from_bytes(header[4:8], 'little')

                    # Basic sanity checks
                    if stack_ptr < 0x20000000 or stack_ptr > 0x20020000:  # Typical SRAM range for STM32
                        logger.warning(f"Stack pointer {hex(stack_ptr)} seems unusual")

                    if reset_vector < 0x08000000 or reset_vector > 0x08080000:  # Typical Flash range
                        logger.warning(f"Reset vector {hex(reset_vector)} seems unusual")

        except Exception as e:
            return False, f"Error validating file content: {str(e)}"

        return True, "Valid firmware file"

    @staticmethod
    def extract_version_info(filepath: str) -> str:
        """Attempt to extract version information from firmware"""
        try:
            # Look for version strings in the binary
            with open(filepath, 'rb') as f:
                content = f.read()

            # Search for common version patterns
            import re
            version_patterns = [
                rb'v\d+\.\d+\.\d+',  # v1.2.3
                rb'V\d+\.\d+\.\d+',  # V1.2.3
                rb'\d+\.\d+\.\d+',   # 1.2.3
            ]

            for pattern in version_patterns:
                match = re.search(pattern, content)
                if match:
                    return match.group().decode('ascii', errors='ignore')

            return "Unknown"

        except Exception as e:
            logger.error(f"Error extracting version from {filepath}: {e}")
            return "Unknown"

class FirmwareUploadDialog(QDialog):
    """Dialog for firmware upload with progress tracking"""

    def __init__(self, firmware_info: FirmwareInfo, parent=None):
        super().__init__(parent)
        self.firmware_info = firmware_info
        self.upload_cancelled = False
        self.init_ui()

    def init_ui(self):
        """Initialize upload dialog UI"""
        self.setWindowTitle("Firmware Upload")
        self.setModal(True)
        self.setFixedSize(500, 300)

        layout = QVBoxLayout()

        # Firmware info
        info_group = QGroupBox("Firmware Information")
        info_layout = QFormLayout()

        info_layout.addRow("File:", QLabel(self.firmware_info.filename))
        info_layout.addRow("Version:", QLabel(self.firmware_info.version))
        info_layout.addRow("Device Type:", QLabel(self.firmware_info.device_type))
        info_layout.addRow("Size:", QLabel(f"{self.firmware_info.size_bytes} bytes"))
        info_layout.addRow("Checksum:", QLabel(self.firmware_info.checksum[:16] + "..."))

        info_group.setLayout(info_layout)
        layout.addWidget(info_group)

        # Warning
        warning_label = QLabel("⚠️ Warning: Firmware upload will temporarily disable the device. "
                              "Make sure the device is not in use and power is stable.")
        warning_label.setWordWrap(True)
        warning_label.setStyleSheet("color: orange; font-weight: bold; padding: 10px; "
                                   "border: 1px solid orange; border-radius: 5px;")
        layout.addWidget(warning_label)

        # Progress
        self.progress_bar = QProgressBar()
        self.progress_bar.setVisible(False)
        layout.addWidget(self.progress_bar)

        self.status_label = QLabel("Ready to upload")
        layout.addWidget(self.status_label)

        # Buttons
        button_layout = QHBoxLayout()

        self.upload_button = QPushButton("Upload Firmware")
        self.upload_button.clicked.connect(self.start_upload)
        button_layout.addWidget(self.upload_button)

        self.cancel_button = QPushButton("Cancel")
        self.cancel_button.clicked.connect(self.cancel_upload)
        button_layout.addWidget(self.cancel_button)

        layout.addLayout(button_layout)

        self.setLayout(layout)

    def start_upload(self):
        """Start firmware upload process"""
        self.upload_button.setEnabled(False)
        self.progress_bar.setVisible(True)
        self.progress_bar.setRange(0, 100)

        self.status_label.setText("Starting upload...")

        # Create upload worker
        self.upload_worker = FirmwareUploadWorker(self.firmware_info)
        self.upload_worker.progress_updated.connect(self.update_progress)
        self.upload_worker.status_updated.connect(self.update_status)
        self.upload_worker.upload_completed.connect(self.upload_finished)
        self.upload_worker.upload_failed.connect(self.upload_error)

        self.upload_worker.start()

    def cancel_upload(self):
        """Cancel firmware upload"""
        if hasattr(self, 'upload_worker') and self.upload_worker.isRunning():
            self.upload_worker.cancel_upload()
            self.status_label.setText("Cancelling upload...")
        else:
            self.reject()

    def update_progress(self, percent: int):
        """Update progress bar"""
        self.progress_bar.setValue(percent)

    def update_status(self, message: str):
        """Update status message"""
        self.status_label.setText(message)

    def upload_finished(self):
        """Handle successful upload completion"""
        self.progress_bar.setValue(100)
        self.status_label.setText("✅ Upload completed successfully!")

        QTimer.singleShot(2000, self.accept)  # Close dialog after 2 seconds

    def upload_error(self, error_message: str):
        """Handle upload error"""
        self.progress_bar.setVisible(False)
        self.status_label.setText(f"❌ Upload failed: {error_message}")
        self.upload_button.setEnabled(True)

class FirmwareUploadWorker(QThread):
    """Worker thread for firmware upload"""

    progress_updated = pyqtSignal(int)
    status_updated = pyqtSignal(str)
    upload_completed = pyqtSignal()
    upload_failed = pyqtSignal(str)

    def __init__(self, firmware_info: FirmwareInfo):
        super().__init__()
        self.firmware_info = firmware_info
        self.cancelled = False

    def cancel_upload(self):
        """Cancel the upload process"""
        self.cancelled = True

    def run(self):
        """Run the firmware upload process"""
        try:
            # Simulate firmware upload process
            # In a real implementation, this would communicate with the bootloader

            stages = [
                ("Connecting to bootloader...", 10),
                ("Erasing flash memory...", 30),
                ("Programming firmware...", 80),
                ("Verifying firmware...", 95),
                ("Resetting device...", 100)
            ]

            for message, progress in stages:
                if self.cancelled:
                    self.upload_failed.emit("Upload cancelled by user")
                    return

                self.status_updated.emit(message)
                self.progress_updated.emit(progress)

                # Simulate work time
                self.msleep(1000)

            self.upload_completed.emit()

        except Exception as e:
            self.upload_failed.emit(str(e))

class FirmwareManagerWidget(QWidget):
    """Widget for managing firmware files and uploads"""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.firmware_files = {}
        self.init_ui()

    def init_ui(self):
        """Initialize firmware manager UI"""
        layout = QVBoxLayout()

        # Header
        header_label = QLabel("Firmware Management")
        header_label.setFont(QFont("Arial", 16, QFont.Bold))
        layout.addWidget(header_label)

        # Control buttons
        button_layout = QHBoxLayout()

        self.load_button = QPushButton("Load Firmware File")
        self.load_button.clicked.connect(self.load_firmware_file)
        button_layout.addWidget(self.load_button)

        self.refresh_button = QPushButton("Refresh List")
        self.refresh_button.clicked.connect(self.refresh_firmware_list)
        button_layout.addWidget(self.refresh_button)

        button_layout.addStretch()
        layout.addLayout(button_layout)

        # Firmware list
        self.firmware_table = QTableWidget()
        self.firmware_table.setColumnCount(6)
        self.firmware_table.setHorizontalHeaderLabels([
            'Filename', 'Version', 'Device Type', 'Size', 'Checksum', 'Actions'
        ])

        # Make table fill width
        header = self.firmware_table.horizontalHeader()
        header.setSectionResizeMode(0, QHeaderView.Stretch)
        header.setSectionResizeMode(1, QHeaderView.ResizeToContents)
        header.setSectionResizeMode(2, QHeaderView.ResizeToContents)
        header.setSectionResizeMode(3, QHeaderView.ResizeToContents)
        header.setSectionResizeMode(4, QHeaderView.ResizeToContents)
        header.setSectionResizeMode(5, QHeaderView.ResizeToContents)

        layout.addWidget(self.firmware_table)

        # Status bar
        self.status_bar = QLabel("Ready")
        self.status_bar.setStyleSheet("padding: 5px; border: 1px solid gray;")
        layout.addWidget(self.status_bar)

        self.setLayout(layout)

        # Load existing firmware files
        self.refresh_firmware_list()

    def load_firmware_file(self):
        """Load a new firmware file"""
        file_dialog = QFileDialog()
        filepath, _ = file_dialog.getOpenFileName(
            self,
            "Select Firmware File",
            "",
            "Firmware Files (*.bin *.hex *.elf);;All Files (*)"
        )

        if filepath:
            self.process_firmware_file(filepath)

    def process_firmware_file(self, filepath: str):
        """Process and validate a firmware file"""
        self.status_bar.setText("Processing firmware file...")

        # Validate file
        is_valid, message = FirmwareValidator.validate_firmware_file(filepath)
        if not is_valid:
            QMessageBox.warning(self, "Invalid Firmware", f"Cannot load firmware: {message}")
            self.status_bar.setText("Ready")
            return

        # Extract information
        filename = os.path.basename(filepath)
        version = FirmwareValidator.extract_version_info(filepath)
        checksum = FirmwareValidator.calculate_checksum(filepath)
        size = os.path.getsize(filepath)

        # Create firmware info
        firmware_info = FirmwareInfo(
            filename=filename,
            filepath=filepath,
            version=version,
            device_type="STM32",  # Default, could be detected from filename
            firmware_type=FirmwareType.STM32_MAVLINK,
            checksum=checksum,
            size_bytes=size,
            description=f"Loaded from {filepath}"
        )

        # Add to list
        self.add_firmware_to_table(firmware_info)
        self.firmware_files[filename] = firmware_info

        self.status_bar.setText(f"Loaded firmware: {filename}")

    def add_firmware_to_table(self, firmware_info: FirmwareInfo):
        """Add firmware entry to the table"""
        row = self.firmware_table.rowCount()
        self.firmware_table.insertRow(row)

        # Set data
        self.firmware_table.setItem(row, 0, QTableWidgetItem(firmware_info.filename))
        self.firmware_table.setItem(row, 1, QTableWidgetItem(firmware_info.version))
        self.firmware_table.setItem(row, 2, QTableWidgetItem(firmware_info.device_type))
        self.firmware_table.setItem(row, 3, QTableWidgetItem(f"{firmware_info.size_bytes:,} bytes"))
        self.firmware_table.setItem(row, 4, QTableWidgetItem(firmware_info.checksum[:8] + "..."))

        # Add action buttons
        actions_widget = QWidget()
        actions_layout = QHBoxLayout()
        actions_layout.setContentsMargins(5, 2, 5, 2)

        upload_btn = QPushButton("Upload")
        upload_btn.setMaximumWidth(60)
        upload_btn.clicked.connect(lambda: self.upload_firmware(firmware_info))

        info_btn = QPushButton("Info")
        info_btn.setMaximumWidth(50)
        info_btn.clicked.connect(lambda: self.show_firmware_info(firmware_info))

        actions_layout.addWidget(upload_btn)
        actions_layout.addWidget(info_btn)
        actions_widget.setLayout(actions_layout)

        self.firmware_table.setCellWidget(row, 5, actions_widget)

    def upload_firmware(self, firmware_info: FirmwareInfo):
        """Upload firmware to device"""
        # Show upload dialog
        upload_dialog = FirmwareUploadDialog(firmware_info, self)
        result = upload_dialog.exec_()

        if result == QDialog.Accepted:
            self.status_bar.setText(f"Successfully uploaded {firmware_info.filename}")
        else:
            self.status_bar.setText("Upload cancelled or failed")

    def show_firmware_info(self, firmware_info: FirmwareInfo):
        """Show detailed firmware information"""
        dialog = QDialog(self)
        dialog.setWindowTitle(f"Firmware Info - {firmware_info.filename}")
        dialog.setFixedSize(400, 300)

        layout = QVBoxLayout()

        info_text = f"""
Filename: {firmware_info.filename}
Version: {firmware_info.version}
Device Type: {firmware_info.device_type}
Firmware Type: {firmware_info.firmware_type.value}
Size: {firmware_info.size_bytes:,} bytes ({firmware_info.size_bytes/1024:.1f} KB)
MD5 Checksum: {firmware_info.checksum}
File Path: {firmware_info.filepath}
Description: {firmware_info.description}
        """.strip()

        info_label = QLabel(info_text)
        info_label.setWordWrap(True)
        info_label.setFont(QFont("Courier", 9))
        layout.addWidget(info_label)

        # Close button
        close_button = QPushButton("Close")
        close_button.clicked.connect(dialog.accept)
        layout.addWidget(close_button)

        dialog.setLayout(layout)
        dialog.exec_()

    def refresh_firmware_list(self):
        """Refresh the firmware list"""
        # Clear existing table
        self.firmware_table.setRowCount(0)

        # In a real implementation, this would scan a firmware directory
        # For now, just update the status
        self.status_bar.setText("Firmware list refreshed")

    def get_available_firmwares(self) -> List[FirmwareInfo]:
        """Get list of available firmware files"""
        return list(self.firmware_files.values())