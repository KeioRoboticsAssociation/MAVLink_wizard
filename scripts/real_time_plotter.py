#!/usr/bin/env python3

import logging
from typing import Dict, List, Any, Optional, Tuple
from collections import deque
import time

import numpy as np
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *

try:
    import pyqtgraph as pg
    PLOTTING_AVAILABLE = True
except ImportError:
    PLOTTING_AVAILABLE = False
    logging.warning("PyQtGraph not available. Real-time plotting disabled. Install with: pip install pyqtgraph")

logger = logging.getLogger(__name__)

class PlotData:
    """Container for plot data with time series management"""

    def __init__(self, max_points: int = 1000):
        self.max_points = max_points
        self.timestamps = deque(maxlen=max_points)
        self.values = deque(maxlen=max_points)
        self.start_time = time.time()

    def add_point(self, value: float, timestamp: Optional[float] = None):
        """Add a data point with timestamp"""
        if timestamp is None:
            timestamp = time.time()

        relative_time = timestamp - self.start_time
        self.timestamps.append(relative_time)
        self.values.append(value)

    def get_arrays(self) -> Tuple[np.ndarray, np.ndarray]:
        """Get numpy arrays for plotting"""
        return np.array(self.timestamps), np.array(self.values)

    def clear(self):
        """Clear all data"""
        self.timestamps.clear()
        self.values.clear()
        self.start_time = time.time()

class DevicePlotWidget(QWidget):
    """Widget for plotting real-time data from a single device"""

    def __init__(self, device_type: str, device_id: int, parent=None):
        super().__init__(parent)
        self.device_type = device_type
        self.device_id = device_id
        self.plot_data = {}
        self.plot_curves = {}

        if not PLOTTING_AVAILABLE:
            self.init_fallback_ui()
        else:
            self.init_plotting_ui()

    def init_fallback_ui(self):
        """Initialize fallback UI when PyQtGraph is not available"""
        layout = QVBoxLayout()

        label = QLabel("Real-time plotting not available.\nInstall PyQtGraph for plotting support.")
        label.setAlignment(Qt.AlignCenter)
        label.setStyleSheet("color: orange; font-size: 14px;")
        layout.addWidget(label)

        # Simple table as fallback
        self.table = QTableWidget()
        self.table.setColumnCount(3)
        self.table.setHorizontalHeaderLabels(['Parameter', 'Value', 'Unit'])
        layout.addWidget(self.table)

        self.setLayout(layout)

    def init_plotting_ui(self):
        """Initialize plotting UI with PyQtGraph"""
        layout = QVBoxLayout()

        # Device info header
        header_label = QLabel(f"{self.device_type.title()} {self.device_id}")
        header_label.setFont(QFont("Arial", 12, QFont.Bold))
        layout.addWidget(header_label)

        # Control panel
        control_panel = self.create_control_panel()
        layout.addWidget(control_panel)

        # Plot widget
        self.plot_widget = pg.PlotWidget()
        self.plot_widget.setLabel('left', 'Value')
        self.plot_widget.setLabel('bottom', 'Time (seconds)')
        self.plot_widget.setTitle(f'{self.device_type.title()} {self.device_id} - Real-time Data')
        self.plot_widget.addLegend()

        layout.addWidget(self.plot_widget)

        # Statistics panel
        stats_panel = self.create_statistics_panel()
        layout.addWidget(stats_panel)

        self.setLayout(layout)

        # Update timer
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self.update_plots)
        self.update_timer.start(100)  # Update every 100ms

    def create_control_panel(self) -> QWidget:
        """Create control panel for plot settings"""
        panel = QGroupBox("Plot Controls")
        layout = QHBoxLayout()

        # Clear button
        clear_btn = QPushButton("Clear Data")
        clear_btn.clicked.connect(self.clear_all_data)
        layout.addWidget(clear_btn)

        # Auto-scale button
        autoscale_btn = QPushButton("Auto Scale")
        autoscale_btn.clicked.connect(self.auto_scale_plots)
        layout.addWidget(autoscale_btn)

        # Time window control
        layout.addWidget(QLabel("Time Window:"))
        self.time_window_spin = QSpinBox()
        self.time_window_spin.setRange(10, 300)
        self.time_window_spin.setValue(60)
        self.time_window_spin.setSuffix(" sec")
        layout.addWidget(self.time_window_spin)

        # Parameter selection
        layout.addWidget(QLabel("Show:"))
        self.parameter_checkboxes = {}

        panel.setLayout(layout)
        return panel

    def create_statistics_panel(self) -> QWidget:
        """Create statistics panel showing current values"""
        panel = QGroupBox("Current Statistics")
        layout = QGridLayout()

        self.stat_labels = {}

        panel.setLayout(layout)
        return panel

    def add_parameter_checkbox(self, param_name: str, color: str):
        """Add checkbox for parameter visibility"""
        if not PLOTTING_AVAILABLE:
            return

        checkbox = QCheckBox(param_name)
        checkbox.setChecked(True)
        checkbox.stateChanged.connect(lambda state: self.toggle_parameter_visibility(param_name, state == Qt.Checked))

        # Color indicator
        checkbox.setStyleSheet(f"QCheckBox::indicator:checked {{ background-color: {color}; }}")

        self.parameter_checkboxes[param_name] = checkbox

        # Add to control panel layout
        control_layout = self.findChild(QGroupBox, "Plot Controls").layout()
        control_layout.addWidget(checkbox)

    def add_data_point(self, parameter: str, value: float, unit: str = "", timestamp: Optional[float] = None):
        """Add a data point for plotting"""
        if not PLOTTING_AVAILABLE:
            self.update_fallback_table(parameter, value, unit)
            return

        # Initialize plot data if needed
        if parameter not in self.plot_data:
            max_points = self.time_window_spin.value() * 10  # Assuming 10Hz update rate
            self.plot_data[parameter] = PlotData(max_points)

            # Create plot curve
            colors = ['r', 'g', 'b', 'c', 'm', 'y', 'w']
            color = colors[len(self.plot_curves) % len(colors)]

            curve = self.plot_widget.plot(pen=color, name=f"{parameter} ({unit})")
            self.plot_curves[parameter] = curve

            # Add parameter checkbox
            self.add_parameter_checkbox(parameter, color)

            # Add to statistics
            self.add_statistic_label(parameter, unit)

        # Add data point
        self.plot_data[parameter].add_point(value, timestamp)

        # Update statistics
        self.update_statistic_label(parameter, value, unit)

    def update_fallback_table(self, parameter: str, value: float, unit: str):
        """Update fallback table when plotting is not available"""
        # Find or create row for this parameter
        row = -1
        for i in range(self.table.rowCount()):
            if self.table.item(i, 0) and self.table.item(i, 0).text() == parameter:
                row = i
                break

        if row == -1:
            row = self.table.rowCount()
            self.table.insertRow(row)
            self.table.setItem(row, 0, QTableWidgetItem(parameter))

        self.table.setItem(row, 1, QTableWidgetItem(f"{value:.3f}"))
        self.table.setItem(row, 2, QTableWidgetItem(unit))

    def add_statistic_label(self, parameter: str, unit: str):
        """Add statistic label for parameter"""
        if not PLOTTING_AVAILABLE:
            return

        stats_panel = self.findChild(QGroupBox, "Current Statistics")
        layout = stats_panel.layout()

        row = len(self.stat_labels)

        name_label = QLabel(f"{parameter}:")
        value_label = QLabel("---")
        unit_label = QLabel(unit)

        layout.addWidget(name_label, row, 0)
        layout.addWidget(value_label, row, 1)
        layout.addWidget(unit_label, row, 2)

        self.stat_labels[parameter] = value_label

    def update_statistic_label(self, parameter: str, value: float, unit: str):
        """Update statistic label with current value"""
        if parameter in self.stat_labels:
            self.stat_labels[parameter].setText(f"{value:.3f}")

    def update_plots(self):
        """Update all plot curves with current data"""
        if not PLOTTING_AVAILABLE:
            return

        for parameter, curve in self.plot_curves.items():
            if parameter in self.plot_data:
                timestamps, values = self.plot_data[parameter].get_arrays()
                if len(timestamps) > 0:
                    curve.setData(timestamps, values)

    def toggle_parameter_visibility(self, parameter: str, visible: bool):
        """Toggle visibility of a parameter plot"""
        if parameter in self.plot_curves:
            curve = self.plot_curves[parameter]
            curve.setVisible(visible)

    def clear_all_data(self):
        """Clear all plot data"""
        for plot_data in self.plot_data.values():
            plot_data.clear()

    def auto_scale_plots(self):
        """Auto-scale all plots"""
        if PLOTTING_AVAILABLE:
            self.plot_widget.autoRange()

class RealTimePlotterWidget(QWidget):
    """Main widget for managing multiple device plots"""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.device_plots = {}
        self.init_ui()

    def init_ui(self):
        """Initialize the main plotting UI"""
        layout = QVBoxLayout()

        # Header
        header_label = QLabel("Real-time Device Monitoring")
        header_label.setFont(QFont("Arial", 16, QFont.Bold))
        header_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(header_label)

        if not PLOTTING_AVAILABLE:
            warning_label = QLabel("⚠️ PyQtGraph not installed. Install it for full plotting support.")
            warning_label.setStyleSheet("color: orange; font-size: 12px; font-weight: bold;")
            warning_label.setAlignment(Qt.AlignCenter)
            layout.addWidget(warning_label)

        # Scroll area for multiple device plots
        self.scroll_area = QScrollArea()
        self.scroll_area.setWidgetResizable(True)

        self.scroll_widget = QWidget()
        self.scroll_layout = QVBoxLayout()
        self.scroll_widget.setLayout(self.scroll_layout)

        self.scroll_area.setWidget(self.scroll_widget)
        layout.addWidget(self.scroll_area)

        self.setLayout(layout)

    def add_device_plot(self, device_type: str, device_id: int) -> DevicePlotWidget:
        """Add a plot widget for a new device"""
        device_key = f"{device_type}_{device_id}"

        if device_key not in self.device_plots:
            plot_widget = DevicePlotWidget(device_type, device_id)
            self.device_plots[device_key] = plot_widget
            self.scroll_layout.addWidget(plot_widget)

            logger.info(f"Added plot for {device_type} {device_id}")

        return self.device_plots[device_key]

    def remove_device_plot(self, device_type: str, device_id: int):
        """Remove a device plot"""
        device_key = f"{device_type}_{device_id}"

        if device_key in self.device_plots:
            plot_widget = self.device_plots[device_key]
            self.scroll_layout.removeWidget(plot_widget)
            plot_widget.deleteLater()
            del self.device_plots[device_key]

            logger.info(f"Removed plot for {device_type} {device_id}")

    def get_device_plot(self, device_type: str, device_id: int) -> Optional[DevicePlotWidget]:
        """Get plot widget for a device"""
        device_key = f"{device_type}_{device_id}"
        return self.device_plots.get(device_key)

    def update_device_data(self, device_type: str, device_id: int, parameter: str, value: float, unit: str = ""):
        """Update data for a specific device parameter"""
        plot_widget = self.get_device_plot(device_type, device_id)
        if not plot_widget:
            plot_widget = self.add_device_plot(device_type, device_id)

        plot_widget.add_data_point(parameter, value, unit)

    def clear_all_plots(self):
        """Clear all plot data"""
        for plot_widget in self.device_plots.values():
            plot_widget.clear_all_data()