# MAVLink Wizard

A comprehensive GUI application for configuring, monitoring, and calibrating MAVLink devices, similar to ROBOTIS Dynamixel Wizard. Built with ROS2 and PyQt5 for professional device management with advanced features including real-time plotting, calibration wizards, and firmware management.

## Features

### 🔧 Device Management
- **Automatic Device Discovery**: Monitors ROS2 topics for active devices
- **Multi-Device Support**: Servos, encoders, and RoboMaster motors
- **Real-time Status Monitoring**: Connection health and communication statistics
- **Device Tree View**: Organized device hierarchy with status indicators

### 📊 Real-time Monitoring & Plotting
- **Live Data Visualization**: Real-time plotting with PyQtGraph (optional)
- **Parameter Monitoring**: Position, velocity, current, temperature tracking
- **Historical Data**: Configurable data retention with circular buffers
- **Multi-parameter Plots**: Overlay multiple parameters on single plots
- **Fallback Mode**: Table-based display when plotting unavailable

### ⚙️ Parameter Management
- **Comprehensive Parameter Sets**: Device-specific parameter definitions
- **Batch Operations**: Update multiple devices simultaneously
- **Input Validation**: Real-time parameter range checking with visual feedback
- **Configuration Templates**: Save and load device configurations
- **Error Handling**: Service call timeouts and retry mechanisms

### 🎯 Calibration Wizards
- **Servo Calibration**: Center position, range limits, and zero offset
- **Encoder Calibration**: Zero position setting with verification
- **Step-by-step Guidance**: Interactive wizards with progress tracking
- **Safety Features**: Movement limits and emergency stop capabilities

### 💾 Firmware Management
- **Firmware Upload**: Support for .bin, .hex, and .elf files
- **Version Detection**: Automatic firmware version extraction
- **Validation**: File integrity and compatibility checks
- **Progress Tracking**: Real-time upload progress with cancellation support

### 🛠️ Advanced Features
- **Error Handling**: Comprehensive error reporting with user feedback dialogs
- **Configuration Management**: Persistent settings with JSON configuration
- **Logging**: Structured logging with configurable levels
- **Performance Optimization**: Bounded data structures and efficient threading

## Prerequisites

### System Requirements
- **ROS2**: Jazzy or Humble
- **Python**: 3.8 or later
- **OS**: Ubuntu 22.04 or compatible Linux distribution

### Required Dependencies
- **PyQt5**: GUI framework
- **stm32_mavlink_interface**: Hardware communication package

### Optional Dependencies
- **PyQtGraph**: For advanced real-time plotting (recommended)
- **NumPy**: For numerical operations
- **SciPy**: For signal processing (future features)

## Installation

### Quick Install

```bash
# Install system dependencies
sudo apt update
sudo apt install python3-pyqt5 python3-numpy

# Optional: Install PyQtGraph for advanced plotting
pip3 install pyqtgraph

# Navigate to your ROS2 workspace
cd ~/ros2_jazzy

# Install ROS2 package dependencies
rosdep install --from-paths src --ignore-src -r -y

# Build the workspace
colcon build --packages-select mavlink_wizard stm32_mavlink_interface
source install/setup.bash
```

### Development Install

For development with additional features:

```bash
# Install all optional dependencies
pip3 install pyqtgraph numpy scipy matplotlib

# Build with debug symbols
colcon build --packages-select mavlink_wizard --cmake-args -DCMAKE_BUILD_TYPE=Debug

# Enable development mode
echo "export MAVLINK_WIZARD_DEV_MODE=1" >> ~/.bashrc
```

## Usage

### Basic Usage

1. Launch the MAVLink Wizard:
```bash
ros2 launch mavlink_wizard mavlink_wizard.launch.py
```

2. Or launch with custom parameters:
```bash
# For USB-to-Serial devices (most common)
ros2 launch mavlink_wizard mavlink_wizard.launch.py serial_port:=/dev/ttyUSB0 baud_rate:=115200

# For CDC/ACM devices (STM32 with USB CDC)
ros2 launch mavlink_wizard mavlink_wizard.launch.py serial_port:=/dev/ttyACM0 baud_rate:=115200
```

### GUI Interface

#### Device Scanner
1. Select serial port and baud rate
2. Click "Scan for Devices" to discover connected devices
3. Devices will appear in the device tree with their status

#### Parameter Configuration
1. Select a device from the device tree
2. Switch to the "Configuration" tab
3. Modify parameters as needed
4. Click "Write Parameters" to send changes to the device

#### Real-time Monitoring
1. Switch to the "Real-time Monitor" tab
2. Click "Start Monitoring" to begin data collection
3. View live device data in the table
4. Use "Clear Data" to reset the display

### Command Line Tools

Run individual components separately:

```bash
# Device scanner only
ros2 run mavlink_wizard device_scanner.py

# Parameter manager only
ros2 run mavlink_wizard parameter_manager.py

# Message monitor only
ros2 run mavlink_wizard message_monitor.py
```

## Configuration

The application uses a YAML configuration file located at:
`~/.config/mavlink_wizard/wizard_config.yaml`

Key configuration sections:
- **serial**: Serial port settings
- **discovery**: Device discovery parameters
- **gui**: Interface appearance and behavior
- **monitoring**: Data collection settings
- **device_types**: Device-specific configurations

## Architecture

### Core Components

1. **MAVLink Wizard GUI** (`mavlink_wizard_gui.py`)
   - Main application window
   - PyQt5-based user interface
   - Integrates all other components

2. **Device Scanner** (`device_scanner.py`)
   - Serial port communication
   - MAVLink message parsing
   - Device discovery and management

3. **Parameter Manager** (`parameter_manager.py`)
   - Parameter definitions and validation
   - Configuration file handling
   - Device parameter operations

4. **Message Monitor** (`message_monitor.py`)
   - Real-time data collection
   - Statistical analysis
   - Message logging and recording

### ROS2 Integration

The application integrates with the `stm32_mavlink_interface` package for:
- Device state messages
- Service calls for configuration
- MAVLink communication protocols

## Supported Devices

### Servo Motors
- Position, velocity, and current control
- Angle limits and speed configuration
- Torque limiting and protection

### Encoders
- Quadrature encoder support
- Configurable resolution (CPR)
- Channel inversion options
- Index channel support

### Brushless Motors
- Current and velocity control
- PID parameter tuning
- Multiple control modes
- Temperature monitoring

## File Formats

### Configuration Files
Device configurations are saved in JSON format:
```json
{
  "device_id": 1,
  "device_type": "servo",
  "parameters": {
    "SPEED": 150,
    "TORQUE_LIMIT": 75,
    ...
  }
}
```

### Monitoring Data
Monitoring data is exported in CSV format:
```csv
timestamp,message_type,device_id,position,velocity,current
1634567890.123,servo_state,1,45.2,12.5,0.8
```

## Development

### Adding New Device Types

1. Define parameters in `parameter_manager.py`:
```python
'new_device': {
    'PARAM_NAME': Parameter(
        name='PARAM_NAME',
        value=default_value,
        param_type=ParameterType.UINT16,
        description='Parameter description',
        min_value=0,
        max_value=1000
    )
}
```

2. Add message handling in `message_monitor.py`:
```python
def _handle_new_device_state(self, msg):
    # Process device-specific message
    pass
```

3. Update GUI widgets in `mavlink_wizard_gui.py` as needed.

### Testing

Run the test suite:
```bash
cd ~/ros2_jazzy
colcon test --packages-select mavlink_wizard
```

## Troubleshooting

### Common Issues

1. **Serial Port Access**
   ```bash
   sudo usermod -a -G dialout $USER
   # Log out and back in
   ```

2. **PyQt5 Import Error**
   ```bash
   sudo apt install python3-pyqt5
   ```

3. **Device Not Found**
   - Check serial port connections
   - Verify baud rate settings
   - Ensure device is powered on

### Debug Mode

Enable debug logging:
```bash
ros2 launch mavlink_wizard mavlink_wizard.launch.py --ros-args --log-level debug
```

## Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Add tests for new functionality
5. Submit a pull request

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## Support

For issues and questions:
- Create an issue on GitHub
- Check the ROS2 MAVLink documentation
- Consult the MAVLink protocol specification

## Acknowledgments

- Inspired by Dynamixel Wizard
- Built on ROS2 and PyQt5 frameworks
- Uses MAVLink protocol for device communication