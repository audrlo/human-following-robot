# Human-Following Robot

An autonomous robot system that uses YOLOv8 computer vision and Intel RealSense depth sensing to follow humans while avoiding obstacles.

## Features

- **Human Detection**: Uses YOLOv8 to identify and track people in real-time
- **Obstacle Avoidance**: Comprehensive collision prevention using depth sensing
- **Smart Following**: Maintains optimal 1-meter distance from people
- **Smooth Movement**: Gradual speed ramping for natural robot behavior
- **Real-time Processing**: Fast inference suitable for robotics applications

## Hardware Requirements

- **Robot Platform**: Differential drive robot with two motors
- **Motor Controller**: Roboclaw motor controller
- **Camera**: Intel RealSense D400 series (RGB + Depth)
- **Computer**: Raspberry Pi 4 or similar single-board computer
- **Power**: 12V battery system

## Software Dependencies

```bash
pip install ultralytics opencv-python pyrealsense2 numpy
```

### Core Libraries
- **ultralytics**: YOLOv8 object detection
- **opencv-python**: Computer vision and image processing
- **pyrealsense2**: Intel RealSense camera interface
- **numpy**: Numerical computing
- **roboclaw**: Motor controller interface (included)

## Installation

1. **Clone the repository**
   ```bash
   git clone <your-repo-url>
   cd follow
   ```

2. **Install dependencies**
   ```bash
   pip install -r requirements.txt
   ```

3. **Connect hardware**
   - Connect Roboclaw to `/dev/ttyACM0` (or update port in code)
   - Connect RealSense camera via USB
   - Ensure proper power connections

4. **Test basic functionality**
   ```bash
   python drive_basic.py      # Test motor control
   python depthcam.py         # Test camera
   python avoid_obstacle.py   # Test obstacle avoidance
   ```

## Usage

### Basic Obstacle Avoidance
```bash
python avoid_obstacle.py
```
Robot moves forward and avoids obstacles using depth sensing.

### Human Following Robot
```bash
python test_follow.py
```
Robot detects people and follows them while avoiding all obstacles.

### Camera Testing
```bash
python depthcam.py
```
Display depth camera feed with distance measurements.

### Motor Testing
```bash
python drive_basic.py
```
Test motor control and calibration.

## How It Works

### 1. Human Detection
- **YOLOv8 Model**: Pre-trained on COCO dataset for person detection
- **Real-time Processing**: ~6ms inference time for responsive behavior
- **Confidence Filtering**: Only follows people with >50% confidence

### 2. Obstacle Avoidance
- **Depth Sensing**: Intel RealSense provides 3D distance data
- **Safety First**: Obstacle avoidance takes priority over following
- **Smart Navigation**: Turns left until clear path is found

### 3. Following Behavior
- **Distance Control**: Maintains 1-meter following distance
- **Lateral Positioning**: Keeps person centered in camera view
- **Speed Management**: Smooth acceleration/deceleration

### 4. Motor Control
- **Differential Drive**: Independent left/right wheel control
- **Speed Ramping**: Gradual speed changes for natural movement
- **Safety Stops**: Immediate halt when obstacles detected

## Configuration

### Motor Control Parameters
```python
RAMP_STEP_QPPS = 50              # Speed increment steps
FORWARD_TARGET_QPPS = 50 * 20    # Target forward speed
TURN_QPPS = 50 * 10              # Turning speed
RAMP_STEP_DELAY_S = 0.05         # Ramp timing
```

### Safety Parameters
```python
OBSTACLE_METERS = 1.0            # Minimum safe distance
PERSON_FOLLOW_DISTANCE = 1.0     # Target following distance
CHECK_INTERVAL_S = 0.05          # Control loop frequency
```

## File Structure

```
follow/
├── README.md              # This file
├── requirements.txt       # Python dependencies
├── avoid_obstacle.py     # Basic obstacle avoidance
├── test_follow.py        # Human following robot (main system)
├── depthcam.py           # Camera testing
├── drive_basic.py        # Motor testing
├── roboclaw.py           # Motor controller library
└── .gitignore            # Git ignore rules
```

## Troubleshooting

### Common Issues

1. **Roboclaw Connection Failed**
   - Check USB connection and port (`/dev/ttyACM0` vs `/dev/ttyUSB0`)
   - Verify baud rate (38400)
   - Check power supply

2. **Camera Not Detected**
   - Ensure RealSense is properly connected
   - Check USB 3.0 connection
   - Install RealSense drivers

3. **YOLOv8 Model Download**
   - First run will download ~6MB model file
   - Ensure internet connection
   - Check disk space

4. **Performance Issues**
   - Use YOLOv8n (nano) for speed
   - Reduce camera resolution if needed
   - Check CPU/GPU capabilities

### Debug Mode
The system includes comprehensive logging and visual feedback:
- **Console Output**: Real-time status and distance information
- **Video Display**: Bounding boxes and distance overlays
- **Error Handling**: Graceful fallbacks for detection failures

## Safety Features

- **Emergency Stop**: Ctrl+C or 'q' key for immediate halt
- **Obstacle Priority**: Collision avoidance takes precedence
- **Distance Limits**: Never gets closer than 1 meter to people
- **Speed Limits**: Configurable maximum speeds
- **Fallback Behavior**: Returns to basic obstacle avoidance if detection fails

## Future Enhancements

- **Multi-person tracking**: Follow specific individuals
- **Gesture recognition**: Respond to hand signals
- **Path planning**: Navigate to specific locations
- **Voice commands**: Speech recognition integration
- **Learning behavior**: Adapt to user preferences

## Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Test thoroughly
5. Submit a pull request

## License

[Add your license here]

## Acknowledgments

- **YOLOv8**: Ultralytics for computer vision
- **Intel RealSense**: Depth sensing technology
- **Roboclaw**: Motor control interface
- **OpenCV**: Computer vision library

## Support

For issues and questions:
- Check the troubleshooting section
- Review console output for error messages
- Ensure all dependencies are properly installed
- Verify hardware connections

---

**Note**: This is a robotics project. Always test in a safe environment and ensure proper safety measures are in place before operation.
