# Sunny Robot Arm 🌞🤖

The project is built by Mikhail Romanov and Jaakko Aalto completely from scratch. Only motors, drivers, linear rail and arms aluminum profile were bought, all the rest is custom made by first designing parts in CAD, then manufacturing them using multiple metalworking techniques like CNC milling, water cutting and metal turning or 3D printing for smaller parts.

Project is a precision solar tracking robotic arm system that automatically follows the sun's path throughout the day. This 2-DOF robotic arm mounted on a linear rail uses advanced kinematics calculations to track solar positions while maintaining safety through collision detection. At the end arm has RGB LED lamp that ajust its brightness and color based on the solar position and system status. This features allow robot to simulate effect of the sunlight on office enviroment, which is then used in the Aalto University research to study the impact of natural light on human well-being and productivity in house automation and smart buildings.

![Basic view of the robot](/images_for_mds/basic_view_of_robot.png)

### Web Dashboard Features

- Real-time 3D robot visualization while executing pathits brightness and color based on the solar position and system status. This features allow robot to simulate effect of the sunlight on office enviroment, which is then used in the Aalto University research to study the impact of natural light on human well-being and productivity in house automation and smart buildings.

## 🌟 Features

- **Automated Solar Tracking**: Follows the sun's path automatically using astronomical calculations
- **Smart RGB LED Lamp**: WLED-controlled end effector with dynamic color and brightness that can be controlled via web browser, mobile app, or automatically synchronized with robot movements
- **Precise Kinematics**: Forward and inverse kinematics for accurate positioning
- **Safety Systems**: Built-in collision detection and safety boundary enforcement
- **Web Interface**: Real-time control and monitoring via Flask web application
- **Multi-Location Support**: Pre-configured solar paths for Finland and Singapore
- **Warning System**: Warning system with visual and audio alerts
- **Temperature Management**: Automatic cooling system with fan control
- **Manual Control**: Direct motor control for testing and calibration

## 🏗️ Hardware Architecture

### Robotic Arm Components
- **Pontto Joint**: Base rotation motor (spinning joint) controlling the yaw angle
- **Paaty Joint**: Pitch/elevation motor (spinning joint) controlling the pitch angle
- **Linear Rail**: Horizontal positioning system
- **LED Strip**: End effector with RGB LED lamp for visual status feedback via WLED (controllable via web browser, mobile app, or automatically synchronized with robot movements)
- **Cooling Fan**: Temperature management system
- **Limit Switches and Induction Sensors**: Safety and homing sensors

### Technical Specifications
- **Arm lengths**: 
  - Pontto: 835mm
  - Paaty: 905mm
- **Rail Travel**: 731.3mm linear range
- **Joint Angles**: 
  - Pontto: -172° to +188° (360° range)
  - Paaty: ±180° range
- **Precision**: Stepper motor control with gear ratios

## 📁 Project Structure

```
sunny-robot-arm/
├── src/                          # Core application code
│   ├── arm.py                    # Main robot arm controller
│   ├── kinematics_and_safety.py  # Forward/inverse kinematics & collision detection
│   ├── sun_helper.py             # Solar position calculations
│   ├── spinning_joints.py        # Rotational motor control
│   ├── linear_rail.py            # Linear actuator control
│   ├── lamp.py                   # LED control interface
│   ├── cooling.py                # Temperature management
│   └── config.py                 # Hardware configuration constants
│   └── warning.py                # Warning system for audio alerts
├── ui/                           # Web interface
│   ├── main.py                   # Flask web server
│   ├── templates/                # HTML templates
│   └── static/                   # CSS/JS assets
├── paths/                        # Pre-calculated solar paths
├── test/                         # Unit tests
├── documentation/                # Technical documentation
└── sounds/                       # Audio feedback files
```

## 🚀 Quick Start

### Were to start

- **Basic Guide** Basic tutorial on how to use the robot via web interface or API, refer to the tutorial md [here](quick_tutorial.md).
- **Documentation**: Detailed documentation in strucuture of the robot (electrical wirings, class structure, manufacturing process, etc.) can be found in the [documentation](documentation/README.md).



### Hardware used in this project
- Raspberry Pi 4
- Python 3.10
- 2x JT-RD6012 Power Supply (70v x 12.5A)
- 3x Leadshine CS-M23445 stepper motors with Leadshine CS-D808 drivers
- WLED-compatible LED strip (with 144 LEDs/m)
- 2x SN04-N Induction Sensors for homing
- 2x Basic limit switches for linear rail position detection
- A lot other unspecified parts like fan, power supplies and thermal sensor that can be found in wiring diagram


### Cooling System

Automatic temperature management with configurable thresholds:

- Fan activation at 30°C
- Maximum cooling at 60°C
- Temperature logging and alerts

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## 🙏 Acknowledgments

- **pvlib**: Solar position calculations
- **Flask**: Web interface framework  
- **NumPy/SciPy**: Mathematical computations
- **Matplotlib**: Visualization tools
- **RPi.GPIO**: Raspberry Pi hardware control

---

**Built with ❤️ and passion for robotics**