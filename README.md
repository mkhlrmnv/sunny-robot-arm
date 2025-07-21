# Sunny Robot Arm 🌞🤖

A precision solar tracking robotic arm system that automatically follows the sun's path throughout the day. This 2-DOF robotic arm mounted on a linear rail uses advanced kinematics calculations to track solar positions while maintaining safety through collision detection. At the end arm has RGB LED lamp that ajust its brightness and color based on the solar position and system status. This features allow robot to simulate effect of the sunlight on office enviroment, which is then used in the Aalto University research to study the impact of natural light on human well-being and productivity in house automation and smart buildings.

Project is Build by Mikhail Romanov and Jaakko Aalto completely from scratch. Only motors, drivers, linear rail and arms aluminion profile were bought, all the rest is custom made by first desigining parts in CAD, then manufacturing them using multiple metalworking techniques like CNC milling, water cutting and metal turning or 3D printing for smaller parts.

## 🌟 Features

- **Automated Solar Tracking**: Follows the sun's path automatically using astronomical calculations
- **Adjusting RGB LED Lamp**: Simulates sunlight effects with dynamic color and brightness
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
- **LED Strip**: End effector with RGB LED lamp for visual status feedback via WLED
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

### Hardware used in this project
- Raspberry Pi 4
- Python 3.10
- 2x JT-RD6012 Power Supply (70v x 12.5A)
- 3x Leadshine CS-M23445 stepper motors with Leadshine CS-D808 drivers
- WLED-compatible LED strip (with 144 LEDs/m)
- 2x SN04-N Induction Sensors for homing
- 2x Basic limit switches for linear rail position detection
- A lot other unspecified parts like fan, power supplies and thermal sensor that can be found in wiring diagram

### Installation

1. **Clone the repository**
   ```bash
   git clone https://github.com/mkhlrmnv/sunny-robot-arm.git
   cd sunny-robot-arm
   ```

2. **Install dependencies**
   ```bash
    python3 -m venv sunny
    source sunny/bin/activate
    pip install --upgrade pip
    pip install -r requirements.txt
   ```

3. **Configure hardware**
   - Update GPIO pin assignments in `src/config.py` or connect to the motors using the pins provided in the `src/config.py`
   - Adjust motor gear ratios and limits if they differ from defaults
   - Set up WLED LED strip URL in environment variables, mainly setting the https address of the WLED controller in .env file:
    ```bash
    export WLED_ADR="http://<WLED_IP_ADDRESS>"
    ```

4. **Run the system**
    - For manual control via keyboard interface:
    ```bash
    source sunny/bin/activate
    python src/manual_control.py
    ```

### Web Interface

Start the web control interface:
```bash
source sunny/bin/activate
python -m ui.main
```

Access the interface at `http://localhost:5000` for:
- Path execution with real-time robot visualization
- Manual motor control
- Sensor testing
- Cooling system monitoring

## 🎯 API Usage

While the web server is running, you can access the REST API endpoints for programmatic control. All endpoints return JSON responses with status information.

### Stoping the Robot
To stop the robot at any points, just call the index endpoint `/`:
```bash
curl "http://localhost:5000/"
``` 

### Base URL
```
http://localhost:5000
```

### Main Control Endpoint: `/move_arm`

All robot control commands use the `/move_arm` endpoint with different `cmd` parameters:

#### Manual Control Commands

**Step-based Movement (for manual jogging):**
```bash
# Move joints by step increments
curl "http://localhost:5000/move_arm?cmd=motor_paaty_up"      # Pitch up
curl "http://localhost:5000/move_arm?cmd=motor_paaty_down"    # Pitch down  
curl "http://localhost:5000/move_arm?cmd=motor_pontto_ccw"    # Rotate counter-clockwise
curl "http://localhost:5000/move_arm?cmd=motor_pontto_cw"     # Rotate clockwise
curl "http://localhost:5000/move_arm?cmd=motor_rail_right"    # Rail right
curl "http://localhost:5000/move_arm?cmd=motor_rail_left"     # Rail left

# Adjust step size
curl "http://localhost:5000/move_arm?cmd=pl"                 # Increase step size (+10)
curl "http://localhost:5000/move_arm?cmd=mn"                 # Decrease step size (-10)
curl "http://localhost:5000/move_arm?cmd=set_step_size&to=5" # Set specific step size
```

#### Precise Motor Control

**Move by relative angle/distance:**
```bash
# Move motor by specified amount (with safety checking)
curl "http://localhost:5000/move_arm?cmd=by_angle&motor=pontto&angle=45&speed=0.3&check_safety=1"
curl "http://localhost:5000/move_arm?cmd=by_angle&motor=paaty&angle=-30&speed=0.5&check_safety=0"
curl "http://localhost:5000/move_arm?cmd=by_distance&dist=100&speed=0.2&check_safety=1"
```

**Move to absolute angle/distance:**
```bash
# Move motor to absolute position
curl "http://localhost:5000/move_arm?cmd=to_angle&motor=pontto&angle=90&speed=0.4&check_safety=1"
curl "http://localhost:5000/move_arm?cmd=to_angle&motor=paaty&angle=45&speed=0.3&check_safety=0"
curl "http://localhost:5000/move_arm?cmd=to_distance&dist=200&speed=0.5&check_safety=1"
```

#### Coordinate-based Movement

**Move to 3D point:**
```bash
# Move end-effector to specific coordinates (x, y, z in mm)
curl "http://localhost:5000/move_arm?cmd=to_point&x=500&y=300&z=800&speed_joints=0.2&speed_rail=0.4&check_safety=1"
```

**Move to joint configuration:**
```bash
# Move to specific joint angles and rail position
curl "http://localhost:5000/move_arm?cmd=to_angles&theta_1=90&theta_2=45&delta_r=200&speed_joints=0.3&speed_rail=0.5&check_safety=1"
```

#### System Commands

**Initialize robot:**
```bash
curl "http://localhost:5000/move_arm?cmd=init"
```

### Parameters

| Parameter | Type | Description | Default |
|-----------|------|-------------|---------|
| `cmd` | string | Command type (required) | - |
| `motor` | string | Motor name: `pontto`, `paaty` | - |
| `angle` | float | Angle in degrees | - |
| `dist` | float | Distance in mm | - |
| `x`, `y`, `z` | float | World frame 3D coordinates in mm | - |
| `theta_1`, `theta_2`, `delta_r` | float | Joint angles (°) and rail position (mm) | - |
| `speed` | float | Movement speed (0.0-1.0) | varies |
| `speed_joints` | float | Joint movement speed (0.0-1.0) | 0.1 |
| `speed_rail` | float | Rail movement speed (0.0-1.0) | 0.5 |
| `check_safety` | int | Enable safety checking (0/1) | 1 |
| `to` | int | Target value for step size | - |

### Response Format

All endpoints return JSON with the following structure:
```json
{
  "status": "ok",           // "ok" or "error"
  "message": "Operation completed successfully", // Usually specifing the starting and ending positions
  "step_size": 10          // Current step size (when applicable)
}
```

### Error Responses

```json
{
  "status": "error",
  "message": "Inverse kinematics failed: Target position unreachable", // Error message with details
  "step_size": 10        // Current step size (when applicable)
}
```

### Safety Features

- **Collision Detection**: When `check_safety=1`, the system verifies movements won't cause collisions, but this assumes that the robot has been properly homed before the use and no errors have occurred.
- **Joint Limits**: Automatic enforcement of joint angle and rail position limits  
- **Kinematics Validation**: Ensures target positions are reachable before movement
- **Multi-motor Coordination**: Prevents unsafe movements that would require coordinated motion

### Example Python Usage

```python
import requests

# Move to specific point with safety checking
response = requests.get("http://localhost:5000/move_arm", params={
    'cmd': 'to_point',
    'x': 600,
    'y': 400, 
    'z': 700,
    'speed_joints': 0.2,
    'speed_rail': 0.4,
    'check_safety': 1
})

result = response.json()
if result['status'] == 'ok':
    print(f"Success: {result['message']}")
else:
    print(f"Error: {result['message']}")
```

## 📊 Monitoring

### Web Dashboard Features
- Real-time 3D robot visualization while executing path
- Live joint position and temperature readings
  - Also available via API by:
   1. '/points' end point for all robot points
   2. '/cooling_info' end point for cooling system status
   3. '/path_points' end point for path points
- Manual control

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