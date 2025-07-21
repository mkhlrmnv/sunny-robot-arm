# Sunny Robot Arm - Class Structure

## 🏗️ Core Architecture Overview

The project follows a modular architecture with clear separation of concerns:

```
Main Controller (Arm) 
    ├── Motor Controllers (SpinningJoints, LinearRail)
    ├── Visual Feedback (Lamp)  
    ├── Audio System (WarningSound)
    ├── Kinematics Engine (Functions)
    ├── Safety Systems (Functions)
    └── Cooling System (FanController)
```

## 📋 Class Diagram

### 🎮 **Main Controller Classes**

#### **`Arm`** - Primary Robot Controller
```python
class Arm:
    # Properties
    - theta_1: float              # Pontto joint angle (°)
    - theta_2: float              # Paaty joint angle (°) 
    - delta_r: float              # Linear rail position (mm)
    - required_theta_1: float     # Target pontto angle
    - required_theta_2: float     # Target paaty angle
    - required_delta_r: float     # Target rail position
    - current_path: np.array      # Path points to follow
    - current_path_colors: list   # Color data for path visualization
    - iteration: int              # Current path point index
    - duration_per_point: float   # Time per path point
    - shared: Namespace           # Multiprocessing shared state
    
    # Component References
    - motor_paaty: SpinningJoints     # Pitch motor controller
    - motor_pontto: SpinningJoints    # Yaw motor controller  
    - motor_rail: LinearRail          # Linear actuator controller
    - lamp: Lamp                      # LED strip controller
    - warning_sound: WarningSound     # Audio feedback system
    
    # Methods
    + __init__(shared, init_pos, dimensions...)
    + init()                          # Initialize and home all motors
    + shutdown()                      # Safe shutdown procedure
    + init_path(path_file, duration)  # Load solar tracking path
    + move(speeds, check_safety)      # Execute movement to next path point
    + _target_not_set() -> bool       # Check if target is defined
    + _step_towards(attr, target)     # Safe incremental movement
    + _compute_next_target()          # Calculate next path target
    + _clear_target()                 # Reset target after reaching point
    + draw_all(index, ax)             # Visualization for matplotlib
```

### 🔧 **Motor Controller Classes**

#### **`SpinningJoints`** - Rotational Motor Controller
```python
class SpinningJoints:
    # Properties
    - shared: Namespace           # Multiprocessing shared state
    - name: str                   # Motor identifier ("pontto"/"paaty")
    - angle: float                # Current angle (degrees)
    - steps: int                  # Step count from home position
    - step_per_rev: int           # Steps per motor revolution (1600)
    - gear_ratio: float           # Gearbox reduction ratio
    - angle_limit: float          # Maximum rotation range
    - min_delay: float            # Fastest step timing
    - max_delay: float            # Slowest step timing
    
    # Hardware Interfaces
    - pulse: DigitalOutputDevice  # Step pulse pin
    - direction: DigitalOutputDevice  # Direction control pin
    - limit_switch: Button        # Homing sensor
    - limit_event: Event          # Thread-safe sensor state
    
    # Methods
    + __init__(shared, pins, name, ratios...)
    + init_motor(speed)           # Homing sequence with sensor detection
    + step(direction, speed)      # Single step execution
    + move_by_angle(angle, speed) # Relative angular movement
    + move_to_angle(angle, speed) # Absolute angular positioning
    + calc_delay(speed_percent)   # Speed-to-timing conversion
    + reset_position()            # Set current position as zero
    + shutdown()                  # Safe parking position
```

#### **`LinearRail`** - Linear Actuator Controller  
```python
class LinearRail:
    # Properties
    - shared: Namespace           # Multiprocessing shared state
    - distance: float             # Current position (mm)
    - steps: int                  # Step count from home
    - angle: float                # Equivalent angular displacement
    - step_per_rev: int           # Motor steps per revolution
    - gear_ratio: float           # Gear reduction factor
    - pitch: float                # Lead screw pitch (mm/rev)
    - min_delay: float            # Timing limits for speed control
    - max_delay: float
    - stop: bool                  # Limit switch state
    
    # Hardware Interfaces  
    - pulse: DigitalOutputDevice  # Step pulse pin
    - direction: DigitalOutputDevice  # Direction control pin
    - limit_switch: Button        # Position limit sensor
    - limit_event: Event          # Thread-safe limit state
    
    # Methods
    + __init__(shared, pins, mechanical_params...)
    + init_motor(direction)       # Homing to limit switch
    + step(direction, speed, ignore_limit)  # Single step with safety
    + move_by_distance(dist, speed)         # Relative linear movement
    + move_to_distance(dist, speed)         # Absolute positioning
    + calc_delay(speed_percent)             # Speed mapping
    + reset_position()                      # Zero position reference
```

### 🎨 **Feedback Systems**

#### **`Lamp`** - RGB LED Controller
```python
class Lamp:
    # Properties
    - brightness: int             # Overall brightness (0-255)
    - https_url: str              # WLED controller URL
    - red: int                    # Current red component (0-255)
    - green: int                  # Current green component (0-255) 
    - blue: int                   # Current blue component (0-255)
    - effect: int                 # WLED effect index (0=solid, 1=blink...)
    - speed: int                  # Effect speed (milliseconds)
    
    # Methods
    + __init__(brightness, https_url)
    + set_brightness(level)       # Control overall brightness
    + set_to_effect(r, g, b, effect, speed)  # Generic effect control
    + set_to_solid(r, g, b)       # Solid color mode
    + set_to_blink(r, g, b, speed)           # Blinking warning mode
    + _send_to_wled(json_data)               # HTTP API communication
```

#### **`WarningSound`** - Audio Alert System
```python
class WarningSound:
    # Properties
    - _stop: threading.Event      # Thread synchronization
    - _thread: Thread             # Background playback thread
    - _is_playing: bool           # Current playback state
    
    # Methods
    + __init__(audio_file)        # Load sound file via pygame
    + start()                     # Begin looped playback
    + stop()                      # Stop playback and cleanup
    + _play_loop()                # Internal playback thread
```

#### **`FanController`** - Thermal Management
```python
class FanController:
    # Properties
    - sensor: W1ThermSensor       # DS18B20 temperature sensor
    - fan: PWMOutputDevice        # PWM-controlled cooling fan
    - min_temp: float             # Fan activation threshold (°C)
    - max_temp: float             # Full-speed threshold (°C)
    - latest_reading: dict        # Current temperature/speed data
    
    # Methods
    + __init__(fan_pin, min_temp, max_temp)
    + map_temp_to_speed(temp)     # Temperature to PWM conversion
    + run(verbal, interval)       # Main control loop
    + shutdown()                  # Safe fan stop and GPIO cleanup
```

### 🧮 **Utility Functions** (Functional Programming)

#### **Kinematics Module** (`kinematics_and_safety.py`)
```python
# Core Kinematics Functions
+ inverse_kinematics(x, y, z, **params) -> List[Tuple[float, float, float]]
    # Solve joint angles for target position
    
+ forward_kinematics(theta1, theta2, delta_r, **params) -> np.array
    # Calculate all joint positions from angles
    
+ choose_solution(solutions, current_state) -> Tuple[float, float, float]
    # Select optimal solution from IK candidates

# Safety and Collision Detection
+ check_solutions_safety(solution, safety_boxes) -> bool
    # Verify movement doesn't cause collisions
    
+ draw_robot(ax, points) -> None
    # 3D visualization of robot configuration
    
+ draw_all_safety_boxes(ax) -> None
    # Render collision boundaries
```

#### **Solar Path Module** (`sun_helper.py`)
```python
# Solar Position Calculations
+ get_sun_path_finland(R, lat, lon, timezone) -> Tuple[np.array, np.array, np.array]
    # Generate solar tracking path for Finland
    
+ get_sun_path_singapore(R, lat, lon, timezone) -> Tuple[np.array, np.array, np.array]
    # Generate solar tracking path for Singapore
    
+ alt_to_color(altitude_deg) -> Tuple[int, int, int]
    # Map solar altitude to RGB color temperature
    
+ jsonify_path(path, colors, filename) -> None
    # Serialize path data to JSON
    
+ un_jsonify_path(filename) -> Tuple[np.array, np.array]
    # Load path data from JSON
```

### 🌐 **Web Interface** (`ui/main.py`)

#### **Flask Application**
```python
# Global State
- app: Flask                      # Web server instance
- arm: Arm                        # Robot controller reference
- shared: Namespace               # Multiprocessing shared state
- motor_lock: Lock                # Thread synchronization
- angles_per_key: int             # Manual control step size

# Core Routes
@app.route('/')                   # Main dashboard
@app.route('/manual')             # Manual control interface
@app.route('/move_arm')           # Primary API endpoint
@app.route('/points')             # Current robot position data
@app.route('/path_points')        # Active path visualization data  
@app.route('/cooling_info')       # Temperature monitoring

# Process Management Functions
+ start_arm(func, args)           # Execute robot function in subprocess
+ stop_arm()                      # Terminate active robot processes
+ is_arm_running() -> bool        # Check subprocess status
+ start_arm_and_wait(func, args) -> int  # Blocking execution with return code
```

## 🔄 **Data Flow Architecture**

### **Multiprocessing Communication**
```python
# Shared State Object (Manager.Namespace)
shared = {
    'theta_1': float,        # Current pontto angle
    'theta_2': float,        # Current paaty angle  
    'delta_r': float,        # Current rail position
    'path_it': int,          # Path iteration counter
    'timer': float,          # Last movement timestamp
    'path': np.array         # Active path points
}
```

### **Control Flow**
1. **Web Interface** receives user commands
2. **Flask API** validates parameters and safety
3. **Process Manager** spawns subprocess for robot control
4. **Arm Controller** coordinates motor movements
5. **Motor Controllers** execute precise positioning
6. **Feedback Systems** provide visual/audio status
7. **Shared State** maintains synchronization across processes

### **Safety Integration**
- All movements validated through **inverse kinematics**
- **Collision detection** prevents unsafe configurations
- **Joint limits** enforced at motor controller level
- **Emergency stop** capability via web interface
- **Homing sequences** ensure accurate position reference

## 📊 **Key Design Patterns**

### **Composition over Inheritance**
- `Arm` class aggregates motor controllers rather than inheriting
- Enables flexible configuration and testing
- Clear component boundaries and responsibilities

### **Shared State Pattern**  
- `multiprocessing.Manager.Namespace` for inter-process communication
- Thread-safe access to robot state across web server and control processes

### **Strategy Pattern**
- Different motor types (`SpinningJoints`, `LinearRail`) with common interface
- Kinematics functions can be swapped for different robot configurations

### **Observer Pattern**
- GPIO limit switches trigger events for safety monitoring
- Real-time status updates to web interface

This architecture provides a robust, safe, and extensible foundation for precise solar tracking while maintaining clear separation between hardware control, safety systems, and user interfaces.
