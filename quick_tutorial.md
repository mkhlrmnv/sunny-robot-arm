# Quick Tutorial

This tutorial will guide you through the basics of using the Sunny Robot Arm.

## Getting Started

### Basics

Once powering the system (turning electricity on), rasperry pi will boot and automatically start the server. You can access the web interface by navigating to `http://...:5000` in your web browser. Also make sure that your computer is connected to the lab network '...' or you are using stationary computer in the lab.

Credentials to connect to correct network are:
- name: '...'
- password: '...'

Booting up usually takes about 30 seconds, so please be patient:)

### Terminology

![Terminology](/images_for_mds/terminology.png)

### Troubleshooting

If after few minutes you still cannot acces the web interface, you have to ssh into the raspberry to manually start the server:

1. Connect to '...' network or use a stationary computer in the lab.
2. Open a terminal on your computer.
3. Type `ssh raspi@raspi.local` and press Enter.
4. Enter the password `...` when prompted.
5. Once logged in, you have to kill all python processes or verify that none are running. You can do this by typing the following command:

```bash
sudo pkill -f python
```

To verify that none are running, run:
```bash
ps aux | grep python
```
If you see any python processes listed, you can kill them using:
```bash
sudo kill <PID>
```

6. After ensuring no python processes are running, start the server by typing:
```bash
cd ~/Desktop/sunny-robot-arm
python -m ui.main
```

7. Wait for the server to start, which may take a few seconds.
8. If none errors occur, follow the instructions in the "Basics" section to access the web interface.

### Alternative Controlling Methods

#### Manual Control via Terminal

If for some reason you want to run manual control straight from the terminal of raspberry pi, you have to first ssh into the raspberry pi as described above. Then you can run the manual control script by typing:

```bash
cd ~/Desktop/sunny-robot-arm
python src/wsd_control.py
```

This will start the script that will listen for keyboard inputs to control the robot arm.

#### Running Path Following Script manually

As always you have to ssh into the raspberry pi as described above. Afterwards navigade to the '~/Desktop/sunny-robot-arm/src/arm.py' file and in the __if __name__ == "__main__":__ section, you can change the code to what ever you want to run. Currently there should be script that runs finnish path but it may been mofidied to something else:)

Once you hacve modified the code, you can run the script by typing:

```bash
cd ~/Desktop/sunny-robot-arm
python src/arm.py
```

This will execute the script and the robot arm will start following the path defined in the script.

## Using the Web Interface

### Basics

In the web ui, you will see following buttons:

- **Init Arm**: This button initializes the robot arm, setting it to a default position.
- **Play Path**: This button will first initialize motors and then display the page from 
which you can select the path to follow, duration and the behavior of the lamp. 
- **Manual Control**: This button will open a new page where you can control the robot arm manually. Lower will be image describing the buttons that are found on this page.
- **Sensor Test**: This button will start the test that will allow you to check that they are working and correctly calibrated. After clicking button there are comprehensive instructions on how to complete the test.
- **Shutdown**: This button will move arm in such position where power can be safely turned off without damaging the arm.

### Manual Control Page

On this page you can control motors via six different buttons, which will turn arm in different directions which are described in the image below. You can control arm by single clicks or by holding the button down. 

![Manual Control Directions](/images_for_mds/directions_of_manual_control.png)

On the bottom of the page you can also set up how much each click will turn motor. This is done by increasing "step size" value.
One step size transfers to 1 degree in rotation joints (pontto / paaty) and 1 cm in linear joint (linear rail).

Also if direction image is not clear enough, you can figure out directions of the arm by setting the step size to 1, and clicking the buttons one by one. This will move the arm slowly and you can see how it moves in real time.

If in any point you can interrupt the manual control by clicking the "Stop" button. This will stop all motors immediately and redirect you to the main page. After this you can continue using the web interface as usual.

### Path Following Page

Page offers in total four different paths to follow:

- **Test path**: Is just basic path of two points, in which arm moves from point A to point B and set up the lamp to blue in the point A and to green in point B.
- **Finnish path**: Is a path that follows path of finnish path on 21st of June 2025. In total path has 99 points that are calculated with 10min intervals. So you want to exact follow of the path, durations should be set to 59400 (as 99 * 10min = 990min = 59400s).
- **Singapore path v1** and **Singapore path v2**: These are two paths of the singaporian sun, on the same day as in the finnish path. The difference between paths in relias in how unreachable points were handeled while creating paths. This differences are described in tha math notebook which can be found in 'documentation/math.md' file. Both paths ahve 69 points that are calculated with 10min intervals. So you want to exact follow of the path, durations should be set to 41400 (as 69 * 10min = 690min = 41400s).
- **Duration**: Is selector of how long the path will take to follow. Timer will start then robot has reached the first point of the path. Notice that robot doesn't take into account the time it takes to move from point to point, so if durations is set to low, robot will go though path as fast as possible, but it will not finnish the path in time
- **Lamp**: This is check box that allows you to set the lamp to change color in each point of the path. If checked, lamp will change color based on the estimations of the sun's color in that point. If not checked, arm will not change color of the lamp, in other occasion then when it has to warn surroundings of the arm's movement. After warning isn't needed anymore, lamp will restore to the color it was set to before warning.

## Controlling the lamp

The lamp is powered by wled, which is a software that allows you to control the lamp via web interface. You can access the wled interface by navigating to `http://...` in your web browser while connected to the lab network.

For more instructions on how to use the wled interface, please refer to the [wled documentation](https://kno.wled.ge)

## API

While the web server is running, you can access the REST API endpoints for programmatic control. All endpoints return JSON responses with status information.

### Base URL
```
http://...:5000
```

### Stoping the Robot
To stop the robot at any points, just call the index endpoint `/`:
```bash
curl "http://...:5000/"
``` 
or 
```bash
curl "http://...:5000/stop_arm"
```

### Main Control Endpoint: `/move_arm`

All robot control commands use the `/move_arm` endpoint with different `cmd` parameters:

#### Manual Control Commands

**Step-based Movement (for manual jogging):**
```bash
# Move joints by step increments
curl "http://...:5000/move_arm?cmd=motor_paaty_up"      # Pitch up
curl "http://...:5000/move_arm?cmd=motor_paaty_down"    # Pitch down  
curl "http://...:5000/move_arm?cmd=motor_pontto_ccw"    # Rotate counter-clockwise
curl "http://...:5000/move_arm?cmd=motor_pontto_cw"     # Rotate clockwise
curl "http://...:5000/move_arm?cmd=motor_rail_right"    # Rail right
curl "http://...:5000/move_arm?cmd=motor_rail_left"     # Rail left

# Adjust step size
curl "http://...:5000/move_arm?cmd=pl"                 # Increase step size (+10)
curl "http://...:5000/move_arm?cmd=mn"                 # Decrease step size (-10)
curl "http://...:5000/move_arm?cmd=set_step_size&to=5" # Set specific step size
```

Directions in which the arm will move are described in "Manual Control Page" section above.

#### Precise Motor Control

**Move by relative angle/distance:**
```bash
# Move motor by specified amount (with safety checking)
curl "http://...:5000/move_arm?cmd=by_angle&motor=pontto&angle=45&speed=0.3&check_safety=1"
curl "http://...:5000/move_arm?cmd=by_angle&motor=paaty&angle=-30&speed=0.5&check_safety=0"
curl "http://...:5000/move_arm?cmd=by_distance&dist=100&speed=0.2&check_safety=1"
```

**Move to absolute angle/distance:**
```bash
# Move motor to absolute position
curl "http://...:5000/move_arm?cmd=to_angle&motor=pontto&angle=90&speed=0.4&check_safety=1"
curl "http://...:5000/move_arm?cmd=to_angle&motor=paaty&angle=45&speed=0.3&check_safety=0"
curl "http://...:5000/move_arm?cmd=to_distance&dist=200&speed=0.5&check_safety=1"
```

#### Coordinate-based Movement

**Move to 3D point:**
```bash
# Move end-effector to specific coordinates (x, y, z in mm)
curl "http://...:5000/move_arm?cmd=to_point&x=500&y=300&z=800&speed_joints=0.2&speed_rail=0.4&check_safety=1"
```

**Move to joint configuration:**
```bash
# Move to specific joint angles and rail position
curl "http://...:5000/move_arm?cmd=to_angles&theta_1=90&theta_2=45&delta_r=200&speed_joints=0.3&speed_rail=0.5&check_safety=1"
```

#### System Commands

**Initialize robot:**
```bash
curl "http://...:5000/move_arm?cmd=init"
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

### Path Execution Endpoint

**Execute complete path sequence:**

```bash
# Execute a saved path with motor initialization and synchronous completion
curl "http://...:5000/api_play_path?name=finish_sun_path.json&duration=3600&lamp=1"
```

| Parameter | Type | Description | Required |
|-----------|------|-------------|----------|
| `name` | string | Path filename in `/paths` directory | Yes |
| `duration` | int | Total execution time in seconds | Yes |
| `lamp` | int | Enable dynamic lamp colors (0/1) | No (default: 1) |

**Response Format:**

```json
{
  "status": "ok",                                    // "ok" or "error"
  "message": "arm reached the end of the path"       // Completion status
}
```

**Error Responses:**

```json
{
  "status": "error",
  "message": "robot stopped due to safety reasons"   // Error code 69
}
```

```json
{
  "status": "error", 
  "message": "robot stopped due to unknown reasons"  // Other exit codes
}
```

### 📊 Monitoring end points

- `/points`: Get all points of the robot arm
- `/angles`: Get current joint angles
- `/path_points`: Get current path points for visualization
- `/cooling_info`: Get current cooling system status (temperature, fan state)

## Updating code

If some code needs to be updated, you can do it by following these steps:

1. Update the code locally in this repository and push the changes to the remote repository.
2. SSH into the Raspberry Pi as described in the "Getting Started / Troubleshooting" section.
3. Navigate to the sunny-robot-arm directory:
   ```bash
   cd ~/Desktop/sunny-robot-arm
   ```
4. Pull the latest changes from the remote repository:
   ```bash
   git pull
   ```
5. Restart server to apply the changes:
```bash
pkill -f python
python -m ui.main
```  