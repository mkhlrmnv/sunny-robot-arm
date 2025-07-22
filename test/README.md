# Test Suite for Sunny Robot Arm

This directory contains comprehensive unit tests and integration tests for the Sunny Robot Arm project.

## Test Files

### `test_kinematics_and_safety.py`
Comprehensive unit tests for the kinematics and safety module including:

- **Forward Kinematics Tests**
  - Home position calculations
  - Known configurations
  - Extreme positions
  - Consistency checks

- **Inverse Kinematics Tests**
  - Reachable point solving
  - Unreachable point handling
  - Multiple solution detection
  - Rail limit enforcement
  - Safety integration

- **Solution Selection Tests**
  - Closest solution selection
  - Edge cases handling

- **Safety & Collision Detection Tests**
  - Safe position verification
  - Unsafe position detection
  - Edge-box intersection testing
  - Safety box validation

- **Round-trip Consistency Tests**
  - Forward→Inverse→Forward consistency
  - Error tolerance validation

- **Performance Tests**
  - Execution time validation
  - Batch processing efficiency

### `test_integration_kinematics.py`
Integration tests for realistic scenarios:

- **Robot Initialization**
  - Home position reachability
  - Workspace validation

- **Solar Tracking Simulation**
  - Typical sun positions
  - Workspace coverage

- **Trajectory Following**
  - Smooth path execution
  - Joint movement validation

- **Workspace Boundaries**
  - Limit testing
  - Edge case handling

- **Safety System Integration**
  - End-to-end safety validation

- **Performance Integration**
  - Batch processing
  - Real-time capability

### `test_sun_helper.py`
Unit tests for sun path serialization utilities:

- **TestJsonifyPath**
  - Path-only JSON serialization
  - Path with colors serialization
  - Array structure validation
  - Output format verification

- **TestUnJsonifyPath**
  - JSON file deserialization
  - Data validation and type checking
  - Error handling for invalid files
  - Color data restoration

- **TestPathSerialization**
  - Round-trip serialization consistency
  - Data integrity verification
  - Precision preservation

- **TestEdgeCases**
  - Empty path handling
  - Invalid input validation
  - File operation error cases
  - Malformed JSON handling

### `test_move_arm_api.py`
Comprehensive Flask API endpoint tests:

- **TestMoveArmAPI**
  - Manual control commands (motor movements)
  - Motor control API (by_angle, to_angle, by_distance, to_distance)
  - Position control (to_point, to_angles)
  - Initialization and safety checking
  - Parameter validation and error handling

- **TestAPIHelpers**
  - Helper function testing
  - Error condition validation
  - Response format verification

- **TestAPIIntegration**
  - End-to-end API workflow testing
  - Multi-command sequences
  - State consistency validation

### `test_spinning_joints.py`
Comprehensive unit tests for the SpinningJoints stepper motor class:

- **TestSpinningJointsInitialization**
  - Parameter validation and object creation
  - GPIO pin configuration validation
  - Motor specifications (steps/rev, gear ratio, delays)

- **TestStepCalculations**
  - Step timing calculations based on speed percentage
  - Motor stepping logic with direction control
  - GPIO pulse generation and state updates

- **TestAngleMovement**
  - Relative angle movement (move_by_angle)
  - Absolute angle positioning (move_to_angle)
  - Angle limit enforcement and validation

- **TestMotorInitialization**
  - Motor-specific initialization sequences (pontto vs paaty)
  - Limit switch detection and homing routines
  - Initialization timeout and error handling

- **TestSafetyValidation**
  - Angle limit checking and enforcement
  - Error handling for invalid parameters
  - Safety constraint validation

- **TestSharedStateManagement**
  - Multiprocessing shared state coordination
  - Motor-specific shared variable updates (theta_1, theta_2)
  - Cross-process state synchronization

- **TestCleanupAndUtilities**
  - GPIO resource cleanup and management
  - Utility functions (get_steps, get_angle, reset_position)
  - Proper resource deallocation

### `test_linear_rail.py`
Comprehensive unit tests for the LinearRail stepper motor class:

- **TestLinearRailInitialization**
  - Parameter validation for linear stage setup
  - GPIO configuration and mechanical parameters
  - Screw drive pitch and gear ratio validation

- **TestDistanceCalculations**
  - Distance-to-steps conversion calculations
  - Step timing based on speed percentage
  - GPIO control for linear movement

- **TestDistanceMovement**
  - Relative distance movement (move_by_distance)
  - Absolute distance positioning (move_to_distance)
  - Movement direction and step calculation

- **TestMotorInitialization**
  - Limit switch-based homing sequences
  - Back-off positioning after limit detection
  - Multi-direction initialization support

- **TestLimitSwitchHandling**
  - Limit switch detection and response
  - Movement blocking when limits are active
  - Override capabilities with ignore_limit flag

- **TestSharedStateManagement**
  - Shared delta_r state coordination
  - Real-time position tracking across processes
  - State synchronization for multi-process control

- **TestCleanupAndUtilities**
  - Linear stage resource management
  - Position reset and calibration functions
  - Proper GPIO cleanup procedures

### `test_api.py`
Simple API testing script for manual verification:

- Live API endpoint testing
- HTTP request/response validation
- Manual command execution
- Basic integration verification

### `run_tests.py`
Enhanced test runner with multiple execution options:

- Individual test module execution
- Combined test suite running
- Verbose output options
- Specific test class/method targeting

## Running Tests

### Option 1: Using test runner (Recommended)

```bash
# From project root directory
python test/run_tests.py                              # Run kinematics unit tests
python test/run_tests.py --verbose                    # Verbose output  
python test/run_tests.py --integration               # Run integration tests
python test/run_tests.py --sun-helper                # Run sun_helper utility tests
python test/run_tests.py --api                       # Run Flask API tests
python test/run_tests.py --all                       # Run all test modules
python test/run_tests.py --specific TestForwardKinematics  # Run specific test class
```

### Option 2: Using pytest

```bash
# From project root directory
python -m pytest test/ -v

# Run specific test file
python -m pytest test/test_kinematics_and_safety.py -v
python -m pytest test/test_sun_helper.py -v
python -m pytest test/test_move_arm_api.py -v

# Run specific test class
python -m pytest test/test_kinematics_and_safety.py::TestForwardKinematics -v

# Run with coverage
python -m pytest test/ --cov=src --cov-report=html
```

### Option 3: Direct execution

```bash
# From project root directory
cd test
python test_kinematics_and_safety.py
python test_integration_kinematics.py
python test_sun_helper.py
python test_move_arm_api.py
```

### Option 4: Manual API testing

```bash
# Start the Flask server first
python ui/main.py

# Then run API tests in another terminal
python test/test_api.py
```

## Test Dependencies

Make sure you have all required packages installed:

```bash
pip install numpy scipy matplotlib pvlib pandas pytest pytest-cov flask requests
```

For API testing specifically:

```bash
# Core testing packages
pip install unittest-mock

# For integration testing  
pip install tempfile json
```

## Test Categories

### Unit Tests
- Individual function testing
- Input validation
- Error handling
- Edge cases
- Performance benchmarks

### Integration Tests  
- Multi-component workflows
- Realistic scenarios
- System-level validation
- Performance under load

## Expected Test Results

### Passing Tests
Most tests should pass when run on properly configured hardware. Some tests may be skipped if:
- Positions are outside robot workspace
- Safety constraints prevent certain movements
- Hardware-specific configurations differ

### Performance Benchmarks
- Inverse kinematics: >50 solutions/second
- Forward kinematics: >1000 calculations/second  
- Safety checking: >100 checks/second
- Position accuracy: <1mm error tolerance

## Debugging Failed Tests

### Import Errors
```bash
# Make sure you're in the project root
pwd  # Should show .../sunny-robot-arm

# Check Python path
python -c "import sys; print(sys.path)"

# Install missing dependencies
pip install -r requirements.txt
```

### Calculation Errors
- Check that config.py values match your hardware
- Verify safety box definitions are appropriate
- Ensure coordinate frames are correctly defined

### Performance Issues
- Run tests on dedicated hardware without GUI interference
- Check for CPU throttling or high system load
- Verify NumPy is using optimized BLAS libraries

## Adding New Tests

### Test Structure
```python
class TestNewFeature(unittest.TestCase):
    def setUp(self):
        """Set up test fixtures before each test method."""
        pass
    
    def test_specific_behavior(self):
        """Test description."""
        # Arrange
        input_data = ...
        
        # Act  
        result = function_under_test(input_data)
        
        # Assert
        self.assertEqual(result, expected)
    
    def tearDown(self):
        """Clean up after each test method."""
        pass
```

### Best Practices
- Use descriptive test names
- Test both success and failure cases
- Include performance tests for critical functions
- Use subTest() for multiple related test cases
- Mock hardware dependencies when testing logic
- Validate numerical accuracy with appropriate tolerances

## Coverage Goals

Target test coverage:

- **Kinematics functions**: >95%
- **Safety functions**: >90%  
- **Utility functions**: >80%
- **Error handling**: >95%
- **API endpoints**: >85%
- **Path serialization**: >95%

Run coverage analysis:

```bash
python -m pytest test/ --cov=src --cov-report=term-missing
```

This will show exactly which lines need additional test coverage.

## Current Test Status

### Fully Tested Modules

- ✅ **sun_helper.py**: 18 tests covering all path serialization functions
- ✅ **kinematics_and_safety.py**: Comprehensive coverage of all kinematics functions
- ✅ **spinning_joints.py**: 22 tests covering stepper motor control for rotating joints
- ✅ **linear_rail.py**: 22 tests covering linear stepper motor stage control
- ✅ **Flask API structure**: All endpoints and parameter validation tested

### Partially Tested Modules  

- 🟡 **Flask API functionality**: Structural tests complete, functional tests with mocking in progress
- 🟡 **Integration workflows**: Basic coverage, expanding for real-world scenarios

### Test Metrics

- **Total test files**: 7
- **Total test classes**: 20+
- **Total test cases**: 120+
- **Estimated coverage**: >90% for core modules
