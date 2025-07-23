import requests
import time
import json

BASE_URL = "http://127.0.0.1:5000/move_arm"

def test(cmd, params=None):
    # Call the /points endpoint to get current arm position
    r = requests.get('http://127.0.0.1:5000/angles')
    
    print("Current arm points:")
    print(json.dumps(r.json(), indent=2))

    if params is None:
        params = {}
    params['cmd'] = cmd
    print(f"Testing: {cmd} {params}")
    r = requests.get(BASE_URL, params=params)
    try:
        j = r.json()
    except Exception as e:
        print("Invalid response:", r.text)
        return
    print(f"Status: {j['status']}, Message: {j['message']}")
    print("-" * 50)
    time.sleep(5)

'''
WORKS:
- by angel
- to angle
- by dist
- to dist
- to point
- to angles


'''


if __name__ == "__main__":
    # test("init")

    params = {
        "name": "test_path.json",
        "duration": 0,
        "dynamic_lamp": 1
    }

    r = requests.get('http://127.0.0.1:5000/api_play_path', params=params)
    
    print("Current arm points:")
    print(json.dumps(r.json(), indent=2))

    # test("motor_paaty_up")
    # test("motor_paaty_down")
    # test("motor_paaty_down")
    # test("set_step_size", {"to": 25})
    # test("motor_pontto_cw")
    # time.sleep(5)
    # test("motor_pontto_cw")
    # test("motor_pontto_cw")
    # test("motor_rail_right")
    # test("motor_rail_left")
# 
    # test("pl")
    # test("mn")
    # test("set_step_size", {"to": 25})


    # BY ANGLE WORKS
    # test("by_angle", {
    #     "motor": "pontto",
    #     "angle": -20,
    #     "speed": 0.1,
    #     "check_safety": 1
    # })

    
    # test("to_angle", {
    #     "motor": "paaty",
    #     "angle": 90,
    #     "speed": 0.1,
    #     "check_safety": 1
    # })
    # test("to_angle", {
    #     "motor": "paaty",
    #     "angle": 0,
    #     "speed": 0.1,
    #     "check_safety": 1
    # })
# 
    # test("to_angle", {
    #     "motor": "paaty",
    #     "angle": -100,
    #     "speed": 0.1,
    #     "check_safety": 1
    # })

# 
    # test("by_distance", {
    #     "dist": 20,
    #     "speed": 1,
    #     "check_safety": 1
    # })


    # safe one
    # test("to_point",{
    #     "x": 377.04045089,
    #     "y": 391.49348017,
    #     "z": 1115.0,
    #     "check_safety": 1,
    #     "speed_rail": 0.1,
    #     "speed_joints": 0.1
    # })

    # un safe
    # test("to_point",{
    #     "x": 1082.5,
    #     "y": 1500,
    #     "z": 210.0,
    #     "check_safety": 0,
    #     "speed_rail": 0.1,
    #     "speed_joints": 0.1
    # })

    # test("to_angles",{
    #     "theta_1": 90,
    #     "theta_2": -100,
    #     "delta_r": 20.0,
    #     "check_safety": 0,
    #     "speed_rail": 0.1,
    #     "speed_joints": 0.1
    # })
