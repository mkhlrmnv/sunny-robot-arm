import requests
import time

BASE_URL = "http://127.0.0.1:5000/move_arm"

def test(cmd, params=None):
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
    time.sleep(1)


if __name__ == "__main__":
    test("init")
    # test("motor_paaty_up")
    # test("motor_paaty_down")
    # test("motor_pontto_ccw")
    # test("motor_pontto_cw")
    # test("motor_rail_right")
    # test("motor_rail_left")
# 
    # test("pl")
    # test("mn")
    # test("set_step_size", {"to": 25})

    test("by_angle", {
        "motor": "paaty",
        "angle": -20,
        "speed": 0.5,
        "check_safety": 1
    })
    
    test("to_angle", {
        "motor": "pontto",
        "angle": 120,
        "speed": 0.1,
        "check_safety": 0
    })
# 
    test("by_distance", {
        "dist": 20,
        "speed": 1,
        "check_safety": 0
    })

    # test("to_point",{
    #     "x": 1197.92882108,
    #     "y": -1020.43108514,
    #     "z": 1050,
    #     "check_safety": 1,
    #     "speed_rail": 0.1,
    #     "speed_joints": 0.1
    # })
