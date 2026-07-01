#!/usr/bin/env python3
import json
import os
import time
import rospy
from RobotChassis import RobotChassis
from std_srvs.srv import Empty
def speak(g):
    os.system(f'espeak "{g}"')
    print(g)
clear_costmaps = None
try:
    rospy.wait_for_service("/move_base/clear_costmaps", timeout=2.0)
    clear_costmaps = rospy.ServiceProxy("/move_base/clear_costmaps", Empty)
except rospy.ROSException:
    rospy.logwarn("Costmap clearing service not available.")


def walk_to(name, target_locations):
    name = name.lower()
    if "none" in name or "unknown" in name:
        rospy.logwarn(f"Invalid location: {name}")
        return

    if name in target_locations:
        speak(f"going to {name}")
        coords = target_locations[name]
        chassis.move_to(coords[0], coords[1], coords[2])
        while not rospy.is_shutdown():
            code = chassis.status_code
            if code == 3:
                speak("arrived")
                break
            elif code == 4:
                speak("failed to reach destination")
                break
            time.sleep(0.2)

        time.sleep(1)
        if clear_costmaps:
            try:
                clear_costmaps
            except rospy.ServiceException as e:
                rospy.logerr(f"Service call failed: {e}")
    else:
        rospy.logerr(f"Location '{name}' not found in database.")
other_mission = {
    "tv table 1": [1.282, 1.508, 1.569],
    "tv table 2": [1.881, 1.260, 1.703],
    "dish washer": [7.417, -1.169, -1.512],
    "cabinet 1": [6.164, 1.755, -3.003],
    "cabinet 2": [1.065, 3.790, -1.481],
    "taking clothes": [3.006, 3.916, 1.652],
    "ri": [6.11, -0.46, 0]
}

json:
{
    "laundry": {
        "laundry": [1.990, 3.655, 2.367],
        "laundry table": [1.459, 4.204, 1.574],
        "washing machine": [2.790, 4.580, 1.495],
        "shelf": [1.179, 3.042, -1.554],
        "laundry trash bin": [2.298, 2.696, -1.799]
    },
    "bedroom":{
        "bedroom": [6.476, 4.525, -1.919],
        "bed": [5.330, 4.019, -1.376],
        "host": [6.688, 4.250, -3.062]
    },
    "living room": {
        "living room": [2.650, -0.111, 2.105],
        "tv stand": [1.724, 0.755, 1.568],
        "sofa": [2.842 ,-0.764, -1.586],
        "coffee table": [4.416, -1.109, -1.557]
    },
    "kitchen": {
        "kitchen": [7.156, 1.411, 0.117],
        "cabinet": [6.411, 1.527, -3.121],
        "refrigerator": [7.324, 1.937, 0.242],
        "counter": [5.644, -1.478, -1.653],
        "sink": [6.203, -1.444, -1.626],
        "cooking table": [7.023, -1.453, -2.123],
        "dishwasher": [8.306, -1.300, -2.099],
        "kitchen trash bin": [8.266, 1.616, 0.117],
        "dinner table": [7.891, -0.548, 2.305]
    }
}

[ INFO] [1782877406.777902458]: Setting pose (1782877406.777841): 1.415 1.083 1.591
[ INFO] [1782877418.739098443]: Setting pose (1782877418.739040): 1.949 1.041 1.510
[ INFO] [1782877431.560141521]: Setting pose (1782877431.560086): 4.492 -0.794 -1.331
[ INFO] [1782877506.517678519]: Setting pose (1782877506.517594): 5.827 -0.947 -1.572
[ INFO] [1782877518.276856845]: Setting pose (1782877518.276815): 6.583 -1.118 -1.649
[ INFO] [1782877590.899479250]: Setting pose (1782877590.899434): 7.501 -1.065 -1.613
cab1
[ INFO] [1782877664.876721119]: Setting pose (1782877664.876680): 1.438 3.457 -1.561
[ INFO] [1782877703.439041361]: Setting pose (1782877703.438993): 2.945 4.846 1.687
[ INFO] [1782877709.359333404]: Setting pose (1782877709.359293): 1.695 4.408 1.624
[ INFO] [1782877717.236324837]: Setting pose (1782877717.236266): 1.035 4.330 1.677
find
[ INFO] [1782877740.878785819]: Setting pose (1782877740.878739): 2.400 3.761 -1.482
cab2
[ INFO] [1782877801.598707558]: Setting pose (1782877801.598653): 6.370 1.411 -3.092


locations = {}
json_path = os.path.join(os.path.dirname(__file__), "locations.json")
try:
    if os.path.exists(json_path):
        with open(json_path, "r") as f:
            raw_data = json.load(f)
        for room, items in raw_data.items():
            for item_name, coordinates in items.items():
                locations[item_name.lower()] = coordinates
    else:
        rospy.logwarn("locations.json not found.")
except Exception as e:
    rospy.logerr(f"Failed to load locations.json: {e}")

if __name__ == "__main__":
    rospy.init_node("demo_node")
    chassis = RobotChassis()
    '''
    if locations:
        speak("Starting GPSR locations")
        for x in locations.keys():
            walk_to(x, locations)
    '''
    speak("Starting other locations")
    for x in other_mission.keys():
        walk_to(x, other_mission)

    speak("all positions completed")
