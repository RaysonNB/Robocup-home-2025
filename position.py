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


def check_item(name):
    corrected = "starting point"
    if name in locations:
        corrected = name
    else:
        corrected = corrected.replace("_", " ")
    return corrected


# Initialize ROS Service Proxy
try:
    rospy.wait_for_service("/move_base/clear_costmaps", timeout=2.0)
    clear_costmaps = rospy.ServiceProxy("/move_base/clear_costmaps", Empty)
except rospy.ROSException:
    rospy.logwarn("Costmap clearing service not available.")
    clear_costmaps = None


def walk_to(name):
    if "none" not in name or "unknow" in name:
        name = name.lower()
        real_name = check_item(name)
        speak("going to " + str(name))

        if real_name in locations:
            num1, num2, num3 = locations[real_name]
            chassis.move_to(num1, num2, num3)

            while not rospy.is_shutdown():
                # Get the chassis status
                code = chassis.status_code
                if code in [3, 4]:  # 3: Success, 4: Aborted/Failed
                    break

            speak("arrived")
            time.sleep(1)

            # Explicitly CALL the service using ()
            if clear_costmaps:
                clear_costmaps()


# --- Load and Process locations.json ---
locations = {}
# Assuming locations.json is in the same directory as your script
json_path = os.path.join(os.path.dirname(__file__), "locations.json")

try:
    with open(json_path, "r") as f:
        raw_data = json.load(f)

    # Flatten the JSON structure so "sofa", "washing machine", etc. are directly accessible
    for room, items in raw_data.items():
        for item_name, coordinates in items.items():
            locations[item_name.lower()] = coordinates
except Exception as e:
    rospy.logerr(f"Failed to load locations.json: {e}")

# ----------------------------------------
other_mission = {
    "tv table 1": [1.282, 1.508, 1.569],
    "tv table 2": [1.881,1.260,1.703],
    "dish washer": [7.417, -1.169, -1.512],
    "cabinet 1": [6.164,1.755,-3.003],
    "cabinet 2": [1.065,3.790,-1.481],
    "taking clothes": [3.006,3.916,1.652],
    "ri": [6.11,-0.46,0]
}
if __name__ == "__main__":
    rospy.init_node("demo")
    rospy.loginfo("demo node start!")

    chassis = RobotChassis()
    speak("GPSR locations")
    # Iterates through all items flattened out of your JSON dictionary
    for x in locations.keys():
        walk_to(x)
    speak("other locations")
    for x in other_mission.keys():
        walk_to(x)
    speak("all position end")
