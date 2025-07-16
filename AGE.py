#!/usr/bin/env python3
from RobotChassis import RobotChassis
from std_srvs.srv import Empty
import rospy
import os
import time


def speak(g):
    os.system(f'espeak "{g}"')
    # rospy.loginfo(g)
    print(g)


def check_item(name):
    corrected = "starting point"
    cnt = 0
    if name in locations:
        corrected = name
    else:
        corrected = corrected.replace("_", " ")
    return corrected


clear_costmaps = rospy.ServiceProxy("/move_base/clear_costmaps", Empty)


def walk_to(name):
    if "none" not in name or "unknow" in name:
        name = name.lower()
        real_name = check_item(name)
        speak("going to " + str(name))
        if real_name in locations:
            num1, num2, num3 = locations[real_name]
            chassis.move_to(num1, num2, num3)
            while not rospy.is_shutdown():
                # 4. Get the chassis status.
                code = chassis.status_code
                text = chassis.status_text
                if code == 3:
                    break
                if code == 4:
                    break
            speak("arrived")
            time.sleep(1)
            clear_costmaps


#sink, waste basket
locations = {
    "bedside table": [4.64, 5.73,0.44],
    "side table": [4.294,6.624,-3.102],
    "bed": [3.810, 5.442, 0.723],
    "kitchen table": [1.765, 5.802, 1.774],
    "dishwasher": [1.481, 6.239, -1.53],
    "microwave": [0.383, 5.861, -2.218],
    "shelf": [1.642, 5.529, 3.14],
    "exit": [0.097, 7.848, -3.0],
    "sink": [0.396, 6.064, -1.611],
    "waste basket": [0.47, 5.11, -3.0],
    "refrigerator": [0.47, 6.95, -3.0],
    "trash bin": [3.980, 0.003, -0.154],
    "desk": [5, 2.647, -1.237],
    "bar": [5.496, 2.394, 1.7],
    "tv stand": [0.670, 1.759, 1.610],
    "cabinet ": [1.35, 2.174, 3.14],
    "sofa": [1.659, 1.330, -1.542],
    "seats": [0.969, 0.905, 0],
    "entry": [0.049, 0.188, -3.14],
    "instruction point": [3.765, 3.795, -1.53],
    "bedroom": [4.1, 6.51, 2.083],
    "kitchen": [0.689, 6.181, -0.775],
    "living room": [1.069,2.017,-0.015],
    "office": [4.357, 2.533, 0.663],
    "guest task3": [0.456, 0.1, -3.1],
    "drinktable task3": [5.454, 2.794, 1.55],
    "seats task3": [0.401, 2.622, -0.812],
    "get":[0.525,2.7,1.55],
    "place":[0.144,2.21,3.14]
}
# front 0 back 3.14 left 90 1.5 right 90 -1.5
cout_location = {
    "bedroom": [3.020, 3.555, 1.109],
    "kitchen": [2.732, 3.355, 2.145],
    "office": [2.216, 3.961, -0.646],
    "living room": [3.644, 3.931, -2.409]
}
# front 0 back 3.14 left 90 1.5 right 90 -1.5


# name
# qestion list
# answer
def walk_to1(name):
    if "none" not in name or "unknow" in name:
        name = name.lower()
        real_name = check_item(name)
        if real_name in cout_location:
            speak("going to " + str(name))
            num1, num2, num3 = cout_location[real_name]
            chassis.move_to(num1, num2, num3)
            while not rospy.is_shutdown():
                # 4. Get the chassis status.
                code = chassis.status_code
                text = chassis.status_text
                if code == 3:
                    break
                if code == 4:
                    break
            speak("arrived")
            time.sleep(1)
            clear_costmaps


if __name__ == "__main__":
    rospy.init_node("demo")
    rospy.loginfo("demo node start!")
    # open things
    chassis = RobotChassis()
    
    for x in locations.keys():
        walk_to(x)
        time.sleep(2)
    for x in cout_location.keys():
        speak("count position")
        walk_to1(x)
        time.sleep(2)
        
    clear_costmaps
