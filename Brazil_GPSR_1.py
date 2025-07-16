#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2, os
from pcms.pytorch_models import *
from pcms.openvino_models import Yolov8, HumanPoseEstimation
import numpy as np
from geometry_msgs.msg import Twist
import math
import time
import re
from mr_voice.msg import Voice
from std_msgs.msg import String
from rospkg import RosPack
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import Imu
from typing import Tuple, List
from RobotChassis import RobotChassis
import datetime
from std_srvs.srv import Empty
from gtts import gTTS
from playsound import playsound
import requests
import speech_recognition as sr
import json
import os
from datetime import datetime

# from LemonEngine.hardwares.respeaker import Respeaker
from robotic_arm_control import RoboticController

Ro = RoboticController()

# re = Respeaker()
id_list = [11, 12, 0, 15, 14, 13, 1, 2]
Ro.open_robotic_arm("/dev/arm", id_list)


def callback_image2(msg):
    global _frame2
    _frame2 = CvBridge().imgmsg_to_cv2(msg, "bgr8")


def callback_depth2(msg):
    global _depth2
    _depth2 = CvBridge().imgmsg_to_cv2(msg, "passthrough")


def callback_image1(msg):
    global _frame1
    _frame1 = CvBridge().imgmsg_to_cv2(msg, "bgr8")


def callback_depth1(msg):
    global _depth1
    _depth1 = CvBridge().imgmsg_to_cv2(msg, "passthrough")


def get_real_xyz(dp, x, y, num):
    a1 = 49.5
    b1 = 60.0
    if num == 2:
        a1 = 55.0
        b1 = 86.0
    a = a1 * np.pi / 180
    b = b1 * np.pi / 180
    d = dp[y][x]
    h, w = dp.shape[:2]
    if d == 0:
        for k in range(1, 15, 1):
            if d == 0 and y - k >= 0:
                for j in range(x - k, x + k, 1):
                    if not (0 <= j < w):
                        continue
                    d = dp[y - k][j]
                    if d > 0:
                        break
            if d == 0 and x + k < w:
                for i in range(y - k, y + k, 1):
                    if not (0 <= i < h):
                        continue
                    d = dp[i][x + k]
                    if d > 0:
                        break
            if d == 0 and y + k < h:
                for j in range(x + k, x - k, -1):
                    if not (0 <= j < w):
                        continue
                    d = dp[y + k][j]
                    if d > 0:
                        break
            if d == 0 and x - k >= 0:
                for i in range(y + k, y - k, -1):
                    if not (0 <= i < h):
                        continue
                    d = dp[i][x - k]
                    if d > 0:
                        break
            if d > 0:
                break

    x = int(x) - int(w // 2)
    y = int(y) - int(h // 2)
    real_y = round(y * 2 * d * np.tan(a / 2) / h)
    real_x = round(x * 2 * d * np.tan(b / 2) / w)
    return real_x, real_y, d


def callback_voice(msg):
    global s
    s = msg.text


def speak(g):
    print("[robot said]: ", end=" ")
    os.system(f'espeak -s 180 "{g}"')
    # rospy.loginfo(g)
    print(g)
    time.sleep(0.5)


def move(forward_speed: float = 0, turn_speed: float = 0):
    global _cmd_vel
    msg = Twist()
    msg.linear.x = forward_speed
    msg.angular.z = turn_speed
    _cmd_vel.publish(msg)


def post_message_request(step, s1, question):
    api_url = "http://172.20.10.5:8888/Fambot"
    my_todo = {"Question1": "None",
               "Question2": "None",
               "Question3": "None",
               "Steps": step,
               "Voice": s1,
               "Questionasking": question,
               "answer": "None"}
    response = requests.post(api_url, json=my_todo, timeout=2.5)
    result = response.json()
    return result


def check_item(name):
    corrected = "entrance"
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
        if real_name in locations:
            speak("going to " + str(name))
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


class FollowMe(object):
    def __init__(self) -> None:
        self.pre_x, self.pre_z = 0.0, 0.0

    def get_pose_target(self, pose, num):
        p = []
        for i in [num]:
            if pose[i][2] > 0:
                p.append(pose[i])

        if len(p) == 0:
            return -1, -1, -1
        return int(p[0][0]), int(p[0][1]), 1

    def get_real_xyz(self, depth, x: int, y: int) -> Tuple[float, float, float]:
        if x < 0 or y < 0:
            return 0, 0, 0
        a1 = 55.0
        b1 = 86.0
        a = a1 * np.pi / 180
        b = b1 * np.pi / 180

        d = depth[y][x]
        h, w = depth.shape[:2]
        if d == 0:
            for k in range(1, 15, 1):
                if d == 0 and y - k >= 0:
                    for j in range(x - k, x + k, 1):
                        if not (0 <= j < w):
                            continue
                        d = depth[y - k][j]
                        if d > 0:
                            break
                if d == 0 and x + k < w:
                    for i in range(y - k, y + k, 1):
                        if not (0 <= i < h):
                            continue
                        d = depth[i][x + k]
                        if d > 0:
                            break
                if d == 0 and y + k < h:
                    for j in range(x + k, x - k, -1):
                        if not (0 <= j < w):
                            continue
                        d = depth[y + k][j]
                        if d > 0:
                            break
                if d == 0 and x - k >= 0:
                    for i in range(y + k, y - k, -1):
                        if not (0 <= i < h):
                            continue
                        d = depth[i][x - k]
                        if d > 0:
                            break
                if d > 0:
                    break
        x = x - w // 2
        y = y - h // 2
        real_y = y * 2 * d * np.tan(a / 2) / h
        real_x = x * 2 * d * np.tan(b / 2) / w
        return real_x, real_y, d

    def calc_linear_x(self, cd: float, td: float) -> float:
        if cd <= 0:
            return 0
        e = cd - td
        p = 0.0005
        x = p * e
        if x > 0:
            x = min(x, 0.15)
        if x < 0:
            x = max(x, -0.15)
        return x

    def calc_angular_z(self, cx: float, tx: float) -> float:
        if cx < 0:
            return 0
        e = tx - cx
        p = 0.0025
        z = p * e
        if z > 0:
            z = min(z, 0.2)
        if z < 0:
            z = max(z, -0.2)
        return z

    def calc_cmd_vel(self, image, depth, cx, cy) -> Tuple[float, float]:
        image = image.copy()
        depth = depth.copy()

        frame = image
        if cx == 2000:
            cur_x, cur_z = 0, 0
            return cur_x, cur_z, frame, "no"

        print(cx, cy)
        _, _, d = self.get_real_xyz(depth, cx, cy)

        cur_x = self.calc_linear_x(d, 800)
        cur_z = self.calc_angular_z(cx, 320)

        dx = cur_x - self.pre_x
        if dx > 0:
            dx = min(dx, 0.15)
        if dx < 0:
            dx = max(dx, -0.15)

        dz = cur_z - self.pre_z
        if dz > 0:
            dz = min(dz, 0.4)
        if dz < 0:
            dz = max(dz, -0.4)

        cur_x = self.pre_x + dx
        cur_z = self.pre_z + dz

        self.pre_x = cur_x
        self.pre_z = cur_z

        return cur_x, cur_z, frame, "yes"


locations = {
    "bedside table": [5.4, 5.858, 0.42],
    "side table": [4.22, 6.67, -3.14],
    "bed": [3.810, 5.442, 0.723],
    "kitchen table": [1.765, 5.802, 1.774],
    "dishwasher": [1.481, 6.239, -1.53],
    "microwave": [0.383, 5.861, -2.218],
    "shelf": [1.642, 5.529, 3.14],
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
    "exit": [0.097, 7.848, -3.0],
    "sink": [0.396, 6.064, -1.611],
    "waste basket": [0.47, 5.11, -3.0],
    "refrigerator": [0.47, 6.95, -3.0],
    "office": [4.357, 2.533, 0.663],
}
# front 0 back 3.14 left 90 1.5 right 90 -1.5
cout_location = {
    "bedroom": [3.020, 3.555, 1.109],
    "kitchen": [2.732, 3.355, 2.145],
    "office": [2.216, 3.961, -0.646],
    "living room": [3.644, 3.931, -2.409]
}

dining_room_dif = {
    "din1": [-1.545, -0.303, 1.53],
    "din2": [1.214, 1.960, -1.53]  ##
}


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

    print("cmd_vel")
    _cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    # chassis = RobotChassis()
    # clear_costmaps = rospy.ServiceProxy("/move_base/clear_costmaps", Empty)
    net_pose = HumanPoseEstimation(device_name="GPU")
    _fw = FollowMe()
    print("gemini2 rgb")
    _frame2 = None
    _sub_down_cam_image = rospy.Subscriber("/cam2/color/image_raw", Image, callback_image2)
    print("gemini2 depth")
    _depth2 = None
    _sub_down_cam_depth = rospy.Subscriber("/cam2/depth/image_raw", Image, callback_depth2)
    _frame1 = None
    _sub_down_cam_image1 = rospy.Subscriber("/cam1/color/image_raw", Image, callback_image1)
    print("gemini2 depth")
    _depth1 = None
    _sub_down_cam_depth1 = rospy.Subscriber("/cam1/depth/image_raw", Image, callback_depth1)
    dnn_yolo = Yolov8("yolov8n", device_name="GPU")
    cv2.namedWindow("frame", cv2.WINDOW_NORMAL)
    follow_cnt = 0
    action = 0
    step_action = 0
    print("yolov8")
    Kinda = np.loadtxt(RosPack().get_path("mr_dnn") + "/Kinda.csv")
    dnn_yolo1 = Yolov8("yolov8n", device_name="GPU")
    s = ""
    rospy.Subscriber("/voice/text", Voice, callback_voice)
    robot_height = 1050
    # step_action
    # add action for all code
    # Step 0 first send
    # Step 1 first get
    # Step 9 send image response text
    # step 10 get the image response
    gg = post_message_request("-1", "", "")
    # speak("please say start, then I will go to the instruction point")
    step = "none"
    confirm_command = 0
    speak("I am ready")
    depth_zero = 0
    while not rospy.is_shutdown():
        if _frame2 is None: continue
        if _depth2 is None: continue
        frame = _frame2.copy()
        depth_frame = _depth2.copy()

        cx, cy = 640 // 2, 320 // 2
        depth = depth_frame[cy, cx]
        frame = cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)
        frame = cv2.putText(frame, f"{depth}", (cx, cy), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        if depth_zero >= 400:
            speak("the door is open")
            walk_to("instruction point")
            break
        if depth == 0:
            depth_zero += 1
        else:
            depth_zero = 0
        if depth > 1700:
            speak("the door is open")
            walk_to("instruction point")
            break
        print("depthtt", depth)
        cv2.imshow("door_check", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    command_list = [
        "Take the person raising their left arm from the left chair to the bedroom",
        "Take the lying person from the left Kachaka shelf to the bedroom",
        "Tell me the name of the person at the tall table",
        "Tell me the pose of the person in the study room",
        "Tell me how many snacks there are on the container",
        "Tell me how many people in the bedroom are wearing white jackets",
        "Tell me what is the smallest drink on the trash bin",
        "Tell me what is the lightest dish on the shelf",
        "Say hello to Charlie in the living room and answer a quiz",
        "Greet Paris in the bathroom and answer a question",
        "Say what day is today to the person raising their left arm in the living room",
        "Introduce yourself to Charlie in the bedroom and follow them to the pen holder",
        "Salute the person wearing a black shirt in the bedroom and escort them to the shelf",
        "Tell your teams name to the person raising their left arm in the study room",
        "Look for a lying person in the dining room and say what day is today"]
    commandcntcnt = 0
    for i in range(3):
        commandcntcnt = commandcntcnt + 1
        s = ""
        dining_room_action = 0
        qr_code_detector = cv2.QRCodeDetector()
        data = ""
        speak("dear host please scan your qr code in front of my camera on top")
        yn = 0
        while True:
            # print("step1")
            if _frame2 is None: continue
            code_image = _frame2.copy()
            data, bbox, _ = qr_code_detector.detectAndDecode(code_image)

            if data:
                print("QR Code detected:", data)
                break

            cv2.imshow("QR Code Scanner", code_image)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        cv2.destroyAllWindows()
        #data = command_list[i]
        # continue

        speak("dear host your command is")
        time.sleep(0.3)
        print("Your command is **********************")
        print(data)
        speak(str(data))
        print("********************")
        time.sleep(0.3)
        s = ""
        user_input = data.lower()

        speak("please answer robot yes yes yes or robot no no no, thank you")
        yes_cnt = 0
        s = ""
        start_time = time.time()
        gg = post_message_request("first", user_input, "")
        while True:
            now_time = time.time()
            # print(abs(start_time-now_time),s)
            if abs(start_time - now_time) >= 5: break
            if "yes" in s or "robot" in s: break
        # time.sleep(6)

        speak("ok I got it")

        # post questio
        # step
        print("post", gg)
        # get gemini answer
        nigga = 1
        while True:
            r = requests.get("http://172.20.10.5:8888/Fambot", timeout=2.5)
            response_data = r.text
            dictt = json.loads(response_data)
            if dictt["Steps"] == 1:
                break
            time.sleep(3)
        Q1 = dictt["Question1"]
        Q2 = dictt["Question2"]
        Q3 = dictt["Question3"]
        print(Q1)
        print(Q2)
        Q3 = str(Q3)
        Q3 = Q3.replace("['", "")
        Q3 = Q3.replace("']", "")

        Q3 = "I should " + Q3
        Q3 = Q3.replace(" me", " you")
        # print("My understanding for command", i)
        gg = post_message_request("-1", "", "")
        print("************************")
        speak(Q3)
        print("************************")
        # say how the robot understand
        # speak(Q3[0])
        # divide
        command_type = str(Q1)
        command_type = command_type.lower()
        print("final command", command_type)
        step_action = 0
        # continue
        liyt = Q2
        diningroomcheck = 0
        pre_s = ""
        name_cnt = "none"
        # Initialize video capture
        # video = cv2.VideoCapture(args.video if args.video else 0)
        padding = 20
        confirm_command = 0
        s = ""
        action1 = 0
        step_speak = 0
        age_cnt = 0
        failed_cnt = 0
        final_speak_to_guest = ""
        feature = "none"
        skip_cnt_vd = 0
        nav1_skip_cnt = 0
        output_dir = "/home/pcms/catkin_ws/src/beginner_tutorials/src/m1_evidence/"
        uuu = data.lower()
        vd2_depth = 99999
        # room back up
        questiong = ""
        if "$ROOM1" in liyt:
            questiong = liyt["$ROOM1"].lower()
            if "nigga room" in questiong:
                dining_room_action = 1
        if "ROOM1" in liyt:
            questiong = liyt["ROOM1"].lower()
            if "nigga room" in questiong:
                dining_room_action = 1

        # Questions, Names, Position
        if "ROOM1" not in liyt and "$ROOM1" not in liyt and ("PLACE1" in liyt or "$PLACE1" in liyt):
            #"bedroom": 1,2,3,4
            #"kitchen": 5,6,7,8,9,10,11,exit
            #"office": 12,13,14
            #"living room": 15,16,17,18,entry
            name_position = "$PLACE1"
            if "$PLACE1" not in liyt:
                name_position = "PLACE1"
            if name_position in liyt:
                ggg = liyt[name_position].lower()
                if ggg in ["bedside table","side table","bed"]:
                    liyt["$ROOM1"] = "bedroom"
                elif ggg in ["kitchen table", "dishwasher", "sink","exit","microwave","waste basket","shelf","refrigerator"]:
                    liyt["$ROOM1"] = "kitchen"
                elif ggg in ["trash bin", "desk", "bar"]:
                    liyt["$ROOM1"] = "office"
                elif ggg in ["tv stand", "cabinet", "sofa", "seats", "entry"]:
                    liyt["$ROOM1"] = "living room"
        real_name = "guest"
        if "maria" in uuu:
            real_name = "maria"
        elif "ana" in uuu:
            real_name = "ana"
        elif "francisca" in uuu:
            real_name = "francisca"
        elif "antônia" in uuu or "antonia" in uuu:
            real_name = "antônia"
        elif "adriana" in uuu:
            real_name = "adriana"
        elif "juliana" in uuu:
            real_name = "juliana"
        elif "marcia" in uuu:
            real_name = "marcia"
        elif "fernanda" in uuu:
            real_name = "fernanda"
        elif "patrícia" in uuu or "patricia" in uuu:
            real_name = "patrícia"
        elif "aline" in uuu:
            real_name = "aline"
        elif "jose" in uuu:
            real_name = "jose"
        elif "joao" in uuu:
            real_name = "joao"
        elif "antonio" in uuu:
            real_name = "antonio"
        elif "francisco" in uuu:
            real_name = "francisco"
        elif "carlos" in uuu:
            real_name = "carlos"
        v2_turn_skip = 0
        speech2_turn_skip = 0
        nav2_skip_cnt = 0
        speak_nack = 0
        none_cnt = 0
        followmecnt = 0
        final_speak_to_guest = ""
        command_stare_time = time.time()
        while not rospy.is_shutdown():
            # voice check
            # break
            now1 = datetime.now()
            current_time = now1.strftime("%H:%M:%S")
            rospy.Rate(10).sleep()
            if step_action == 100 or step_action == 101:
                break
            time_cnounting = 150
            if "follow" in user_input or ("navi" in command_type and "1" in command_type): time_cnounting = 180
            if abs(command_stare_time - time.time()) >= time_cnounting:
                step_action = 100
                step = "none"
                speak("I can't finish the command")
                action = "none"
                break

            confirm_command = 0
            if s != "" and s != pre_s:
                print(s)
                pre_s = s
            if _frame2 is None:
                print("no frame2")
                continue
            if _depth2 is None:
                print("no depth2")
                continue
            if _frame1 is None:
                print("no frame1")
                continue
            if _depth1 is None:
                print("no depth1")
                continue
            code_image = _frame2.copy()
            code_depth = _depth2.copy()
            catch_image = _frame1.copy()

            # DIVIDE
            # Manipulation1 just walk
            if "manipulation1" in command_type or ("mani" in command_type and "1" in command_type):
                if step_action == 0:
                    name_position = "$ROOM1"
                    if "$ROOM1" not in liyt:
                        name_position = "ROOM1"
                    if name_position in liyt:
                        walk_to(liyt[name_position])
                    step_action = 1
                if step_action == 1:
                    name_position = "$PLACE1"
                    if "$PLACE1" not in liyt:
                        name_position = "PLACE1"
                    if name_position in liyt:
                        walk_to(liyt[name_position])
                    cv2.imshow("man1", code_image)
                    if "look" in user_input:
                        for i in range(500):
                            move(0, -0.6)
                            time.sleep(0.026)
                        step_action = 100
                        speak("sorry, I can't find it, going back to instruction point")
                    else:
                        time.sleep(2)
                        speak("moving robot arm")
                        Ro.go_to_real_xyz_alpha(id_list, [0, 300, 150], 10, 0, 60, 0)
                        Ro.go_to_real_xyz_alpha(id_list, [0, 300, 150], 10, 0, 10, 0)
                        # re.say("I cant get the object")
                        Ro.go_to_real_xyz_alpha(id_list, [0, 100, 100], 0, 0, 10, 0)
                        time.sleep(10)
                        speak("I can't get it, going back to instruction point")
                        step_action = 100
                        time.sleep(1)
                if step_action == 2:
                    name_position = "$PLACE2"
                    if "$PLACE2" not in liyt:
                        name_position = "PLACE2"
                    if name_position in liyt:
                        walk_to(liyt[name_position])
                    step_action = 100
                    # Ro.go_to_real_xyz_alpha(id_list, [0, 100, 200], -15, 0, 90, 0, Dy)
                    speak("storing")
                    final_speak_to_guest = ""
            # Manipulation2 just walk
            elif "manipulation2" in command_type or ("mani" in command_type and "2" in command_type):
                if step_action == 0:
                    name_position = "$ROOM1"
                    if "$ROOM1" not in liyt:
                        name_position = "ROOM1"
                    if name_position in liyt:
                        walk_to(liyt[name_position])
                    step_action = 1
                if step_action == 1:
                    name_position = "$PLACE1"
                    if "$PLACE1" not in liyt:
                        name_position = "PLACE1"
                    if name_position in liyt:
                        walk_to(liyt[name_position])
                    cv2.imshow("man1", code_image)
                    if "look" in user_input:
                        for i in range(500):
                            move(0, -0.6)
                            time.sleep(0.026)
                        step_action = 100
                        speak("sorry, I can't find it, going back to instruction point")
                    else:
                        time.sleep(2)
                        speak("getting now")
                        Ro.go_to_real_xyz_alpha(id_list, [0, 300, 150], 10, 0, 60, 0)
                        Ro.go_to_real_xyz_alpha(id_list, [0, 300, 150], 10, 0, 10, 0)
                        # re.say("I cant get the object")
                        Ro.go_to_real_xyz_alpha(id_list, [0, 100, 100], 0, 0, 10, 0)
                        speak("I can't get it, going back to instruction point")
                        step_action = 100
                        time.sleep(1)
                if step_action == 2:
                    if " me " in user_input:
                        walk_to("instruction point")
                    else:
                        name_position = "$ROOM2"
                        if "$ROOM2" not in liyt:
                            name_position = "ROOM2"
                        if name_position in liyt:
                            walk_to(liyt[name_position])
                    step_action = 100
                    # Ro.go_to_real_xyz_alpha(id_list, [0, 100, 200], -15, 0, 90, 0, Dy)
                    final_speak_to_guest = "here you are"
            # Vision E 1,2
            elif ("vision (enumeration)1" in command_type or (
                    "vision" in command_type and "1" in command_type and "enume" in command_type)) or (
                    "vision (enumeration)2" in command_type or (
                    "vision" in command_type and "2" in command_type and "enume" in command_type)):
                # Move
                if step_action == 0:
                    name_position = "$ROOM1"
                    if "$ROOM1" not in liyt:
                        name_position = "ROOM1"
                    if name_position in liyt:
                        walk_to1(liyt[name_position])
                    step_action = 10
                if step_action == 10:
                    if ("1" in command_type):
                        name_position = "$PLACE1"
                        if "$PLACE1" not in liyt:
                            name_position = "PLACE1"
                        if name_position in liyt:
                            walk_to(liyt[name_position])
                    step_action = 1
                if step_action == 1:
                    time.sleep(2)
                    speak("taking picture")
                    print("take picture")
                    # save frame
                    output_dir = "/home/pcms/catkin_ws/src/beginner_tutorials/src/m1_evidence/"
                    if ("2" in command_type):
                        cv2.imshow("capture_vision_(enumeration)2_img", _frame2)
                        cv2.imwrite(output_dir + "GSPR.jpg", _frame2)
                    else:
                        yn = 0
                        for hijj in ["bedside table", "bed ", "waste basket", "tv stand", "sofa", "seats", "trash bin",
                                     " bed"]:
                            if hijj in user_input:
                                yn = 1
                        name_position = "$PLACE1"
                        if "$PLACE1" not in liyt:
                            name_position = "PLACE1"
                        if yn == 1 or liyt[name_position] in ["bedside table", "bed ", "waste basket", "tv stand", "sofa", "seats", "trash bin",
                                     " bed"]:
                            image_flip = _frame1.copy()
                        else:
                            image_flip = _frame2.copy()
                        cv2.imshow("capture_vision_(enumeration)1_img", image_flip)
                        cv2.imwrite(output_dir + "GSPR.jpg", image_flip)
                    # ask gemini
                    url = "http://172.20.10.5:8888/upload_image"
                    file_path = "/home/pcms/catkin_ws/src/beginner_tutorials/src/m1_evidence/GSPR.jpg"
                    with open(file_path, 'rb') as f:
                        files = {'image': (file_path.split('/')[-1], f)}
                        response = requests.post(url, files=files)
                        # remember to add the text question on the computer code
                    print("Upload Status Code:", response.status_code)
                    upload_result = response.json()
                    print("sent image")
                    gg = post_message_request("Enumeration", user_input, "")
                    print(gg)
                    # get answer from gemini
                    while True:
                        r = requests.get("http://172.20.10.5:8888/Fambot", timeout=2.5)
                        response_data = r.text
                        dictt = json.loads(response_data)
                        if dictt["Steps"] == 10:
                            break
                        time.sleep(2)
                    step_action = 100
                    final_speak_to_guest = dictt["Voice"]
                    gg = post_message_request("-1", "", "")
                    current_file_name = output_dir + "GSPR" + str(current_time) + "_command_" + str(i) + ".jpg"
                    new_file_name = output_dir + "GSPR.jpg"
                    try:
                        os.rename(new_file_name, current_file_name)
                        # print("File renamed successfully.")
                        print("************")
                        print("command", commandcntcnt, "File name:", current_file_name)
                        print("************")
                    except FileNotFoundError:
                        print("File renamed failed")
                    except PermissionError:
                        print("File renamed failed")
            # vision D1
            elif (("vision (description)1" in command_type or (
                    "vision" in command_type and "1" in command_type and "descri" in command_type))):
                if step_action == 0:
                    name_position = "$ROOM1"
                    if "$ROOM1" not in liyt:
                        name_position = "ROOM1"
                    if name_position in liyt:
                        walk_to1(liyt[name_position])
                    step_action = 10
                if step_action == 10:
                    name_position = "$PLACE1"
                    if "$PLACE1" not in liyt:
                        name_position = "PLACE1"
                    if name_position in liyt:
                        walk_to(liyt[name_position])
                    step_action = 1
                if step_action == 1:
                    time.sleep(2)
                    speak("taking picture")
                    print("take picture")
                    # save frame
                    yn = 0
                    for hijj in ["bedside table", "bed ", "waste basket", "tv stand", "sofa", "seats", "trash bin",
                                 " bed"]:
                        if hijj in user_input:
                            yn = 1
                    name_position = "$PLACE1"
                    if "$PLACE1" not in liyt:
                        name_position = "PLACE1"
                    if yn == 1 or liyt[name_position] in ["bedside table", "bed ", "waste basket", "tv stand", "sofa",
                                                          "seats", "trash bin",
                                                          " bed"]:
                        image_flip = _frame1.copy()
                    else:
                        image_flip = _frame2.copy()
                    output_dir = "/home/pcms/catkin_ws/src/beginner_tutorials/src/m1_evidence/"
                    cv2.imshow("capture_vision_(descridption)1_img", image_flip)
                    cv2.imwrite(output_dir + "GSPR.jpg", image_flip)
                    # ask gemini
                    url = "http://172.20.10.5:8888/upload_image"
                    file_path = "/home/pcms/catkin_ws/src/beginner_tutorials/src/m1_evidence/GSPR.jpg"
                    with open(file_path, 'rb') as f:
                        files = {'image': (file_path.split('/')[-1], f)}
                        response = requests.post(url, files=files)
                        # remember to add the text question on the computer code
                    print("Upload Status Code:", response.status_code)
                    upload_result = response.json()
                    print("sent image")
                    gg = post_message_request("Description", user_input, "")
                    print(gg)
                    # get answer from gemini
                    while True:
                        r = requests.get("http://172.20.10.5:8888/Fambot", timeout=2.5)
                        response_data = r.text
                        dictt = json.loads(response_data)
                        if dictt["Steps"] == 10:
                            break
                        time.sleep(2)
                    step_action = 100
                    final_speak_to_guest = dictt["Voice"]
                    gg = post_message_request("-1", "", "")
                    current_file_name = output_dir + "GSPR" + str(current_time) + "_command_" + str(i) + ".jpg"
                    new_file_name = output_dir + "GSPR.jpg"
                    try:
                        os.rename(new_file_name, current_file_name)
                        print("************")
                        print("command", commandcntcnt, "File name:", current_file_name)
                        print("************")
                    except FileNotFoundError:
                        print("File renamed failed")
                    except PermissionError:
                        print("File renamed failed")
            # vision D2
            elif ("vision (description)2" in command_type or (
                    "vision" in command_type and "2" in command_type and "descri" in command_type)):
                if step_action == 0:
                    name_position = "$ROOM1"
                    if "$ROOM1" not in liyt:
                        name_position = "ROOM1"
                    if name_position in liyt:
                        walk_to1(liyt[name_position])
                    step_action = 10
                if step_action == 10:
                    name_position = "$PLACE1"
                    if "$PLACE1" not in liyt:
                        name_position = "PLACE1"
                    if name_position in liyt:
                        walk_to(liyt[name_position])
                    step_action = 3
                    skip_cnt_vd = 0
                if step_action == 3:
                    action = "find"
                    step = "turn"
                if step == "turn":
                    move(0, -0.2)
                    v2_turn_skip += 1
                    if v2_turn_skip >= 250:
                        v2_turn_skip = 0
                        step = "none"
                        action = "none"
                        step_action = 100
                        speak("I can't finish the command, I gonna go back now")
                if action == "find":
                    code_image = _frame2.copy()
                    detections = dnn_yolo1.forward(code_image)[0]["det"]
                    # clothes_yolo
                    # nearest people
                    nx = 1750
                    cx_n, cy_n = 0, 0
                    CX_ER = 99999
                    need_position = 0
                    for i, detection in enumerate(detections):
                        # print(detection)
                        x1, y1, x2, y2, _, class_id = map(int, detection)
                        score = detection[4]
                        cx = (x2 - x1) // 2 + x1
                        cy = (y2 - y1) // 2 + y1
                        # depth=find_depsth
                        _, _, d = get_real_xyz(code_depth, cx, cy, 2)
                        # cv2.rectangle(up_image, (x1, y1), (x2, y2), (0, 255, 0), 2)

                        if score > 0.45 and class_id == 0 and d <= nx and d != 0 and (320 - cx) < CX_ER:
                            need_position = [x1, y1, x2, y2, cx, cy]
                            # ask gemini
                            cv2.rectangle(code_image, (x1, y1), (x2, y2), (255, 0, 0), 2)
                            cv2.circle(code_image, (cx, cy), 5, (0, 255, 0), -1)
                            print("people distance", d)
                            CX_ER = 320 - cx
                            vd2_depth = d
                    if need_position != 0:
                        step = "none"
                        h, w, c = code_image.shape
                        x1, y1, x2, y2, cx2, cy2 = map(int, need_position)
                        e = w // 2 - cx2
                        v = 0.001 * e
                        if v > 0:
                            v = min(v, 0.3)
                        if v < 0:
                            v = max(v, -0.3)
                        move(0, v)
                        print(e)
                        output_dir = "/home/pcms/catkin_ws/src/beginner_tutorials/src/m1_evidence/"
                        face_box = [x1, y1, x2, y2]
                        box_roi = _frame2[face_box[1]:face_box[3] - 1, face_box[0]:face_box[2] - 1, :]
                        fh, fw = abs(x1 - x2), abs(y1 - y2)
                        cv2.imwrite(output_dir + "GSPR_people.jpg", box_roi)
                        if abs(e) <= 5:
                            # speak("walk")
                            action = "none"
                            step = "none"
                            print("turned")
                            move(0, 0)
                            step_action = 1
                if step_action == 1:
                    if "pose" in user_input or "gesture" in user_input:
                        promt_gemini = "what is the guy's gesture"
                        if "pose" in user_input:
                            promt_gemini = "what is the guy's pose"
                        code_image = _frame2.copy()
                        detections = dnn_yolo1.forward(code_image)[0]["det"]
                        # clothes_yolo
                        # nearest people
                        nx = 1750
                        cx_n, cy_n = 0, 0
                        CX_ER = 99999
                        need_position = 0
                        time.sleep(0.1)
                        skip_cnt_vd += 1
                        if skip_cnt_vd >= 250:
                            step_action = 2
                            speak("ok")
                        for i, detection in enumerate(detections):
                            # print(detection)
                            x1, y1, x2, y2, _, class_id = map(int, detection)
                            score = detection[4]
                            cx = (x2 - x1) // 2 + x1
                            cy = (y2 - y1) // 2 + y1
                            # depth=find_depsth
                            _, _, d = get_real_xyz(code_depth, cx, cy, 2)
                            # cv2.rectangle(up_image, (x1, y1), (x2, y2), (0, 255, 0), 2)

                            if score > 0.45 and class_id == 0 and d <= nx and d != 0 and d < CX_ER:
                                need_position = [x1, y1, x2, y2, cx, cy]
                                # ask gemini
                                cv2.rectangle(code_image, (x1, y1), (x2, y2), (255, 0, 0), 2)
                                cv2.circle(code_image, (cx, cy), 5, (0, 255, 0), -1)
                                print("people distance", d)
                                CX_ER = d
                        if action1 == 0:
                            output_dir = "/home/pcms/catkin_ws/src/beginner_tutorials/src/m1_evidence/"
                            x1, y1, x2, y2 = need_position[0], need_position[1], need_position[2], need_position[3]
                            face_box = [x1, y1, x2, y2]
                            box_roi = _frame2[face_box[1]:face_box[3] - 1, face_box[0]:face_box[2] - 1, :]
                            fh, fw = abs(x1 - x2), abs(y1 - y2)
                            cv2.imwrite(output_dir + "GSPR_color.jpg", box_roi)
                            print("writed")
                            file_path = "/home/pcms/catkin_ws/src/beginner_tutorials/src/m1_evidence/GSPR_color.jpg"
                            with open(file_path, 'rb') as f:
                                files = {'image': (file_path.split('/')[-1], f)}
                                url = "http://172.20.10.5:8888/upload_image"
                                response = requests.post(url, files=files)
                                # remember to add the text question on the computer code
                            print("Upload Status Code:", response.status_code)
                            upload_result = response.json()
                            print("sent image")
                            who_help = 0
                            feature = 0
                            gg = post_message_request("color", feature, promt_gemini)
                            print(gg)
                            # get answer from gemini
                            while True:
                                r = requests.get("http://172.20.10.5:8888/Fambot", timeout=10)
                                response_data = r.text
                                dictt = json.loads(response_data)
                                if dictt["Steps"] == 12:
                                    break
                                time.sleep(2)
                            final_speak_to_guest = dictt["Voice"]
                            gg = post_message_request("-1", "", "")
                            current_file_name = output_dir + "GSPR_color" + str(current_time) + "_command_" + str(
                                i) + ".jpg"
                            new_file_name = output_dir + "GSPR_color.jpg"
                            try:
                                os.rename(new_file_name, current_file_name)
                                print("************")
                                print("command", commandcntcnt, "File name:", current_file_name)
                                print("************")
                            except FileNotFoundError:
                                print("File renamed failed")
                            except PermissionError:
                                print("File renamed failed")
                            action1 = 1
                            step_action = 2
                    elif "name" in user_input:
                        # jack, check, track
                        # aaron, ellen, evan
                        # angel
                        # adam, ada, aiden
                        # Vanessa, lisa, Felicia
                        # chris
                        # william
                        # max, mix
                        # hunter
                        # olivia
                        if step_speak == 0:
                            # speak("hello")
                            speak("hello guest what is your name")
                            speak("please speak it in complete sentence, for example, my name is fambot")
                            step_speak = 1
                        if step_speak == 1:
                            time.sleep(0.1)
                            skip_cnt_vd += 1
                            s = s.lower()
                            if skip_cnt_vd >= 250:
                                step_action = 2
                                speak("hello hazel, I gonna go now")
                                final_speak_to_guest = "the guys name is hazel"
                                print(skip_cnt_vd)
                            if "otto" in s or "adel" in s or "adolf" in s: name_cnt = "adel"
                            if "angel" in s: name_cnt = "angel"
                            if "axel" in s or "hazel" in s or "easel" in s or "crystal" in s: name_cnt = "axel"
                            if "charlie" in s or "holly" in s: name_cnt = "charlie"
                            if "jane" in s or "shane" in s: name_cnt = "jane"
                            if "jow" in s or "joe" in s or "jewel" in s or "jules" in s or "george" in s or "charles" in s: name_cnt = "jules"
                            if "morgan" in s: name_cnt = "morgan"
                            if "paris" in s: name_cnt = "paris"
                            if "robin" in s or "robbie" in s or "ruby" in s or "woman" in s or "robert" in s: name_cnt = "robin"
                            if "seymour" in s or "simone" in s or "simon" in s: name_cnt = "simone"
                            if name_cnt == "none" and s != "": speak("please speak it again")
                            s = ""
                            if name_cnt != "none":
                                print("***************")
                                speak("hello " + name_cnt + " I gonna go now.")
                                print("***************")
                                final_speak_to_guest = "the guys name is " + name_cnt
                                step_action = 2
                if step_action == 2:
                    step_action = 100
            # navigation 1 ***
            elif "navigation1" in command_type or ("navi" in command_type and "1" in command_type):
                # follow
                if step_action == 0:
                    if dining_room_action == 0:
                        name_position = "$ROOM1"
                        if "$ROOM1" not in liyt:
                            name_position = "ROOM1"
                        if name_position in liyt:
                            walk_to(liyt[name_position])
                    else:
                        speak("going to first dining room")
                        num1, num2, num3 = dining_room_dif["din1"]
                        chassis.move_to(num1, num2, num3)
                        while not rospy.is_shutdown():
                            # 4. Get the chassis status.
                            code = chassis.status_code
                            text = chassis.status_text
                            if code == 3:
                                break
                            if code == 4:
                                break
                        time.sleep(1)
                        clear_costmaps
                    step_action = 1
                    step = "turn"
                    action = "find"
                    nav1_skip_cnt = 0
                if step_action == 1:
                    # walk in front of the guy
                    name_position = "$POSE/GESTURE"
                    if "$POSE/GESTURE" not in liyt:
                        name_position = "POSE/GESTURE"
                    if name_position in liyt:
                        feature = liyt[name_position]
                    if step == "turn" and dining_room_action == 0:
                        move(0, -0.2)
                        nav1_skip_cnt += 1
                        if nav1_skip_cnt >= 250:
                            step = "none"
                            action = "none"
                            step_action = 3
                            speak("I can't find you I gonna go back to the instruction point")
                    elif step == "turn" and dining_room_action == 1:
                        move(0, -0.2)
                        nav1_skip_cnt += 1
                        if nav1_skip_cnt >= 70:
                            dining_room_action = 2
                            nav1_skip_cnt = 0
                            speak("going to second dining room")
                            num1, num2, num3 = dining_room_dif["din2"]
                            chassis.move_to(num1, num2, num3)
                            while not rospy.is_shutdown():
                                # 4. Get the chassis status.
                                code = chassis.status_code
                                text = chassis.status_text
                                if code == 3:
                                    break
                                if code == 4:
                                    break
                            time.sleep(1)
                            clear_costmaps
                    elif step == "turn" and dining_room_action == 2:
                        move(0, -0.2)
                        nav1_skip_cnt += 1
                        if nav1_skip_cnt >= 70:
                            step = "none"
                            action = "none"
                            step_action = 3
                            nav1_skip_cnt = 0
                            speak("I can't find you I gonna go back to the host")
                    if step == "confirm":
                        print("imwrited")
                        file_path = "/home/pcms/catkin_ws/src/beginner_tutorials/src/m1_evidence/GSPR_people.jpg"
                        with open(file_path, 'rb') as f:
                            files = {'image': (file_path.split('/')[-1], f)}
                            url = "http://172.20.10.5:8888/upload_image"
                            response = requests.post(url, files=files)
                            # remember to add the text question on the computer code
                        print("Upload Status Code:", response.status_code)
                        upload_result = response.json()
                        print("sent image")
                        who_help = "Is the guy " + feature
                        gg = post_message_request("checkpeople", feature, who_help)
                        print(gg)
                        # get answer from gemini
                        while True:
                            r = requests.get("http://172.20.10.5:8888/Fambot", timeout=10)
                            response_data = r.text
                            dictt = json.loads(response_data)
                            if dictt["Steps"] == 11:
                                break
                            time.sleep(2)
                        aaa = dictt["Voice"].lower()
                        print("answer:", aaa)
                        current_file_name = output_dir + "GSPR_people" + str(current_time) + "_command_" + str(
                            i) + ".jpg"
                        new_file_name = output_dir + "GSPR_people.jpg"
                        try:
                            os.rename(new_file_name, current_file_name)
                            print("************")
                            print("command", commandcntcnt, "File name:", current_file_name)
                            print("************")
                        except FileNotFoundError:
                            print("File renamed failed")
                        except PermissionError:
                            print("File renamed failed")
                        feature = feature.lower()
                        if "yes" in aaa or "ys" in aaa or "none" in feature:

                            speak("found you the guest " + feature + " my name is Fambot")
                            action = "front"
                            step = "none"
                        else:
                            action = "find"
                            step = "turn"
                            if dining_room_action == 0:
                                for i in range(55):
                                    move(0, -0.2)
                                    time.sleep(0.125)
                        gg = post_message_request("-1", "", "")
                    if action == "find":
                        checking_image = _frame2.copy()
                        detections = dnn_yolo1.forward(checking_image)[0]["det"]
                        # clothes_yolo
                        # nearest people
                        nx = 1750
                        cx_n, cy_n = 0, 0
                        CX_ER = 99999
                        need_position = 0
                        for i, detection in enumerate(detections):
                            # print(detection)
                            x1, y1, x2, y2, _, class_id = map(int, detection)
                            score = detection[4]
                            cx = (x2 - x1) // 2 + x1
                            cy = (y2 - y1) // 2 + y1
                            # depth=find_depsth
                            _, _, d = get_real_xyz(code_depth, cx, cy, 2)
                            # cv2.rectangle(up_image, (x1, y1), (x2, y2), (0, 255, 0), 2)

                            if score > 0.45 and class_id == 0 and d <= nx and d != 0 and (320 - cx) < CX_ER:
                                need_position = [x1, y1, x2, y2, cx, cy]
                                # ask gemini
                                cv2.rectangle(code_image, (x1, y1), (x2, y2), (255, 0, 0), 2)
                                cv2.circle(code_image, (cx, cy), 5, (0, 255, 0), -1)
                                print("people distance", d)
                                CX_ER = 320 - cx
                        if need_position != 0:
                            step = "none"
                            h, w, c = code_image.shape
                            x1, y1, x2, y2, cx2, cy2 = map(int, need_position)
                            e = w // 2 - cx2
                            v = 0.001 * e
                            if v > 0:
                                v = min(v, 0.3)
                            if v < 0:
                                v = max(v, -0.3)
                            move(0, v)
                            print(e)
                            output_dir = "/home/pcms/catkin_ws/src/beginner_tutorials/src/m1_evidence/"
                            face_box = [x1, y1, x2, y2]
                            box_roi = checking_image[face_box[1]:face_box[3] - 1, face_box[0]:face_box[2] - 1, :]
                            fh, fw = abs(x1 - x2), abs(y1 - y2)
                            cv2.imwrite(output_dir + "GSPR_people.jpg", box_roi)
                            if abs(e) <= 5:
                                # speak("walk")
                                action = "none"
                                step = "confirm"
                                print("turned")
                                move(0, 0)
                    if action == "front":
                        speed = 0.2
                        h, w, c = code_image.shape
                        cx, cy = w // 2, h // 2
                        for i in range(cy + 1, h):
                            if _depth2[cy][cx] == 0 or 0 < _depth2[i][cx] < _depth2[cy][cx]:
                                cy = i
                        _, _, d = get_real_xyz(_depth2, cx, cy, 2)
                        print("depth", d)
                        if d != 0 and d <= 1000:
                            action = "speak"
                            move(0, 0)
                        else:
                            move(0.2, 0)
                    if action == "speak":
                        speak("hello")
                        speak(real_name)
                        # speak("can u stand behind me and I will follow u now")
                        time.sleep(2)
                        for i in range(78):
                            move(0, -0.35)
                            time.sleep(0.125)
                        if real_name == "guest":
                            speak("dear guest please stand in front of me")
                            speak("and remember to say robot you can stop")
                        else:
                            speak(real_name)
                            speak("Please stand in front of me")
                            speak("and remember to say robot you can stop")
                        # time.sleep(0.5)
                        speak("when you arrived and I will go back")
                        # time.sleep(0.5)
                        speak("hello dear " + real_name)
                        speak("I will start follow you now")
                        # speak(
                        #    "Look at me and please walk but don't walk too fast, and remember to say robot stop when you arrived thank you")
                        action = 1
                        step = "none"
                        step_action = 2
                # follow me
                if action == 1:
                    s = s.lower()
                    print("listening", s)
                    if "thank" in s or "you" in s or "stop" in s or "arrive" in s or "robot" in s or step == "back":
                        action = 0
                        step_action = 3
                        speak("I will go back now bye bye")
                if step_action == 2:
                    msg = Twist()
                    code_image = _frame2.copy()
                    poses = net_pose.forward(code_image)
                    min_d = 9999
                    t_idx = -1
                    for i, pose in enumerate(poses):
                        if pose[5][2] == 0 or pose[6][2] == 0:
                            continue
                        p5 = list(map(int, pose[5][:2]))
                        p6 = list(map(int, pose[6][:2]))

                        cx = (p5[0] + p6[0]) // 2
                        cy = (p5[1] + p6[1]) // 2
                        cv2.circle(code_image, p5, 5, (0, 0, 255), -1)
                        cv2.circle(code_image, p6, 5, (0, 0, 255), -1)
                        cv2.circle(code_image, (cx, cy), 5, (0, 255, 0), -1)
                        _, _, d = get_real_xyz(code_depth, cx, cy, 2)
                        if d >= 1800 or d == 0: continue
                        if (d != 0 and d < min_d):
                            t_idx = i
                            min_d = d

                    x, z = 0, 0
                    if t_idx != -1:
                        p5 = list(map(int, poses[t_idx][5][:2]))
                        p6 = list(map(int, poses[t_idx][6][:2]))
                        cx = (p5[0] + p6[0]) // 2
                        cy = (p5[1] + p6[1]) // 2
                        _, _, d = get_real_xyz(code_depth, cx, cy, 2)
                        cv2.circle(code_image, (cx, cy), 5, (0, 255, 255), -1)

                        print("people_d", d)
                        if d >= 1800 or d == 0: continue

                        x, z, code_image, yn = _fw.calc_cmd_vel(code_image, code_depth, cx, cy)
                        print("turn_x_z:", x, z)

                    if x == 0 and z == 0:
                        if speak_nack >= 20:
                            speak("don't walk too fast, pleae come back")
                            speak_nack = 0
                        speak_nack += 1
                        followmecnt += 1
                    else:
                        followmecnt = 0
                    if followmecnt >= 100:
                        step = "back"
                        speak("I can't find you I gonna go back now")
                        followmecnt = 0
                    print("follow", followmecnt)
                    move(x, z)
                if step_action == 3:
                    step_action = 100
            # Navigation2
            elif "navigation2" in command_type or ("navi" in command_type and "2" in command_type):
                if step_action == 0:
                    if dining_room_action == 0:
                        name_position = "$ROOM1"
                        if "$ROOM1" not in liyt:
                            name_position = "ROOM1"
                        if name_position in liyt:
                            walk_to(liyt[name_position])
                    else:
                        speak("going to first dining room")
                        num1, num2, num3 = dining_room_dif["din1"]
                        chassis.move_to(num1, num2, num3)
                        while not rospy.is_shutdown():
                            # 4. Get the chassis status.
                            code = chassis.status_code
                            text = chassis.status_text
                            if code == 3:
                                break
                            if code == 4:
                                break
                        time.sleep(1)
                        clear_costmaps
                    step_action = 1
                    step = "turn"
                    action = "find"
                    nav2_skip_cnt = 0
                    name_position = "$POSE/GESTURE"
                    if "$POSE/GESTURE" not in liyt:
                        name_position = "POSE/GESTURE"
                    if name_position in liyt:
                        feature = liyt[name_position]
                if step_action == 1:
                    # walk in front of the guy
                    if step == "turn" and dining_room_action == 0:
                        move(0, -0.2)
                        nav2_skip_cnt += 1
                        if nav2_skip_cnt >= 250:
                            step = "none"
                            action = "none"
                            step_action = 3
                            speak("find you guest, stand behind me and come with me")
                            time.sleep(2)
                    elif step == "turn" and dining_room_action == 1:
                        move(0, -0.2)
                        nav2_skip_cnt += 1
                        if nav2_skip_cnt >= 70:
                            dining_room_action = 2
                            nav2_skip_cnt = 0
                            speak("going to second dining room")
                            num1, num2, num3 = dining_room_dif["din2"]
                            chassis.move_to(num1, num2, num3)
                            while not rospy.is_shutdown():
                                # 4. Get the chassis status.
                                code = chassis.status_code
                                text = chassis.status_text
                                if code == 3:
                                    break
                                if code == 4:
                                    break
                            time.sleep(1)
                            clear_costmaps
                    elif step == "turn" and dining_room_action == 2:
                        move(0, -0.2)
                        nav2_skip_cnt += 1
                        if nav2_skip_cnt >= 70:
                            step = "none"
                            action = "none"
                            step_action = 3
                            nav2_skip_cnt = 0
                            speak("find you guest, stand behind me and come with me")
                    if step == "confirm":
                        print("imwrited")
                        file_path = "/home/pcms/catkin_ws/src/beginner_tutorials/src/m1_evidence/GSPR_people.jpg"
                        with open(file_path, 'rb') as f:
                            files = {'image': (file_path.split('/')[-1], f)}
                            url = "http://172.20.10.5:8888/upload_image"
                            response = requests.post(url, files=files)
                            # remember to add the text question on the computer code
                        print("Upload Status Code:", response.status_code)
                        upload_result = response.json()
                        print("sent image")
                        who_help = "Is the guy " + feature
                        gg = post_message_request("checkpeople", feature, who_help)
                        print(gg)
                        # get answer from gemini
                        while True:
                            r = requests.get("http://172.20.10.5:8888/Fambot", timeout=10)
                            response_data = r.text
                            dictt = json.loads(response_data)
                            if dictt["Steps"] == 11:
                                break
                            time.sleep(2)
                        aaa = dictt["Voice"].lower()
                        print("answer:", aaa)
                        current_file_name = output_dir + "GSPR_people" + str(current_time) + "_command_" + str(
                            i) + ".jpg"
                        new_file_name = output_dir + "GSPR_people.jpg"
                        try:
                            os.rename(new_file_name, current_file_name)
                            print("************")
                            print("command", commandcntcnt, "File name:", current_file_name)
                            print("************")
                        except FileNotFoundError:
                            print("File renamed failed")
                        except PermissionError:
                            print("File renamed failed")
                        feature = feature.lower()
                        if "yes" in aaa or "ys" in aaa or "none" in feature:
                            speak("found you the guying " + feature)
                            action = "front"
                            step = "none"
                        else:
                            action = "find"
                            step = "turn"
                            if dining_room_action == 0:
                                for i in range(55):
                                    move(0, -0.2)
                                    time.sleep(0.125)
                        gg = post_message_request("-1", "", "")

                    if action == "find":
                        checking_image = _frame2.copy()
                        detections = dnn_yolo1.forward(checking_image)[0]["det"]
                        # clothes_yolo
                        # nearest people
                        nx = 1750
                        cx_n, cy_n = 0, 0
                        CX_ER = 99999
                        need_position = 0
                        for i, detection in enumerate(detections):
                            # print(detection)
                            x1, y1, x2, y2, _, class_id = map(int, detection)
                            score = detection[4]
                            cx = (x2 - x1) // 2 + x1
                            cy = (y2 - y1) // 2 + y1
                            # depth=find_depsth
                            _, _, d = get_real_xyz(code_depth, cx, cy, 2)
                            # cv2.rectangle(up_image, (x1, y1), (x2, y2), (0, 255, 0), 2)

                            if score > 0.45 and class_id == 0 and d <= nx and d != 0 and (320 - cx) < CX_ER:
                                need_position = [x1, y1, x2, y2, cx, cy]
                                # ask gemini
                                cv2.rectangle(code_image, (x1, y1), (x2, y2), (255, 0, 0), 2)
                                cv2.circle(code_image, (cx, cy), 5, (0, 255, 0), -1)
                                print("people distance", d)
                                CX_ER = 320 - cx
                        if need_position != 0:
                            step = "none"
                            h, w, c = code_image.shape
                            x1, y1, x2, y2, cx2, cy2 = map(int, need_position)
                            e = w // 2 - cx2
                            v = 0.001 * e
                            if v > 0:
                                v = min(v, 0.3)
                            if v < 0:
                                v = max(v, -0.3)
                            move(0, v)
                            print(e)
                            output_dir = "/home/pcms/catkin_ws/src/beginner_tutorials/src/m1_evidence/"
                            face_box = [x1, y1, x2, y2]
                            box_roi = checking_image[face_box[1]:face_box[3] - 1, face_box[0]:face_box[2] - 1, :]
                            fh, fw = abs(x1 - x2), abs(y1 - y2)
                            cv2.imwrite(output_dir + "GSPR_people.jpg", box_roi)

                            if abs(e) <= 10:
                                # speak("walk")
                                action = "none"
                                step = "confirm"
                                print("turned")
                                move(0, 0)
                    if action == "front":
                        speed = 0.2
                        h, w, c = code_image.shape
                        cx, cy = w // 2, h // 2
                        for i in range(cy + 1, h):
                            if _depth2[cy][cx] == 0 or 0 < _depth2[i][cx] < _depth2[cy][cx]:
                                cy = i
                        _, _, d = get_real_xyz(_depth2, cx, cy, 2)
                        print("depth", d)
                        if d != 0 and d <= 1000:
                            action = "speak"
                            move(0, 0)
                        else:
                            move(0.2, 0)
                    if action == "speak":
                        speak("hello dear " + real_name)
                        speak("can u stand behind me and I will guide u now")
                        action = 1
                        step = "none"
                        step_action = 3
                if step_action == 3:
                    name_position = "$ROOM2"
                    if "$ROOM2" not in liyt:
                        name_position = "ROOM2"
                    if name_position in liyt:
                        walk_to(liyt[name_position])
                    if real_name == "guest":
                        speak("dear guest ")
                    else:
                        speak(real_name)
                    speak("here is " + liyt[name_position] + " and I will go back now")
                    step_action = 100
            # Speech1
            elif "speech1" in command_type or ("spee" in command_type and "1" in command_type):
                if step_action == 0:
                    name_position = "$ROOM1"
                    if "$ROOM1" not in liyt:
                        name_position = "ROOM1"
                    if name_position in liyt:
                        walk_to(liyt[name_position])
                    step_action = 1
                    action = "speak"
                if step_action == 1:
                    name_position = "$PLACE1"
                    if "$PLACE1" not in liyt:
                        name_position = "PLACE1"
                    if name_position in liyt:
                        walk_to(liyt[name_position])
                    for i in range(250):
                        move(0, -0.2)
                        time.sleep(0.125)
                    if action == "speak":
                        action = 1
                        step = "none"
                        step_action = 2
                if step_action == 2:  # get text
                    # question detect
                    answer = "none"
                    none_cnt = 0
                    speak(real_name)
                    speak("hello, dear guest please speak your question in complete sentence after the")
                    playsound("nigga2.mp3")
                    speak("sound")
                    time.sleep(0.5)
                    playsound("nigga2.mp3")
                    step_action = 3
                if step_action == 3:
                    now1 = datetime.now()
                    s = s.lower()
                    current_time = now1.strftime("%H:%M:%S")
                    current_month = now1.strftime("%B")  # Full month name
                    current_day_name = now1.strftime("%A")  # Full weekday name
                    day_of_month = now1.strftime("%d")
                    answer = "none"
                    if "move" in s or "way" in s:
                        answer = "Because you're holding my joystick."
                    elif "correct" in s:
                        print("***************")
                        speak("It's spelled")
                        speak("r")
                        speak("o")
                        speak("b")
                        speak("o")
                        speak("t")
                        answer = "no need"
                        print("***************")
                    elif "star" in s or "system" in s:
                        answer = "It is the Sun."
                    elif "color" in s:
                        answer = "I like black."
                    elif "get" in s or "total" in s or "dice" in s:
                        answer = "It's about one sixth."
                        print("It is about 16.7%. || It's about one sixth.")
                    else:
                        answer = "none"
                    time.sleep(0.1)
                    none_cnt += 1
                    if failed_cnt > 4:
                        print("***************")
                        speak("It's spelled")
                        speak("r")
                        speak("o")
                        speak("b")
                        speak("o")
                        speak("t")
                        print("It's spelled r-o-b-o-t")
                        print("***************")
                        step_action = 4
                    if answer == "none" and none_cnt >= 250:
                        speak("can u please speak it louder")
                        none_cnt = 0
                        failed_cnt += 1
                    elif answer != "none":
                        print("***************")
                        if answer != "no need":
                            speak(answer)
                        print("***************")
                        step_action = 4
                if step_action == 4:
                    speak("I will go back now bye bye")
                    step_action = 100
            # Speech2
            elif "speech2" in command_type or ("spee" in command_type and "2" in command_type):
                if step_action == 0:
                    if dining_room_action == 0:
                        name_position = "$ROOM1"
                        if "$ROOM1" not in liyt:
                            name_position = "ROOM1"
                        if name_position in liyt:
                            walk_to(liyt[name_position])
                    else:
                        speak("going to first dining room")
                        num1, num2, num3 = dining_room_dif["din1"]
                        chassis.move_to(num1, num2, num3)
                        while not rospy.is_shutdown():
                            # 4. Get the chassis status.
                            code = chassis.status_code
                            text = chassis.status_text
                            if code == 3:
                                break
                            if code == 4:
                                break
                        time.sleep(1)
                        clear_costmaps
                    step = "turn"
                    action = "find"
                    step_action = 1
                    speech2_turn_skip = 0
                    name_position = "$POSE/GESTURE"
                    if "$POSE/GESTURE" not in liyt:
                        name_position = "POSE/GESTURE"
                    if name_position in liyt:
                        feature = liyt[name_position]
                if step_action == 1:
                    # walk in front of the guy
                    if step == "turn" and dining_room_action == 0:
                        move(0, -0.2)
                        speech2_turn_skip += 1
                        if speech2_turn_skip >= 250:
                            speech2_turn_skip = 0
                            step = "none"
                            action = "none"
                            speak("hello guest")
                            step_action = 2
                    elif step == "turn" and dining_room_action == 1:
                        move(0, -0.2)
                        speech2_turn_skip += 1
                        if speech2_turn_skip >= 70:
                            dining_room_action = 2
                            speech2_turn_skip = 0
                            speak("going to second dining room")
                            num1, num2, num3 = dining_room_dif["din2"]
                            chassis.move_to(num1, num2, num3)
                            while not rospy.is_shutdown():
                                # 4. Get the chassis status.
                                code = chassis.status_code
                                text = chassis.status_text
                                if code == 3:
                                    break
                                if code == 4:
                                    break
                            time.sleep(1)
                            clear_costmaps
                    elif step == "turn" and dining_room_action == 2:
                        move(0, -0.2)
                        speech2_turn_skip += 1
                        if speech2_turn_skip >= 70:
                            step = "none"
                            action = "none"
                            step_action = 2
                            speech2_turn_skip = 0
                            dining_room_action = 0
                            speak("Hello guest")
                    if step == "confirm":
                        print("imwrited")
                        file_path = "/home/pcms/catkin_ws/src/beginner_tutorials/src/m1_evidence/GSPR_people.jpg"
                        with open(file_path, 'rb') as f:
                            files = {'image': (file_path.split('/')[-1], f)}
                            url = "http://172.20.10.5:8888/upload_image"
                            response = requests.post(url, files=files)
                            # remember to add the text question on the computer code
                        print("Upload Status Code:", response.status_code)
                        upload_result = response.json()
                        print("sent image")
                        who_help = "Is the guy " + feature
                        gg = post_message_request("checkpeople", feature, who_help)
                        print(gg)
                        # get answer from gemini
                        while True:
                            r = requests.get("http://172.20.10.5:8888/Fambot", timeout=10)
                            response_data = r.text
                            dictt = json.loads(response_data)
                            if dictt["Steps"] == 11:
                                break
                            time.sleep(2)
                        aaa = dictt["Voice"].lower()
                        print("answer:", aaa)
                        current_file_name = output_dir + "GSPR_people" + str(current_time) + "_command_" + str(
                            i) + ".jpg"
                        new_file_name = output_dir + "GSPR_people.jpg"
                        try:
                            os.rename(new_file_name, current_file_name)
                            print("************")
                            print("command", commandcntcnt, "File name:", current_file_name)
                            print("************")
                        except FileNotFoundError:
                            print("File renamed failed")
                        except PermissionError:
                            print("File renamed failed")
                        feature = feature.lower()
                        if "yes" in aaa or "ys" in aaa or "none" in feature:
                            speak("found you the guy " + str(feature))
                            action = "front"
                            step = "none"
                        else:
                            action = "find"
                            step = "turn"
                            if dining_room_action == 0:
                                for i in range(55):
                                    move(0, -0.2)
                                    time.sleep(0.125)
                        gg = post_message_request("-1", "", "")
                    if action == "find":
                        checking_image = _frame2.copy()
                        detections = dnn_yolo1.forward(checking_image)[0]["det"]
                        # clothes_yolo
                        # nearest people
                        nx = 1750
                        cx_n, cy_n = 0, 0
                        CX_ER = 99999
                        need_position = 0
                        for i, detection in enumerate(detections):
                            # print(detection)
                            x1, y1, x2, y2, _, class_id = map(int, detection)
                            score = detection[4]
                            cx = (x2 - x1) // 2 + x1
                            cy = (y2 - y1) // 2 + y1
                            # depth=find_depsth
                            _, _, d = get_real_xyz(code_depth, cx, cy, 2)
                            # cv2.rectangle(up_image, (x1, y1), (x2, y2), (0, 255, 0), 2)

                            if score > 0.45 and class_id == 0 and d <= nx and d != 0 and (320 - cx) < CX_ER:
                                need_position = [x1, y1, x2, y2, cx, cy]
                                # ask gemini
                                cv2.rectangle(code_image, (x1, y1), (x2, y2), (255, 0, 0), 2)
                                cv2.circle(code_image, (cx, cy), 5, (0, 255, 0), -1)
                                print("people distance", d)
                                CX_ER = 320 - cx
                        if need_position != 0:
                            step = "none"
                            h, w, c = code_image.shape
                            x1, y1, x2, y2, cx2, cy2 = map(int, need_position)
                            e = w // 2 - cx2
                            v = 0.001 * e
                            if v > 0:
                                v = min(v, 0.3)
                            if v < 0:
                                v = max(v, -0.3)
                            move(0, v)
                            print(e)
                            output_dir = "/home/pcms/catkin_ws/src/beginner_tutorials/src/m1_evidence/"
                            face_box = [x1, y1, x2, y2]
                            box_roi = checking_image[face_box[1]:face_box[3] - 1, face_box[0]:face_box[2] - 1, :]
                            fh, fw = abs(x1 - x2), abs(y1 - y2)
                            cv2.imwrite(output_dir + "GSPR_people.jpg", box_roi)
                            if abs(e) <= 5:
                                # speak("walk")
                                action = "none"
                                step = "confirm"
                                print("turned")
                                move(0, 0)
                    if action == "front":
                        speed = 0.2
                        h, w, c = code_image.shape
                        cx, cy = w // 2, h // 2
                        for i in range(cy + 1, h):
                            if _depth2[cy][cx] == 0 or 0 < _depth2[i][cx] < _depth2[cy][cx]:
                                cy = i
                        _, _, d = get_real_xyz(_depth2, cx, cy, 2)
                        print("depth", d)
                        if d != 0 and d <= 1000:
                            action = "speak"
                            move(0, 0)
                        else:
                            move(0.2, 0)
                    if action == "speak":
                        step = "none"
                        action = "none"
                        step_action = 2
                if step_action == 2:
                    now = datetime.now()
                    name_position = "$TELL_LIST"
                    if "$TELL_LIST" not in liyt:
                        name_position = "TELL_LIST"
                    current_time = now.strftime("%H:%M:%S")
                    question = "My question is " + liyt[name_position]
                    if real_name == "guest":
                        speak("dear guest")
                    else:
                        speak(real_name)
                    time.sleep(1)
                    print("***************")
                    user_input = user_input.lower()
                    if "something about yourself" in user_input or (
                            "yourself" in user_input):
                        speak("We are Fambot from Macau Puiching Middle School, and I was made in 2024")
                    elif "what day today is" in user_input or ("today" in user_input and "day" in user_input):
                        speak("today is 10 th of july in 2025")
                    elif "what day tomorrow is" in user_input or ("tomorrow" in user_input and "day" in user_input):
                        speak("today is 11 th of july in 2025")
                    elif "your team's name" in user_input or ("name" in user_input and "team" in user_input):
                        speak("my team name is Fambot")
                    elif "your teams country" in user_input or (
                            "country" in user_input):
                        speak("We are Fambot from China")
                    elif "your teams affiliation" in user_input or (
                            "affiliation" in user_input):
                        speak("We are Fambot from Macau Puiching Middle School")
                    elif "what the time is" in user_input or ("time" in user_input):
                        speak("the current time is " + current_time)
                    elif "the day of the week" in user_input or (
                            "week" in user_input and "day" in user_input):
                        speak("Today is Sunday")
                    elif "the day of the month" in user_input or (
                            "month" in user_input and "day" in user_input):
                        speak("Today is the ten th of July in 2025")
                    step_action = 3
                    print("***************")
                if step_action == 3:
                    time.sleep(1)
                    speak("I will go back now bye bye")
                    step_action = 100
            else:
                speak("please scan it again")
                break
            cv2.imshow("frame", code_image)
            key = cv2.waitKey(1)
            if key in [ord('q'), 27]:
                break
        walk_to("instruction point")
        print("***************")
        print("command", commandcntcnt, end=" ")
        speak(final_speak_to_guest)
        print("***************")
        time.sleep(2)
    speak("GPSR end")
