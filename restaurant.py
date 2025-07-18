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

'''
def callback_voice(msg):
    global s
    s = msg.text
'''

def speak(g):
    print("[robot said]: ", end=" ")
    os.system(f'espeak -s 150 "{g}"')
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
    api_url = "http://192.168.50.147:8888/Fambot"
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
clear_costmaps = rospy.ServiceProxy("/move_base/clear_costmaps", Empty)

def walk_to():
        num1, num2, num3 = 1.069, 2.017, -0.015
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
if __name__ == "__main__":
    rospy.init_node("demo")
    rospy.loginfo("demo node start!")
    # open things
    chassis = RobotChassis()

    net_pose = HumanPoseEstimation(device_name="GPU")
    print("gemini2 rgb")
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
    #s = ""
    print("sever1")
    gg = post_message_request("-1", "", "")
    print("sever2")
    step = "none"
    confirm_command = 0
    speak("I am ready")
    speak_j=""
    for i in range(2):
        step_action=0
        skip_voice_cnt=0
        while not rospy.is_shutdown():
            # voice check
            # break
            now1 = datetime.now()
            current_time = now1.strftime("%H:%M:%S")
            rospy.Rate(10).sleep()
            confirm_command = 0
            if _frame1 is None:
                print("no frame")
            if _depth1 is None:
                print("no depth")
            code_image = _frame1.copy()
            code_depth = _depth1.copy()
            catch_image = _frame1.copy()
            cv2.imshow("frame", code_image)
            key = cv2.waitKey(1)
            if key in [ord('q'), 27]:
                break
            if step_action == 0:
                speak("guys, please raising your hand if u need a order")
                step="turn"
                action="find"
                speak("finding person")
            if step_action == 1:
                # walk in front of the guy
                if step == "turn":
                    move(0, -0.2)
                if step == "confirm":
                    print("imwrited")
                    file_path = "/home/pcms/catkin_ws/src/beginner_tutorials/src/m1_evidence/GSPR_people.jpg"
                    with open(file_path, 'rb') as f:
                        files = {'image': (file_path.split('/')[-1], f)}
                        url = "http://192.168.50.147:8888/upload_image"
                        response = requests.post(url, files=files)
                        # remember to add the text question on the computer code
                    print("Upload Status Code:", response.status_code)
                    upload_result = response.json()
                    print("sent image")
                    feature="raising hand"
                    who_help = "Is the guy " + "raising hand or calling(speaking)"
                    gg = post_message_request("checkpeople", "raising hand", who_help)
                    print(gg)
                    # get answer from gemini
                    while True:
                        r = requests.get("http://192.168.50.147:8888/Fambot", timeout=10)
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
                        print("command", i, "File name:", current_file_name)
                        print("************")
                    except FileNotFoundError:
                        print("File renamed failed")
                    except PermissionError:
                        print("File renamed failed")
                    feature = feature.lower()
                    gg = post_message_request("-1", "", "")
                    if "yes" in aaa or "ys" in aaa:
                        speak("found you the guy " + str(feature))
                        action = "front"
                        step = "none"
                    else:
                        action = "find"
                        step = "turn"
                        for i in range(55):
                            move(0, -0.2)
                            time.sleep(0.125)
                if action == "find":
                    code_image = _frame1.copy()
                    detections = dnn_yolo1.forward(code_image)[0]["det"]
                    # clothes_yolo
                    # nearest people
                    nx = 2500
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
    
                        if score > 0.65 and class_id == 0 and d <= nx and d != 0 and (320 - cx) < CX_ER:
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
                            v = min(v, 0.45)
                        if v < 0:
                            v = max(v, -0.45)
                        move(0, v)
                        print(e)
                        output_dir = "/home/pcms/catkin_ws/src/beginner_tutorials/src/m1_evidence/"
                        face_box = [x1, y1, x2, y2]
                        box_roi = _frame1[face_box[1]:face_box[3] - 1, face_box[0]:face_box[2] - 1, :]
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
                        if _depth1[cy][cx] == 0 or 0 < _depth1[i][cx] < _depth1[cy][cx]:
                            cy = i
                    _, _, d = get_real_xyz(_depth1, cx, cy, 2)
                    print("depth", d)
                    if d != 0 and d <= 1200:
                        action = "speak"
                        move(0, 0)
                    else:
                        move(0.3, 0)
                if action == "speak":
                    step = "none"
                    action = "none"
                    step_action = 2
            if step_action == 2:
                speak("please tell me what you want, you can take the keyboard and type")
                answer=input("*******************please type it here:   ")
                gg = post_message_request("color", "", answer)
                while True:
                    r = requests.get("http://172.20.10.5:8888/Fambot", timeout=10)
                    response_data = r.text
                    dictt = json.loads(response_data)
                    if dictt["Steps"] == 12:
                        break
                    time.sleep(2)
                final_speak_to_guest = dictt["Voice"]
                gg = post_message_request("-1", "", "")
                speak(final_speak_to_guest)
                time.sleep(1)
                speak("sorry guest, I can't help you")
                move(0,0.2)
                time.sleep(5)
                move(0,0)
                break
            speak("mission end")
