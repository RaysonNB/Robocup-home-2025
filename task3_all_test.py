#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2, os
import time
from RobotChassis import RobotChassis
from std_srvs.srv import Empty
from geometry_msgs.msg import Twist

def callback_image2(msg):
    global _frame2
    _frame2 = CvBridge().imgmsg_to_cv2(msg, "bgr8")


def callback_depth2(msg):
    global _depth2
    _depth2 = CvBridge().imgmsg_to_cv2(msg, "passthrough")


def callback_image1(msg):
    global _frame1
    src = CvBridge().imgmsg_to_cv2(msg, "bgr8")
    _frame1 = cv2.flip(src, 0)
    _frame1 = cv2.flip(_frame1, 1)


def callback_depth1(msg):
    global _depth1
    _depth1 = CvBridge().imgmsg_to_cv2(msg, "passthrough")

locations = {
    # Furniture and objects
    "first": [-0.927, 0.086, 0.1],
    "seats": [-0.927, 0.086, 0.1],
    "guest": [1.193, 2.021, 1.53],
    "drinktable": [2.47, 3.36, -1.607],
}

def speak1(g):
    print("[robot say]:", end=" ")
    os.system(f'espeak -s 170 "{g}"')
    # rospy.loginfo(g)
    print(g)
    time.sleep(0.3)

def walk_to(name):
    name = name.lower()
    num1, num2, num3 = locations[name]
    chassis.move_to(num1, num2, num3)
    while not rospy.is_shutdown():
        # 4. Get the chassis status.
        code = chassis.status_code
        text = chassis.status_text
        if code == 3:
            break
        if code == 4:
            break
    speak1("arrived")
    time.sleep(1)
    clear_costmaps
clear_costmaps = rospy.ServiceProxy("/move_base/clear_costmaps", Empty)

def move(forward_speed: float = 0, turn_speed: float = 0):
    global _cmd_vel
    msg = Twist()
    msg.linear.x = forward_speed
    msg.angular.z = turn_speed
    _cmd_vel.publish(msg)

def turn(angle):
    print("hi")
    if angle==-1:
        move(0, 0.35)
    elif angle==1:
        move(0, -0.35)
    else:
        if angle < 0:
            for i in range(-angle):
                move(0, 0.6)
                time.sleep(0.026)
        else:
            for i in range(angle):
                move(0, 0 - .6)
                time.sleep(0.026)
        time.sleep(1.5)
def seat_turn(num12):
    check_num = str(num12)
    angle1 = -2
    angle2 = -1
    angle3 = 0
    angle4 = 1
    angle5 = 2
    if "1" in check_num:
        turn(angle1)
    elif "2" in check_num:
        turn(angle2)
    elif "3" in check_num:
        turn(angle3)
    elif "4" in check_num:
        turn(angle4)
    elif "5" in check_num:
        turn(angle5)
    time.sleep(1)
if __name__ == "__main__":
    rospy.init_node("demo")
    rospy.loginfo("demo node start!")
    # open things
    chassis = RobotChassis()

    print("cmd_vel")
    _cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
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
    yn="yes"
    speak1("going to guest")
    walk_to("guest")
    speak1("going to seats")
    walk_to("seats")
    speak1("going to drink table")
    walk_to("drinktable")
    if yn=="yes":
        speak1("check turning angles")
        for i in [1,2,3,4,5]:
            speak1("going to "+str(i))
            walk_to("first")
            walk_to("seats")
            time.sleep(1)
            seat_turn(str(i))
            time.sleep(1)
            a=input()
    while not rospy.is_shutdown():
        rospy.Rate(10).sleep()
        confirm_command = 0
        if _frame2 is None:
            print("no frame")
        if _depth2 is None:
            print("no depth")
        code_image = _frame2.copy()
        code_depth = _depth2.copy()
        check_empty_img = _frame1.copy()
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 1.5
        font_color = (255, 255, 255)
        font_thickness = 5
        width = 640
        height = 320
        # correct the numbers**********************
        positions = {
            1: (int(width * 0.08), int(height * 0.4)),
            2: (int(width * 0.32), int(height * 0.4)),
            3: (int(width * 0.5), int(height * 0.4)),
            4: (int(width * 0.68), int(height * 0.4)),
            5: (int(width * 0.85), int(height * 0.4))
        }
        for number, pos in positions.items():
            text_size = cv2.getTextSize(str(number), font, font_scale, font_thickness)[0]
            rectangle_start = (pos[0] - 10, pos[1] + 10)  # Adjust margins
            rectangle_end = (pos[0] + text_size[0] + 10, pos[1] - text_size[1] - 10)
            cv2.rectangle(check_empty_img, rectangle_start, rectangle_end, (0, 0, 0),
                          cv2.FILLED)
            cv2.putText(check_empty_img, str(number), pos, font, font_scale, font_color, font_thickness,
                        cv2.LINE_AA)
        cv2.imshow("frame", check_empty_img)
        key = cv2.waitKey(1)
        if key in [ord('q'), 27]:
            break
