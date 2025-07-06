#!/usr/bin/env python3
import rospy
from mr_voice.msg import Voice
import time
from std_msgs.msg import String
import re
def callback_voice(msg):
    global s
    s = msg.text
if __name__ == "__main__":
    rospy.init_node('your_node_name')
    s = ""
    rospy.Subscriber("/voice/text", Voice, callback_voice)
    pre_s = ""
    name_cnt= pre_name_cnt= "nigga"
    while not rospy.is_shutdown():
        rospy.Rate(10).sleep()
        confirm_command = 0
        if s != "" and s != pre_s:
            print(s)
            pre_s = s
        s = s.lower()
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
        if name_cnt != pre_name_cnt:
            print("name", name_cnt)
        pre_name_cnt=name_cnt
