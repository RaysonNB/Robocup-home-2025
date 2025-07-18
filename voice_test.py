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
        if "populous" in s or "most" in s: name_cnt = "São Paulo is the most populous city in Brazil with 12.03 million residents."
        if "month" in s or "independ" in s: name_cnt = "On September 7, 1822, Brazil’s independence was declared."
        if "space" in s or "first" in s: name_cnt = "In March 2006, Pontes became the first Brazilian to go to space."
        if "lake" in s or "tourist" in s or "spot" in s: name_cnt = "	Belo Horizonte"
        if "small" in s or "extension" in s: name_cnt = "Sergipe"
        if "locate" in s or "palace" in s: name_cnt = "Brasília"
        if "new" in s or "nearest" in s: name_cnt = "Tocantins"
        if "capital" in s or "bahia" in s: name_cnt = "Salvador"
        if "typical" in s or "food" in s or "foot" in s: name_cnt = "Bahia"
        if "color" in s or "flag" in s: name_cnt = "White, red and blue"
        if name_cnt != pre_name_cnt:
            print("answer", name_cnt)
        pre_name_cnt=name_cnt
