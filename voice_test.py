import rospy
from mr_voice.msg import Voice
def callback_voice(msg):
    global s
    s = msg.text
if __name__ == "__main__":
    s = ""
    rospy.Subscriber("/voice/text", Voice, callback_voice)
    pre_s = ""
    while not rospy.is_shutdown():
        rospy.Rate(10).sleep()
        confirm_command = 0
        if s != "" and s != pre_s:
            print(s)
            pre_s = s
        s = s.lower()
        if "charcoal" in s or "chicago" in s or "chikako" in s: name_cnt = "chikako"
        if "yoshimura" in s or "shima" in s or "shi" in s or "tsushima" in s: name_cnt = "yoshimura"
        if "basil" in s or "stac" in s or "stace" in s or "bas" in s or "basel" in s or "special" in s: name_cnt = "basil"
        if "angel" in s: name_cnt = "angel"
        if "check" in s or "track" in s or "jack" in s: name_cnt = "jack"
        if "andrew" in s or "angelo" in s: name_cnt = "andrew"
        if "sophia" in s: name_cnt = "sophia"
        if "mike" in s: name_cnt = "mike"
        if "leo" in s: name_cnt = "leo"
        if "tom" in s: name_cnt = "tom"
