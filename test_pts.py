import time
import rospy
import numpy as np
from loguru import logger

from LemonEngine.hardwares.respeaker import Respeaker
from LemonEngine.hardwares.chassis import Navigator

POINTS = {
    "Kitchen Table": [0.650, 6.924, 0.822],
    "Rubbis Bin": [-0.099, 4.990, -3.103]
}

def main():
    navigator = Navigator()
    respeaker = Respeaker()
    for name, position in POINTS:
        respeaker.say(f"Going to {name}")
        success = navigator.move_to(*position, max_retry=3)
        if success:
            respeaker.say(f"reached {name}")
        else:
            respeaker.say(f"Fail to go {name}")
        time.sleep(1)

if __name__ == '__main__':
    rospy.init_node('test_point', anonymous=True)
    main()
