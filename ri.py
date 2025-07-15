import rospy
import cv2, os, time
from loguru import logger
from std_srvs.srv import Empty
from LemonEngine.sensors import Camera
from LemonEngine.hardwares.respeaker import Respeaker
from LemonEngine.hardwares.chassis import Chassis
from RobotChassis import RobotChassis

POINT_DINING_ROOM = (-0,0,0.001) ##

def main():
    clear_costmaps = rospy.ServiceProxy("/move_base/clear_costmaps", Empty)
    chassis_move = Chassis()
    chassis = RobotChassis()
    # navigator = Navigator()
    
    respeaker = Respeaker(enable_espeak_fix=True)
    cam1 = Camera("/cam2/color/image_raw", "bgr8")
    cam2 = Camera("/cam2/depth/image_raw", "passthrough")
    width, height = cam1.width, cam1.height
    cx, cy = width // 2, height // 2
    rate = rospy.Rate(20)

    def walk_to(point):
        # navigate
        chassis.move_to(*point)
        while not rospy.is_shutdown():
            # 4. Get the chassis status.
            rate.sleep()
            code = chassis.status_code
            text = chassis.status_text
            if code == 3:
                break
            if code == 4:
                logger.error("Fail to get a plan")
                respeaker.say("Fail to get a plan")
                time.sleep(1)
                clear_costmaps
                chassis.move_to(*point)

        clear_costmaps

    zero_count = 0

    while not rospy.is_shutdown():
        rate.sleep()
        frame = cam1.get_frame()
        depth_frame = cam2.get_frame()
        depth_frame = cv2.resize(depth_frame, (width, height))
        depth = depth_frame[cy, cx]

        frame = cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)
        frame = cv2.putText(frame, f"{depth}", (cx, cy), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        if depth == 0:
            zero_count += 1
        else:
            zero_count = 0
        
        if depth > 2000 or zero_count > 100:
            respeaker.say("The door is opened")

            time.sleep(5)
            for i in range(50):
                chassis_move.set_linear(0.25)
                time.sleep(0.1)
            time.sleep(3)
            clear_costmaps
            break

        cv2.imshow("depth", depth_frame)
        cv2.imshow("frame", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    cv2.destroyAllWindows()

    walk_to([4.130, 3.064, -3.074])
    time.sleep(2)
    respeaker.say("Testing testing")
    prompt = ""
    while prompt != "ok":
        prompt = input(">>")

    walk_to([-1.032, 7.978, -3.092])
    time.sleep(2)

    respeaker.say("Done")

if __name__ == '__main__':
    rospy.init_node('test_camera', anonymous=True)
    main()
