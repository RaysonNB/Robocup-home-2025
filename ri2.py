import rospy
import cv2, time
from loguru import logger
from LemonEngine.sensors import Camera
from LemonEngine.hardwares.respeaker import Respeaker
from LemonEngine.hardwares.chassis import Chassis, Navigator


def main():
    chassis_move = Chassis()
    navigator = Navigator()
    
    respeaker = Respeaker(enable_espeak_fix=True)
    cam = Camera("/camera/depth/image_raw", "passthrough")
    width, height = cam.width, cam.height
    cx, cy = width // 2, height // 2
    rate = rospy.Rate(20)

    zero_count = 0
    while not rospy.is_shutdown():
        rate.sleep()
        frame = cam.get_frame()
        depth = frame[cy, cx]
        logger.debug(f"Depth: {depth}, Captured Continue {zero_count} zero points")

        zero_count += 1 if depth == 0 else (-zero_count)
        if depth > 2000 or zero_count > 100:
            respeaker.say("The door is opened")

            time.sleep(5)
            for _ in range(80):
                chassis_move.set_linear(0.25)
                rate.sleep()
            time.sleep(3)
            navigator.clear_costmaps()
            break

        cv2.imshow("frame", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cv2.destroyAllWindows()

    navigator.move_to(*[3.500, 3.064, -3.074])
    time.sleep(2)
    respeaker.say("Testing testing")
    time.sleep(5)
    respeaker.say("i am going to exit after 3 seconds")
    time.sleep(3)

    navigator.move_to(*[-1.032, 7.978, -3.092])
    time.sleep(2)

    respeaker.say("Done")

if __name__ == '__main__':
    rospy.init_node('test_camera', anonymous=True)
    main()
