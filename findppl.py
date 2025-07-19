import rospy, tf
import cv2, time
import numpy as np
import ros_numpy as rnp
from loguru import logger
from LemonEngine.sensors import Camera
from LemonEngine.hardwares.respeaker import Respeaker
from LemonEngine.hardwares.chassis import Chassis, Navigator
from LemonEngine.ai.openvino import YoloPose
from LemonEngine.utils.seqtool import Streak

from visualization_msgs.msg import Marker
from std_msgs.msg import Header, ColorRGBA
from geometry_msgs.msg import Point, Quaternion, Vector3, Pose

frame_id = "human"

def get_real_xyz(dp, x, y):
    a = 55.0 * np.pi / 180
    b = 86.0 * np.pi / 180
    h, w = dp.shape[:2]
    y_min = np.clip(y-8, 0, h-1)
    y_max = np.clip(y+8, 0, h-1)
    x_min = np.clip(x-8, 0, w-1)
    x_max = np.clip(x+8, 0, w-1)

    ys, xs = np.nonzero(dp[y_min:y_max, x_min:x_max])
    if ys.size == 0: 
        d = 0
    else:
        xs += x_min; ys += y_min
        i = ((xs - x)**2 + (ys - y)**2).argmin()
        d = dp[ys[i], xs[i]]

    x = int(x) - int(w // 2)
    y = int(y) - int(h // 2)
    real_y = round(y * 2 * d * np.tan(a / 2) / h)
    real_x = round(x * 2 * d * np.tan(b / 2) / w)
    return real_x, real_y, d

def make_marker(frame_id, position: Point, color: ColorRGBA = ColorRGBA(1, 0, 0, 1)):
    now = rospy.Time.now()

    return Marker(
        header=Header(stamp=now, frame_id=frame_id),
        ns="human",
        type=Marker.CYLINDER,
        action=Marker.ADD,
        pose=Pose(
            position=position,
            orientation=Quaternion(0, 0, 0, 1)
        ),
        scale=Vector3(0.15, 0.15, 0.5),
        color=color
    )

def keypoint_to_xyz(depth, kpts, index):
    x = kpts[index, 0]
    y = kpts[index, 1]
    return get_real_xyz(depth, x, y)


def main():
    chassis = Chassis()
    navigator = Navigator()
    cam = Camera("/camera/color/image_raw", "bgr8")
    cam_depth = Camera("/camera/depth/image_raw", "passthrough")
    respeaker = Respeaker(enable_espeak_fix=True)
    model = YoloPose(device_name="CPU")
    clock = time.time()
    frame_cnt = 0
    stop_cnt = 0
    rate = rospy.Rate(10)
    br = tf.TransformBroadcaster()
    li = tf.TransformListener()
    rospy.sleep(1)
    logger.success("TF initialized!")

    marker_pub = rospy.Publisher('human_marker', Marker, queue_size=10)
    marker_pub2 = rospy.Publisher('goal_marker', Marker, queue_size=10)
    
    respeaker.say("Dear Human, Please do not move, i will find you")

    speed_streak = Streak(lambda x: x<0.1)
    best_match = {"box":None, "pt1": None, "pt2":None, "kpt":None, "score":0}

    while not rospy.is_shutdown():
        rate.sleep()
        frame = cam.get_frame()
        depth = cam_depth.get_frame()
        output = model.predict(frame)
        boxes, kpts = output

        best_match = {"box":None, "pt1": None, "pt2":None, "kpt":None, "score":0}

        for box, kpt in zip(boxes, kpts):
            box = box.astype(np.int32)

            cv2.circle(frame, (kpt[YoloPose.Left_Wrist, :2].astype(np.int32)), 3, (255, 100, 232), 2)
            
            if kpt[YoloPose.Left_Shoulder, 2] < 0.1 or \
                kpt[YoloPose.Right_Shoulder, 2] < 0.1 or \
                max(kpt[YoloPose.Left_Wrist, 2], kpt[YoloPose.Right_Wrist, 2]) < 0.1:
                continue

            pt1 = kpt[YoloPose.Right_Shoulder]
            pt2 = kpt[YoloPose.Left_Shoulder]
            mid_y = int((pt1[1] + pt2[1]) / 2)
            mid_x = int((pt1[0] + pt2[0]) / 2)
            depth_mid = get_real_xyz(depth, mid_x, mid_y)[2]

            if depth_mid > 2600:
                continue

            wrist_y = min(kpt[YoloPose.Right_Wrist, 1], kpt[YoloPose.Left_Wrist, 1])
            score = (mid_y - wrist_y) / 100


            if score > best_match["score"]:
                best_match["box"] = box
                best_match["pt1"] = pt1
                best_match["pt2"] = pt2
                best_match["score"] = score

        if best_match["box"] is not None:
            box, kpt = best_match["box"], best_match["kpt"]

            cx, cy = (box[0]+box[2])//2, (box[1]+box[3])//2
            cv2.rectangle(frame, (box[0], box[1]), (box[2], box[3]), (255, 100, 232), 2)
            cv2.circle(frame, (cx, cy), 3, (255, 100, 232), 2)

            _x, _y, _z = get_real_xyz(depth, cx, cy)
            _z -= 200
            angle = np.arctan2(-_x, _z)

            br.sendTransform(
                translation=np.array([_z, -_x, 0]) / 1000,
                rotation=tf.transformations.quaternion_from_euler(0,0,0),
                time=rospy.Time.now(),
                child=frame_id,
                parent="base_link",
            )
            marker = make_marker(frame_id, position=Point(0, 0, 0.25))
            marker_pub.publish(marker)

            speed = angle / 6
        else:
            speed = 0.15

        # logger.debug(f"Speed: {speed}")
        # chassis.set_angular(speed)
        # if speed @ speed_streak > 20:
        #     respeaker.say("I found you")
        #     cv2.imwrite("evidence.jpg", frame)
        #     chassis.stop_moving()
        #     break
        
        cv2.putText(frame, f"FPS: {frame_cnt/ (time.time() - clock):.3f}", (10, 30), cv2.FONT_HERSHEY_PLAIN, 1, (255, 0, 0))
        cv2.imshow("frame1", frame)

        frame_cnt += 1
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
 



if __name__ == '__main__':
    rospy.init_node('test_camera', anonymous=True)
    main()
