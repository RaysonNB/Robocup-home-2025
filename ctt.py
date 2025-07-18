import cv2
import time
import json
import rospy
import base64
import numpy as np
from loguru import logger
from openai import OpenAI

from robotic_arm_control import RoboticController, Dy
from LemonEngine.sensors import Camera
from LemonEngine.hardwares.respeaker import Respeaker
from LemonEngine.hardwares.chassis import Chassis, Navigator

client = OpenAI(
    api_key="AIzaSyCYSxtdvQnullmuGnJqI49kwGlqT1ZAmpo",
    base_url="https://generativelanguage.googleapis.com/v1beta/openai/"
)

id_list = [11, 12, 0, 15, 14, 13, 1, 2]
Ro = RoboticController()
Ro.open_robotic_arm("/dev/arm", id_list)


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

def draw_bbox(img, boxes_json: list, label="", color=(255, 0, 0), thickness=2):
    if "bounding_boxes" in boxes_json:
        boxes_json = boxes_json["bounding_boxes"]
    height, width = img.shape[:2]
    for box in boxes_json:
        abs_y1 = int(box["box_2d"][0] / 1000 * height)
        abs_x1 = int(box["box_2d"][1] / 1000 * width)
        abs_y2 = int(box["box_2d"][2] / 1000 * height)
        abs_x2 = int(box["box_2d"][3] / 1000 * width)
        cv2.rectangle(img, (abs_x1, abs_y1), (abs_x2, abs_y2), color, thickness=thickness)
        if "label" in box and label == "":
            label = str(box.get("label"))

        cv2.putText(
            img, label, (abs_x1, abs_y1 - 10),
            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2
        )

    return img

def get_bboxes(img: np.ndarray, prompt: str, api_client: OpenAI):
    """
    Uses Gemini to detect bounding boxes

    Args:
        img (np.ndarray): The input image in cv2 format (NumPy array).
        prompt (str): The text prompt describing what to detect (e.g., "the car").
        api_client (OpenAI): The initialized OpenAI client instance.

    Returns:
        [{label: xxx, box_2d: [x, x, x, x]}, ...]
    """

    # 1. Encode the cv2 image to a base64 string
    success, encoded_image = cv2.imencode('.jpeg', img)
    if not success:
        print("Error: Could not encode image to JPEG format.")
        return None
    base64_image = base64.b64encode(encoded_image).decode('utf-8')

    # 2. Define the system prompt, now asking for a specific JSON object structure
    system_instruction = (
        "You are an expert in object detection. "
        "Only output a json list where each entry contains the 2D bounding box in \"box_2d\" and a text label in \"label\"."
        "The 'box_2d' should be an array of [ymin, xmin, ymax, xmax] with coordinates normalized to 0-1000. "
    )
    
    user_prompt = f"Detect {prompt}"
    logger.debug(f"Prompt: {user_prompt}")
    # 3. Call the API with the new `reasoning_effort` parameter
    response = api_client.chat.completions.create(
        model="gemini-2.5-flash",
        reasoning_effort="none",
        messages=[
            {"role": "system", "content": system_instruction},
            {
                "role": "user",
                "content": [
                    {"type": "text", "text": user_prompt},
                    {
                        "type": "image_url",
                        "image_url": {"url": f"data:image/jpeg;base64,{base64_image}"}
                    }
                ]
            }
        ],
        temperature=0,
    )

    content = response.choices[0].message.content
    print(content)
    if "```" in content:
        content = content.replace("```json", "")
        content = content.replace("```", "")
    return json.loads(content)

def close_grip(grip_id):
    logger.info("Start Closing Grip")
    Dy.profile_velocity(grip_id, 20)
    final_angle = 0
    last_angle = Dy.present_position(grip_id)
    target_angle = 90
    dt = 0.2
    i = 0
    while target_angle > 5:
        
        i += 1
        angle = Dy.present_position(grip_id)
        dangle = abs( last_angle - Dy.present_position(grip_id) )
        time.sleep(dt)
        last_angle = angle

        angle = Dy.present_position(grip_id)
        angle_speed = dangle / dt
        logger.debug(f"{angle}, {last_angle}, {angle_speed}, {target_angle}, {i}")

        target_angle = angle - 10
        Dy.goal_absolute_direction(grip_id, target_angle)

        if angle_speed <= 20.0 and i > 3:
        # if False:
            logger.debug(f"Stop at {Dy.present_position(grip_id)}")
            final_angle = Dy.present_position(grip_id) + 3
            Dy.goal_absolute_direction(grip_id, final_angle)
            break

    time.sleep(1)
    Ro.go_to_real_xyz_alpha(id_list, [0, 250, 150], -25, 0, final_angle, 0)
    return final_angle

def main():
    chassis_move = Chassis()
    navigator = Navigator()
    
    respeaker = Respeaker(enable_espeak_fix=True)
    cam = Camera("/camera/color/image_raw", "bgr8")
    cam_depth = Camera("/camera/depth/image_raw", "passthrough")
    rate = rospy.Rate(20)
    navigator.move_to(0.650, 6.924, 0.822, max_retry=3) # kitchen table
    time.sleep(1)
    respeaker.say("Now taking picture")
    frame = cam.get_frame()
    frame_depth = cam.get_frame()

    boxes_json = get_bboxes(frame, "2 drinks on the table") # + get_bboxes(frame, "cup on the table")[:1]

    frame_base = frame.copy()
    frame_base = draw_bbox(frame_base, boxes_json)
    cv2.imwrite("evidence", frame_base)

    for i, bbox in enumerate(boxes_json):

        label = bbox.get('label') if bbox.get('label') else "Object"
        
        Ro.go_to_real_xyz_alpha(id_list, [0, 150, 300], 0, 0, 90, 0)

        frame_base = frame.copy()
        frame_base = draw_bbox(frame_base, [bbox])
        cv2.imwrite("image1.jpg", frame_base)

        ## Take by myself
        OFFSET_Z = 220 # horz
        OFFSET_Y = 600 # height
        mid_point = ((bbox[1], bbox[3]) // 2, (bbox[0] + bbox[2]) // 2) # x, y
        point = get_real_xyz(frame_depth, *mid_point)
        arm_x = point[0]
        arm_y = point[2] - OFFSET_Y
        arm_z = point[1] + OFFSET_Z
        success = Ro.go_to_real_xyz_alpha(id_list, [arm_x, arm_y, arm_z], 0, 0, 90, 0)
        if success:
            final_angle = close_grip(id_list[-1])
            Ro.go_to_real_xyz_alpha(id_list, [0, 150, 300], 0, 0, final_angle, 0)
        else:
            respeaker.say("Sorry, i cant get it")
        ####

        ## Request to take
        # 
        # respeaker.say(f"I detected {label} with a bounding box ")
        # respeaker.say(f"Dear Referee Please help me take {label} on the table and place it on my robot arm")
        # time.sleep(3)
        # respeaker.say(f"Please wait for the gripper close")
        # final_angle = close_grip(id_list[-1])
        ###

        navigator.move_to(-0.099, 4.990, -3.103) # rubbish bin
        # open arm
        Ro.go_to_real_xyz_alpha(id_list, [0, 300, 150], 0, 0, final_angle, 0)
        Ro.go_to_real_xyz_alpha(id_list, [0, 300, 150], 0, 0, 90, 0)




if __name__ == '__main__':
    rospy.init_node('clean_the_table', anonymous=True)
    main()

