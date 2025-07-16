import rospy
import re, json, requests
import cv2, os, time
import mimetypes
import random as r
from loguru import logger
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty
from LemonEngine.sensors import Camera
from LemonEngine.hardwares.respeaker import Respeaker
from LemonEngine.hardwares.chassis import Chassis
from RobotChassis import RobotChassis

from robotic_arm_control import RoboticController, Dy


# TABLE_P = (0.284, 3.317, -1.57)
TABLE_P = (0.525,2.7,1.55)
GOAL_P = (0.144,2.21,3.14)
# FOOD_POINT = (6.34, 3.07, 1.5)
# TASK_POINT = (5.13, 2.90, 1.5)
# UNKNOWN_POINT = (3.97, 2.85, 1.5)
# KITCHEN_POINT = (2.05, 3.72, 0)

# TABLE_P = (1.772, -0.118, 0.8)
# FOOD_POINT = (0.278, -0.139, -2.462)


# UNKNOWN_POINT = (1.638, -0.231, -0.910)
# KITCHEN_POINT = (1.638, -0.231, -0.910)



PROMPT = """
# Instruction
Analyze the input image. Detect distinct objects on the table and try your best to classify them using the `Object List` below. 
If the object is not listed, use the class "Unknown" and **fill in a short description of the item in the key `"name"`**.
Be careful not to leave any items behide

You Must output *only* a JSON list containing objects with keys `"name"` and `"category"`. 
If no object here, please output a empty json list 

```json
[]
```

# Object List
| Name | Category | Appearance |
|:---|:---|:---|
| mayo | food | Squeeze bottle of green mayonnaise. |
| tuna | food | Round, blue metal can of tuna. |
| ketchup | food | Three red plastic squeeze bottles of ketchup. |
| oats | food | Two cardboard boxes of Nestlé oats. |
| broth | food | Knorr broth box with two cubes. |
| corn_flower | food | Yellow and blue bag of corn flour. |
| peanuts | snack | Golden-brown bag of Japanese-style peanuts. |
| cornflakes | snack | Red Nescau Duo cereal box. |
| crisps | snack | Green and blue Ruffles chips bag. |
| pringles | snack | Three tall cylindrical cans of Pringles. |
| cheese_snack | snack | Yellow bag of Fandangos cheese snacks. |
| chocolate_bar | snack | Blue metallic package of Bis chocolate. |
| gum_balls | snack | Orange bag of Fini sour gum balls. |
| apple | fruit | Round red apple with a stem. |
| lemon | fruit | Bright yellow lemon with textured skin. |
| tangerine | fruit | Bright orange tangerine with bumpy skin. |
| pear | fruit | Light green, bell-shaped pear with stem. |
| spoon | dish | Metal spoon with a red handle. |
| plate | dish | Solid dark red round dinner plate. |
| cup | dish | Red speckled enamel mug with handle. |
| fork | dish | Metal fork with a red handle. |
| bowl | dish | Round red speckled enamel bowl. |
| knife | dish | Small metal knife with a red handle. |
| cloth | cleaning_supply | One red and one orange cloth. |
| polish | cleaning_supply | Yellow bottle of Bravo furniture polish. |
| brush | cleaning_supply | Blue scrub brush with green bristles. |
| sponge | cleaning_supply | Yellow sponge with a green scrubber. |
| coffee | drink | Two cartons of protein coffee drinks. |
| kuat | drink | Green aluminum can of Kuat soda. |
| milk | drink | Blue and white carton of milk. |
| orange_juice | drink | Large plastic bottle of orange soda. |
| fanta | drink | Orange aluminum can of Fanta soda. |
| coke | drink | Red can of Coca-Cola Zero Sugar. |

* Furnitures (i.e. Table, Chair) is not an object

# Example Output
```json
[
  {"name": "mayo", "category": "food"},
  {"name": "ketchup", "category": "food"},
  {"name": "sponge", "category": "cleaning_supply"}
  {"name": "a red bottle", "category": "unknown"}
]
```
"""

object_list_dict = {
    "mayo": {
        "category": "food",
        "appearance": "Squeeze bottle of green mayonnaise."
    },
    "tuna": {
        "category": "food",
        "appearance": "Round, blue metal can of tuna."
    },
    "ketchup": {
        "category": "food",
        "appearance": "Three red plastic squeeze bottles of ketchup."
    },
    "oats": {
        "category": "food",
        "appearance": "Two cardboard boxes of Nestlé oats."
    },
    "broth": {
        "category": "food",
        "appearance": "Knorr broth box with two cubes."
    },
    "corn_flower": {
        "category": "food",
        "appearance": "Yellow and blue bag of corn flour."
    },
    "peanuts": {
        "category": "snack",
        "appearance": "Golden-brown bag of Japanese-style peanuts."
    },
    "cornflakes": {
        "category": "snack",
        "appearance": "Red Nescau Duo cereal box."
    },
    "crisps": {
        "category": "snack",
        "appearance": "Green and blue Ruffles chips bag."
    },
    "pringles": {
        "category": "snack",
        "appearance": "Three tall cylindrical cans of Pringles."
    },
    "cheese_snack": {
        "category": "snack",
        "appearance": "Yellow bag of Fandangos cheese snacks."
    },
    "chocolate_bar": {
        "category": "snack",
        "appearance": "Blue metallic package of Bis chocolate."
    },
    "gum_balls": {
        "category": "snack",
        "appearance": "Orange bag of Fini sour gum balls."
    },
    "apple": {
        "category": "fruit",
        "appearance": "Round red apple with a stem."
    },
    "lemon": {
        "category": "fruit",
        "appearance": "Bright yellow lemon with textured skin."
    },
    "tangerine": {
        "category": "fruit",
        "appearance": "Bright orange tangerine with bumpy skin."
    },
    "pear": {
        "category": "fruit",
        "appearance": "Light green, bell-shaped pear with stem."
    },
    "spoon": {
        "category": "dish",
        "appearance": "Metal spoon with a red handle."
    },
    "plate": {
        "category": "dish",
        "appearance": "Solid dark red round dinner plate."
    },
    "cup": {
        "category": "dish",
        "appearance": "Red speckled enamel mug with handle."
    },
    "fork": {
        "category": "dish",
        "appearance": "Metal fork with a red handle."
    },
    "bowl": {
        "category": "dish",
        "appearance": "Round red speckled enamel bowl."
    },
    "knife": {
        "category": "dish",
        "appearance": "Small metal knife with a red handle."
    },
    "cloth": {
        "category": "cleaning_supply",
        "appearance": "One red and one orange cloth."
    },
    "polish": {
        "category": "cleaning_supply",
        "appearance": "Yellow bottle of Bravo furniture polish."
    },
    "brush": {
        "category": "cleaning_supply",
        "appearance": "Blue scrub brush with green bristles."
    },
    "sponge": {
        "category": "cleaning_supply",
        "appearance": "Yellow sponge with a green scrubber."
    },
    "coffee": {
        "category": "drink",
        "appearance": "Two cartons of protein coffee drinks."
    },
    "kuat": {
        "category": "drink",
        "appearance": "Green aluminum can of Kuat soda."
    },
    "milk": {
        "category": "drink",
        "appearance": "Blue and white carton of milk."
    },
    "orange_juice": {
        "category": "drink",
        "appearance": "Large plastic bottle of orange soda."
    },
    "fanta": {
        "category": "drink",
        "appearance": "Orange aluminum can of Fanta soda."
    },
    "coke": {
        "category": "drink",
        "appearance": "Red can of Coca-Cola Zero Sugar."
    }
}

cmd_vel=rospy.Publisher("/cmd_vel",Twist,queue_size=10)


def generate_content(prompt_text: str = None, image_path: str = None) -> dict:
    """
    Sends a request to the Gemini Flask API to generate content.

    Args:
        prompt_text:  Optional text prompt to send to the API.
        image_path:   Optional path to an image file to send to the API.

    Returns:
        A dictionary containing the API response, or None if an error occurred.
    """
    url = "http://192.168.50.143:5000/generate"  # Adjust if your server is running on a different host/port
    files = {}
    data = {}
    logger.info(f"Generate Content (\"{prompt_text}\", {image_path})")
    if prompt_text:
        data['prompt'] = prompt_text
    if image_path:
        try:
            mime_type, _ = mimetypes.guess_type(image_path)
            if not mime_type:
                print("Error: Could not determine MIME type of image file.")
                return None
            with open(image_path, 'rb') as image_file:
                image_data = image_file.read()
            files['image'] = (image_path, image_data, mime_type)
        except FileNotFoundError:
            print(f"Error: Image file not found at {image_path}")
            return None
        except Exception as e:
            print(f"Error opening image file: {e}")
            return None

    try:
        response = requests.post(url, files=files, data=data)
        response.raise_for_status()  # Raise HTTPError for bad responses (4xx or 5xx)
        result = response.json()
        logger.info(f"Output... {result}")
        return result  # Parse JSON response

    except requests.exceptions.RequestException as e:
        print(f"Request failed: {e}")
        return None

def move(forward_speed: float = 0, turn_speed: float = 0):
    msg = Twist()
    msg.linear.x = forward_speed
    msg.angular.z = turn_speed
    cmd_vel.publish(msg)

def main():
    clear_costmaps = rospy.ServiceProxy("/move_base/clear_costmaps", Empty)
    
    
    chassis = RobotChassis()
    respeaker = Respeaker(enable_espeak_fix=True)
    
    id_list = [11, 12, 0, 15, 14, 13, 1, 2]
    Ro = RoboticController()
    Ro.open_robotic_arm("/dev/arm", id_list)

    # navigator = Navigator()
    cam1 = Camera("/camera/color/image_raw", "bgr8")
    cam2 = Camera("/camera/depth/image_raw", "passthrough")
    width, height = cam1.width, cam1.height
    cx, cy = width // 2, height // 2
    rate = rospy.Rate(20)
    
    def walk_to(point):
        logger.info(f"Walk to {point}")
        tried = 0
        chassis.move_to(*point)
        clear_costmaps
        while not rospy.is_shutdown():
            # 4. Get the chassis status.
            rate = rospy.Rate(20)
            code = chassis.status_code
            text = chassis.status_text
            if code == 3:
                logger.success("Point Reached!")
                return True
                
            if code == 4:
                logger.error(f"Plan Failed (tried {tried})")
                respeaker.say("I am blocked, please move aside")
                clear_costmaps
                chassis.move_to(*point)
                if tried > 3:
                    break
                tried += 1

        chassis.move_base.cancel_all_goals()
        return False
                
    def ask_gemini(text):
        match = False
        while True:
            logger.info("Asking Gemini for objects")
            frame = cam1.get_frame()
            cv2.imwrite("./image.jpg", frame)
            text = generate_content(text, "./image.jpg").get('generated_text')
            if text is None:
                respeaker.say("Failed")
                continue
            text = text.replace("\n", "")
            text = text.replace("\r", "")
            print("Gemini Res", text)
    
            pattern = r"```json(.*?)```"  # Corrected regex pattern
            match = re.search(pattern, text)
    
            if match:
                json_string = match.group(1)  # Extract the content inside ```json ... ```
                logger.debug(json_string)
                json_object = json.loads(json_string)
                return json_object

    def ask_gemini_for_bbox(text, path=None):
        while True:
            try:
                logger.info("Asking Gemini for bbox")
                if path is None:
                    frame = cam1.get_frame()
                    cv2.imwrite("./image.jpg", frame)
                    bboxes = generate_content("$bbox$"+text, "./image.jpg")
                else:
                    bboxes = generate_content("$bbox$"+text, path)
                return bboxes
            except:
                logger.error("Error @ ask_gemini_for_bbox")

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
                final_angle = Dy.present_position(grip_id) + 5
                Dy.goal_absolute_direction(grip_id, final_angle)
                break

        time.sleep(1)
        Ro.go_to_real_xyz_alpha(id_list, [0, 250, 150], -25, 0, final_angle, 0)

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
        
    respeaker.say("I am waiting for the door open")

    depth_count = 0
    while not rospy.is_shutdown():
        frame = cam1.get_frame()
        rate.sleep()
        depth_frame = cam2.get_frame()
        depth_frame = cv2.resize(depth_frame, (width, height))
        depth = depth_frame[cy, cx]

        frame = cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)
        frame = cv2.putText(frame, f"{depth}", (cx, cy), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        if depth == 0:
            depth_count += 1
        else:
            depth_count = 0
        if depth > 2000 or depth_count > 150:
            respeaker.say("The door is opened")

            for i in range(10):
                move(0.25, 0)
                time.sleep(0.1)
            
            move(0, 0)
            break

        cv2.imshow("depth", depth_frame)
        cv2.imshow("frame", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    
    clear_costmaps
    # walk_to(FULL_CABINET)
    # respeaker.say("Huamn, Please open the cabinet door for me")
    # time.sleep(10)
    # img_cpy = cam1.get_frame()
    # cv2.imwrite("./cabinet.jpg", img_cpy)
    # respeaker.say("Ok")
    
    walk_to(TABLE_P)


    saved_bboxes = []

    respeaker.say("I am recognizing objects")
    json_object = ask_gemini(PROMPT)
    respeaker.say("I am detecting and categorying objects, it might take some time")
    img_cpy = cam1.get_frame()
    cv2.imwrite("./image2.jpg", img_cpy)
    img_cpy = cv2.resize(img_cpy, (1920, 1440))
    # respeaker.say("Dear human, can you please help me open the cabinet door?")

    for obj in json_object:
        logger.info(f"Detecting {obj['name']}")
        name, categ = obj["name"].lower(), obj["category"].lower()
        if name.lower().strip() == "unknown":
            continue
        desc = object_list_dict.get(name.lower().strip())
        if desc is not None:
            desc = desc.get("appearance")
        bboxes = ask_gemini_for_bbox(f"{name},({desc})", "./image2.jpg")
        saved_bboxes.append(obj)
        saved_bboxes[-1]["bbox"] = bboxes
        
        draw_bbox(img_cpy, bboxes, label=f"{name}:{categ}")
        cv2.imwrite("./image3.jpg", img_cpy)
    cv2.imwrite("./image_all_box_evidence.jpg", img_cpy)

    img_cpy = cv2.imread("./image2.jpg")

    Ro.go_to_real_xyz_alpha(id_list, [0, 300, 150], 0, 0, 90, 0)

    respeaker.say("I see")
    for a_object in saved_bboxes[:5]:
        img_base = img_cpy.copy()
        logger.info(f"Requesting for... {a_object}")


        draw_bbox(img_base, a_object["bbox"], label=f"{a_object['name']}:{a_object['category']}", color=(0, 9, 255), thickness=3)
        cv2.imwrite("./image.jpg", img_base)

        respeaker.say(f"Please help me take {a_object['name']} inside the red bounding box")
        time.sleep(5)
        respeaker.say("Help me put it in my robot arm and wait for the gripper close")
        time.sleep(10)

        print("**CLOSE_ARM")
        close_grip(id_list[-1])
        respeaker.say("Thank you")
        time.sleep(5)

        respeaker.say(a_object["category"])
        walk_to(GOAL_P)
        
        respeaker.say("Putting Object")
        for i in range(20):
            move(0.25, 0)
            rate.sleep()
            
        move(0, 0)
        print("**OPEN_ARM")
        time.sleep(5)
        Ro.go_to_real_xyz_alpha(id_list, [0, 300, 150], 0, 0, 90, 0)
        for i in range(20):
            move(-0.25, 0)
            rate.sleep()
            
        move(0, 0)

        walk_to(TABLE_P)
    

if __name__ == '__main__':
    rospy.init_node('tidyup', anonymous=True)
    main()
