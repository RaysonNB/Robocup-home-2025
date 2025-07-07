import google.generativeai as genai
import json
import time
import requests
import google.generativeai as genai
import os
import PIL.Image
import cv2
import numpy as np
from datetime import datetime

pathnum = r"C:/Users/rayso/Desktop/python/"
from Generate_command import kitchen_items

genai.configure(api_key='AIzaSyBdTRu-rcBKbf86gjiMNtezBu1dEuxrWyE')
model = genai.GenerativeModel("gemini-2.0-flash")
cnt_yy = 0
while True:

    while True:
        r = requests.get("http://192.168.50.147:8888/Fambot", timeout=2.5)
        response_data = r.text
        print("Response_data", response_data)
        dictt = json.loads(response_data)
        if dictt["Steps"] == "-1" or dictt["Voice"] == "":
            time.sleep(2)
        else:
            break
    if dictt["Steps"] == "first":
        s1 = dictt["Voice"]
        s = "***The Sentence:" + s1
        print("question", s)
        sample_txt = """

        (The Sentence)(Task: Sentence Structure)(I given u)
        Manipulation1: Go to the $ROOM1, grasp the $OBJECT on the $PLACE1 and place it on the $PLACE2.
        Manipulation2: Go to the $ROOM1, grasp the $OBJECT on the $PLACE1 and give it to $PERSON on the $ROOM2.(if &PERSON is me than $ROOM2:"instruction point" just edit $ROOM2)
        Vision (Enumeration)1: Tell me how many $CATEGORY_OBJ here are on the $PLACE1.
        Vision (Enumeration)2: Tell me how many people in the $ROOM1 are $POSE/GESTURE.
        Vision (Description)1: Tell me what is the $OBJ_COMP object on the $PLACE1.
        Vision (Description)2: Tell me the $PERS_INFO of the person at the $PLACE1
        Navigation1: Go to the $ROOM1, find $POSE/GESTURE person and follow (him | her).
        Navigation2: Go to the $ROOM1, find $POSE/GESTURE person and guide (him|her) to the $ROOM2.
        Speech1: Go to the $ROOM1, find $PERSON at the $PLACE1 and answer (his | her) question.
        Speech2: Go to the $ROOM1, find the person who is $POSE/GESTURE and tell (him | her) $TELL_LIST.

        %possible information options
        %ROOM         : bedroom, kitchen, office, living room, bathroom
        %PLACE        : bed, bedside table, shelf, trashbin, dishwasher, potted plant, kitchen table, chairs, pantry, refrigerator, sink, cabinet, coatrack, desk, armchair, desk lamp, waste basket, tv stand, storage rack, lamp, side tables, sofa, bookshelf, entrance, exit
        $OBJECT       : orange juice, red wine, milk, iced tea, cola, tropical juice, juice pack, apple, pear, lemon, peach, banana, strawberry, orange, plum, cheezit, cornflakes, pringles, tuna, sugar, strawberry jello, tomato soup, mustard, chocolate jello, spam, coffee grounds, plate, fork, spoon, cup, knife, bowl, rubiks cube, soccer ball, dice, tennis ball, baseball, cleanser, sponge
        $PERS_INFO    : name, pose, gesture
        $CATEGORY_OBJ : drinks, fruits, snacks, foods, dishes, toys, cleaning supplies 
        $POSE/GESTURE : waving persons, persons raising their left arm, persons raising their right arm, persons pointing to the left, persons pointing to the right, sitting persons, standing persons, lying persons # or some of the clothes color
        $OBJ_COMP     : biggest, largest, smallest, heaviest, lightest, thinnest
        $TELL_LIST    : something about yourself, the time, what day is today, what day is tomorrow, your teams name, your teams country, your teams affiliation, the day of the week, the day of the month
        
        
        (Questions)
        Question1: which Task is it(just one) [Manipulation1, Manipulation2, Vision (Enumeration)1, Vision (Enumeration)2, Vision (Description)1, Vision (Description)2, Navigation1, Navigation2, Speech1, Speech2] ?
        Question2: give me the $informations(make it in dictionary), for example {"$ROOM1":"Living room","$PLACE1":"Tray A"} ?
        Question3: what the sentence mean, and what I should do(20words)(just give me one sentence)?


        here is the answer_format (in python_dictionary_format)

        *** {"1":[],"2":[],"3":[]} ***
        """
        response = model.generate_content([s, sample_txt])
        file_data_string = response.text
        print(file_data_string)
        file_data_string = file_data_string.replace("```", "")
        file_data_string = file_data_string.replace("python", "")
        file_data_string = file_data_string.replace("***", "")
        file_data_string = file_data_string.replace("json", "")
        dict = json.loads(file_data_string)
        Question1 = dict["1"]
        s = str(dict["2"])
        if (s[0] == "["):
            Question2 = dict["2"][0]
        else:
            Question2 = dict["2"]
        Question3 = dict["3"]
        time.sleep(1)
        questions = {
            "Question1": Question1,
            "Question2": Question2,
            "Question3": Question3,
            "Steps": 1,
            "Voice": "Voice",
            "Questionasking": "None",
            "answer": "None"
        }
        api_url = "http://192.168.50.147:8888/Fambot"
        response = requests.post(api_url, json=questions)
        result = response.json()
        print(result)
        print("sent")
        time.sleep(2)
    elif dictt["Steps"] == "checkpeople":
        promt = dictt["Questionasking"] + " answer my question ys or no only"
        image_url = f"http://192.168.50.147:8888{'/uploads/GSPR_people.jpg'}"
        print("Fetching image from:", image_url)
        image_response = requests.get(image_url)

        image_array = np.frombuffer(image_response.content, dtype=np.uint8)

        # Decode the image using OpenCV
        img = cv2.imdecode(image_array, cv2.IMREAD_COLOR)
        name_img = 'Robot_view' + str(cnt_yy) + '.jpg'
        print(name_img)
        # Save the image using OpenCV

        cv2.imwrite(pathnum + name_img, img)
        print("Image saved successfully using OpenCV!")
        # Configuration and setup
        genai.configure(api_key='AIzaSyBdTRu-rcBKbf86gjiMNtezBu1dEuxrWyE')  # Replace with your actual API key
        model = genai.GenerativeModel("gemini-2.0-flash")
        path_sample = "C:/Users/rayso/Desktop/python/" + name_img  # Use raw string to handle backslashes
        # Prepare the prompt template
        sample_txt = promt
        img = PIL.Image.open(path_sample)
        response = model.generate_content([img, sample_txt])
        file_data_string = response.text
        print(file_data_string)
        questions = {
            "Question1": "None",
            "Question2": "None",
            "Question3": "None",
            "Steps": 11,
            "Voice": file_data_string,
            "Questionasking": "None",
            "answer": "None"
        }
        api_url = "http://192.168.50.147:8888/Fambot"
        response = requests.post(api_url, json=questions)
        result = response.json()
        print(result)
        time.sleep(2)
    elif dictt["Steps"] == "Description":
        promt2 = dictt["Voice"]
        objects = '''
        The Objects can only be : orange juice, red wine, milk, iced tea, cola, tropical juice, juice pack, apple, pear, lemon, peach, banana, strawberry, orange, plum, cheezit, cornflakes, pringles, tuna, sugar, strawberry jello, tomato soup, mustard, chocolate jello, spam, coffee grounds, plate, fork, spoon, cup, knife, bowl, rubiks cube, soccer ball, dice, tennis ball, baseball, cleanser, sponge
        '''
        promt = promt2 + objects
        image_url = f"http://192.168.50.147:8888{'/uploads/GSPR.jpg'}"
        print("Fetching image from:", image_url)
        image_response = requests.get(image_url)

        image_array = np.frombuffer(image_response.content, dtype=np.uint8)

        # Decode the image using OpenCV
        img = cv2.imdecode(image_array, cv2.IMREAD_COLOR)
        # Save the image using OpenCV
        cv2.imwrite(pathnum + "Robot_view.jpg", img)
        print("Image saved successfully using OpenCV!")
        # Configuration and setup
        genai.configure(api_key='AIzaSyBdTRu-rcBKbf86gjiMNtezBu1dEuxrWyE')  # Replace with your actual API key
        model = genai.GenerativeModel("gemini-2.0-flash")
        path_sample = "C:/Users/rayso/Desktop/python/Robot_view.jpg"  # Use raw string to handle backslashes
        # Prepare the prompt template
        sample_txt = promt
        img = PIL.Image.open(path_sample)
        response = model.generate_content([img, sample_txt])
        file_data_string = response.text
        print(file_data_string)
        questions = {
            "Question1": "None",
            "Question2": "None",
            "Question3": "None",
            "Steps": 10,
            "Voice": file_data_string,
            "Questionasking": "None",
            "answer": "None"
        }
        api_url = "http://192.168.50.147:8888/Fambot"
        response = requests.post(api_url, json=questions)
        result = response.json()
        print(result)
        time.sleep(2)
    elif dictt["Steps"] == "Enumeration":
        promt2 = dictt["Voice"].lower()
        if "food" in promt2 or "kitchen" in promt2 or "item" in promt2:
            promt1 = '''
                    (Category)         (Object)
                    drinks:            orange juice, red wine, milk, iced tea, cola, tropical juice, juice pack
                    fruits:            apple, pear, lemon, peach, banana, strawberry, orange, plum
                    snacks:            cheezit, cornflakes, pringles
                    foods:             tuna, sugar, strawberry jello, tomato soup, mustard, chocolate jello, spam, coffee grounds
                    dishes:            plate, fork, spoon, cup, knife, bowl
                    toys:              rubiks cube, soccer ball, dice, tennis ball, baseball
                    cleaning supplies: cleanser, sponge
                    
                    '''
            promt = promt1 + promt2
        else:
            promt = promt2

        image_url = f"http://192.168.50.147:8888{'/uploads/GSPR.jpg'}"
        print("Fetching image from:", image_url)
        image_response = requests.get(image_url)

        image_array = np.frombuffer(image_response.content, dtype=np.uint8)

        # Decode the image using OpenCV
        img = cv2.imdecode(image_array, cv2.IMREAD_COLOR)
        # Save the image using OpenCV
        cv2.imwrite(pathnum + "Robot_view.jpg", img)
        print("Image saved successfully using OpenCV!")
        # Configuration and setup
        genai.configure(api_key='AIzaSyBdTRu-rcBKbf86gjiMNtezBu1dEuxrWyE')  # Replace with your actual API key
        model = genai.GenerativeModel("gemini-2.0-flash")
        path_sample = "C:/Users/rayso/Desktop/python/Robot_view.jpg"  # Use raw string to handle backslashes
        # Prepare the prompt template
        sample_txt = promt
        img = PIL.Image.open(path_sample)
        response = model.generate_content([img, sample_txt])
        file_data_string = response.text
        print(file_data_string)
        questions = {
            "Question1": "None",
            "Question2": "None",
            "Question3": "None",
            "Steps": 10,
            "Voice": file_data_string,
            "Questionasking": "None",
            "answer": "None"
        }
        api_url = "http://192.168.50.147:8888/Fambot"
        response = requests.post(api_url, json=questions)
        result = response.json()
        print(result)
        time.sleep(2)

    elif dictt["Steps"] == "color":
        sample_txt = dictt["Questionasking"].lower()
        image_url = f"http://192.168.50.147:8888{'/uploads/GSPR_color.jpg'}"
        print("Fetching image from:", image_url)
        image_response = requests.get(image_url)
        image_array = np.frombuffer(image_response.content, dtype=np.uint8)
        # Decode the image using OpenCV
        img = cv2.imdecode(image_array, cv2.IMREAD_COLOR)
        # Save the image using OpenCV
        cv2.imwrite(pathnum + "Robot_view.jpg", img)
        print("Image saved successfully using OpenCV!")
        # Configuration and setup
        genai.configure(api_key='AIzaSyBdTRu-rcBKbf86gjiMNtezBu1dEuxrWyE')  # Replace with your actual API key
        model = genai.GenerativeModel("gemini-2.0-flash")
        path_sample = "C:/Users/rayso/Desktop/python/Robot_view.jpg"  # Use raw string to handle backslashes
        # Prepare the prompt template
        img = PIL.Image.open(path_sample)
        response = model.generate_content([img, sample_txt])
        file_data_string = response.text
        print(file_data_string)
        file_data_string=file_data_string.replace("**","")
        questions = {
            "Question1": "None",
            "Question2": "None",
            "Question3": "None",
            "Steps": 12,
            "Voice": file_data_string,
            "Questionasking": "None",
            "answer": "None"
        }
        api_url = "http://192.168.50.147:8888/Fambot"
        response = requests.post(api_url, json=questions)
        result = response.json()
        print(result)
        time.sleep(2)
