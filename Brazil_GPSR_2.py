import json
import time
import requests
import google.generativeai as genai
import PIL.Image
import cv2
import numpy as np

pathnum = r"C:/Users/rayso/Desktop/python/"
genai.configure(api_key='AIzaSyBdTRu-rcBKbf86gjiMNtezBu1dEuxrWyE')
model = genai.GenerativeModel("gemini-2.0-flash")
cnt_yy = 0

# all big promt
while True:
    while True:
        r = requests.get("http://172.20.10.5:8888/Fambot", timeout=2.5)
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
        *** room name will not be place name
        *** bedroom: bedside table, side table, bed
        *** kitchen: kitchen table, dishwasher, sink, microwave, waste basket, shelf, refrigerator
        *** office: trash bin, desk, bar
        *** living room: cabinet, sofa, seats
        I am in the instruction point
        *** please remember Manipulation1, Manipulation2 is only for graping object not for taking/graping person
        (The Sentence)(Task: Sentence Structure)(I given u)
        Manipulation1(only for object): Go to the $ROOM1, grasp the $OBJECT on the $PLACE1 and place it on the $PLACE2.
        Manipulation2(only for object): Go to the $ROOM1, grasp the $OBJECT on the $PLACE1 and give it to $PERSON on the $ROOM2.(if &PERSON is me than $ROOM2:"instruction point" just edit $ROOM2)
        Vision (Enumeration)1: Tell me how many $CATEGORY_OBJ here are on the $PLACE1.
        Vision (Enumeration)2: Tell me how many people in the $ROOM1 are $POSE/GESTURE.
        Vision (Description)1: Tell me what is the $OBJ_COMP object on the $PLACE1.
        Vision (Description)2: Tell me the $PERS_INFO of the person at the $PLACE1
        Navigation1: Go to the $ROOM1, find $POSE/GESTURE person and follow (him | her).
        Navigation2: Go to the $ROOM1, find $POSE/GESTURE person and guide (him|her) to the $ROOM2.
        Speech1: Go to the $ROOM1, find $PERSON at the $PLACE1 and answer (his | her) question.
        Speech2: Go to the $ROOM1, find the person who is $POSE/GESTURE and tell (him | her) $TELL_LIST.
        
        %possible information options
        %ROOM         : bedroom, kitchen, office, living room
        %PLACE        : bedside table, side table, bed, kitchen table, dishwasher, sink, microwave, waste basket, shelf, refrigerator, trash bin, desk, bar, tv stand, cabinet, sofa, seats
        $OBJECT       : mayo, tuna, ketchup, oats, broth, corn flower, peanuts, cornflakes, crisps, pringles, cheese snack, chocolate bar, gum balls, apple, lemon, tangerine, pear, spoon, plate, cup, fork, bowl, knife, cloth, polish, brush, sponge, coffee, kuat, milk, orange juice, fanta, coke
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
        api_url = "http://172.20.10.5:8888/Fambot"
        response = requests.post(api_url, json=questions)
        result = response.json()
        print(result)
        print("sent")
        time.sleep(2)
    elif dictt["Steps"] == "checkpeople":
        promt = dictt["Questionasking"] + " answer my question yes or no only"
        image_url = f"http://172.20.10.5:8888{'/uploads/GSPR_people.jpg'}"
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
        api_url = "http://172.20.10.5:8888/Fambot"
        response = requests.post(api_url, json=questions)
        result = response.json()
        print(result)
        time.sleep(2)
    elif dictt["Steps"] == "Description":
        promt2 = dictt["Voice"]
        objects = '''
        The Objects can only be: mayo, tuna, ketchup, oats, broth, corn flower, peanuts, cornflakes, crisps, pringles, cheese snack, chocolate bar, gum balls, apple, lemon, tangerine, pear, spoon, plate, cup, fork, bowl, knife, cloth, polish, brush, sponge, coffee, kuat, milk, orange juice, fanta, coke
        
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
        '''
        promt = promt2 + objects
        image_url = f"http://172.20.10.5:8888{'/uploads/GSPR.jpg'}"
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
        api_url = "http://172.20.10.5:8888/Fambot"
        response = requests.post(api_url, json=questions)
        result = response.json()
        print(result)
        time.sleep(2)
    elif dictt["Steps"] == "Enumeration":
        promt2 = dictt["Voice"].lower()
        if "food" in promt2 or "kitchen" in promt2 or "item" in promt2:
            promt1 = '''
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

                    just one sentence(15 words) is enough
                    
                    '''
            promt = promt1 + promt2
        else:
            promt = promt2

        image_url = f"http://172.20.10.5:8888{'/uploads/GSPR.jpg'}"
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
        api_url = "http://172.20.10.5:8888/Fambot"
        response = requests.post(api_url, json=questions)
        result = response.json()
        print(result)
        time.sleep(2)
    elif dictt["Steps"] == "color":
        sample_txt = dictt["Questionasking"].lower()
        sample_txt+=" just give me 20 words"
        image_url = f"http://172.20.10.5:8888{'/uploads/GSPR_color.jpg'}"
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
        file_data_string = file_data_string.replace("**", "")
        questions = {
            "Question1": "None",
            "Question2": "None",
            "Question3": "None",
            "Steps": 12,
            "Voice": file_data_string,
            "Questionasking": "None",
            "answer": "None"
        }
        api_url = "http://172.20.10.5:8888/Fambot"
        response = requests.post(api_url, json=questions)
        result = response.json()
        print(result)
        time.sleep(2)
