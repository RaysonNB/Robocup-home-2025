from openai import OpenAI
import numpy as np

def nearest_nonzero(img, x, y):
    ys, xs = np.nonzero(img[y-7:y+8, x-7:x+8])
    if ys.size == 0: 
        return None
    xs += x - 7; ys += y - 7
    i = ((xs - x)**2 + (ys - y)**2).argmin()
    return xs[i], ys[i]


Here is a brief description of each room's features based on the provided floor plan:

Bedroom: This room contains a large bed positioned against the top wall, with a bedside table to its right. A plant is in the top-left corner, and a side table is against the bottom wall.
Office: The office is set up with a desk and three chairs in the upper right corner, next to a trash bin and a plant. A bar is located against the wall separating the office from the bedroom.
Kitchen: On the left, there is a round kitchen table with four chairs. The area to the right includes a sink placed between a dishwasher and a microwave. Along the bottom wall, next to the exit, are a refrigerator, a shelf, and a waste basket.
Living Room: This room features a sofa against the right wall, facing two seats. A cabinet is placed near the main entry, and a TV stand is against the wall shared with the kitchen.
client = OpenAI(
    api_key="AIzaSyCYSxtdvQnullmuGnJqI49kwGlqT1ZAmpo",
    base_url="https://generativelanguage.googleapis.com/v1beta/openai/"
)

cout_location = {
    "bedroom": [3.020, 3.555, 1.109],
    "kitchen": [2.732, 3.355, 2.145],
    "office": [2.216, 3.961, -0.646],
    "living room": [3.644, 3.931, -2.409]
}

response = client.chat.completions.create(
    model="gemini-2.5-flash",
    reasoning_effort="none",
    messages=[
        {"role": "system", "content": "You are a helpful assistant."},
        {
            "role": "user",
            "content": "Explain to me how AI works"
        }
    ]
)

print(response.choices[0].message)
