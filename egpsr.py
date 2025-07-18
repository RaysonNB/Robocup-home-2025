from openai import OpenAI
import numpy as np

def get_bboxes(
    img: np.ndarray,
    prompt: str,
    api_client: OpenAI
):
    """
    Uses Gemini to detect bounding boxes, validating the output with Pydantic.

    Args:
        img (np.ndarray): The input image in cv2 format (NumPy array).
        prompt (str): The text prompt describing what to detect (e.g., "the car").
        api_client (OpenAI): The initialized OpenAI client instance.

    Returns:
        Optional[BoundingBoxDetections]: A Pydantic object containing a list
        of validated bounding boxes, or None if an error occurs.
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
        "Output a json list where each entry contains the 2D bounding box in \"box_2d\" and a text label in \"label\"."
        "The 'box_2d' should be an array of [ymin, xmin, ymax, xmax] with coordinates normalized to 0-1000. "
    )
    
    user_prompt = f"Detect {prompt} in the image"

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
