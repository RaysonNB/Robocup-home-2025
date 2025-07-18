from openai import OpenAI

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
