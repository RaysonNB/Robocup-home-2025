import cv2
import numpy as np
import time
import math

# --- Parameters ---
IMG_WIDTH, IMG_HEIGHT = 1000, 1000
CENTER = (IMG_WIDTH // 2, IMG_HEIGHT // 2)

# --- Color Palette (Sampled from the image) ---
COLOR_BG = (216, 178, 0)  # Cyan-Blue background
COLOR_HOODIE = (231, 226, 225)
COLOR_HAT_PURPLE = (207, 60, 142)
COLOR_HAT_YELLOW = (36, 209, 253)
COLOR_GLASSES_MAGENTA = (137, 17, 208)
COLOR_FACE_VOID = (20, 20, 20)
COLOR_PUPIL = (250, 250, 250)
COLOR_OUTLINE = (10, 10, 10)

# --- Animation & Interaction State ---
is_blinking = False
blink_end_time = 0
next_blink_time = time.time() + np.random.uniform(1, 5)

is_talking = False
mouth_openness = 0.0 # For smoothing mouth animation

next_saccade_time = 0
look_target = (0, 0)

current_emotion = "neutral"
previous_emotion = "neutral"
transition_start_time = 0
TRANSITION_DURATION = 0.4

next_emotion_change_time = time.time() + np.random.uniform(3, 7)
RANDOM_EMOTIONS = ["neutral", "neutral", "happy", "happy", "cool", "sad", "surprise"]

# --- Simplified Emotion Pupil Shapes (width, height) ---
EMOTION_SHAPES = {
    "neutral":  (60, 100),
    "happy":    (65, 95),
    "sad":      (65, 95),
    "surprise": (70, 70),  # Becomes a circle
    "cool":     (70, 45),  # Squinted
}

# --- Mouth Parameters ---
MOUTH_Y_POS = CENTER[1] + 180  # Vertical position of the mouth
MOUTH_THICKNESS = 12

# --- Drawing Helpers ---
def lerp(a, b, t):
    """Linear interpolation."""
    return a + (b - a) * t

def draw_rounded_rectangle(img, pt1, pt2, color, radius, thickness=-1, outline_color=None, outline_width=0):
    """Draw a rounded rectangle."""
    x1, y1 = pt1
    x2, y2 = pt2
    
    if outline_color is not None and outline_width > 0:
        o = outline_width // 2
        draw_rounded_rectangle(img, (x1-o, y1-o), (x2+o, y2+o), outline_color, radius + o, -1)

    cv2.circle(img, (x1 + radius, y1 + radius), radius, color, -1)
    cv2.circle(img, (x2 - radius, y1 + radius), radius, color, -1)
    cv2.circle(img, (x1 + radius, y2 - radius), radius, color, -1)
    cv2.circle(img, (x2 - radius, y2 - radius), radius, color, -1)
    cv2.rectangle(img, (x1 + radius, y1), (x2 - radius, y2), color, -1)
    cv2.rectangle(img, (x1, y1 + radius), (x2, y2 - radius), color, -1)

def draw_mouth(image, current_time):
    """Draws the character's mouth based on emotion and talking state."""
    global mouth_openness

    # Smoothly interpolate the mouth's openness based on the talking state
    target_openness = 1.0 if is_talking else 0.0
    mouth_openness = lerp(mouth_openness, target_openness, 0.5)  # 0.5 controls how fast it opens/closes

    # --- Talking Animation ---
    if mouth_openness > 0.05:  # Use a threshold to draw
        # Use a sine wave to make the mouth bob up and down while talking
        talk_anim = (math.sin(current_time * 30) + 1) / 2  # A 0-1 value
        
        # Interpolate width and height based on both overall openness and the bobbing animation
        mouth_w = int(lerp(40, 80, mouth_openness))
        mouth_h = int(lerp(5, 55, mouth_openness) * talk_anim) + 3  # Ensure it's never fully closed while talking
        
        mouth_center = (CENTER[0], MOUTH_Y_POS)

        # Draw the mouth (outline and fill)
        cv2.ellipse(image, mouth_center, (mouth_w // 2 + 4, mouth_h // 2 + 4), 0, 0, 360, COLOR_OUTLINE, -1)
        cv2.ellipse(image, mouth_center, (mouth_w // 2, mouth_h // 2), 0, 0, 360, COLOR_FACE_VOID, -1)
        return  # Skip emotional mouths if talking

    # --- Emotion-based Mouths (when not talking) ---
    mouth_center_x, mouth_center_y = CENTER[0], MOUTH_Y_POS

    if current_emotion == 'happy':
        # Wide U-shaped smile
        cv2.ellipse(image, (mouth_center_x, mouth_center_y - 15), (60, 45), 0, 0, 180, COLOR_OUTLINE, MOUTH_THICKNESS)
    elif current_emotion == 'sad':
        # Downward arc for a frown
        cv2.ellipse(image, (mouth_center_x, mouth_center_y + 15), (50, 35), 0, 180, 360, COLOR_OUTLINE, MOUTH_THICKNESS)
    elif current_emotion == 'surprise':
        # A small, open 'O' mouth
        cv2.ellipse(image, (mouth_center_x, mouth_center_y), (25 + 2, 40 + 2), 0, 0, 360, COLOR_OUTLINE, -1)
        cv2.ellipse(image, (mouth_center_x, mouth_center_y), (25, 40), 0, 0, 360, COLOR_FACE_VOID, -1)
    elif current_emotion == 'cool':
         # A slight smirk
        pt1 = (mouth_center_x - 45, mouth_center_y + 5)
        pt2 = (mouth_center_x + 45, mouth_center_y - 5)
        cv2.line(image, pt1, pt2, COLOR_OUTLINE, MOUTH_THICKNESS)
    else:  # Neutral
        # A simple straight line
        cv2.line(image, (mouth_center_x - 40, mouth_center_y), (mouth_center_x + 40, mouth_center_y), COLOR_OUTLINE, MOUTH_THICKNESS)


def draw_character(image):
    """Draws the character's head, hat, glasses, and mouth."""
    global is_blinking, blink_end_time, next_blink_time
    global next_saccade_time, look_target, current_emotion, previous_emotion, transition_start_time

    current_time = time.time()
    
    # Define vertical anchor point to center the head
    HEAD_CENTER_Y = IMG_HEIGHT // 2

    # --- Head and Hat ---
    # Head (Hoodie Shape)
    cv2.ellipse(image, (CENTER[0], HEAD_CENTER_Y + 50), (330, 330), 0, 0, 360, COLOR_OUTLINE, -1)
    cv2.ellipse(image, (CENTER[0], HEAD_CENTER_Y + 50), (320, 320), 0, 0, 360, COLOR_HOODIE, -1)
    
    # Hat
    hat_center_y = HEAD_CENTER_Y - 110
    
    draw_rounded_rectangle(image, (350, hat_center_y + 120), (650, hat_center_y + 165), COLOR_OUTLINE, 20 + 3)
    draw_rounded_rectangle(image, (350, hat_center_y + 120), (650, hat_center_y + 165), COLOR_HAT_PURPLE, 20)
    draw_rounded_rectangle(image, (530, hat_center_y + 130), (630, hat_center_y + 155), COLOR_OUTLINE, 5, -1)
    cv2.circle(image, (400, hat_center_y + 142), 18, COLOR_OUTLINE, -1)
    cv2.circle(image, (400, hat_center_y + 142), 13, COLOR_HAT_YELLOW, -1)
    cv2.circle(image, (CENTER[0], hat_center_y-160), 22, COLOR_OUTLINE, -1)
    cv2.circle(image, (CENTER[0], hat_center_y-160), 16, COLOR_HAT_YELLOW, -1)

    # --- Glasses ---
    face_y = HEAD_CENTER_Y + 20

    # Blinking and Saccade logic
    if current_time > next_blink_time and not is_blinking:
        is_blinking = True
        blink_end_time = current_time + 0.15
    if is_blinking and current_time > blink_end_time:
        is_blinking = False
        next_blink_time = current_time + np.random.uniform(1.5, 6)
    if current_time > next_saccade_time:
        look_target = (np.random.uniform(-40, 40), np.random.uniform(-30, 30))
        next_saccade_time = current_time + np.random.uniform(0.8, 2.5)
        
    glasses_y = face_y - 20
    frame_w, frame_h = 220, 180
    spacing = 130
    draw_rounded_rectangle(image, (CENTER[0]-spacing-frame_w//2, glasses_y-frame_h//2), (CENTER[0]-spacing+frame_w//2, glasses_y+frame_h//2), COLOR_OUTLINE, 40 + 3)
    draw_rounded_rectangle(image, (CENTER[0]-spacing-frame_w//2, glasses_y-frame_h//2), (CENTER[0]-spacing+frame_w//2, glasses_y+frame_h//2), COLOR_GLASSES_MAGENTA, 40)
    draw_rounded_rectangle(image, (CENTER[0]-spacing-180//2, glasses_y-140//2), (CENTER[0]-spacing+180//2, glasses_y+140//2), COLOR_FACE_VOID, 25)
    draw_rounded_rectangle(image, (CENTER[0]+spacing-frame_w//2, glasses_y-frame_h//2), (CENTER[0]+spacing+frame_w//2, glasses_y+frame_h//2), COLOR_OUTLINE, 40 + 3)
    draw_rounded_rectangle(image, (CENTER[0]+spacing-frame_w//2, glasses_y-frame_h//2), (CENTER[0]+spacing+frame_w//2, glasses_y+frame_h//2), COLOR_GLASSES_MAGENTA, 40)
    draw_rounded_rectangle(image, (CENTER[0]+spacing-180//2, glasses_y-140//2), (CENTER[0]+spacing+180//2, glasses_y+140//2), COLOR_FACE_VOID, 25)
    cv2.rectangle(image, (CENTER[0]-spacing+frame_w//2-30, glasses_y-20), (CENTER[0]+spacing-frame_w//2+30, glasses_y+20), COLOR_GLASSES_MAGENTA, -1)

    # --- Pupil Drawing ---
    if not is_blinking:
        time_in_transition = current_time - transition_start_time
        t = min(time_in_transition / TRANSITION_DURATION, 1.0)
        t = (1 - math.cos(t * math.pi)) / 2

        start_shape = EMOTION_SHAPES.get(previous_emotion, EMOTION_SHAPES["neutral"])
        end_shape = EMOTION_SHAPES.get(current_emotion, EMOTION_SHAPES["neutral"])
        
        pupil_w = int(lerp(start_shape[0], end_shape[0], t))
        pupil_h = int(lerp(start_shape[1], end_shape[1], t))
        
        talk_bounce = (math.sin(current_time * 40) * 8) if is_talking else 0
        
        pupil_centers = [
            (CENTER[0] - spacing + int(look_target[0]), glasses_y + int(look_target[1]) + int(talk_bounce)),
            (CENTER[0] + spacing + int(look_target[0]), glasses_y + int(look_target[1]) + int(talk_bounce))
        ]

        for center in pupil_centers:
            cv2.ellipse(image, center, (pupil_w//2 + 2, pupil_h//2 + 2), 0, 0, 360, COLOR_OUTLINE, -1, cv2.LINE_AA)
            cv2.ellipse(image, center, (pupil_w//2, pupil_h//2), 0, 0, 360, COLOR_PUPIL, -1, cv2.LINE_AA)

            if current_emotion == 'happy':
                cv2.rectangle(image, (center[0] - pupil_w//2, center[1]), (center[0] + pupil_w//2, center[1] + pupil_h//2), COLOR_FACE_VOID, -1)
            elif current_emotion == 'sad':
                cv2.rectangle(image, (center[0] - pupil_w//2, center[1] - pupil_h//2), (center[0] + pupil_w//2, center[1]), COLOR_FACE_VOID, -1)
    
    # --- Mouth Drawing ---
    draw_mouth(image, current_time)


def main():
    """Main function to run the application."""
    global current_emotion, previous_emotion, transition_start_time, next_emotion_change_time, is_talking
    
    window_name = "Character Face"
    cv2.namedWindow(window_name)

    while True:
        current_time = time.time()
        
        if not is_talking and current_time > next_emotion_change_time:
            possible_emotions = [e for e in RANDOM_EMOTIONS if e != current_emotion]
            new_emotion = np.random.choice(possible_emotions)
            
            previous_emotion = current_emotion
            current_emotion = new_emotion
            transition_start_time = time.time()
            next_emotion_change_time = current_time + np.random.uniform(3.0, 7.0)

        frame = np.full((IMG_HEIGHT, IMG_WIDTH, 3), COLOR_BG, dtype=np.uint8)
        draw_character(frame)

        status_text = "Talking..." if is_talking else f"Emotion: {current_emotion.capitalize()}"
        cv2.putText(frame, status_text, (20, 40), cv2.FONT_HERSHEY_DUPLEX, 1, (10,10,10), 6, cv2.LINE_AA)
        cv2.putText(frame, status_text, (20, 40), cv2.FONT_HERSHEY_DUPLEX, 1, (255,255,255), 2, cv2.LINE_AA)
        cv2.putText(frame, "Press [T] to talk | [Q] to quit", (20, IMG_HEIGHT - 20), cv2.FONT_HERSHEY_DUPLEX, 0.7, (10,10,10), 6, cv2.LINE_AA)
        cv2.putText(frame, "Press [T] to talk | [Q] to quit", (20, IMG_HEIGHT - 20), cv2.FONT_HERSHEY_DUPLEX, 0.7, (255,255,255), 1, cv2.LINE_AA)

        cv2.imshow(window_name, frame)

        key = cv2.waitKey(16) & 0xFF
        if key == ord('q'):
            break
        
        if key == ord('t'):
            is_talking = not is_talking
            if not is_talking:
                next_emotion_change_time = time.time() + np.random.uniform(2.0, 5.0)
                
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
