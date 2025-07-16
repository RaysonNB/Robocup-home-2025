import cv2
import numpy as np
import time
import math

# --- Parameters ---
IMG_WIDTH, IMG_HEIGHT = 2560, 1600
# IMG_WIDTH, IMG_HEIGHT = 1280, 768 # You can switch back and it will still work
CENTER = (IMG_WIDTH // 2, IMG_HEIGHT // 2)

# --- Resolution Scaling ---
# Define the base resolution the character was designed for.
BASE_WIDTH, BASE_HEIGHT = 1280.0, 768.0
# Calculate scaling factors.
SCALE_X = IMG_WIDTH / BASE_WIDTH
SCALE_Y = IMG_HEIGHT / BASE_HEIGHT
# Use a general scale for non-axis-specific things like radius and thickness
SCALE = min(SCALE_X, SCALE_Y)


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

# --- Simplified Emotion Pupil Shapes (width, height) - Will be scaled at runtime ---
EMOTION_SHAPES = {
    "neutral":  (60, 100),
    "happy":    (65, 95),
    "sad":      (65, 95),
    "surprise": (70, 70),
    "cool":     (70, 45),
}

# --- Mouth Parameters (Scaled at runtime) ---
MOUTH_Y_POS_OFFSET = 180
MOUTH_THICKNESS_BASE = 12

# --- Drawing Helpers ---
def lerp(a, b, t):
    """Linear interpolation."""
    return a + (b - a) * t

def draw_rounded_rectangle(img, pt1, pt2, color, radius, thickness=-1, outline_color=None, outline_width=0):
    """Draw a rounded rectangle with an optional outline."""
    x1, y1 = pt1
    x2, y2 = pt2

    # Ensure radius is not larger than half the rectangle's dimensions
    radius = int(min(radius, (x2 - x1) / 2, (y2 - y1) / 2))
    if radius < 0: radius = 0

    # Draw the outline first by drawing a larger version behind the main shape
    if outline_color is not None and outline_width > 0:
        o = int(outline_width) # Outline thickness
        draw_rounded_rectangle(img, (x1-o, y1-o), (x2+o, y2+o), outline_color, radius + o, -1)

    # Draw the main filled rounded rectangle
    cv2.circle(img, (x1 + radius, y1 + radius), radius, color, -1)
    cv2.circle(img, (x2 - radius, y1 + radius), radius, color, -1)
    cv2.circle(img, (x1 + radius, y2 - radius), radius, color, -1)
    cv2.circle(img, (x2 - radius, y2 - radius), radius, color, -1)
    cv2.rectangle(img, (x1 + radius, y1), (x2 - radius, y2), color, -1)
    cv2.rectangle(img, (x1, y1 + radius), (x2, y2 - radius), color, -1)

def draw_mouth(image, current_time, head_center_y):
    """Draws the character's mouth based on emotion and talking state."""
    global mouth_openness

    # The mouth is now positioned relative to the head's vertical center
    mouth_y_pos = head_center_y + int(MOUTH_Y_POS_OFFSET * SCALE_Y)
    mouth_thickness = int(MOUTH_THICKNESS_BASE * SCALE)

    target_openness = 1.0 if is_talking else 0.0
    mouth_openness = lerp(mouth_openness, target_openness, 0.5)

    if mouth_openness > 0.05:
        talk_anim = (math.sin(current_time * 30) + 1) / 2
        
        mouth_w = int(lerp(40 * SCALE_X, 80 * SCALE_X, mouth_openness))
        mouth_h = int(lerp(5 * SCALE_Y, 55 * SCALE_Y, mouth_openness) * talk_anim) + int(3 * SCALE)
        
        mouth_center = (CENTER[0], mouth_y_pos)
        
        cv2.ellipse(image, mouth_center, (mouth_w // 2 + int(4*SCALE), mouth_h // 2 + int(4*SCALE)), 0, 0, 360, COLOR_OUTLINE, -1)
        cv2.ellipse(image, mouth_center, (mouth_w // 2, mouth_h // 2), 0, 0, 360, COLOR_FACE_VOID, -1)
        return

    mouth_center_x = CENTER[0]
    
    if current_emotion == 'happy':
        cv2.ellipse(image, (mouth_center_x, mouth_y_pos - int(15 * SCALE_Y)), (int(60 * SCALE_X), int(45 * SCALE_Y)), 0, 0, 180, COLOR_OUTLINE, mouth_thickness)
    elif current_emotion == 'sad':
        cv2.ellipse(image, (mouth_center_x, mouth_y_pos + int(15 * SCALE_Y)), (int(50 * SCALE_X), int(35 * SCALE_Y)), 0, 180, 360, COLOR_OUTLINE, mouth_thickness)
    elif current_emotion == 'surprise':
        cv2.ellipse(image, (mouth_center_x, mouth_y_pos), (int(25 * SCALE_X + 2*SCALE), int(40 * SCALE_Y + 2*SCALE)), 0, 0, 360, COLOR_OUTLINE, -1)
        cv2.ellipse(image, (mouth_center_x, mouth_y_pos), (int(25 * SCALE_X), int(40 * SCALE_Y)), 0, 0, 360, COLOR_FACE_VOID, -1)
    elif current_emotion == 'cool':
        pt1 = (mouth_center_x - int(45 * SCALE_X), mouth_y_pos + int(5 * SCALE_Y))
        pt2 = (mouth_center_x + int(45 * SCALE_X), mouth_y_pos - int(5 * SCALE_Y))
        cv2.line(image, pt1, pt2, COLOR_OUTLINE, mouth_thickness)
    else: # Neutral
        cv2.line(image, (mouth_center_x - int(40 * SCALE_X), mouth_y_pos), (mouth_center_x + int(40 * SCALE_X), mouth_y_pos), COLOR_OUTLINE, mouth_thickness)

def draw_character(image):
    """Draws the character's head, hat, glasses, and mouth."""
    global is_blinking, blink_end_time, next_blink_time
    global next_saccade_time, look_target, current_emotion, previous_emotion, transition_start_time

    current_time = time.time()
    # Define a single vertical anchor for the entire character, moved up by 10px
    HEAD_CENTER_Y = IMG_HEIGHT // 2 - 10

    # --- Head and Hoodie (All values scaled and relative to HEAD_CENTER_Y) ---
    head_outline_radius = int(330 * SCALE)
    head_fill_radius = int(320 * SCALE)
    cv2.ellipse(image, (CENTER[0], HEAD_CENTER_Y + int(50 * SCALE_Y)), (head_outline_radius, head_outline_radius), 0, 0, 360, COLOR_OUTLINE, -1)
    cv2.ellipse(image, (CENTER[0], HEAD_CENTER_Y + int(50 * SCALE_Y)), (head_fill_radius, head_fill_radius), 0, 0, 360, COLOR_HOODIE, -1)

    # --- Hat ---
    # Position the hat relative to the main head center Y
    hat_y_offset = -200 # Base offset in pixels from the head's anchor
    hat_center_y = HEAD_CENTER_Y + int(hat_y_offset * SCALE_Y)
    outline_width = int(10 * SCALE)

    # Hat Brim (Purple)
    brim_w = int(500 * SCALE_X)
    brim_h = int(70 * SCALE_Y)
    brim_radius = int(35 * SCALE)
    brim_pt1 = (CENTER[0] - brim_w // 2, hat_center_y - brim_h // 2)
    brim_pt2 = (CENTER[0] + brim_w // 2, hat_center_y + brim_h // 2)
    draw_rounded_rectangle(image, brim_pt1, brim_pt2, COLOR_HAT_PURPLE, brim_radius, outline_color=COLOR_OUTLINE, outline_width=outline_width)

    # Hat Top (Yellow)
    top_w = int(250 * SCALE_X)
    top_h = int(220 * SCALE_Y)
    # Position the top part so it slightly overlaps the brim
    top_center_y = hat_center_y - int(30 * SCALE_Y)
    top_center = (CENTER[0], top_center_y)
    # Draw outline first, then fill
    cv2.ellipse(image, top_center, (top_w // 2 + outline_width, top_h // 2 + outline_width), 0, 180, 360, COLOR_OUTLINE, -1, cv2.LINE_AA)
    cv2.ellipse(image, top_center, (top_w // 2, top_h // 2), 0, 180, 360, COLOR_HAT_YELLOW, -1, cv2.LINE_AA)


    # --- Glasses (All values scaled and relative to HEAD_CENTER_Y) ---
    face_y = HEAD_CENTER_Y + int(20 * SCALE_Y)
    
    # Blinking and Saccade logic
    if current_time > next_blink_time and not is_blinking:
        is_blinking = True
        blink_end_time = current_time + 0.15
    if is_blinking and current_time > blink_end_time:
        is_blinking = False
        next_blink_time = current_time + np.random.uniform(1.5, 6)
    if current_time > next_saccade_time:
        look_target = (np.random.uniform(-40, 40) * SCALE_X, np.random.uniform(-30, 30) * SCALE_Y)
        next_saccade_time = current_time + np.random.uniform(0.8, 2.5)
        
    glasses_y = face_y - int(20 * SCALE_Y)
    frame_w, frame_h = int(220 * SCALE_X), int(180 * SCALE_Y)
    spacing = int(130 * SCALE_X)
    
    # Left Lens
    draw_rounded_rectangle(image, (CENTER[0]-spacing-frame_w//2, glasses_y-frame_h//2), (CENTER[0]-spacing+frame_w//2, glasses_y+frame_h//2), COLOR_GLASSES_MAGENTA, int(40 * SCALE), outline_color=COLOR_OUTLINE, outline_width=int(10*SCALE))
    draw_rounded_rectangle(image, (CENTER[0]-spacing-int(180*SCALE_X)//2, glasses_y-int(140*SCALE_Y)//2), (CENTER[0]-spacing+int(180*SCALE_X)//2, glasses_y+int(140*SCALE_Y)//2), COLOR_FACE_VOID, int(25 * SCALE))
    
    # Right Lens
    draw_rounded_rectangle(image, (CENTER[0]+spacing-frame_w//2, glasses_y-frame_h//2), (CENTER[0]+spacing+frame_w//2, glasses_y+frame_h//2), COLOR_GLASSES_MAGENTA, int(40 * SCALE), outline_color=COLOR_OUTLINE, outline_width=int(10*SCALE))
    draw_rounded_rectangle(image, (CENTER[0]+spacing-int(180*SCALE_X)//2, glasses_y-int(140*SCALE_Y)//2), (CENTER[0]+spacing+int(180*SCALE_X)//2, glasses_y+int(140*SCALE_Y)//2), COLOR_FACE_VOID, int(25 * SCALE))
    
    # Bridge
    cv2.rectangle(image, (CENTER[0]-spacing+frame_w//2-int(30*SCALE_X), glasses_y-int(20*SCALE_Y)), (CENTER[0]+spacing-frame_w//2+int(30*SCALE_X), glasses_y+int(20*SCALE_Y)), COLOR_GLASSES_MAGENTA, -1)
    
    # --- Pupil Drawing (All values scaled and relative to glasses_y) ---
    if not is_blinking:
        time_in_transition = current_time - transition_start_time
        t = min(time_in_transition / TRANSITION_DURATION, 1.0)
        t = (1 - math.cos(t * math.pi)) / 2 # Ease-in-out

        start_shape = EMOTION_SHAPES.get(previous_emotion, EMOTION_SHAPES["neutral"])
        end_shape = EMOTION_SHAPES.get(current_emotion, EMOTION_SHAPES["neutral"])
        
        pupil_w_base = lerp(start_shape[0], end_shape[0], t)
        pupil_h_base = lerp(start_shape[1], end_shape[1], t)
        
        pupil_w = int(pupil_w_base * SCALE_X)
        pupil_h = int(pupil_h_base * SCALE_Y)
        
        talk_bounce = (math.sin(current_time * 40) * 8 * SCALE_Y) if is_talking else 0
        
        pupil_centers = [
            (CENTER[0] - spacing + int(look_target[0]), glasses_y + int(look_target[1]) + int(talk_bounce)),
            (CENTER[0] + spacing + int(look_target[0]), glasses_y + int(look_target[1]) + int(talk_bounce))
        ]

        for p_center in pupil_centers:
            outline_size = int(4 * SCALE)
            cv2.ellipse(image, p_center, (pupil_w//2 + outline_size, pupil_h//2 + outline_size), 0, 0, 360, COLOR_OUTLINE, -1, cv2.LINE_AA)
            cv2.ellipse(image, p_center, (pupil_w//2, pupil_h//2), 0, 0, 360, COLOR_PUPIL, -1, cv2.LINE_AA)

            if current_emotion == 'happy':
                cv2.rectangle(image, (p_center[0] - pupil_w//2, p_center[1]), (p_center[0] + pupil_w//2, p_center[1] + pupil_h//2), COLOR_FACE_VOID, -1)
            elif current_emotion == 'sad':
                cv2.rectangle(image, (p_center[0] - pupil_w//2, p_center[1] - pupil_h//2), (p_center[0] + pupil_w//2, p_center[1]), COLOR_FACE_VOID, -1)
    
    # Pass the head's Y center to the mouth drawing function
    draw_mouth(image, current_time, HEAD_CENTER_Y)


def main():
    """Main function to run the application."""
    global current_emotion, previous_emotion, transition_start_time, next_emotion_change_time, is_talking
    
    window_name = "Character Face"
    cv2.namedWindow(window_name, cv2.WND_PROP_FULLSCREEN)
    cv2.setWindowProperty(window_name, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)

    while True:
        current_time = time.time()
        
        # Change emotion randomly if not talking
        if not is_talking and current_time > next_emotion_change_time:
            possible_emotions = [e for e in RANDOM_EMOTIONS if e != current_emotion]
            new_emotion = np.random.choice(possible_emotions)
            
            previous_emotion = current_emotion
            current_emotion = new_emotion
            transition_start_time = time.time()
            next_emotion_change_time = current_time + np.random.uniform(3.0, 7.0)

        # Create a new frame for this iteration
        frame = np.full((IMG_HEIGHT, IMG_WIDTH, 3), COLOR_BG, dtype=np.uint8)
        draw_character(frame)

        # Scale fonts and text outlines for UI
        font_scale = 1.0 * SCALE
        font_thickness = int(2 * SCALE)
        outline_thickness = int(6 * SCALE)
        
        status_text = "Talking..." if is_talking else f"Emotion: {current_emotion.capitalize()}"
        cv2.putText(frame, status_text, (20, 50), cv2.FONT_HERSHEY_DUPLEX, font_scale, (10,10,10), outline_thickness, cv2.LINE_AA)
        cv2.putText(frame, status_text, (20, 50), cv2.FONT_HERSHEY_DUPLEX, font_scale, (255,255,255), font_thickness, cv2.LINE_AA)

        help_font_scale = 0.7 * SCALE
        help_font_thickness = int(1 * SCALE)
        help_outline_thickness = int(6 * SCALE)
        cv2.putText(frame, "Press [T] to talk | [Q] to quit", (20, IMG_HEIGHT - 20), cv2.FONT_HERSHEY_DUPLEX, help_font_scale, (10,10,10), help_outline_thickness, cv2.LINE_AA)
        cv2.putText(frame, "Press [T] to talk | [Q] to quit", (20, IMG_HEIGHT - 20), cv2.FONT_HERSHEY_DUPLEX, help_font_scale, (255,255,255), help_font_thickness, cv2.LINE_AA)
        
        cv2.imshow(window_name, frame)

        key = cv2.waitKey(16) & 0xFF
        if key == ord('q'):
            break
        
        if key == ord('t'):
            is_talking = not is_talking
            # If we just stopped talking, schedule the next emotion change
            if not is_talking:
                next_emotion_change_time = time.time() + np.random.uniform(2.0, 5.0)
                
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
