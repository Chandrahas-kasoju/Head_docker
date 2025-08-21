import pygame
import random
import sys

# --- Configuration Settings ---
# You can easily adjust these values to customize your robot's look and feel.

# Set the screen dimensions.
SCREEN_WIDTH = 1080
SCREEN_HEIGHT = 1080

# Colors (RGB format)
BACKGROUND_COLOR = (0, 0, 0)          # Black
FACE_COLOR = (0, 200, 255)       # Bright Cyan

# Eye Shape & Position
EYE_SIZE = 180                  # The width and normal height of the eyes
EYE_SPACING = 250               # Distance between the centers of the two eyes
EYE_BORDER_RADIUS = 25              # The roundness of the corners

# Mouth Shape & Position
MOUTH_WIDTH = 280
MOUTH_HEIGHT = 40
MOUTH_VERTICAL_OFFSET = 160     # How far below the center the mouth is
MOUTH_BORDER_RADIUS = 10

# Animation Timings for Smooth Blinking
BLINK_INTERVAL_MIN = 3.0        # Minimum seconds between blinks
BLINK_INTERVAL_MAX = 7.0        # Maximum seconds between blinks
BLINK_ANIMATION_SPEED = 0.075   # How fast the "eyelids" close and open

# --- End of Configuration ---

def main():
    """Main function to initialize and run the face animation."""
    pygame.init()
    pygame.mixer.quit() # Prevents common ALSA errors on Linux

    # Use pygame.NOFRAME for a borderless window, which is highly compatible.
    screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT), pygame.NOFRAME)
    pygame.display.set_caption("Robot Face")
    pygame.mouse.set_visible(False)
    clock = pygame.time.Clock()

    # --- Positioning Calculations ---
    screen_center_x = SCREEN_WIDTH // 2
    screen_center_y = SCREEN_HEIGHT // 2
    left_eye_center = (screen_center_x - (EYE_SPACING // 2), screen_center_y)
    right_eye_center = (screen_center_x + (EYE_SPACING // 2), screen_center_y)
    mouth_rect = pygame.Rect(0, 0, MOUTH_WIDTH, MOUTH_HEIGHT)
    mouth_rect.center = (screen_center_x, screen_center_y + MOUTH_VERTICAL_OFFSET)

    # --- Animation State ---
    eye_state = 'OPEN'
    animation_start_time = 0
    next_blink_trigger_time = pygame.time.get_ticks() + random.uniform(BLINK_INTERVAL_MIN, BLINK_INTERVAL_MAX) * 1000
    
    # --- Main Loop ---
    running = True
    while running:
        # Event handling (to quit with ESC key or by closing the window)
        for event in pygame.event.get():
            if event.type == pygame.QUIT or (event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE):
                running = False

        current_time = pygame.time.get_ticks()

        # --- Blinking Logic ---
        if eye_state == 'OPEN':
            current_eye_height = EYE_SIZE
            if current_time >= next_blink_trigger_time:
                eye_state = 'CLOSING'
                animation_start_time = current_time
        elif eye_state == 'CLOSING':
            progress = (current_time - animation_start_time) / (BLINK_ANIMATION_SPEED * 1000)
            if progress >= 1.0:
                eye_state = 'OPENING'
                animation_start_time = current_time
                current_eye_height = 0
            else:
                current_eye_height = EYE_SIZE * (1.0 - progress)
        elif eye_state == 'OPENING':
            progress = (current_time - animation_start_time) / (BLINK_ANIMATION_SPEED * 1000)
            if progress >= 1.0:
                eye_state = 'OPEN'
                next_blink_trigger_time = current_time + random.uniform(BLINK_INTERVAL_MIN, BLINK_INTERVAL_MAX) * 1000
                current_eye_height = EYE_SIZE
            else:
                current_eye_height = EYE_SIZE * progress

        # --- Drawing Frame by Frame ---
        screen.fill(BACKGROUND_COLOR)

        if current_eye_height > 0:
            left_eye_rect = pygame.Rect(0, 0, EYE_SIZE, current_eye_height)
            right_eye_rect = pygame.Rect(0, 0, EYE_SIZE, current_eye_height)
            left_eye_rect.center = left_eye_center
            right_eye_rect.center = right_eye_center
            pygame.draw.rect(screen, FACE_COLOR, left_eye_rect, border_radius=EYE_BORDER_RADIUS)
            pygame.draw.rect(screen, FACE_COLOR, right_eye_rect, border_radius=EYE_BORDER_RADIUS)
        
        pygame.draw.rect(screen, FACE_COLOR, mouth_rect, border_radius=MOUTH_BORDER_RADIUS)
        
        pygame.display.flip()
        clock.tick(60)

    # --- Cleanup ---
    pygame.quit()
    print("Robot face program has shut down.")
    sys.exit()

if __name__ == '__main__':
    main()