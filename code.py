import cv2
import numpy as np
import RPi.GPIO as GPIO
from RPi import PWM
from picamera.array import PiRGBArray
from picamera import PiCamera
import time

# Configurable parameters
FORWARD_SPEED = 75
MAX_TURN_SPEED = 75
CENTER_TOLERANCE = 20
POSTIT_MIN_AREA = 200
LOST_FRAMES_THRESHOLD = 20
HSV_LOWER = np.array([20, 125, 75])
HSV_UPPER = np.array([40, 255, 255])
FINISH_HSV_LOWER = np.array([0, 0, 200])  # Example: white post-it for finish
FINISH_HSV_UPPER = np.array([180, 30, 255])

# GPIO motor control setup
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Motor GPIO pin setup
LEFT_MOTOR_FORWARD = 17
LEFT_MOTOR_BACKWARD = 18
RIGHT_MOTOR_FORWARD = 22
RIGHT_MOTOR_BACKWARD = 23

GPIO.setup(LEFT_MOTOR_FORWARD, GPIO.OUT)
GPIO.setup(LEFT_MOTOR_BACKWARD, GPIO.OUT)
GPIO.setup(RIGHT_MOTOR_FORWARD, GPIO.OUT)
GPIO.setup(RIGHT_MOTOR_BACKWARD, GPIO.OUT)

# Set up PWM for each motor pin using RPi PWM library
left_motor_forward_pwm = PWM.PWM(LEFT_MOTOR_FORWARD, 100)
left_motor_backward_pwm = PWM.PWM(LEFT_MOTOR_BACKWARD, 100)
right_motor_forward_pwm = PWM.PWM(RIGHT_MOTOR_FORWARD, 100)
right_motor_backward_pwm = PWM.PWM(RIGHT_MOTOR_BACKWARD, 100)

left_motor_forward_pwm.start(0)
left_motor_backward_pwm.start(0)
right_motor_forward_pwm.start(0)
right_motor_backward_pwm.start(0)

last_action = None
lost_frames = 0

def move_forward():
    global last_action
    left_motor_forward_pwm.ChangeDutyCycle(75)
    left_motor_backward_pwm.ChangeDutyCycle(0)
    right_motor_forward_pwm.ChangeDutyCycle(75)
    right_motor_backward_pwm.ChangeDutyCycle(0)
    last_action = 'forward'
    print('Moving Forward')

def turn_left():
    global last_action
    left_motor_forward_pwm.ChangeDutyCycle(0)
    left_motor_backward_pwm.ChangeDutyCycle(50)
    right_motor_forward_pwm.ChangeDutyCycle(75)
    right_motor_backward_pwm.ChangeDutyCycle(0)
    last_action = 'left'
    print('Turning Left')

def turn_right():
    global last_action
    left_motor_forward_pwm.ChangeDutyCycle(75)
    left_motor_backward_pwm.ChangeDutyCycle(0)
    right_motor_forward_pwm.ChangeDutyCycle(0)
    right_motor_backward_pwm.ChangeDutyCycle(50)
    last_action = 'right'
    print('Turning Right')

def turn_left_in_place():
    left_motor_forward_pwm.ChangeDutyCycle(0)
    left_motor_backward_pwm.ChangeDutyCycle(75)
    right_motor_forward_pwm.ChangeDutyCycle(75)
    right_motor_backward_pwm.ChangeDutyCycle(0)
    print('Turning Left In Place')

def turn_right_in_place():
    left_motor_forward_pwm.ChangeDutyCycle(75)
    left_motor_backward_pwm.ChangeDutyCycle(0)
    right_motor_forward_pwm.ChangeDutyCycle(0)
    right_motor_backward_pwm.ChangeDutyCycle(75)
    print('Turning Right In Place')

def panic():
    if last_action == 'left':
        print('PANIC: Turning left in place')
        turn_left_in_place()
    elif last_action == 'right':
        print('PANIC: Turning right in place')
        turn_right_in_place()
    else:
        print('PANIC: Stopping')
        left_motor_forward_pwm.ChangeDutyCycle(0)
        left_motor_backward_pwm.ChangeDutyCycle(0)
        right_motor_forward_pwm.ChangeDutyCycle(0)
        right_motor_backward_pwm.ChangeDutyCycle(0)
    # Optionally, add a slow 360 turn logic here

def robot_kinematics(left_duty, right_duty):
    left_motor_forward_pwm.ChangeDutyCycle(left_duty)
    left_motor_backward_pwm.ChangeDutyCycle(0)
    right_motor_forward_pwm.ChangeDutyCycle(right_duty)
    right_motor_backward_pwm.ChangeDutyCycle(0)

# Utility functions

def find_largest_postit(mask):
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    largest = None
    max_area = 0
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area > POSTIT_MIN_AREA and area > max_area:
            largest = cnt
            max_area = area
    return largest

def get_postit_center(contour):
    M = cv2.moments(contour)
    if M['m00'] == 0:
        return None
    cx = int(M['m10'] / M['m00'])
    cy = int(M['m01'] / M['m00'])
    return cx, cy

def nothing(x):
    pass

cv2.namedWindow('Trackbars')
cv2.createTrackbar('LH', 'Trackbars', HSV_LOWER[0], 180, nothing)
cv2.createTrackbar('LS', 'Trackbars', HSV_LOWER[1], 255, nothing)
cv2.createTrackbar('LV', 'Trackbars', HSV_LOWER[2], 255, nothing)
cv2.createTrackbar('UH', 'Trackbars', HSV_UPPER[0], 180, nothing)
cv2.createTrackbar('US', 'Trackbars', HSV_UPPER[1], 255, nothing)
cv2.createTrackbar('UV', 'Trackbars', HSV_UPPER[2], 255, nothing)

# Camera setup
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(640, 480))
time.sleep(0.1)

print("Press CTRL+C to end the program.")
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    image = frame.array
    image_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    # Detect finish post-it
    finish_mask = cv2.inRange(image_hsv, FINISH_HSV_LOWER, FINISH_HSV_UPPER)
    finish_contour = find_largest_postit(finish_mask)
    if finish_contour is not None:
        print("Finish post-it detected! Stopping robot.")
        robot_kinematics(0, 0)
        break
    # Detect colored post-it
    lh = cv2.getTrackbarPos('LH', 'Trackbars')
    ls = cv2.getTrackbarPos('LS', 'Trackbars')
    lv = cv2.getTrackbarPos('LV', 'Trackbars')
    uh = cv2.getTrackbarPos('UH', 'Trackbars')
    us = cv2.getTrackbarPos('US', 'Trackbars')
    uv = cv2.getTrackbarPos('UV', 'Trackbars')
    HSV_LOWER = np.array([lh, ls, lv])
    HSV_UPPER = np.array([uh, us, uv])
    mask = cv2.inRange(image_hsv, HSV_LOWER, HSV_UPPER)
    mask = cv2.medianBlur(mask, 5)  # Reduce noise
    contour = find_largest_postit(mask)
    if contour is not None:
        lost_frames = 0
        cx, cy = get_postit_center(contour)
        center_x = mask.shape[1] // 2
        offset = cx - center_x
        max_offset = center_x
        turn_speed = int(MAX_TURN_SPEED * min(abs(offset) / max_offset, 1))
        if abs(offset) < CENTER_TOLERANCE:
            robot_kinematics(FORWARD_SPEED, FORWARD_SPEED)
            print('Moving Forward (centered horizontally)')
        elif offset < 0:
            robot_kinematics(FORWARD_SPEED - turn_speed, FORWARD_SPEED)
            print(f'Turning Left (speed {FORWARD_SPEED - turn_speed})')
        else:
            robot_kinematics(FORWARD_SPEED, FORWARD_SPEED - turn_speed)
            print(f'Turning Right (speed {FORWARD_SPEED - turn_speed})')
    else:
        lost_frames += 1
        if lost_frames > LOST_FRAMES_THRESHOLD:
            print('Post-it lost! Searching...')
            # Search pattern: alternate left/right turns
            if last_action == 'left':
                robot_kinematics(0, FORWARD_SPEED)
                print('Searching: Turning right')
            else:
                robot_kinematics(FORWARD_SPEED, 0)
                print('Searching: Turning left')
        else:
            print('No post-it detected, waiting...')
            robot_kinematics(0, 0)
    cv2.imshow("Frame", image)
    cv2.imshow("Mask", mask)
    rawCapture.truncate(0)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
cv2.destroyAllWindows()
GPIO.cleanup()
# Stop PWM on exit
left_motor_forward_pwm.stop()
left_motor_backward_pwm.stop()
right_motor_forward_pwm.stop()
right_motor_backward_pwm.stop()
