import cv2
import numpy as np
import RPi.GPIO as GPIO
from RPi import PWM
from picamera.array import PiRGBArray
from picamera import PiCamera
import time

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
    lowerColorThreshold = np.array([20, 125, 75])
    upperColorThreshold = np.array([40, 255, 255])
    mask = cv2.inRange(image_hsv, lowerColorThreshold, upperColorThreshold)
    image_masked = cv2.bitwise_and(image, image, mask=mask)
    pixelleft = cv2.countNonZero(mask[:, :300])
    pixelright = cv2.countNonZero(mask[:, 340:])
    pixelmiddle = cv2.countNonZero(mask[:, 300:340])
    totalpixel = cv2.countNonZero(mask)
    if totalpixel < 50:
        panic()
    else:
        # Find the weighted average x position of detected color
        indices = np.where(mask > 0)
        if len(indices[1]) > 0:
            avg_x = np.mean(indices[1])
            center_x = mask.shape[1] // 2
            offset = avg_x - center_x
            max_offset = center_x
            # Calculate turn speed: slower when closer to horizontal center
            turn_speed = int(75 * min(abs(offset) / max_offset, 1))
            forward_speed = 75
            if abs(offset) < 20:
                robot_kinematics(forward_speed, forward_speed)
                print('Moving Forward (centered horizontally)')
            elif offset < 0:
                robot_kinematics(forward_speed - turn_speed, forward_speed)
                print(f'Turning Left (speed {forward_speed - turn_speed})')
            else:
                robot_kinematics(forward_speed, forward_speed - turn_speed)
                print(f'Turning Right (speed {forward_speed - turn_speed})')
        else:
            panic()
    cv2.imshow("Frame", image)
    cv2.imshow("Mask", mask)
    cv2.imshow("Result", image_masked)
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
