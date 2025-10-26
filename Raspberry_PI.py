import RPi.GPIO as GPIO
import time
import cv2
import pickle

ENA = 11  
IN1 = 12 
IN2 = 13  
ENB = 15  
IN3 = 16  
IN4 = 18 
TRIG = 38
ECHO = 40
CAMERA_PIN = 22

print(‘Loading the model ,please wait’)
model = pickle.load(open('/home/hemanth/Downloads/best_estimator.pkl', 'rb'))
print('Model loaded successfully')

GPIO.setmode(GPIO.BOARD)


GPIO.setup([ENA, IN1, IN2, ENB, IN3, IN4, TRIG, ECHO, CAMERA_PIN], GPIO.OUT)

pwm_motor1 = GPIO.PWM(ENA, 100)
pwm_motor2 = GPIO.PWM(ENB, 100)
pwm_motor1.start(0)
pwm_motor2.start(0)

def set_motor_speed(speed1, speed2):
    pwm_motor1.ChangeDutyCycle(speed1)
    pwm_motor2.ChangeDutyCycle(speed2)

def move_forward(speed):
    GPIO.output([IN1, IN2], [GPIO.LOW, GPIO.HIGH])
    GPIO.output([IN3, IN4], [GPIO.HIGH, GPIO.LOW])
    set_motor_speed(speed, speed)

def move_backward(speed):
    GPIO.output([IN1, IN2], [GPIO.HIGH, GPIO.LOW])
    GPIO.output([IN3, IN4], [GPIO.LOW, GPIO.HIGH])
    set_motor_speed(speed, speed)

def move_left(speed):
    GPIO.output([IN1, IN2], [GPIO.HIGH, GPIO.LOW])
    GPIO.output([IN3, IN4], [GPIO.HIGH, GPIO.LOW])
    set_motor_speed(speed, speed)

def move_right(speed):
    GPIO.output([IN1, IN2], [GPIO.LOW, GPIO.HIGH])
    GPIO.output([IN3, IN4], [GPIO.LOW, GPIO.HIGH])
    set_motor_speed(speed, speed)


def stop_motors():
    GPIO.output([IN1, IN2, IN3, IN4], GPIO.LOW)
    set_motor_speed(0, 0)

def read_distance():
    GPIO.output(TRIG, True)
    time.sleep(0.00001)
    GPIO.output(TRIG, False)

    while GPIO.input(ECHO) == 0:
        pulse_start = time.time()
    while GPIO.input(ECHO) == 1:
        pulse_end = time.time()

    pulse_duration = pulse_end - pulse_start
    distance = pulse_duration * 17150
    return distance

def avoid_obstacle():
    distance = read_distance()
    if distance < 30:  
        stop_motors()

def preprocess_image(image):
    processed_image = cv2.resize(image, (224, 224))
    processed_image = processed_image.astype('float32') / 255.0
    return processed_image.flatten()

def predict_occupant_change(image):
    preprocessed_image = preprocess_image(image)
    prediction = model.predict([preprocessed_image])
    return prediction

def capture_image(angle):
    pwm_camera = GPIO.PWM(CAMERA_PIN, 50)
    pwm_camera.start(0)
    duty = angle / 18 + 2
    GPIO.output(CAMERA_PIN, True)
    pwm_camera.ChangeDutyCycle(duty)
    time.sleep(1)
    GPIO.output(CAMERA_PIN, False)
    pwm_camera.stop()
    time.sleep(1)
    camera = cv2.VideoCapture(0)
    ret, frame = camera.read()
    camera.release()
    return frame

def detect_parking_space(image):
    prediction = predict_occupant_change(image)
    if prediction == 1:
        return True
    else:
        return False

def find_parking_space():
    for row in range(10):
        left_angle = 0
        left_image = capture_image(left_angle)
        if detect_parking_space(left_image):
            return "Left", row + 1
        
        right_angle = 180
        right_image = capture_image(right_angle)
        if detect_parking_space(right_image):
            return "Right", row + 1
        
        avoid_obstacle()
        move_forward(30)
        time.sleep(0.5)
        stop_motors()
    
    return None, None  

def park_car():
    side, row = find_parking_space()
    if side is not None:
        print("Parking space found at {} side in row {}".format(side, row))
        if side == "Left":
            move_left(30)
            time.sleep(.75)  
            stop_motors()
            time.sleep(2)
            move_forward(30)
            time.sleep(1)  
            stop_motors()
            time.sleep(1)
        elif side == "Right":
            move_right(30)
            time.sleep(.75)  
            stop_motors()
            time.sleep(2)
            move_forward(30)
            time.sleep(1)  
            stop_motors()
            time.sleep(1)
    else:
        print("No free parking space available in the area.")

parked = False


def main():
    try:
            if not parked:
                park_car()
                parked = True 
    except KeyboardInterrupt:
        stop_motors()
        GPIO.cleanup()
    finally:
        stop_motors()
        GPIO.cleanup()

if __name__ == "__main__":
    main()

