import RPi.GPIO as GPIO  # type: ignore
import pygame
import time
import cv2  # type: ignore
import numpy as np
from enum import Enum

# ---------------- Камера ----------------
cap = cv2.VideoCapture(0)
KERNEL = np.ones((5, 5), np.uint8)

LOWER_RED_1 = np.array([0, 100, 100])
UPPER_RED_1 = np.array([10, 255, 255])
LOWER_RED_2 = np.array([160, 100, 100])
UPPER_RED_2 = np.array([179, 255, 255])

# ---------------- GPIO ----------------
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

TRIG, ECHO = 23, 24
GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)

ENA, IN1, IN2 = 26, 20, 21
ENB, IN3, IN4 = 12, 6, 13
GPIO.setup([ENA, IN1, IN2, ENB, IN3, IN4], GPIO.OUT)

SERVO_PIN = 18
GPIO.setup(SERVO_PIN, GPIO.OUT)

RELAY_PIN = 25
GPIO.setup(RELAY_PIN, GPIO.OUT)

pwm_a = GPIO.PWM(ENA, 1000)
pwm_b = GPIO.PWM(ENB, 1000)
servo = GPIO.PWM(SERVO_PIN, 50)

pwm_a.start(0)
pwm_b.start(0)
servo.start(0)

# ---------------- Константы ----------------
WALL_DISTANCE = 30
SAFE_DISTANCE = 20
MOVE_SPEED = 35

# ---------------- Состояния ----------------
class AutoState(Enum):
    EXIT_BASE = 1
    MOVE_FORWARD = 2
    SEARCH_WALL = 3
    ALIGN_WALL = 4
    CLEANING = 5
    GO_BACK = 6
    FINISH = 7

# ---------------- Движение ----------------
def forward(speed=MOVE_SPEED):
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    pwm_a.ChangeDutyCycle(speed)
    pwm_b.ChangeDutyCycle(speed)

def backward(speed=MOVE_SPEED):
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)
    pwm_a.ChangeDutyCycle(speed)
    pwm_b.ChangeDutyCycle(speed)

def left(speed=MOVE_SPEED):
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    pwm_a.ChangeDutyCycle(speed)
    pwm_b.ChangeDutyCycle(speed)

def right(speed=MOVE_SPEED):
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)
    pwm_a.ChangeDutyCycle(speed)
    pwm_b.ChangeDutyCycle(speed)

def stop():
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.LOW)
    pwm_a.ChangeDutyCycle(0)
    pwm_b.ChangeDutyCycle(0)

# ---------------- Датчики ----------------
def get_distance():
    GPIO.output(TRIG, False)
    time.sleep(0.002)
    GPIO.output(TRIG, True)
    time.sleep(0.00001)
    GPIO.output(TRIG, False)

    start, end = time.time(), time.time()
    while GPIO.input(ECHO) == 0:
        start = time.time()
    while GPIO.input(ECHO) == 1:
        end = time.time()

    return (end - start) * 17150

# ---------------- Манипулятор ----------------
def set_servo(angle):
    duty = 5 + angle / 180 * 5
    servo.ChangeDutyCycle(duty)
    time.sleep(0.2)
    servo.ChangeDutyCycle(0)

def pump(state):
    GPIO.output(RELAY_PIN, GPIO.HIGH if state else GPIO.LOW)

# ---------------- Компьютерное зрение ----------------
def detect_red():
    ret, frame = cap.read()
    if not ret:
        return False

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask1 = cv2.inRange(hsv, LOWER_RED_1, UPPER_RED_1)
    mask2 = cv2.inRange(hsv, LOWER_RED_2, UPPER_RED_2)
    mask = cv2.bitwise_or(mask1, mask2)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, KERNEL)

    return cv2.countNonZero(mask) > 3000

def camera_detect_green():
    # обнаружение зелёного цвета камерой
    ret, frame = cap.read()
    if not ret:
        return False

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    lower_green = np.array([35, 40, 40])
    upper_green = np.array([85, 255, 255])

    mask = cv2.inRange(hsv, lower_green, upper_green)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, KERNEL)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, KERNEL)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if not contours:
        return False

    largest = max(contours, key=cv2.contourArea)
    return cv2.contourArea(largest) > 500

# ---------------- Автономный режим ----------------
def run_autonomous_mode():
   def run_autonomous_mode():
    # выезд из базы
    forward(30)
    time.sleep(1.5)
    stop()

    # поиск красной линии на тоннеле
    while not detect_red():
        left(25)
        time.sleep(0.1)
        stop()

    # подъезд задом к тоннелю
    backward(25)
    time.sleep(0.5)
    stop()

    # заезд в тоннель задним ходом по красной линии
    while detect_red():
        backward(25)
        time.sleep(0.05)
        stop()

    # движение по тоннелю по одному сонару
    while True:
        dist = get_distance()

        if dist > 50:
            break

        if dist > 7.7:
            left(20)
        else:
            right(20)

        forward(30)
        time.sleep(0.05)
        stop()

    # поиск зелёного круга после тоннеля
    while not camera_detect_green():
        left(25)
        time.sleep(0.1)
        stop()

    # подъезд к зелёному кругу
    while get_distance() > 15:
        forward(25)
        time.sleep(0.05)
        stop()

    # разворот боком к стене по сонару
    right(25)
    time.sleep(0.6)
    stop()

    # поиск самой верхней точки пятна
    for angle in range(30, 150, 5):
        set_servo(angle)
        if camera_detect_green():
            break

    # покраска сверху вниз
    pump(True)
    for angle in range(angle, 20, -5):
        set_servo(angle)
        if not camera_detect_green():
            time.sleep(0.1)
    pump(False)

    # опускание подъемного механизма
    set_servo(0)

    # разворот на 180 градусов
    right(30)
    time.sleep(1.7)
    stop()

    # возврат к тоннелю
    forward(30)
    time.sleep(2)
    stop()

    # проход тоннеля обратно по сонару
    while True:
        dist = get_distance()

        if dist > 50:
            break

        if dist > 7.7:
            left(20)
        else:
            right(20)

        forward(30)
        time.sleep(0.05)
        stop()

    # возврат в базу по запомненному маршруту
    backward(30)
    time.sleep(2)
    stop()


# ---------------- Ручное управление ----------------
def joystick_control():
    """Ручное управление роботом с джойстика"""
    pygame.init()
    pygame.joystick.init()
    joystick = pygame.joystick.Joystick(0)
    joystick.init()

    while True:
        for event in pygame.event.get():
            if event.type == pygame.JOYBUTTONDOWN:
                if event.button == 0:
                    forward()
                elif event.button == 3:
                    backward()
                elif event.button == 1:
                    right()
                elif event.button == 2:
                    left()
                elif event.button == 7:
                    run_autonomous_mode()
            elif event.type == pygame.JOYBUTTONUP:
                stop()

# ---------------- Точка входа ----------------
def main():
    """Запуск программы"""
    joystick_control()

if __name__ == "__main__":
    main()
