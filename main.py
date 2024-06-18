#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor, UltrasonicSensor
from pybricks.parameters import Port, Button
from pybricks.media.ev3dev import Font
from pybricks.tools import wait

# Initialize objects
ev3 = EV3Brick()
left_motor = Motor(Port.A)
right_motor = Motor(Port.D)
color_sensor = ColorSensor(Port.S1)
ultrasonic_sensor = UltrasonicSensor(Port.S4)

small_font = Font(size=9)
ev3.screen.set_font(small_font)

# Constants
TOLERANCE = 10
BLUE_TOLERANCE = 17
GREEN_TOLERANCE = 7.4
WHITE_TOLERANCE = 6.7
BASE_SPEED = 75  # Base speed for the motors
KP = 30  # Proportional gain
KI = 0  # Integral gain
KD = 0  # Derivative gain

# Variables
integral = 0
prev_error = 0
on_blue = False
object_action_performed = False
motors_enabled = False

# Target color values
GREEN_R, GREEN_G, GREEN_B = 14, 45, 10
BLUE_R, BLUE_G, BLUE_B = 8, 11, 18
WHITE_R, WHITE_G, WHITE_B = 58, 58, 43

# Light intensity calculations
green_intensity = (GREEN_R + GREEN_G + GREEN_B) / 3
blue_intensity = (BLUE_R + BLUE_G + BLUE_B) / 3
white_intensity = (WHITE_R + WHITE_G + WHITE_B) / 3

# Thresholds
THRESHOLD = (green_intensity + white_intensity) / 2
BLUE_THRESHOLD = (blue_intensity + white_intensity) / 2
WHITE_DIFFERENCE = 30  # white_R is significantly higher.
BLUE_DIFFERENCE = 35  # blue_G is significantly lower.

def toggle_motors():
    global motors_enabled
    motors_enabled = not motors_enabled
    if not motors_enabled:
        left_motor.stop()
        right_motor.stop()
    wait(500)  # Wait for 0.5 seconds to avoid rapid toggling

def adjust_base_speed(delta):
    global BASE_SPEED
    BASE_SPEED = min(max(BASE_SPEED + delta, 0), 150)
    wait(200)

def adjust_kp(delta):
    global KP
    KP = min(max(KP + delta, 1), 100)
    wait(200)

def read_sensors():
    return color_sensor.rgb(), ultrasonic_sensor.distance()

def display_data(red_value, green_value, blue_value, distance, error, left_speed, right_speed):
    ev3.screen.clear()
    ev3.screen.draw_text(0, 0, "R: {}".format(red_value))
    ev3.screen.draw_text(0, 10, "G: {}".format(green_value))
    ev3.screen.draw_text(0, 20, "B: {}".format(blue_value))
    ev3.screen.draw_text(0, 30, "Object Distance: {}".format(distance))
    ev3.screen.draw_text(0, 60, "Error ({}): ".format(round(error,4)))
    ev3.screen.draw_text(0, 70, "Motors Enabled: {}".format(motors_enabled))
    ev3.screen.draw_text(0, 80, "Base Speed: {}".format(BASE_SPEED))
    ev3.screen.draw_text(0, 90, "Right Speed: {}".format(round(right_speed,3)))
    ev3.screen.draw_text(0, 100, "Left Speed: {}".format(round(left_speed,3)))
    ev3.screen.draw_text(0, 110, "Proportional Gain (Kp): {}".format(KP))

def handle_object_detection(distance):
    global object_action_performed
    if distance <= 100 and not object_action_performed and motors_enabled:
        left_motor.stop()
        right_motor.stop()
        ev3.speaker.beep()
        object_action_performed = True
        wait(2000)
        if on_blue:
            rotate_180()
        else:
            perform_green_line_action()
        object_action_performed = False

def rotate_180():
    left_motor.run(-BASE_SPEED * 6)
    right_motor.run(BASE_SPEED * 6)
    wait(900)
    left_motor.stop()
    right_motor.stop()

def perform_green_line_action():
    left_motor.run(BASE_SPEED * 6)
    right_motor.run(BASE_SPEED * 6)
    wait(375)
    left_motor.stop()
    right_motor.stop()
    wait(400)
    left_motor.run(-BASE_SPEED * 6)
    right_motor.run(BASE_SPEED * 6)
    wait(375)
    left_motor.stop()
    right_motor.stop()
    wait(400)
    left_motor.run(BASE_SPEED * 6)
    right_motor.run(-BASE_SPEED * 6)
    wait(430)
    left_motor.stop()
    right_motor.stop()
    wait(400)
    left_motor.run(-BASE_SPEED * 6)
    right_motor.run(-BASE_SPEED * 6)
    wait(400)
    left_motor.stop()
    right_motor.stop()
    wait(500)

def adjust_motor_speeds(error):
    global integral, prev_error
    integral += abs(error)
    derivative = error - prev_error
    prev_error = error
    error_diff = abs(error) - TOLERANCE
    left_speed = BASE_SPEED
    right_speed = BASE_SPEED
    if error > TOLERANCE:
        left_speed += (KP * error_diff) + (KI * integral) + (KD * derivative)
        right_speed -= (KP * error_diff) + (KI * integral) + (KD * derivative)
    elif error < -TOLERANCE:
        left_speed -= (KP * error_diff) - (KI * integral) - (KD * derivative)
        right_speed += (KP * error_diff) - (KI * integral) - (KD * derivative)
    if motors_enabled:
        left_motor.run(left_speed)
        right_motor.run(right_speed)
    return left_speed, right_speed

def main_loop():
    global on_blue, TOLERANCE
    while True:
        if Button.CENTER in ev3.buttons.pressed():
            toggle_motors()
        
        if Button.LEFT in ev3.buttons.pressed():
            adjust_base_speed(-5)
        
        if Button.RIGHT in ev3.buttons.pressed():
            adjust_base_speed(5)
        
        if Button.UP in ev3.buttons.pressed():
            adjust_kp(1)
        
        if Button.DOWN in ev3.buttons.pressed():
            adjust_kp(-1)

        (red_value, green_value, blue_value), distance = read_sensors()
        current_intensity = (red_value + green_value + blue_value) / 3
        error = THRESHOLD - current_intensity
        handle_object_detection(distance)
        left_speed, right_speed = adjust_motor_speeds(error)

        display_data(red_value, green_value, blue_value, distance, error, left_speed, right_speed)
        
        if green_value < BLUE_DIFFERENCE:
            TOLERANCE = BLUE_TOLERANCE
            on_blue = True
            ev3.screen.draw_text(0, 40, "BLUE")
        elif red_value <= WHITE_DIFFERENCE:
            TOLERANCE = GREEN_TOLERANCE
            on_blue = False
            ev3.screen.draw_text(0, 40, "GREEN")
        else:
            TOLERANCE = WHITE_TOLERANCE
            ev3.screen.draw_text(0, 40, "WHITE")

        wait(100)

main_loop()
