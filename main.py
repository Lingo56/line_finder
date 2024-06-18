#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor, UltrasonicSensor
from pybricks.parameters import Port, Button
from pybricks.media.ev3dev import Font
from pybricks.tools import wait

# Create your objects here.
ev3 = EV3Brick()
left_motor = Motor(Port.A)
right_motor = Motor(Port.D)
color_sensor = ColorSensor(Port.S1)
ultrasonic_sensor = UltrasonicSensor(Port.S4)

small_font = Font(size=9)
ev3.screen.set_font(small_font)

# Write your program here.
# Define variables and constants
tolerance = 10
blue_tolerance = 17
green_tolerance = 7.4
white_tolerance = 6.7
base_speed = 75  # Base speed for the motors
Kp = 30  # Proportional gain
Ki = 0  # Integral gain
Kd = 0  # Derivative gain
integral = 0
prev_error = 0

on_blue = False
object_action_performed = False
motors_enabled = False

# Define target color values
green_R, green_G, green_B = 14, 45, 10
blue_R, blue_G, blue_B = 8, 11, 18
white_R, white_G, white_B = 58, 58, 43

# Calculate light intensity for each color
green_intensity = (green_R + green_G + green_B) / 3
blue_intensity = (blue_R + blue_G + blue_B) / 3
white_intensity = (white_R + white_G + white_B) / 3

# Calculate thresholds
threshold = (green_intensity + white_intensity) / 2
blue_threshold = (blue_intensity + white_intensity) / 2
white_difference = 30  # white_R is significantly higher.
blue_difference = 35  # blue_G is significantly lower.

# Main loop
while True:
    # Check if the button is pressed to toggle motors
    if Button.CENTER in ev3.buttons.pressed():
        motors_enabled = not motors_enabled
        if not motors_enabled:
            left_motor.stop()
            right_motor.stop()
        wait(500)  # Wait for 0.5 seconds to avoid rapid toggling
        
    # Check if the left button is pressed to decrease speed
    if Button.LEFT in ev3.buttons.pressed():
        base_speed -= 5
        if base_speed < 0:
            base_speed = 0
        wait(200)  # Wait for 0.5 seconds to avoid rapid toggling
    
    # Check if the right button is pressed to increase speed
    if Button.RIGHT in ev3.buttons.pressed():
        base_speed += 5
        if base_speed > 150:  # You can adjust the maximum speed limit as needed
            base_speed = 150
        wait(200)  # Wait for 0.5 seconds to avoid rapid toggling
        
    # Check if the right button is pressed to increase speed
    if Button.UP in ev3.buttons.pressed():
        Kp += 1
        if Kp > 100:  # You can adjust the maximum speed limit as needed
            Kp = 100
        wait(200)  # Wait for 0.5 seconds to avoid rapid toggling
        
    # Check if the left button is pressed to decrease speed
    if Button.DOWN in ev3.buttons.pressed():
        Kp -= 1
        if Kp < 1:
            Kp = 1
        wait(200)  # Wait for 0.5 seconds to avoid rapid toggling

    # Read the RGB values from the light sensor
    red_value, green_value, blue_value = color_sensor.rgb()

    # Read distance from ultrasonic sensor
    distance = ultrasonic_sensor.distance()

    # Clear the screen before drawing new text
    ev3.screen.clear()

    # Display the RGB values and distance on the EV3 brick screen
    ev3.screen.draw_text(0, 0, "R: {}".format(red_value))
    ev3.screen.draw_text(0, 10, "G: {}".format(green_value))
    ev3.screen.draw_text(0, 20, "B: {}".format(blue_value))
    ev3.screen.draw_text(0, 30, "Object Distance: {}".format(distance))

    # Check if an object is within 10cm of the ultrasonic sensor
    if distance <= 100 and not object_action_performed and motors_enabled:
        # Stop the motors
        left_motor.stop()
        right_motor.stop()
        # Beep for 2 seconds
        ev3.speaker.beep()
        object_action_performed = True
        wait(2000)

        # Check color under the robot
        if on_blue:  # If on blue line
            # Rotate 180 degrees and go backwards
            left_motor.run(-base_speed * 6)
            right_motor.run(base_speed * 6)
            wait(900)  # Rotate for 0.5 second
            left_motor.stop()
            right_motor.stop()
            object_action_performed = False
        else:  # If on green line
            left_motor.run(base_speed * 6)
            right_motor.run(base_speed * 6)
            wait(375)  # Go forward
            left_motor.stop()
            right_motor.stop()
            wait(400)
            left_motor.run(-base_speed * 6)
            right_motor.run(base_speed * 6)
            wait(375)  # Turn left
            left_motor.stop()
            right_motor.stop()
            wait(400)
            left_motor.run(base_speed * 6)
            right_motor.run(-base_speed * 6)
            wait(430)  # Turn Right
            left_motor.stop()
            right_motor.stop()
            wait(400)
            left_motor.run(-base_speed * 6)
            right_motor.run(-base_speed * 6)
            wait(400)  # Go Backwards
            left_motor.stop()
            right_motor.stop()
            wait(500)
            object_action_performed = False

    # Calculate current light intensity
    current_intensity = (red_value + green_value + blue_value) / 3

    # Calculate the error term
    error = threshold - current_intensity

    # Integral term
    integral += abs(error)

    # Derivative term
    derivative = error - prev_error
    prev_error = error

    # Calculate the difference between error and tolerance
    error_diff = abs(error) - tolerance
    
    right_speed = 1
    left_speed = 1

    # Adjust motor speeds based on the error difference
    if error > tolerance:
        # Too dark, turn left
        left_speed = base_speed + (Kp * error_diff) + (Ki * integral) + (Kd * derivative)
        right_speed = base_speed - (Kp * error_diff) - (Ki * integral) - (Kd * derivative)
        if motors_enabled:
            left_motor.run(left_speed)
            right_motor.run(right_speed)
        ev3.screen.draw_text(0, 50, "Turn Right")
        ev3.screen.draw_text(0, 60, "Error({}): Too dark".format(round(error,4)))
    elif error < -tolerance:
        # Too bright, turn right
        left_speed = base_speed - (Kp * error_diff) - (Ki * integral) - (Kd * derivative)
        right_speed = base_speed + (Kp * error_diff) + (Ki * integral) + (Kd * derivative)
        if motors_enabled:
            left_motor.run(left_speed)
            right_motor.run(right_speed)
        ev3.screen.draw_text(0, 50, "Turn Left")
        ev3.screen.draw_text(0, 60, "Error ({}): Too bright".format(round(error,4)))
    else:
        # In the desired range, go straight
        if motors_enabled:
            left_motor.run(base_speed)
            right_motor.run(base_speed)
        ev3.screen.draw_text(0, 50, "Straight")
        ev3.screen.draw_text(0, 60, "Error ({}): In range".format(round(error,4)))

    # Set tolerances based on color
    if green_value < blue_difference:  # if blue
        tolerance = blue_tolerance
        on_blue = True
        ev3.screen.draw_text(0, 40, "BLUE")
    elif red_value <= white_difference:  # if green
        tolerance = green_tolerance
        on_blue = False
        ev3.screen.draw_text(0, 40, "GREEN")
    else:  # if white
        tolerance = white_tolerance
        ev3.screen.draw_text(0, 40, "WHITE")

    # Display info about if motors
    ev3.screen.draw_text(0, 70, "Motors Enabled: {}".format(motors_enabled))
    ev3.screen.draw_text(0, 80, "Base Speed: {}".format(base_speed))
    ev3.screen.draw_text(0, 90, "Right Speed: {}".format(right_speed))
    ev3.screen.draw_text(0, 100, "Left Speed: {}".format(left_speed))
    ev3.screen.draw_text(0, 110, "Proportional Gain (Kp): {}".format(Kp))

    # Wait briefly before repeating the loop
    wait(100)
