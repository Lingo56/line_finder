#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor, UltrasonicSensor
from pybricks.parameters import Port
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
blue_tolerance = 20
green_tolerance = 8
white_tolerance = 9
base_speed = 20  # Base speed for the motors
Kp = 10  # Proportional gain
Ki = 0.000004  # Integral gain
Kd = 0.00001  # Derivative gain
integral = 0
prev_error = 0

on_blue = False
object_action_performed = False

# Define target color values
green_R, green_G, green_B = 13, 40, 6
blue_R, blue_G, blue_B = 8, 10, 10
white_R, white_G, white_B = 55, 50, 20

# Calculate light intensity for each color
green_intensity = (green_R + green_G + green_B) / 3
blue_intensity = (blue_R + blue_G + blue_B) / 3
white_intensity = (white_R + white_G + white_B) / 3

# Calculate thresholds
threshold = (green_intensity + white_intensity) / 2
blue_threshold = (blue_intensity + white_intensity) / 2
white_difference = 15  # white_R is significantly higher.
blue_difference = 20  # blue_G is significantly lower.

# Main loop
while True:
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
    if distance <= 100 and not object_action_performed:
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
            left_motor.run_time(-base_speed * 5, 500)
            right_motor.run_time(base_speed * 5, 500)
            wait(500)  # Rotate for 0.5 second
            left_motor.stop()
            right_motor.stop()
            object_action_performed = False
        else:  # If on green line
            # No need to check if the object is still in the way
            # Continue moving forward
            left_motor.run_time(base_speed * 4, 375)
            right_motor.run_time(base_speed * 4, 375)
            wait(375)  # Move object
            left_motor.stop()
            right_motor.stop()
            wait(250)
            left_motor.run_time(-base_speed * 4, 375)
            right_motor.run_time(base_speed * 4, 375)
            wait(375)  # Move object
            left_motor.run_time(base_speed * 4, 325)
            right_motor.run_time(-base_speed * 4, 325)
            wait(325)  # Move object
            left_motor.run_time(-base_speed * 4, 375)
            right_motor.run_time(-base_speed * 4, 375)
            wait(375)  # Return to original position
            left_motor.stop()
            right_motor.stop()
            wait(500)
            object_action_performed = False

    # Calculate current light intensity
    current_intensity = (red_value + green_value + blue_value) / 3

    # Calculate the error term
    error = threshold - current_intensity

    # Integral term
    integral += error

    # Derivative term
    derivative = error - prev_error
    prev_error = error

    # Adjust motor speeds proportionally based on the error, integral, and derivative terms
    left_speed = base_speed + Kp * error + Ki * integral + Kd * derivative
    right_speed = base_speed - Kp * error - Ki * integral - Kd * derivative

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

    # Apply tolerance
    if error > tolerance:
        # Too dark, turn left
        left_motor.run(left_speed)
        right_motor.run(right_speed)
        ev3.screen.draw_text(0, 50, "Turn Right")
        ev3.screen.draw_text(0, 60, "Error({}): Too dark".format(round(error,4)))
    elif error < -tolerance:
        # Too bright, turn right
        left_motor.run(left_speed)
        right_motor.run(right_speed)
        ev3.screen.draw_text(0, 50, "Turn Left")
        ev3.screen.draw_text(0, 60, "Error ({}): Too bright".format(round(error,4)))
    else:
        # In the desire```  d range, go straight
        left_motor.run(base_speed)
        right_motor.run(base_speed)
        ev3.screen.draw_text(0, 50, "Straight")
        ev3.screen.draw_text(0, 60, "Error ({}): In range".format(round(error,4)))

    # Wait briefly before repeating the loop
    wait(100)
