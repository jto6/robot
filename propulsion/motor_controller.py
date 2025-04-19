#!/usr/bin/env python3

import RPi.GPIO as GPIO
import time
import curses
from enum import Enum

class RobotState(Enum):
    STOPPED = 1
    FORWARD = 2
    BACKWARD = 3
    TURNING_RIGHT = 4
    TURNING_LEFT = 5

pwm_hz = 500  # in Hz

class Robot(object):
    def __init__(self):
        # Pin Definitions
        self.motorLin1_pin = 22
        self.motorLin2_pin = 24
        self.motorLenA_pin = 10
        self.motorRin1_pin = 19
        self.motorRin2_pin = 21
        self.motorRenB_pin = 32

        GPIO.setmode(GPIO.BOARD)

        self.state = RobotState.STOPPED
        self.RM_speed = GPIO.PWM(self.motorRenB_pin, pwm_hz)
        self.LM_speed = GPIO.PWM(self.motorLenA_pin, pwm_hz)
        self.RM_speed.start(0)
        self.LM_speed.start(0)

        self.speed = 30

        # set pin as an output pin with optional initial state of LOW
        GPIO.setup(self.motorRin1_pin, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.motorRin2_pin, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.motorLin1_pin, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.motorLin2_pin, GPIO.OUT, initial=GPIO.LOW)

    def shutdown(self):
        # Shut down motor
        GPIO.output(self.motorRin1_pin, GPIO.LOW)
        GPIO.output(self.motorRin2_pin, GPIO.LOW)
        GPIO.output(self.motorLin1_pin, GPIO.LOW)
        GPIO.output(self.motorLin2_pin, GPIO.LOW)
        self.RM_speed.stop()
        self.LM_speed.stop()

        GPIO.cleanup()

    def set_speed(self, new_state, speed=None):
        if speed is None:
            speed = self.speed

        self.RM_speed.ChangeDutyCycle(speed)
        self.LM_speed.ChangeDutyCycle(speed)

        match new_state:
            case RobotState.FORWARD:
                GPIO.output(self.motorRin1_pin, GPIO.LOW)
                GPIO.output(self.motorLin1_pin, GPIO.LOW)
                GPIO.output(self.motorRin2_pin, GPIO.HIGH)
                GPIO.output(self.motorLin2_pin, GPIO.HIGH)
                print(f"Moving forward at {speed}!\n")

            case RobotState.BACKWARD:
                GPIO.output(self.motorRin1_pin, GPIO.HIGH)
                GPIO.output(self.motorLin1_pin, GPIO.HIGH)
                GPIO.output(self.motorRin2_pin, GPIO.LOW)
                GPIO.output(self.motorLin2_pin, GPIO.LOW)
                print(f"Moving backward at {speed}!\n")

            case RobotState.TURNING_LEFT:
                GPIO.output(self.motorRin1_pin, GPIO.LOW)
                GPIO.output(self.motorLin1_pin, GPIO.LOW)
                GPIO.output(self.motorRin2_pin, GPIO.HIGH)
                GPIO.output(self.motorLin2_pin, GPIO.LOW)
                print("Turning left!\n")

            case RobotState.TURNING_RIGHT:
                GPIO.output(self.motorRin1_pin, GPIO.LOW)
                GPIO.output(self.motorLin1_pin, GPIO.LOW)
                GPIO.output(self.motorRin2_pin, GPIO.LOW)
                GPIO.output(self.motorLin2_pin, GPIO.HIGH)
                print("Turning right!\n")

            case RobotState.STOPPED:
                GPIO.output(self.motorRin1_pin, GPIO.LOW)
                GPIO.output(self.motorLin1_pin, GPIO.LOW)
                GPIO.output(self.motorRin2_pin, GPIO.LOW)
                GPIO.output(self.motorLin2_pin, GPIO.LOW)
                print("Stoped!\n")

        self.state = new_state
        self.speed = speed

    def go_forward(self):
        if self.state == RobotState.BACKWARD:
            self.set_speed(RobotState.STOPPED)
        else:
            self.set_speed(RobotState.FORWARD)

    def go_left(self):
        self.set_speed(RobotState.TURNING_LEFT)

    def go_right(self):
        self.set_speed(RobotState.TURNING_RIGHT)

    def backward(self):
        if self.state == RobotState.FORWARD:
            self.set_speed(RobotState.STOPPED)
        else:
            self.set_speed(RobotState.BACKWARD)

    def accelerate(self):
        new_speed = self.speed + 10
        if new_speed > 100:
            new_speed = 100
        print(f"Accelerating to {new_speed}!\n")

        self.set_speed(self.state, new_speed)

    def deaccelerate(self):
        new_speed = self.speed - 10
        if new_speed < 0:
            new_speed = 0
        print(f"Deaccelerating to {new_speed}!\n")

        self.set_speed(self.state, new_speed)

# Main loop using curses
def control_loop(stdscr, robot):
    stdscr.clear()
    stdscr.addstr("Use the arrow keys to control, and press 'q' to quit.\n")
    stdscr.refresh()

    while True:
        key = stdscr.getch()  # Wait for a single key press

        if key == curses.KEY_UP:
            robot.go_forward()
        elif key == curses.KEY_LEFT:
            robot.go_left()
        elif key == curses.KEY_RIGHT:
            robot.go_right()
        elif key == curses.KEY_DOWN:
            robot.backward()
        elif key == ord('='):
            robot.accelerate()
        elif key == ord('-'):
            robot.deaccelerate()
        elif key == ord('q'):  # Use 'q' to quit
            print("Exiting the control loop.")
            break

# Run the control loop
if __name__ == "__main__":
    robot = Robot()

    curses.wrapper(control_loop, robot)
    robot.shutdown()
