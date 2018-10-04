#!/usr/bin/env python

''' This node manages all the control signals sent from the Raspberry Pi to
manipulate actuators. Currently this includes braking, acceleration, and
steering.'''


import rospy
from std_msgs.msg import Float32, Float64
import RPi.GPIO as gpio
import time


class ControllerNode():
    def __init__(self):
        rospy.init_node("controller_node")
        self.rate = 60

        # Commands
        self.brake_pub = rospy.Publisher("controls/brake/input", Float32, queue_size=10)
        self.steering_sub = rospy.Subscriber("controls/steering/input", Float64, self.steering_callback)
        self.accelerator_sub = rospy.Subscriber("controls/accelerator/input", Float64, self.accelerator_callback)

        # GPIO Pin Setup
        self.ACCELERATOR_PIN = 37
        gpio.setmode(gpio.BOARD)
        gpio.setup(self.ACCELERATOR_PIN, gpio.OUT)
        self.accelerator_pwm = gpio.PWM(self.ACCELERATOR_PIN, 1000)
        self.accelerator_pwm.start(0)

    def spin(self):
        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.update()
            r.sleep()

    def update(self):
        #
        for i in range(0,101):
            self.accelerator_pwm.ChangeDutyCycle(i)
            print i
            time.sleep(0.05)

        time.sleep(1)
        self.accelerator_pwm.stop()
        time.sleep(1)

        self.accelerator_pwm.start(100)
        for i in range(100, -1, -1):
            self.accelerator_pwm.ChangeDutyCycle(i)
            print i
            time.sleep(0.05)
            
        time.sleep(1)

    def steering_callback(self, msg):
        pass

    def accelerator_callback(self, msg):
        # Convert fraction into PWM duty cycle (0 -> 100%)
        fraction = 0
        if (msg.data < 0.):
            fraction = 0.
        if (msg.data > 1.):
            fraction = 1.

        duty_cycle = fraction*100
        self.accelerator_pwm.start(duty_cycle)


if __name__ == '__main__':
    try:
        controller_node = ControllerNode()
        controller_node.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        gpio.cleanup()
