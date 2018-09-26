#!/usr/bin/env python

''' This is the node for reading the encoder raw values and publishing them. 
The tick values from this node are read by encoder_odom and converted into 
odometry messages.'''

import time
import RPi.GPIO as GPIO
import rospy
from std_msgs.msg import Int16

class EncoderRead():
    def __init__(self):
        # Define Global Variables
        self.counter1 = Int16(0)
        self.counter2 = Int16(0)

        self.ir1 = 0
        self.ir2 = 0
        self.ir3 = 0
        self.ir4 = 0

        self.oldenc_1 = 0
        self.oldenc_2 = 0
        self.oldenc_3 = 0
        self.oldenc_4 = 0

        self.state1 = 0
        self.oldstate1 = 0
        self.state2 = 0
        self.oldstate2 = 0

        self.ir1_pin = 29
        self.ir2_pin = 31
        self.ir3_pin = 33
        self.ir4_pin = 35
        self.switch_pin = 37
        
        # Create publishers
        self.rate = 60 # Hertz
        rospy.init_node("encoder_read")
        self.right_pub = rospy.Publisher('enc/front_wheel_right_ticks', Int16, queue_size=10);
        self.left_pub = rospy.Publisher('enc/front_wheel_left_ticks', Int16, queue_size=10);
        
        self.setup()


    def setup(self):
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.ir1_pin, GPIO.IN)
        GPIO.setup(self.ir2_pin, GPIO.IN)
        GPIO.setup(self.ir3_pin, GPIO.IN)
        GPIO.setup(self.ir4_pin, GPIO.IN)
        GPIO.setup(self.switch_pin, GPIO.OUT)

        GPIO.output(self.switch_pin, 1)
        time.sleep(2)

        if GPIO.input(self.ir1_pin):
            self.ir1 = 1
        else:
            self.ir1 = 0
        if GPIO.input(self.ir2_pin):
            self.ir2 = 1
        else:
            self.ir2 = 0
        if GPIO.input(self.ir3_pin):
            self.ir3 = 1
        else:
            self.ir3 = 0
        if GPIO.input(self.ir4_pin):
            self.ir4 = 1
        else:
            self.ir4 = 0

        # state initialization
        if self.ir1 == 0:
            if self.ir2 == 0:
                self.oldstate1 = 0
            else:
                self.oldstate1 = 1
        else:
            if self.ir2 == 0:
                self.oldstate1 = 2
            else:
                self.oldstate1 = 3

        if self.ir3 == 0:
            if self.ir4 == 0:
                self.oldstate2 = 0
            else:
                self.oldstate2 = 1
        else:
            if self.ir4 == 0:
                self.oldstate1 = 2
            else:
                self.oldstate2 = 3
                

    def loop(self):
        GPIO.output(self.switch_pin, 1)

        if GPIO.input(self.ir1_pin):
            self.ir1 = 1
        else:
            self.ir1 = 0
        if GPIO.input(self.ir2_pin):
            self.ir2 = 1
        else:
            self.ir2 = 0
        if GPIO.input(self.ir3_pin):
            self.ir3 = 1
        else:
            self.ir3 = 0
        if GPIO.input(self.ir4_pin):
            self.ir4 = 1
        else:
            self.ir4 = 0

        if self.ir1 == 0:
            if self.ir2 == 0:
                self.state1 = 0
            else:
                self.state1 = 1
        else:
            if self.ir2 == 0:
                self.state1 = 2
            else:
                self.state1 = 3

        if self.ir3 == 0:
            if self.ir4 == 0:
                self.state2 = 0
            else:
                self.state2 = 1
        else:
            if self.ir4 == 0:
                self.state2 = 2
            else:
                self.state2 = 3

        # main state function
        if self.oldstate1 == 0:
            if self.state1 == 1:
                self.returnvalue1 = 1
            elif self.state1 == 2:
                self.returnvalue1 = -1
            else:
                self.returnvalue1 = 0

        elif self.oldstate1 == 1:
            if self.state1 == 3:
                self.returnvalue1 = 1
            elif self.state1 == 0:
                self.returnvalue1 = -1
            else:
                self.returnvalue1 = 0

        elif self.oldstate1 == 2:
            if self.state1 == 0:
                self.returnvalue1 = 1
            elif self.state1 == 3:
                self.returnvalue1 = -1
            else:
                self.returnvalue1 = 0

        elif self.oldstate1 == 3:
            if self.state1 == 2:
                self.returnvalue1 = 1
            elif self.state1 == 1:
                self.returnvalue1 = -1
            else:
                self.returnvalue1 = 0

        ####

        if self.oldstate2 == 0:
            if self.state2 == 1:
                self.returnvalue2 = 1
            elif self.state2 == 2:
                self.returnvalue2 = -1
            else:
                self.returnvalue2 = 0

        elif self.oldstate2 == 1:
            if self.state2 == 3:
                self.returnvalue2 = 1
            elif self.state2 == 0:
                self.returnvalue2 = -1
            else:
                self.returnvalue2 = 0

        elif self.oldstate2 == 2:
            if self.state2 == 0:
                self.returnvalue2 = 1
            elif self.state2 == 3:
                self.returnvalue2 = -1
            else:
                self.returnvalue2 = 0

        elif self.oldstate2 == 3:
            if self.state2 == 2:
                self.returnvalue2 = 1
            elif self.state2 == 1:
                self.returnvalue2 = -1
            else:
                self.returnvalue2 = 0

        if self.ir1 != self.oldenc_1:
            self.counter1 = self.counter1 + self.returnvalue1
        if self.ir3 != self.oldenc_3:
            self.counter2 = self.counter2 + self.returnvalue2

        # Publish counter values
        #print("counter1: %d, counter2: %d" % (counter1, counter2))
        self.right_pub.publish(self.counter1);
        self.left_pub.publish(self.counter2);

        # Update states
        self.oldenc_1 = self.ir1
        self.oldenc_2 = self.ir2
        self.oldenc_3 = self.ir3
        self.oldenc_4 = self.ir4

        self.oldstate1 = self.state1
        self.oldstate2 = self.state2
    
    def spin(self):
        r = rospy.Rate(self.rate)
        while (not rospy.is_shutdown()):
            self.loop()
            r.sleep()
            


def main():
    try:
        encoder_read = EncoderRead()
        encoder_read.spin()
    finally:
        print "Cleaning up pins."
        GPIO.cleanup()


if __name__ == "__main__":
    main()
    print("hI")
