''' This is the node for reading the encoder raw values and publishing them.'''

import time
import RPi.GPIO as GPIO


def setup():
    global ir1_pin
    global ir2_pin
    global ir3_pin
    global ir4_pin
    global switch_pin
    
    global ir1
    global ir2
    global ir3
    global ir4
    
    global oldstate1
    global oldstate2

    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(ir1_pin, GPIO.IN)
    GPIO.setup(ir2_pin, GPIO.IN)
    GPIO.setup(ir3_pin, GPIO.IN)
    GPIO.setup(ir4_pin, GPIO.IN)
    GPIO.setup(switch_pin, GPIO.OUT)

    GPIO.output(switch_pin, 1)
    time.sleep(2)

    if GPIO.input(ir1_pin):
        ir1 = 1
    else:
        ir1 = 0
    if GPIO.input(ir2_pin):
        ir2 = 1
    else:
        ir2 = 0
    if GPIO.input(ir3_pin):
        ir3 = 1
    else:
        ir3 = 0
    if GPIO.input(ir4_pin):
        ir4 = 1
    else:
        ir4 = 0

    # state initialization
    if ir1 == 0:
        if ir2 == 0:
            oldstate1 = 0
        else:
            oldstate1 = 1
    else:
        if ir2 == 0:
            oldstate1 = 2
        else:
            oldstate1 = 3

    if ir3 == 0:
        if ir4 == 0:
            oldstate2 = 0
        else:
            oldstate2 = 1
    else:
        if ir4 == 0:
            oldstate1 = 2
        else:
            oldstate2 = 3

    # ROS stuff goes here
    # and IMU stuff

############################


def loop():
    global ir1_pin
    global ir2_pin
    global ir3_pin
    global ir4_pin
    global switch_pin
    
    global ir1
    global ir2
    global ir3
    global ir4
    
    global oldenc_1
    global oldenc_2
    global oldenc_3
    global oldenc_4

    global state1
    global state2

    global oldstate1
    global oldstate2

    global counter1
    global counter2

    GPIO.output(switch_pin, 1)

    if GPIO.input(ir1_pin):
        ir1 = 1
    else:
        ir1 = 0
    if GPIO.input(ir2_pin):
        ir2 = 1
    else:
        ir2 = 0
    if GPIO.input(ir3_pin):
        ir3 = 1
    else:
        ir3 = 0
    if GPIO.input(ir4_pin):
        ir4 = 1
    else:
        ir4 = 0

    if ir1 == 0:
        if ir2 == 0:
            state1 = 0
        else:
            state1 = 1
    else:
        if ir2 == 0:
            state1 = 2
        else:
            state1 = 3

    if ir3 == 0:
        if ir4 == 0:
            state2 = 0
        else:
            state2 = 1
    else:
        if ir4 == 0:
            state2 = 2
        else:
            state2 = 3

    # main state function
    if oldstate1 == 0:
        if state1 == 1:
            returnvalue1 = 1
        elif state1 == 2:
            returnvalue1 = -1
        else:
            returnvalue1 = 0

    elif oldstate1 == 1:
        if state1 == 3:
            returnvalue1 = 1
        elif state1 == 0:
            returnvalue1 = -1
        else:
            returnvalue1 = 0

    elif oldstate1 == 2:
        if state1 == 0:
            returnvalue1 = 1
        elif state1 == 3:
            returnvalue1 = -1
        else:
            returnvalue1 = 0

    elif oldstate1 == 3:
        if state1 == 2:
            returnvalue1 = 1
        elif state1 == 1:
            returnvalue1 = -1
        else:
            returnvalue1 = 0

    ####

    if oldstate2 == 0:
        if state2 == 1:
            returnvalue2 = 1
        elif state2 == 2:
            returnvalue2 = -1
        else:
            returnvalue2 = 0

    elif oldstate2 == 1:
        if state2 == 3:
            returnvalue2 = 1
        elif state2 == 0:
            returnvalue2 = -1
        else:
            returnvalue2 = 0

    elif oldstate2 == 2:
        if state2 == 0:
            returnvalue2 = 1
        elif state2 == 3:
            returnvalue2 = -1
        else:
            returnvalue2 = 0

    elif oldstate2 == 3:
        if state2 == 2:
            returnvalue2 = 1
        elif state2 == 1:
            returnvalue2 = -1
        else:
            returnvalue2 = 0

    if ir1 != oldenc_1:
        counter1 = counter1 + returnvalue1
    if ir3 != oldenc_3:
        counter2 = counter2 + returnvalue2

    print("counter1: %d, counter2: %d" % (counter1, counter2))

    oldenc_1 = ir1
    oldenc_2 = ir2
    oldenc_3 = ir3
    oldenc_4 = ir4

    oldstate1 = state1
    oldstate2 = state2


# Main Function
counter1 = 0
counter2 = 0

ir1 = 0
ir2 = 0
ir3 = 0
ir4 = 0

oldenc_1 = 0
oldenc_2 = 0
oldenc_3 = 0
oldenc_4 = 0

state1 = 0
oldstate1 = 0
state2 = 0
oldstate2 = 0

ir1_pin = 29
ir2_pin = 31
ir3_pin = 33
ir4_pin = 35
switch_pin = 37

def main():
    print "Beginning setup function"
    setup()
    try:
        print "Entering loop..."
        while True:
            loop()
    finally:
        print "Cleaning up pins."
        GPIO.cleanup()


if __name__ == "__main__":
    main()
    print("hI")
