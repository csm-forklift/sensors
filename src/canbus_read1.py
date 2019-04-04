import serial
import time
from sys import exit

try:
    # Open Serial Connection with Arduino
    port_name = "/dev/ttyACM0"
    baudrate = 115200
    arduino_serial = serial.Serial("/dev/ttyACM0", baudrate, timeout=1);
    # Give time for the connection
    time.sleep(2)

    # Send signal to confirm connection
    timeout = 10 # sec
    not_connected = True
    arduino_serial.reset_input_buffer()
    arduino_serial.reset_output_buffer()
    arduino_serial.write('0');

    # Wait for response
    tic = time.time()
    while not_connected:
        rx = arduino_serial.read(1);
        if rx == '1': # 49 = ascii number for '1' character
            print("Connected to Arduino at %s" % port_name)
            not_connected = False

        # If connection takes too long, end the program
        toc = time.time()
        if (toc - tic) > timeout:
            print("Connection is taking too long. Ending program.")
            exit("Could not connect to Arduino.")

    # Process serial data (Continue reading lines and printing them)
    while True:
        line = arduino_serial.readline()
        if (line != ''):
            print(line)
            if (line == "CAN Init ok\r\n"):
                print("="*60)
                print("Begin Data Collection")
                print("="*60)


except KeyboardInterrupt:
    # Close port
    print("Closing serial connection.")
    arduino_serial.close()
