#include <SerialPort.h>
#include <SerialStream.h>
#include <iostream>

using namespace LibSerial;


int main(int argc, char** argv)
{
    // Open Serial Connection with Arduino
    SerialStream arduino_serial;
    arduino_serial.Open("/dev/ttyACM0");

    // Send signal to confirm connection


    // Wait for response

    // Process serial data

    // Close port
    arduino_serial.Close();

    return 0;
}
