# RPi-Serial
A USB Serial library for integration with Arduino, various UART devices and the Raspberry Pi. Utilized a dynamic file descriptor to establish bi-directional communication with a virtualized /dev/tty device.

This project was created for use in various projects for Dr. Karydis' ARCS Lab. Seeing as no viable library was available that allowed communication via a virtual serial port via USB I created a library to handle this kind of communication. This library has been tested on Arduino devices and ODrive BLDC motor controllers.
