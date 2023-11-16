#include "../include/RPiSerial.h"

RPiSerial::RPiSerial::RPiSerial(std::string port, int baud) {
    this->_portName = port;
    this->_baud = baud;

    this->_numRead = 0;
    this->_fd = -1;
    this->_isOpen = false;
}

RPiSerial::RPiSerial::~RPiSerial() {
    // If the serial port is open, close it
    if(this->_isOpen)
        close(_fd);
}

bool RPiSerial::RPiSerial::openPort() {
    this->_fd = open(this->_portName.c_str(), O_RDWR | O_NONBLOCK | O_NOCTTY);

    // If file descriptor cannot be opened, return false.
    if(this->_fd == -1)
        return false;

    // Get Serial configuration from Linux kernel, check if successful, if not exit
    if(tcgetattr(this->_fd, &this->_tty) != 0) {
        printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
        return false;
    }

    // Set Serial port flags for Teensy
    this->_tty.c_cflag &= ~PARENB;         // Clear parity bit, disabling parity (most common)
    this->_tty.c_cflag &= ~CSTOPB;         // Clear stop field, only one stop bit used in communication (most common)
    this->_tty.c_cflag &= ~CSIZE;          // Clear all bits that set the data size
    this->_tty.c_cflag |= CS8;             // 8 bits per byte (most common)
    this->_tty.c_cflag &= ~CRTSCTS;        // Disable RTS/CTS hardware flow control (most common)
    this->_tty.c_cflag |= CREAD | CLOCAL;  // Turn on READ & ignore ctrl lines (CLOCAL = 1)

    this->_tty.c_lflag &= ~ICANON;
    this->_tty.c_lflag &= ~ECHO;                   // Disable echo
    this->_tty.c_lflag &= ~ECHOE;                  // Disable erasure
    this->_tty.c_lflag &= ~ECHONL;                 // Disable new-line echo
    this->_tty.c_lflag &= ~ISIG;                   // Disable interpretation of INTR, QUIT and SUSP
    this->_tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
    this->_tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);    // Disable any special handling of received bytes

    this->_tty.c_oflag &= ~OPOST;          // Prevent special interpretation of output bytes (e.g. newline chars)
    this->_tty.c_oflag &= ~ONLCR;          // Prevent conversion of newline to carriage return/line feed
    this->_tty.c_cc[VTIME] = 10;           // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
    this->_tty.c_cc[VMIN] = 0;

    // Set in/out baud rate to be 115200
    cfsetispeed(&this->_tty, this->_baud);
    cfsetospeed(&this->_tty, this->_baud);

    // Save tty settings, also checking for error
    if (tcsetattr(this->_fd, TCSANOW, &this->_tty) != 0) {
        printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
        return false;
    }

    // Flush file descriptor buffer
    tcflush(this->_fd, TCIFLUSH);

    return this->_isOpen = true;
}

void RPiSerial::RPiSerial::closePort() {
    if(this->_isOpen)
        close(_fd);

    this->_fd = -1;
}

long RPiSerial::RPiSerial::recv(unsigned char *data, long len) {
    // Check to ensure that the virtual interface is open before attempting a read.
    if(!this->_isOpen)
        return -1;

    long bytesRead = read(this->_fd, data, len); // Read from the virtual interface
    this->_numRead += bytesRead; // Increment the private bytes read counter

    return bytesRead;
}

bool RPiSerial::RPiSerial::send(unsigned char *data, long len) {
    if(!this->_isOpen)
        return false;

    return (write(this->_fd, data, len) == len);
}

bool RPiSerial::RPiSerial::send(unsigned char data) {
    if(!this->_isOpen)
        return false;

    return (write(this->_fd, &data, 1) == 1);
}

bool RPiSerial::RPiSerial::send(std::string data) {
    if(!this->_isOpen)
        return false;

    return (write(this->_fd, data.c_str(), data.size()) == data.size());
}

bool RPiSerial::RPiSerial::getIsOpen() const {
    return this->_isOpen;
}

long RPiSerial::RPiSerial::getNumRecv() const {
    return this->_numRead;
}
