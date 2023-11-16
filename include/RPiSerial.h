#pragma once

#include <stdio.h>
#include <string.h>
#include <string>

// *nix headers
#include <fcntl.h>      // Contains file controls like O_RDWR
#include <errno.h>      // Error integer and strerror() function
#include <termios.h>    // Contains POSIX terminal control definitions
#include <unistd.h>     // write(), read(), close()

/** @file RPiSerial.h
 *  @addtogroup RPiSerial
 *  @brief USB Serial library for use on a RaspberryPi
 *  @namespace RPiSerial
 *  @{
 */
namespace RPiSerial {
    /**
    * @class RPiSerial::RPiSerial RPiSerial.h "RPiSerial.h"
    * @brief Creates and administrates a Serial port communication via the USB ports on a RaspberryPi.
    * @details Serial is a class that can open any available TTY port that the Linux kernel can see. This includes USB
    * ports. Low level library designed to replace WiringPi (special thanks to Gordon for supporting it while he did).
    * Note, being Arduino like was not a design consideration of this library, however, it can be used as such. Finally, while this
    * library was designed with the RaspberryPi in mind it will work on any machine running a Unix-like kernel including macOS.
    */
    class RPiSerial {
    public:
        /**
         * @brief Instantiate a new RPiSerial object
         * @param[in] port <strong>[std::string]</strong> - Desired TTY port to be used, eg /ttyACM0
         * @param[in] baud <strong>[int]</strong> - Baud rate of TTY port (req. by RS-232) can be set to any baud rate since the host interface is USB
         */
        explicit RPiSerial(std::string port, int baud);

        /**
        * @brief Destroys the Serial object. If a Serial port is open the file descriptor will be closed before freeing memory.
        */
        ~RPiSerial();

        /**
        * @brief Sends a message via the virtual serial interface
        * @param[in] data <strong>[char *]</strong> - Buffer containing character data
        * @param[in] len <strong>[long]</strong> - Length of the character buffer to be sent <strong>(MUST NOT EXCEED BUFFER SIZE)</strong>
        */
        bool send(unsigned char  * data, long len);

        /**
        * @brief Sends a single character via the virtual serial interface
        * @param[in] data <strong>[unsigned char]</strong> - Desired ASCII character to be sent
        */
        bool send(unsigned char data);

        /**
        * @brief Sends a string via the virtual serial interface
        * @param[in] data <strong>[std::string]</strong> - Desired string to be sent
        */
        bool send(std::string data);

        /**
        * @brief Reads the virtual serial interface's file descriptor
        * @param[in] len <strong>[long]</strong> - Length of the character buffer <strong>(MUST NOT EXCEED BUFFER SIZE)</strong>
        * @param[out] data <strong>[unsigned char *]</strong> - The character buffer in which to store the contents of the interface file descriptor
        * @return readLength <strong>[int]</strong> - The length (in bytes) read from the virtual interface file descriptor.
        * Returns -1 if the file descriptor could not be read.
        */
        long recv( unsigned char * data, long len);

        /**
        * @brief Opens the file descriptor of the virtual interface
        * @return status <strong>[bool]</strong> - Returns if the virtual interface could be opened (aka if the file descriptor could be opened)
        */
        bool openPort();

        /**
        * @brief Closes the open file descriptor of the virtual interface
        */
        void closePort();

        /**
        * @brief Returns the state of the private _isOpen member variable
        * @return status <strong>[bool]</strong> - The state of the private _isOpen member variable
        */
        bool getIsOpen() const ;

        /**
        * @brief Returns the number of bytes that have been read over the virtual interface
        * @return bytesRead <strong>[long]</strong> - The current value of bytes read over the virtual interface
        */
        long getNumRecv() const;

    private:
        int _fd, _baud;
        long _numRead;
        bool _isOpen;
        std::string _portName;
        struct termios _tty;
    };
}

/** @} */
