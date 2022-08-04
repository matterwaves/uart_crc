# uart-crc

uart is great becuase of how simple it is.
However, I need to send some sensitive information for experimental control.
So I need to make sure the messages are sent and recieved correctly.
This driver implements a CRC-16 CCIT-False redundancy check and ACK/NACK.

It should work with mbed-os (tested on mbed-5) and arduino.

## STM32 platform
    * included as a submodule in the git repository

## Arduino platform
    * include from ~/Arduino/libraries/uart_crc 
