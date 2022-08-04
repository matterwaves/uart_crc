#ifndef _UART_CRC_H_
#define _UART_CRC_H_


#include <stdint.h>
#ifdef MBED_H
  #include "mbed.h"
  #include "cycle_count_delay.h" //cycle_delay_ms()
#endif

#ifdef Arduino_h
  #include <Arduino.h>
#endif

class UART_CRC {

public:
  enum CmdResult {
    /// Failed operation
    Failure,
    /// Successful operation
    Success
  };


#ifdef MBED_H
    ///@brief uart_crc constructor [mbed]
    ///@param[in] buff_size - size of tx/rx buffer to keep (bytes), last 2 bytes for CRC 
    ///@param[in] max_attempts - number of packet resend attempts
    ///@param[in] timeout_ms - timeout for waiting for ACK/NACK
    UART_CRC(PinName tx, PinName rx, PinName tx_active, PinName rx_active, 
            uint16_t baud = 9600, uint8_t max_attempts = 16,uint8_t timeout_ms = 100);
    UARTSerial uart_;
    Serial debug_;
    DigitalOut tx_active;
    DigitalIn rx_active;
#endif

#ifdef Arduino_h
    ///@brief uart_crc constructor [arduino]
    ///@param[in] buff_size - size of tx/rx buffer to keep (bytes), last 2 bytes for CRC 
    ///@param[in] max_attempts - number of packet resend attempts
    ///@param[in] timeout_ms - timeout for waiting for ACK/NACK
    UART_CRC( uint8_t tx_active, uint8_t rx_active, 
            uint16_t baud = 9600, uint8_t max_attempts = 16,uint8_t timeout_ms = 100);

    void begin();

    //Change this if using a different serial port on the arduino
    HardwareSerial &uart_ = Serial1;
    uint16_t baud;
    uint8_t tx_active;
    uint8_t rx_active;

#endif

    ///@brief uart_crc destructor
    ~UART_CRC(void);

    ///@brief uart_crc intializer
    void init(void);

    ///@brief
    CmdResult tx_message();

    ///@brief
    CmdResult rx_message();

    ///@brief checks if a message is pending on the uart line
    //@returns True or False
    bool pending();
    
    ///@brief compute the CCIT-16-False CRC
    ///@param[in] ptr - pointer to buffer which contains message
    ///@param[in] number of bytes in buffer to compute crc for
    ///@returns - 16-bit CRC polynomial remainder
    uint16_t calcrc(char *ptr, uint16_t len);

    ///@brief reset rx_buff to 0000...
    void flush_rx();
    ///@brief reset tx_buff to 0000...
    void flush_tx();
    
    ///@brief stage a message for UART comm
    ///@brief cumulatively tracks bytes_wrote
    ///@param[in] to_write - pointer to buffer to write from
    ///@param[in] len - number of bytes to write  
    void write_tx_buff(char *to_write, uint16_t len);

    ///@brief Set the state of tx_active pin to indicate
    ///@brief to receiver that a message is incoming
    ///@param[in] desired state of the tx_active pin
    void set_tx_active(bool STATE);


    ///@brief flush rx buff if no rx_active flag
    void flush_if_no_msg();


    static const uint16_t buff_size = 32; 
    uint8_t  max_attempts;
    uint8_t timeout_ms;


    char rx_buff[buff_size];
    char tx_buff[buff_size];
    uint16_t bytes_wrote=0;


private:
    char crc_buff[2]={0,0};
    uint16_t crc=0;
    uint16_t bytes_read=0;
    const char NACK[2]={0,0};
    uint8_t ack_bytes=0;
    char trash=0;
#ifdef Arduino_h
    elapsedMillis ack_timer;
#endif
#ifdef MBED_H
    Timer ack_timer;
#endif

    ///@brief dump the buffer thats managed by the serial driver
    void flush_uart_rx();
    ///@brief wait until message has been sent
    void wait_for_send();
    ///@brief check if there is any data to read on the uart line
    bool readable();
    ///@brief read a single char from the stream
    //@param[in] buff - pointer to buffer to store the char
    void get_c(char *buff);

    ///@brief interface for blocking delays
    void wait_ms(uint16_t ms);

    ///@brief interface for resetting stop-watch
    // Resets count, and starts timer
    void reset_ack_timer();
    ///@brief interface for reading elapsed time in ms
    uint16_t check_ack_timer();
};




#endif //_UART_CRC_H_
