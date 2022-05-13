#ifndef _UART_CRC_H_
#define _UART_CRC_H_

#include "mbed.h"
#include "cycle_count_delay.h" //cycle_delay_ms()
class UART_CRC {

public:
  enum CmdResult {
    /// Failed operation
    Failure,
    /// Successful operation
    Success
  };

    ///@brief uart_crc constructor
    ///@param[in] buff_size - size of tx/rx buffer to keep (bytes), last 2 bytes for CRC 
    ///@param[in] max_attempts - number of packet resend attempts
    ///@param[in] timeout_ms - timeout for waiting for ACK/NACK
    UART_CRC(PinName tx, PinName rx, PinName tx_active, PinName rx_active, 
            uint16_t baud = 9600, uint8_t max_attempts = 16,uint8_t timeout_ms = 100);

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
    bool available();
    
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


    static const uint16_t buff_size = 32; 
    uint8_t  max_attempts;
    uint8_t timeout_ms;

    UARTSerial uart_;

    char rx_buff[buff_size];


private:
    char crc_buff[2]={0,0};
    uint16_t crc=0;
    uint16_t bytes_read=0;
    uint16_t bytes_wrote=0;
    const char NACK[2]={0,0};
    char tx_buff[buff_size];
    uint8_t ack_bytes=0;
    char trash=0;
    DigitalOut tx_active;
    DigitalIn rx_active;
};




#endif //_UART_CRC_H_
