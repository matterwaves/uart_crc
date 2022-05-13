#include "uart_crc.h"
    

#ifdef MBED_H
UART_CRC::UART_CRC(PinName tx, PinName rx, PinName tx_active, PinName rx_active,
        uint16_t baud /*9600*/, uint8_t max_attempts /*15*/,uint8_t timeout_ms/*100*/): 
    tx_active(tx_active,0),rx_active(rx_active),
    max_attempts(max_attempts),timeout_ms(timeout_ms),
    uart_(tx,rx,baud){
      flush_tx();
      flush_rx();
    }

#endif

#ifdef Arduino_h
UART_CRC::UART_CRC(uint8_t tx, uint8_t rx, uint8_t tx_active, uint8_t rx_active,
        uint16_t baud /*9600*/, uint8_t max_attempts /*15*/,uint8_t timeout_ms/*100*/): 
    tx_active(tx_active),rx_active(rx_active),
    max_attempts(max_attempts),timeout_ms(timeout_ms),
    {
        pinMode(tx_active,OUTPUT);
        pinMode(rx_active,INPUT);
        digitalWrite(tx_active,LOW);
      flush_tx();
      flush_rx();
      Serial1.begin(baud);
      uart_ = Serial1;
     //uart_(tx,rx,baud)
    }
#endif



void UART_CRC::init(){
  //Create and intialize tx/rx buffer
  flush_tx();
  flush_rx();
}

UART_CRC::~UART_CRC(){}

void UART_CRC::flush_rx(){
    for (int i =0; i<buff_size; i++){
        rx_buff[i]=0;
    }
    bytes_read=0;
}

void UART_CRC::flush_tx(){
    for (int i =0; i<buff_size; i++){
        tx_buff[i]=0;
    }
    bytes_wrote=0;
}

void UART_CRC::write_tx_buff(char *to_write, uint16_t len){
  len++;
  while (--len >0){
     tx_buff[bytes_wrote++]= *to_write++;
  }
}
void UART_CRC::flush_uart_rx(){
#ifdef MBED_H
    while(uart_.readable()){uart_.read(&trash,1)};
#endif
#ifdef Arduino_h
    while(uart_.available()){uart_.read()};
#endif
}

void UART_CRC::wait_for_send(){
#ifdef MBED_H
    uart_.sync();
#endif
#ifdef Arduino_h
    uart_.flush();
#endif
}
 
bool UART_CRC::pending(){
#ifdef MBED_H
    return (uart_.readable() && rx_active);
#endif
#ifdef Arduino_h
    return (uart_.available() && digitalRead(rx_active) );
#endif
}

bool UART_CRC::readable(){
#ifdef MBED_H
    return uart_.readable();
#endif
#ifdef Arduino_h
    return uart_.available();
#endif
}

void UART_CRC::wait_ms(uint16_t ms){
#ifdef MBED_H
    cycle_delay_ms(ms);
#endif
#ifdef Arduino_h
    delay(ms);
#endif
}


UART_CRC::CmdResult UART_CRC::rx_message(){
        /*
         * todo:
         * Add timeout
         *
         */
    //Attempt to recv message until max attempts exceeded
      for (uint8_t attempt = 0; attempt < max_attempts; attempt++){
        //Read all chars in the buffer
        while (this->uart_.readable()){
          this->uart_.read(&rx_buff[bytes_read++],1);
        }
        

        //If buffer is nonempty, do a CRC
        if (bytes_read >0 ){
          crc= calcrc(rx_buff,buff_size);
          if (crc == 0 ){
            //CRC Passed, send ACK: last two bytes of buffer
            crc_buff[0]=rx_buff[buff_size-1];
            crc_buff[1]=rx_buff[buff_size-2];
            this->uart_.write(crc_buff,2);
            this->uart_.sync();
            return CmdResult::Success;
          }
          else{
            //CRC Failed, send NACK to request resend
            this->uart_.write(NACK,2);
            this->uart_.sync();
            if (attempt< max_attempts-1){
                flush_rx();
            }
          }
        }
        //cycle_delay_ms(10);
        wait_ms(10);
      }
      //msg recv failed
      flush_rx();
      return CmdResult::Failure;
    }

UART_CRC::CmdResult UART_CRC::tx_message(){
  crc=calcrc(tx_buff,buff_size-2);
  tx_buff[buff_size-1]=crc & 0xFF;
  tx_buff[buff_size-2]=crc >> 8;

  tx_active=1;
  //Main loop for sending message
  for (uint8_t num_attempts=0; num_attempts < max_attempts; num_attempts++){
    
    //Flush the UART rx buff
    //Make sure that next bytes read will be the ACK/NACK
    flush_uart_rx();
    //while(uart_.readable()){uart_.read(&trash,1);}
    
    //Send over UART
    uart_.write(tx_buff,buff_size);
    //Wait until send it complete
    //uart_.sync();
    wait_for_send();
    //cycle_delay_ms(10);
    wait_ms(10);
    
    //Wait for ACK/NACK response
    //Serial.println(F("Waiting for ACK/NACK"));
    while (ack_bytes < 2){
      //if (uart_.readable()){
      if (readable()){
        uart_.read( &crc_buff[ack_bytes++],1 );
      }
     }
     ack_bytes=0;
    //Flush UART rx buff 
    //while(uart_.readable()){uart_.read(&trash,1);}
     if ( (crc&0xFF) == crc_buff[0] && (crc>>8) == crc_buff[1] ) {
      //msg sent successfully, we're done here
      flush_tx();
      tx_active=0;
      return CmdResult::Success ; 
     }
     else{
     }
     //Wait 50ms between send attempts
     //cycle_delay_ms(50);
     wait_ms(50);
   }
  //max number of attempts failed.
  //time to give up
  flush_tx();
  tx_active=0;
  return CmdResult::Failure;
}
uint16_t UART_CRC::calcrc(char *ptr, uint16_t len)
    {
      //init=0xFFFF for CCIT-16-False
      //init=0x0000 for XMODEM
      //len = number of bytes
        uint16_t  crc=0xFFFF;
        char i;
        const uint16_t polynome = 0x1021;
    
        //Initialized to 0
        len++;
        while (--len > 0)
        {
            // Load next byte to the left of the current CRC
            // Why << 8 instead of <<16? the CRCpolynomial is 16 bit
            crc = crc ^ (int) *ptr++ << 8;
            i = 8;
            do
            {
                //Detect leading 1         
                if (crc & 0x8000)
                    //XOR with polynomial
                    crc = crc << 1 ^ polynome;
                else
                    //Shift till next leading 1
                    crc = crc << 1;
            } while(--i);
        }
        return (crc);
    }



