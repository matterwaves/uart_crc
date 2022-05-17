#include <Arduino.h>
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
UART_CRC::UART_CRC(uint8_t tx_active, uint8_t rx_active,
        uint16_t baud /*9600*/, uint8_t max_attempts /*15*/,uint8_t timeout_ms/*100*/): 
    tx_active(tx_active),rx_active(rx_active), baud(baud),
    max_attempts(max_attempts),timeout_ms(timeout_ms)
    {
      //HardwareSerial &uart_ = Serial1;
    }

void UART_CRC::begin(){
      pinMode(tx_active,OUTPUT);
      pinMode(rx_active,INPUT);
      digitalWrite(tx_active,LOW);
      flush_tx();
      flush_rx();
    Serial1.begin(baud);
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
    while(uart_.readable()){
        uart_.read(&trash,1);
    };
#endif
#ifdef Arduino_h
    while(uart_.available()){uart_.read();};
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
    //Serial.println("msg pending");
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

void UART_CRC::get_c(char *buff){
#ifdef MBED_H
    uart_.read(buff,1);
#endif

#ifdef Arduino_h
    *buff = uart_.read();
#endif
}


void UART_CRC::set_tx_active(bool STATE){
#ifdef MBED_H
    tx_active = STATE;
#endif
#ifdef Arduino_h
    digitalWrite(tx_active, STATE);
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
        //At low baud rates, this delay is necessary to make sure msg is complete
        // minimum delay (ms) = 1000*buff_size*8/baud
        wait_ms(27);
        //Read all chars in the buffer
        while (readable()){
          get_c(&rx_buff[bytes_read++]);
        }  
        //If buffer is nonempty, do a CRC
        if (bytes_read >0 ){
#ifdef Arduino_h
          Serial.print("rx'd: ");
          for (int i=0; i< buff_size;i++){
              Serial.print(rx_buff[i]);
          }
          Serial.println();
          Serial.print("Bytes read: ");
          Serial.println(bytes_read);
#endif
          crc= calcrc(rx_buff,buff_size);
          if (crc == 0 ){
            //CRC Passed, send ACK: last two bytes of buffer
            crc_buff[0]=rx_buff[buff_size-1];
            crc_buff[1]=rx_buff[buff_size-2];
            uart_.write(crc_buff,2);
            wait_for_send();
#ifdef Arduino_h
            Serial.print("rx success @ attempt: ");
            Serial.print(attempt);
            Serial.print(" CRC token: ");
            Serial.print(crc_buff[0],HEX);
            Serial.print(crc_buff[1],HEX);
            Serial.println();
#endif
            return CmdResult::Success;
          }
          else{
            //CRC Failed, send NACK to request resend
            uart_.write(NACK,2);
#ifdef Arduino_h
            Serial.print("rx fail @ attempt: ");
            Serial.print(attempt);
            Serial.print("  Calc'd CRC: ");
            Serial.print(crc,HEX);
            Serial.println();
#endif
            wait_for_send();
            if (attempt< max_attempts-1){
                flush_rx();
            }
          }
        }
        else{
            //No bytes read dont count this as an attempt
            attempt--;
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

  set_tx_active(1);
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
        get_c( &crc_buff[ack_bytes++]);
      }
     }
     ack_bytes=0;
    //Flush UART rx buff 
    //while(uart_.readable()){uart_.read(&trash,1);}
     if ( (crc&0xFF) == crc_buff[0] && (crc>>8) == crc_buff[1] ) {
      //msg sent successfully, we're done here
      flush_tx();
      set_tx_active(0);
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
  set_tx_active(0);
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



