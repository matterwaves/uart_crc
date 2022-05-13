#include "uart_crc.h"
    
UART_CRC::UART_CRC(PinName tx, PinName rx, PinName tx_active, PinName rx_active,
        uint16_t baud /*9600*/, uint8_t max_attempts /*15*/,uint8_t timeout_ms/*100*/): 
    tx_active(tx_active,0),rx_active(rx_active),
    max_attempts(max_attempts),timeout_ms(timeout_ms),
    uart_(tx,rx,baud){
      flush_tx();
      flush_rx();
    }

void UART_CRC::init(){
  //Create and intialize tx/rx buffer
  flush_tx();
  flush_rx();
}

UART_CRC::~UART_CRC(){}

void UART_CRC::flush_rx(){
      memset(&(this->rx_buff),0,buff_size);
      bytes_read=0;
    }
void UART_CRC::flush_tx(){
      memset(&(this->tx_buff),0,buff_size);
      bytes_wrote=0;
    }
void UART_CRC::write_tx_buff(char *to_write, uint16_t len){
  len++;
  while (--len >0){
     tx_buff[bytes_wrote++]= *to_write++;
  }
}
 
bool UART_CRC::available(){
    return (uart_.readable() && rx_active);
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
        cycle_delay_ms(10);
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
    while(uart_.readable()){uart_.read(&trash,1);}
    
    //Send over UART
    uart_.write(tx_buff,buff_size);
    //Wait until send it complete
    uart_.sync();
    cycle_delay_ms(10);
    
    //Wait for ACK/NACK response
    //Serial.println(F("Waiting for ACK/NACK"));
    while (ack_bytes < 2){
      if (uart_.readable()){
        uart_.read( &crc_buff[ack_bytes++],1 );
      }
     }
     ack_bytes=0;
    //Flush UART rx buff 
    while(uart_.readable()){uart_.read(&trash,1);}
     if ( (crc&0xFF) == crc_buff[0] && (crc>>8) == crc_buff[1] ) {
      //msg sent successfully, we're done here
      flush_tx();
      tx_active=0;
      return CmdResult::Success ; 
     }
     else{
     }
     //Wait 50ms between send attempts
     cycle_delay_ms(50);
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



