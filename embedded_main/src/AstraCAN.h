
#include <FlexCAN_T4.h>
//FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can0;


//Convert float to little endian decimal representation
void Float2LEDec(float x, uint8_t (& buffer_data)[8])
{
  unsigned char b[8]={0};
  memcpy(b,&x,4);
  //int* buffer_data[4];
  for(int i=0; i<4; i++){
    buffer_data[i] = b[i];
  }
  for(int i=4; i<8; i++){
    buffer_data[i] = 0;
  }
}


void identifyDevice(FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> &Can0, int can_id)
{
  CAN_message_t msg; 
  msg.flags.extended = 1;

  msg.id = 0x2051D80 + can_id;
  for ( uint8_t i = 0; i < 8; i++ ) msg.buf[i] = 0;
  msg.buf[0] = can_id;
  Can0.write(msg);
}



void sendDutyCycle(FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> &Can0, int can_id, float duty_cycle)
{
  CAN_message_t msg; 
  msg.flags.extended = 1;

  msg.id = 0x2050080 + can_id;
  Float2LEDec(duty_cycle, msg.buf);
  Can0.write(msg);
}

void sendHeartbeat(FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> &Can0, int can_id)
{
  CAN_message_t msg; 
  msg.flags.extended = 1;

  msg.id = 0x2052C80; //non-Rio heartbeat
  for ( uint8_t i = 0; i < 8; i++ ) msg.buf[i] = 0;
  msg.buf[0] = pow(2, can_id);
  Can0.write(msg);
  //Serial.println(msg.id);
}

