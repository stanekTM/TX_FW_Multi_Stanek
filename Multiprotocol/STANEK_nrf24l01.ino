/*
  This project is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Multiprotocol is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Multiprotocol.  If not, see <http://www.gnu.org/licenses/>.
*/

//***************************************************************************************
// Support for custom Arduino-based DIY receivers with RF24 library from this repository:
// https://github.com/stanekTM/RX_nRF24L01_Telemetry_Motor_Servo
//
// Included communication nRF24L01P "Stanek". Fixed RF channel, fixed address.
// Channel reduction in sub protocols 2, 3, 4, 5, 6, 8, 10 and 12ch.
//***************************************************************************************


#if defined(STANEK_NRF24L01_INO)

#include "iface_nrf24l01.h"


uint8_t TX_RX_ADDRESS[] = "jirka";      // setting RF channels address (5 bytes number or character)

#define STANEK_RF_CHANNEL      76       // which RF channel to communicate on (0-125, 2.4Ghz + 76 = 2.476Ghz)

#define STANEK_PACKET_PERIOD   3000     // do not set too low or else next packet may not be finished transmitting before the channel is changed next time around

#define STANEK_RC_CHANNELS     12       // number of RC channels that can be sent in one packet

#define STANEK_RC_PACKET_SIZE  24       // STANEK_RC_PACKET_SIZE = STANEK_RC_CHANNELS * 2

#define STANEK_TELEMETRY_PACKET_SIZE  3 // RSSI, A1, A2

//**********************************************************************************************************************************
//**********************************************************************************************************************************
//**********************************************************************************************************************************
static void __attribute__((unused)) STANEK_setAddress()
{
  rf_ch_num = STANEK_RF_CHANNEL; // initialize the channel
  
  NRF24L01_WriteRegisterMulti(NRF24L01_10_TX_ADDR,    (uint8_t*)(&TX_RX_ADDRESS), 5);
  NRF24L01_WriteRegisterMulti(NRF24L01_0A_RX_ADDR_P0, (uint8_t*)(&TX_RX_ADDRESS), 5);
}

//**********************************************************************************************************************************
//**********************************************************************************************************************************
//**********************************************************************************************************************************
static void __attribute__((unused)) STANEK_RF_init()
{
  NRF24L01_Initialize();
  
  STANEK_setAddress();
  
  NRF24L01_SetBitrate(NRF24L01_BR_250K);        // NRF24L01_BR_250K (fails for units without +), NRF24L01_BR_1M, NRF24L01_BR_2M
  NRF24L01_WriteReg(NRF24L01_1C_DYNPD, 0x3F);   // enable dynamic payload length on all pipes
  NRF24L01_WriteReg(NRF24L01_1D_FEATURE, 0x04); // enable dynamic Payload Length
  NRF24L01_SetTxRxMode(TX_EN);                  // clear data ready, data sent, retransmit and enable CRC 16bits, ready for TX
  delayMilliseconds(10);
}

//**********************************************************************************************************************************
//**********************************************************************************************************************************
//**********************************************************************************************************************************
static void __attribute__((unused)) STANEK_get_telemetry()
{
  // calculate TX rssi based on past 250 expected telemetry packets. Cannot use full second count because telemetry_counter is not large enough
  state++;
  
  if (state > 250)
  {
    TX_RSSI = telemetry_counter;
    telemetry_counter = 0;
    state = 0;
    telemetry_lost = 0;
  }
  
  // process incoming telemetry packet of it was received
  if (NRF24L01_ReadReg(NRF24L01_07_STATUS) & _BV(NRF24L01_07_RX_DR))
  {
    // data received from model
    NRF24L01_ReadPayload(packet, STANEK_TELEMETRY_PACKET_SIZE);
    
    RX_RSSI = packet[0]; // packet rate 0 to 255 where 255 is 100% packet rate
    v_lipo1 = packet[1]; // directly from analog input of receiver, but reduced to 8-bit depth (0 to 255). Scaling depends on the input to the analog pin of the receiver
    v_lipo2 = packet[2]; // directly from analog input of receiver, but reduced to 8-bit depth (0 to 255). Scaling depends on the input to the analog pin of the receiver
    
    telemetry_counter++;
    
    if (telemetry_lost == 0)
    {
      telemetry_link = 1;
    }
  }
  else
  {
    // if no telemetry packet was received then delay by the typical telemetry packet processing time.
    // This is done to try to keep the sendPacket process timing more consistent. Since the SPI payload read takes some time
    delayMicroseconds(50);
  }
  
  NRF24L01_SetTxRxMode(TX_EN);
  NRF24L01_FlushRx();
}

//**********************************************************************************************************************************
//**********************************************************************************************************************************
//**********************************************************************************************************************************
static void __attribute__((unused)) STANEK_send_packet()
{
  STANEK_get_telemetry();
  
  switch (sub_protocol)
  {
    case 1: num_ch = 3;
    break;
    case 2: num_ch = 4;
    break;
    case 3: num_ch = 5;
    break;
    case 4: num_ch = 6;
    break;
    case 5: num_ch = 8;
    break;
    case 6: num_ch = 10;
    break;
    case 7: num_ch = 12;
    break;
    default: num_ch = 2;
    break;
  }
  
  uint8_t rc_channels_reduction = constrain(12 - num_ch, 0, STANEK_RC_CHANNELS);
  
  uint8_t packet_size = STANEK_RC_PACKET_SIZE - (rc_channels_reduction * 2); // 2ch = 20, 18, 16, 14, ... -> 10ch = 4, 12ch = 0
  
  uint8_t packet[STANEK_RC_PACKET_SIZE] = {0};
  
  
  uint8_t payload_index = 0;
  uint16_t hold_value;
  
  for (uint8_t x = 0; (x < STANEK_RC_CHANNELS - rc_channels_reduction); x++)
  {
    hold_value = convert_channel_16b_limit(x, 1000, 2000); // valid channel values are 1000 to 2000
    
    // use 12 bits per value
    hold_value &= 0x0FFF; // 4095
    
    packet[0 + payload_index] |= hold_value & 0xFF; // 255
    payload_index++;
    packet[0 + payload_index] |= hold_value >> 8;
    payload_index++;
      
    /*
    packet[0 + payload_index] |= (uint8_t)(hold_value & 0x00FF); // 255
    payload_index++;
    packet[0 + payload_index] |= (uint8_t)((hold_value >> 8) & 0x00FF);
    payload_index++;
    */
  }
  
  NRF24L01_WriteReg(NRF24L01_05_RF_CH, rf_ch_num); // send channel
  NRF24L01_SetPower();
  NRF24L01_WritePayload(packet, packet_size);      // and payload
  
  
  // switch radio to rx as soon as packet is sent
  // calculate transmit time based on packet size and data rate of 1MB per sec
  // this is done because polling the status register during xmit caused issues.
  // bits = packet_size * 8  +  73 bits overhead
  // at 250 Kbs per sec, one bit is 4 uS
  // then add 140 uS which is 130 uS to begin the xmit and 10 uS fudge factor
  delayMicroseconds(((((unsigned long)packet_size * 8ul)  +  73ul) * 4ul) + 140ul);
  
  // increase packet period by 100 us for each channel over 6
  packet_period = STANEK_PACKET_PERIOD + (constrain(((int16_t)(STANEK_RC_CHANNELS - rc_channels_reduction) - (int16_t)6), (int16_t)0, (int16_t)10) * (int16_t)100);
  
  NRF24L01_WriteReg(NRF24L01_00_CONFIG, 0x7F); // RX mode with 16 bit CRC no IRQ
  //NRF24L01_WriteReg(NRF24L01_00_CONFIG, 0x0F); // RX mode with 16 bit CRC
}

//**********************************************************************************************************************************
//**********************************************************************************************************************************
//**********************************************************************************************************************************
uint16_t STANEK_callback()
{
  STANEK_send_packet(); // packet_period is set/adjusted in STANEK_send_packet

#ifdef MULTI_SYNC
  telemetry_set_input_sync(packet_period);
#endif

  return packet_period;
}

//**********************************************************************************************************************************
//**********************************************************************************************************************************
//**********************************************************************************************************************************
void STANEK_init(void)
{
  BIND_DONE;
  STANEK_RF_init();
}

#endif
 
