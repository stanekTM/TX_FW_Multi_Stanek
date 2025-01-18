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
 
 Thanks to  Goebish ,Ported  from his deviation firmware
 */

#if defined(YUXIANG_NRF24L01_INO)

#include "iface_xn297.h"

#define YUXIANG_FORCE_ID

#define YUXIANG_PACKET_PERIOD		12422
#define YUXIANG_PACKET_SIZE			9
#define YUXIANG_BIND_COUNT			150
#define YUXIANG_BIND_FREQ			0x30
#define YUXIANG_RF_NUM_CHANNELS		4

#define YUXIANG_WRITE_TIME			1000
//#define YUXIANG_TELEM_DEBUG

enum 
{
	YUXIANG_DATA = 0,
	YUXIANG_RX
};

static void __attribute__((unused)) YUXIANG_send_packet()
{
	if(bind_counter && packet_sent < 5 && (hopping_frequency_no & 0x07) == 0)
	{
		bind_counter--;
		if(!bind_counter)
			BIND_DONE;
	#if 0
		debug("B C:%d, ",YUXIANG_BIND_FREQ);
	#endif
		XN297_RFChannel(YUXIANG_BIND_FREQ);
		XN297_SetTXAddr((uint8_t*)"\x00\x00\x00\x00\x00", 5);
		bind_phase = 1;
		packet_sent++;
	}
	else
	{//Normal operation
		XN297_Hopping(hopping_frequency_no & 0x03);
	#if 0
		debug("C:%d, ",hopping_frequency[hopping_frequency_no & 0x03]);
	#endif
		hopping_frequency_no++;
		if(bind_phase)
		{
			XN297_SetTXAddr(rx_tx_addr, 5);
			XN297_SetRXAddr(rx_tx_addr, YUXIANG_PACKET_SIZE);
			bind_phase = 0;
			packet_sent = 0;
		}
	}

	packet[0] = GET_FLAG(!bind_phase, 0x80)		// Bind packet
			| GET_FLAG(telemetry_lost, 0x20)	// No telem
			| GET_FLAG(!CH5_SW, 0x10)			// Lock
			| GET_FLAG(CH6_SW, 0x08)			// High
			| GET_FLAG(CH11_SW, 0x01);			// Screw pitch -> temporary
	
	packet[1] = GET_FLAG(CH7_SW, 0x08)			// Land only when unlock
			| GET_FLAG(CH10_SW, 0x20);			// Mode

	packet[2] = GET_FLAG(CH5_SW, 0x02)			// Altitude hold set when unlock
			| GET_FLAG(CH8_SW, 0x01)			// Manual
			| GET_FLAG(CH9_SW, 0x40);			// Flip

	uint16_t value = convert_channel_16b_limit(AILERON,0,1000);
	packet[3] = value;
	packet[7] = value >> 8;
	value = convert_channel_16b_limit(ELEVATOR,0,1000);
	packet[4] = value;
	packet[7] |= (value >> 6) & 0x0C;
	value = convert_channel_16b_limit(THROTTLE,0,1000);
	packet[5] = value;
	packet[7] |= (value >> 4) & 0x30;
	value = convert_channel_16b_limit(RUDDER,0,1000);
	packet[6] = value;
	packet[7] |= (value >> 2) & 0xC0;

	if(bind_phase)
		memcpy(&packet[3], rx_tx_addr, 4);

	uint8_t checksum = 0;
	for(uint8_t i=0; i<YUXIANG_PACKET_SIZE-1; i++)
		checksum += packet[i];
	packet[8] = checksum;
	
	#if 0
		debug("P:");
		for(uint8_t i=0;i<YUXIANG_PACKET_SIZE;i++)
			debug(" %02X",packet[i]);
		debugln("");
	#endif
	// Send
	XN297_SetPower();
	XN297_SetTxRxMode(TX_EN);
	XN297_WritePayload(packet, YUXIANG_PACKET_SIZE);
}

static void __attribute__((unused)) YUXIANG_RF_init()
{
	XN297_Configure(XN297_CRCEN, XN297_SCRAMBLED, XN297_1M);
}

static void __attribute__((unused)) YUXIANG_initialize_txid()
{
	#ifdef YUXIANG_FORCE_ID
		if(RX_num==0)
		{//TX1
			memcpy(rx_tx_addr,(uint8_t *)"\xB3\x13\x36\xDD",4); //rx_tx_addr[4]=0xD9
			memcpy(hopping_frequency,(uint8_t *)"\x49\x32\x35\x42",4);
		}
		else
		{//TX2
			memcpy(rx_tx_addr,(uint8_t *)"\xEB\x13\x36\xAC",4); //rx_tx_addr[4]=0xE0
			memcpy(hopping_frequency,(uint8_t *)"\x4D\x3A\x3E\x47",4);
		}
	#endif
	uint8_t sum=0;
	for(uint8_t i=0; i<4; i++)
		sum += rx_tx_addr[i];
	rx_tx_addr[4] = sum;
	debugln("ID: %02X %02X %02X %02X %02X , HOP: %02X %02X %02X %02X",rx_tx_addr[0],rx_tx_addr[1],rx_tx_addr[2],rx_tx_addr[3],rx_tx_addr[4],hopping_frequency[0],hopping_frequency[1],hopping_frequency[2],hopping_frequency[3]);
}

uint16_t YUXIANG_callback()
{
	bool rx = false;
	
	switch(phase)
	{
		case YUXIANG_DATA:
			rx = XN297_IsRX();					// Needed for the NRF24L01 since otherwise the bit gets cleared
			XN297_SetTxRxMode(TXRX_OFF);
	#ifdef YUXIANG_HUB_TELEMETRY
			if(packet_count > 240)				// Around 3sec with no telemetry
				telemetry_lost = 1;
			else
				packet_count++;
	#endif
			#ifdef MULTI_SYNC
				telemetry_set_input_sync(YUXIANG_PACKET_PERIOD);
			#endif
			YUXIANG_send_packet();
			if(rx)
			{ // Check if a packet has been received
				#ifdef YUXIANG_TELEM_DEBUG
					debug("RX ");
				#endif
				if(XN297_ReadPayload(packet_in, YUXIANG_PACKET_SIZE))
				{ // packet with good CRC and length
					uint8_t checksum = 0;
					for(uint8_t i=0; i<YUXIANG_PACKET_SIZE-1; i++)
						checksum += packet_in[i];
					if(packet_in[8] == checksum)
					{
	#ifdef YUXIANG_HUB_TELEMETRY
						if(packet_in[0]==0x78)
						{
							#ifdef YUXIANG_TELEM_DEBUG
								debug("OK:");
								for(uint8_t i=0;i<YUXIANG_PACKET_SIZE;i++)
									debug(" %02X",packet_in[i]);
							#endif
							v_lipo1 = packet_in[4];
							v_lipo2 = packet_in[6];
						}
						telemetry_link = 1;
	#endif
						telemetry_lost = 0;
						packet_count = 0;
						bind_counter = 0;	// Stop bind
						BIND_DONE;
					}
					#ifdef YUXIANG_TELEM_DEBUG
					else // Bad packet
						debug(" NOK");
					#endif
				}
				#ifdef YUXIANG_TELEM_DEBUG
				else // Bad packet
					debug("NOK");
				debugln("");
				#endif
				rx = false;
			}
			phase++;
			return YUXIANG_WRITE_TIME;
		default:
			// RX
			{ // Wait for packet to be sent before switching to receive mode
				uint16_t start=(uint16_t)micros();
				while ((uint16_t)((uint16_t)micros()-(uint16_t)start) < 500)
					if(XN297_IsPacketSent())
						break;
			}
			//if(bind_phase)
			//	XN297_Hopping(3);
			XN297_SetTxRxMode(RX_EN);
			phase = YUXIANG_DATA;
			return YUXIANG_PACKET_PERIOD - YUXIANG_WRITE_TIME;
	}
	return 0;
}

void YUXIANG_init(void)
{
	YUXIANG_initialize_txid();
	YUXIANG_RF_init();

	if(IS_BIND_IN_PROGRESS)
		bind_counter = YUXIANG_BIND_COUNT;
	else
		bind_counter = 0;
		
	phase = YUXIANG_DATA;
	hopping_frequency_no = 0;
	bind_phase = 1;
	packet_sent = 8;
	#ifdef YUXIANG_HUB_TELEMETRY
		packet_count = 0;
		telemetry_lost = 1;
		RX_RSSI = 100;		// Dummy value
	#endif
}

#endif