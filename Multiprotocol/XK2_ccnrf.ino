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
// Compatible with XK TX X4 and model A160S.

#if defined(XK2_CCNRF_INO)

#include "iface_xn297.h"

#define FORCE_XK2_ID

#define XK2_RF_BIND_CHANNEL	71
#define XK2_PAYLOAD_SIZE	9
#define XK2_PACKET_PERIOD	4911
#define XK2_RF_NUM_CHANNELS	4

enum {
	XK2_BIND1,
	XK2_BIND2,
	XK2_DATA_PREP,
	XK2_DATA
};

static void __attribute__((unused)) XK2_send_packet()
{
	static uint8_t trim_ch=0;
	
	if(IS_BIND_IN_PROGRESS)
	{
		packet[0] = 0x9D;
		//TXID
		memcpy(&packet[1], rx_tx_addr, 3);
		//RXID
		//memcpy(&packet[4], rx_id     , 3);
		//Unknown
		packet[7] = 0x00;
		//Checksum seed
		packet[8] = 0xC0;												//Constant?
	}
	else
	{
		XN297_Hopping(hopping_frequency_no);
		hopping_frequency_no++;
		hopping_frequency_no &= 0x03;
		//Channels
		packet[0] = convert_channel_16b_limit(AILERON ,0x00,0x64);		//Aileron
		packet[1] = convert_channel_16b_limit(ELEVATOR,0x00,0x64);		//Elevator
		packet[2] = convert_channel_16b_limit(THROTTLE,0x00,0x64);		//Throttle
		packet[3] = convert_channel_16b_limit(RUDDER  ,0x00,0x64);		//Rudder
		//Center the trims
		trim_ch++;
		if(trim_ch > 2) trim_ch = 0;
		packet[4] = 0x20 + 0x40 * trim_ch;								//Trims are A=01..20..3F/E=41..60..7F/R=81..A0..BF, E0 appears when telemetry is received, C1 when p[6] changes from 00->08, C0 when p[6] changes from 08->00
		if(trim_ch == 2)												//Drive rudder trim since otherwise there is no control...
		{
			packet[4] = 0x80 + (convert_channel_8b(RUDDER)>>2);
			if(packet[4] <= 0x81) packet[4] = 0x81;
		}
		//Flags
		packet[5] = GET_FLAG(CH5_SW, 0x01)								//Rate
				  | GET_FLAG(CH6_SW, 0x08)								//Mode
				  | GET_FLAG(CH7_SW, 0x20);								//Hover
		//Telemetry not received=00, Telemetry received=01 but sometimes switch to 1 even if telemetry is not there...
		packet[6] = 0x00;
		//Unknown
		packet[7] = 0x5A;												//Constant?
		//Checksum seed
		packet[8] = 0x7F;												//Constant?
	}
	//Checksum
	for(uint8_t i=0; i<XK2_PAYLOAD_SIZE-1; i++)
		packet[8] += packet[i];

	// Send
	XN297_SetPower();
	XN297_SetTxRxMode(TX_EN);
	XN297_WritePayload(packet, XK2_PAYLOAD_SIZE);
	#if 0
		debug("P");
		for(uint8_t i=0; i<XK2_PAYLOAD_SIZE; i++)
			debug(" %02X",packet[i]);
		debugln();
	#endif
}

static void __attribute__((unused)) XK2_RF_init()
{
	XN297_Configure(XN297_CRCEN, XN297_SCRAMBLED, XN297_250K);
	
	XN297_SetTXAddr((uint8_t*)"\xcc\xcc\xcc\xcc\xcc", 5);
	XN297_SetRXAddr((uint8_t*)"\xcc\xcc\xcc\xcc\xcc", XK2_PAYLOAD_SIZE);

	XN297_HoppingCalib(XK2_RF_NUM_CHANNELS);
	XN297_RFChannel(XK2_RF_BIND_CHANNEL);
}

static void __attribute__((unused)) XK2_initialize_txid()
{
	#ifdef FORCE_XK2_ID
		rx_tx_addr[0] = 0x66;
		rx_tx_addr[1] = 0x4F;
		rx_tx_addr[2] = 0x47;
		for(uint8_t i=0;i<XK2_RF_NUM_CHANNELS;i++)
			hopping_frequency[i] = 65 + i*4;	//65=0x41, 69=0x45, 73=0x49, 77=0x4D
	#endif
	rx_tx_addr[3] = 0xCC;
	rx_tx_addr[4] = 0xCC;
}

uint16_t XK2_callback()
{
	switch(phase)
	{
		case XK2_BIND1:
			// switch to RX mode
			XN297_SetTxRxMode(TXRX_OFF);
			XN297_SetTxRxMode(RX_EN);
			phase++;
			return 5000;
		case XK2_BIND2:
			if(XN297_IsRX())
			{
				XN297_ReadPayload(packet, XK2_PAYLOAD_SIZE);
				#if 0
					debug("RX");
					for(uint8_t i=0; i<XK2_PAYLOAD_SIZE; i++)
						debug(" %02X",packet[i]);
					debugln("");
				#endif
				crc8 = 0xBF;
				for(uint8_t i=0; i<XK2_PAYLOAD_SIZE-1; i++)
					crc8 += packet[i];
				//phase = XK2_BIND1;
				//return 500;
				if(crc8 != packet[8])
				{
					phase = XK2_BIND1;
					return 1000;
				}
				if(packet[0] == 0x9B)
					phase++;
				else
				{
					XN297_SetTxRxMode(TXRX_OFF);
					XN297_SetTxRxMode(TX_EN);
					bind_counter = 10;
					phase = XK2_DATA;
				}
			}
			return 1000;
		case XK2_DATA_PREP:
			XN297_SetTxRxMode(TXRX_OFF);
			XN297_SetTxRxMode(TX_EN);
			XN297_SetTXAddr(rx_tx_addr, 5);
			BIND_DONE;
		case XK2_DATA:
			#ifdef MULTI_SYNC
				telemetry_set_input_sync(XK2_PACKET_PERIOD);
			#endif
			if(bind_counter)
			{
				bind_counter--;
				if(bind_counter == 0)
				{
					phase = XK2_DATA_PREP;
					//phase = XK2_BIND1;
				}
			}
			XK2_send_packet();
			break;
	}
	return XK2_PACKET_PERIOD;
}

void XK2_init()
{
	//BIND_IN_PROGRESS;	// autobind protocol
	XK2_initialize_txid();
	XK2_RF_init();
	
	if(IS_BIND_IN_PROGRESS)
		phase = XK2_BIND1;
	else
		phase = XK2_DATA_PREP;
	bind_counter = 0;
	hopping_frequency_no = 0;
}

#endif

