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
// Radiolink surface protocol. TXs: RC4GS,RC6GS. Compatible RXs:R7FG(Std),R6FG,R6F,R8EF,R8FM,R8F,R4FGM 

#if defined(RLINK_CC2500_INO)

#include "iface_cc2500.h"

//#define RLINK_DEBUG
//#define RLINK_DEBUG_TELEM

//#define RLINK_FORCE_ID
//#define RLINK_RC4G_FORCE_ID

#define RLINK_TX_PACKET_LEN	33
#define RLINK_RX_PACKET_LEN	15
#define RLINK_TX_ID_LEN		4
#define RLINK_HOP			16

enum {
	RLINK_DATA	= 0x00,
	RLINK_RX1	= 0x01,
	RLINK_RX2	= 0x02,
};

uint32_t RLINK_rand1;
uint32_t RLINK_rand2;

static uint32_t __attribute__((unused)) RLINK_prng_next(uint32_t r)
{
	return 0xA5E2A705 * r + 0x754DB79B;
}

static void __attribute__((unused)) RLINK_init_random(uint32_t id)
{
	uint32_t result = id;

	RLINK_rand2 = result;
	for (uint8_t i=0; i<31; i++)
		result = RLINK_prng_next(result);
	RLINK_rand1 = result;
}

static uint8_t __attribute__((unused)) RLINK_next_random_swap()
{
	uint8_t result = (RLINK_rand2 >> 16) + RLINK_rand2 + (RLINK_rand1 >> 16) + RLINK_rand1;

	RLINK_rand2 = RLINK_prng_next(RLINK_rand2);
	RLINK_rand1 = RLINK_prng_next(RLINK_rand1);

	return result & 0x0F;
}

static uint32_t __attribute__((unused)) RLINK_compute_start_id(uint32_t id)
{
	return id * 0xF65EF9F9u + 0x2EDDF6CAu;
}

static void __attribute__((unused)) RLINK_shuffle_freqs(uint32_t seed)
{
	RLINK_init_random(seed);

	for(uint8_t i=0; i<RLINK_HOP; i++)
	{
		uint8_t r   = RLINK_next_random_swap();
		uint8_t tmp = hopping_frequency[r];
		hopping_frequency[r] = hopping_frequency[i];
		hopping_frequency[i] = tmp;
	}
}

static void __attribute__((unused)) RLINK_hop()
{
	uint8_t inc=3*(rx_tx_addr[0]&3);
	
	// init hop table
	for(uint8_t i=0; i<RLINK_HOP; i++)
		hopping_frequency[i] = (12*i) + inc;

	// shuffle
	RLINK_shuffle_freqs(RLINK_compute_start_id(rx_tx_addr[0] + (rx_tx_addr[1] << 8)));
	RLINK_shuffle_freqs(RLINK_compute_start_id(rx_tx_addr[2] + (rx_tx_addr[3] << 8)));

	// replace one of the channel randomely
	rf_ch_num=random(0xfefefefe)%0x11;		// 0x00..0x10
	if(inc==9) inc=6;						// frequency exception
	hopping_frequency[rf_ch_num]=12*16+inc;
}

static void __attribute__((unused)) RLINK_TXID_init()
{
	#ifdef RLINK_RC4G_FORCE_ID
		//TODO: test any ID
		if(sub_protocol==RLINK_RC4G)
		{
			rx_tx_addr[1]=0x77;
			rx_tx_addr[2]=0x00;
			rx_tx_addr[3]=0x00;
		}
	#endif
	#ifdef RLINK_FORCE_ID
		if(sub_protocol==RLINK_SURFACE)
			memcpy(rx_tx_addr,"\x3A\x99\x22\x3A",RLINK_TX_ID_LEN);	//surface RC6GS
		else
			memcpy(rx_tx_addr,"\xFC\x11\x0D\x20",RLINK_TX_ID_LEN);	//air T8FB
	#endif
	// channels order depend on ID
	if(sub_protocol!=RLINK_RC4G)
		RLINK_hop();
	else
	{//RLINK_RC4G
		// Find 2 unused channels
		//  first  channel is a multiple of 3 between 00 and 5D
		//  second channel is a multiple of 3 between 63 and BD
		CC2500_Strobe(CC2500_SIDLE);
		CC2500_WriteReg(CC2500_17_MCSM1,0x3C);
		CC2500_Strobe(CC2500_SFRX);
		CC2500_SetTxRxMode(RX_EN);
		CC2500_Strobe(CC2500_SRX);
		delayMilliseconds(1);						//wait for RX mode
		uint16_t val;
		uint8_t val_low = 0xFF;
		hopping_frequency[0] = 0x00;
		hopping_frequency[1] = 0x63;
		for(uint8_t ch=0; ch<=0xBD; ch+=3)
		{
			if(ch==0x63)
				val_low	= 0xFF;						//init for second block
			if(ch==0x60)
				continue;							//skip channel
			CC2500_WriteReg(CC2500_0A_CHANNR, ch);	//switch channel
			delayMicroseconds(370);					//wait to read
			val = 0;
			for(uint8_t i=0;i<16;i++)
				val += CC2500_ReadReg(CC2500_34_RSSI | CC2500_READ_BURST);
			val >>= 4;
			debug("C:%02X RSSI:%02X",ch,val);
			if(val_low > val)
			{
				debug(" OK");
				val_low = val;
				hopping_frequency[ch<0x63?0:1]=ch;	//save best channel
			}
			debugln("");
		}
		CC2500_WriteReg(CC2500_17_MCSM1,0x30);
		CC2500_Strobe(CC2500_SIDLE);
		CC2500_SetTxRxMode(TX_EN);
		#ifdef RLINK_RC4G_FORCE_ID
			hopping_frequency[0] = 0x03;
			hopping_frequency[1] = 0x6F;
		#endif
	}

 	#ifdef RLINK_DEBUG
		debug("ID:");
		for(uint8_t i=0;i<RLINK_TX_ID_LEN;i++)
			debug(" 0x%02X",rx_tx_addr[i]);
		debugln("");
		debug("Hop(%d):", rf_ch_num);
		for(uint8_t i=0;i<RLINK_HOP;i++)
			debug(" 0x%02X",hopping_frequency[i]);
		debugln("");
	#endif
 }

const PROGMEM uint8_t RLINK_init_values[] = {
  /* 00 */ 0x5B, 0x06, 0x5C, 0x07, 0xAB, 0xCD, 0x40, 0x04,
  /* 08 */ 0x45, 0x00, 0x00, 0x06, 0x00, 0x5C, 0x62, 0x76,
  /* 10 */ 0x7A, 0x7F, 0x13, 0x23, 0xF8, 0x44, 0x07, 0x30,
  /* 18 */ 0x18, 0x16, 0x6C, 0x43, 0x40, 0x91, 0x87, 0x6B,
  /* 20 */ 0xF8, 0x56, 0x10, 0xA9, 0x0A, 0x00, 0x11
};

static void __attribute__((unused)) RLINK_rf_init()
{
	CC2500_Strobe(CC2500_SIDLE);

	for (uint8_t i = 0; i < 39; ++i)
		CC2500_WriteReg(i, pgm_read_byte_near(&RLINK_init_values[i]));

	if(sub_protocol==RLINK_DUMBORC)
	{
		CC2500_WriteReg(4, 0xBA);
		CC2500_WriteReg(5, 0xDC);
	}
	else if(sub_protocol==RLINK_RC4G)
		CC2500_WriteReg(5, 0xA5);
		
	CC2500_WriteReg(CC2500_0C_FSCTRL0, option);
	
	CC2500_SetTxRxMode(TX_EN);
}

static void __attribute__((unused)) RLINK_send_packet()
{
	static uint32_t pseudo=0;
	uint32_t bits = 0;
	uint8_t bitsavailable = 0;
	uint8_t idx = 6;

	CC2500_Strobe(CC2500_SIDLE);

	// packet length
	packet[0] = RLINK_TX_PACKET_LEN;
	// header
	if(packet_count>3)
		packet[1] = 0x02;					// 0x02 telemetry request flag
	else
		packet[1] = 0x00;					// no telemetry

	switch(sub_protocol)
	{
		case RLINK_SURFACE:
			packet[1] |= 0x01;
			//radiolink additionnal ID which is working only on a small set of RXs
			//if(RX_num) packet[1] |= ((RX_num+2)<<4)+4;	// RX number limited to 10 values, 0 is a wildcard
			break;
		case RLINK_AIR:
			packet[1] |= 0x21;					//air 0x21 on dump but it looks to support telemetry at least RSSI
			break;
		case RLINK_DUMBORC:
			packet[1] |= 0x01;					//always 0x00 on dump but does appear to support telemtry on newer transmitters
			break;
	}
	
	// ID
	memcpy(&packet[2],rx_tx_addr,RLINK_TX_ID_LEN);

	// pack 16 channels on 11 bits values between 170 and 1876, 1023 middle. The last 8 channels are failsafe values associated to the first 8 values.
	for (uint8_t i = 0; i < 16; i++)
	{
		uint32_t val = convert_channel_16b_nolimit(i,170,1876,false);		// allow extended limits
		if (val & 0x8000)
			val = 0;
		else if (val > 2047)
			val=2047;

		bits |= val << bitsavailable;
		bitsavailable += 11;
		while (bitsavailable >= 8) {
			packet[idx++] = bits & 0xff;
			bits >>= 8;
			bitsavailable -= 8;
		}
	}
	
	// hop
	pseudo=((pseudo * 0xAA) + 0x03) % 0x7673;	// calc next pseudo random value
	CC2500_WriteReg(CC2500_0A_CHANNR, hopping_frequency[pseudo & 0x0F]);
	packet[28]= pseudo;
	packet[29]= pseudo >> 8;
	packet[30]= 0x00;						// unknown
	packet[31]= 0x00;						// unknown
	packet[32]= rf_ch_num;					// index of value changed in the RF table
	
	// check
	uint8_t sum=0;
	for(uint8_t i=1;i<33;i++)
		sum+=packet[i];
	packet[33]=sum;

	// send packet
	CC2500_WriteData(packet, RLINK_TX_PACKET_LEN+1);
	
	// packets type
	packet_count++;
	if(packet_count>5) packet_count=0;

	#ifdef RLINK_DEBUG
		debugln("C= 0x%02X",hopping_frequency[pseudo & 0x0F]);
		debug("P=");
		for(uint8_t i=1;i<RLINK_TX_PACKET_LEN+1;i++)
			debug(" 0x%02X",packet[i]);
		debugln("");
	#endif
}

#ifndef MULTI_AIR
static void __attribute__((unused)) RLINK_RC4G_send_packet()
{
	uint32_t val;
	//hop
	CC2500_WriteReg(CC2500_0A_CHANNR, hopping_frequency[packet_count>>1]);
	#ifdef RLINK_DEBUG
		debug("C= 0x%02X ",hopping_frequency[packet_count>>1]);
	#endif
	// packet length
	packet[0] = 0x0F;
	//address
	memcpy(&packet[1], &rx_tx_addr[1], 3);
	//channels
	for(uint8_t i=0;i<2;i++)
	{
		val = Channel_data[2*i  ] +400 -24;
		packet[4+i*2] = val;
		packet[8+i  ] = val>>8;
		val = Channel_data[2*i+1] +400 -24;
		packet[5+i*2] = val;
		packet[8+i  ] |= (val>>4) & 0xF0;
	}
	//special channel which is linked to gyro on the orginal TX but allocating it on CH5 here
	packet[10] = convert_channel_16b_limit(CH5,0,100);
	//failsafe
	for(uint8_t i=0;i<4;i++)
		packet[11+i] = convert_channel_16b_limit(CH6+i,0,200);
	//next hop
	packet_count++;
	packet_count &= 0x03;
	packet[15] = hopping_frequency[packet_count>>1];
	// send packet
	CC2500_WriteData(packet, 16);

	#ifdef RLINK_DEBUG
		debug("P=");
		for(uint8_t i=1;i<16;i++)
			debug(" 0x%02X",packet[i]);
		debugln("");
	#endif
}
#endif

#define RLINK_TIMING_PROTO	20000-100		// -100 for compatibility with R8EF
#define RLINK_TIMING_RFSEND	10500
#define RLINK_TIMING_CHECK	2000
#define RLINK_RC4G_TIMING_PROTO 14460
uint16_t RLINK_callback()
{
	if(sub_protocol == RLINK_RC4G)
	{
		#ifndef MULTI_AIR
			#ifdef MULTI_SYNC
				telemetry_set_input_sync(RLINK_RC4G_TIMING_PROTO);
			#endif
			CC2500_SetPower();
			CC2500_SetFreqOffset();
			RLINK_RC4G_send_packet();
		#else
			SUB_PROTO_INVALID;
		#endif
		return RLINK_RC4G_TIMING_PROTO;
	}
	switch(phase)
	{
		case RLINK_DATA:
			#ifdef MULTI_SYNC
				telemetry_set_input_sync(RLINK_TIMING_PROTO);
			#endif
			CC2500_SetPower();
			CC2500_SetFreqOffset();
			RLINK_send_packet();
#if not defined RLINK_HUB_TELEMETRY
			return RLINK_TIMING_PROTO;
#else
			if(!(packet[1]&0x02))
				return RLINK_TIMING_PROTO;					//Normal packet
															//Telemetry packet
			phase++;										// RX1
			return RLINK_TIMING_RFSEND;
		case RLINK_RX1:
			CC2500_Strobe(CC2500_SIDLE);
			CC2500_Strobe(CC2500_SFRX);
			CC2500_SetTxRxMode(RX_EN);
			CC2500_Strobe(CC2500_SRX);
			phase++;										// RX2
			return RLINK_TIMING_PROTO-RLINK_TIMING_RFSEND-RLINK_TIMING_CHECK;
		case RLINK_RX2:
			len = CC2500_ReadReg(CC2500_3B_RXBYTES | CC2500_READ_BURST) & 0x7F;	
			if (len == RLINK_RX_PACKET_LEN + 1 + 2)			//Telemetry frame is 15 bytes + 1 byte for length + 2 bytes for RSSI&LQI&CRC
			{
				#ifdef RLINK_DEBUG_TELEM
					debug("Telem:");
				#endif
				CC2500_ReadData(packet_in, len);
				if(packet_in[0]==RLINK_RX_PACKET_LEN && (packet_in[len-1] & 0x80) && memcmp(&packet[2],rx_tx_addr,RLINK_TX_ID_LEN)==0 && (packet_in[6]==packet[1] || sub_protocol == RLINK_DUMBORC))
				{//Correct telemetry received: length, CRC, ID and type
				 //packet_in[6] is 0x00 on almost all DumboRC RX so assume it is always valid
					#ifdef RLINK_DEBUG_TELEM
						for(uint8_t i=0;i<len;i++)
							debug(" %02X",packet_in[i]);
					#endif
					TX_RSSI = packet_in[len-2];
					if(TX_RSSI >=128)
						TX_RSSI -= 128;
					else
						TX_RSSI += 128;
					RX_RSSI=packet_in[7]&0x7F;				//Should be packet_in[7]-256 but since it's an uint8_t...
					v_lipo1=packet_in[8]<<1;				//RX Batt
					v_lipo2=packet_in[9];					//Batt
					telemetry_link=1;						//Send telemetry out
					pps_counter++;
					packet_count=0;
				}
				#ifdef RLINK_DEBUG_TELEM
					debugln("");
				#endif
			}
			if (millis() - pps_timer >= 2000)
			{//1 telemetry packet every 100ms
				pps_timer = millis();
				if(pps_counter<20)
					pps_counter*=5;
				else
					pps_counter=100;
				debugln("%d pps", pps_counter);
				TX_LQI = pps_counter;						//0..100%
				pps_counter = 0;
			}
			CC2500_SetTxRxMode(TX_EN);
			phase=RLINK_DATA;								// DATA
			return RLINK_TIMING_CHECK;
#endif
	}
	return 0;
}

void RLINK_init()
{
	BIND_DONE;	// Not a TX bind protocol
	RLINK_TXID_init();
	RLINK_rf_init();
	packet_count = 0;
	phase = RLINK_DATA;
}

#endif