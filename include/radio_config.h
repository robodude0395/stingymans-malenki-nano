// include/radio_config.h — shared nRF24L01 settings for TX and RX
//
// Simple single-channel link (no frequency hopping):
//   - TX sends one 8-byte packet every 10 ms (100 Hz) on RADIO_CHANNEL.
//   - RX auto-ACKs each packet with a 4-byte telemetry payload.
//   - Pair isolation: the 5-byte pipe address, a 16-bit ID tag and a CRC-8 key
//     are all derived from TX_ID (see link_protocol.h), so other teams'
//     transmitters on the same channel are ignored.

#pragma once

#include <Arduino.h>
#include <RF24.h>
#include "link_protocol.h"

// ─── Pairing ID ───
// Change TX_ID to any random 32-bit value (e.g. `openssl rand -hex 4`) and
// flash BOTH boards. Every TX/RX pair at the event must use a different value.
#ifndef TX_ID
#define TX_ID 0x5EED1234UL
#endif
#if TX_ID == 0x5EED1234UL
#warning "TX_ID is still the default 0x5EED1234UL: set a unique value per TX/RX pair (include/radio_config.h)"
#endif

// ─── RF channel (same on both boards) ───
// nRF channel n = 2400 + n MHz. 2.4 GHz ISM band is 2400-2483 MHz, so stay
// within 0..83. Channel 50 (2450 MHz) sits between WiFi channels 6 and 11.
// If it is busy at the event, try e.g. 22 or 80 and flash both boards.
// Avoid 76: RF24 library default, used by lots of hobby gear.
#ifndef RADIO_CHANNEL
#define RADIO_CHANNEL 50
#endif
static_assert(RADIO_CHANNEL <= 125, "RADIO_CHANNEL must be 0..125");

// ─── Transmit power (same on both boards) ───
// 0 = MIN, 1 = LOW, 2 = HIGH, 3 = MAX.
// Bench (radios < 2 m apart): 1. PA_MAX at close range overloads the receiver
// and packets get silently dropped. Competition: 3 (or 2 if 3 is unstable).
#ifndef RADIO_PA_LEVEL
#define RADIO_PA_LEVEL 3
#endif
static_assert(RADIO_PA_LEVEL <= 3, "RADIO_PA_LEVEL must be 0..3");

// ─── Data rate (same on both boards) ───
// 0 = 1 Mbps, 1 = 2 Mbps, 2 = 250 kbps (RF24 rf24_datarate_e values).
// 1 Mbps is the most compatible: many clone modules (Si24R1 etc. sold as
// "nRF24L01+") and non-plus nRF24L01 chips are unreliable or deaf at 250 kbps.
// 250 kbps gives ~9 dB more range on genuine nRF24L01+ modules only.
#ifndef RADIO_DATA_RATE
#define RADIO_DATA_RATE 0
#endif
static_assert(RADIO_DATA_RATE <= 2, "RADIO_DATA_RATE must be 0..2");

// ─── Timing ───
static const uint32_t PACKET_PERIOD_US = 10000;   // 100 Hz
// Auto retransmit: delay (3+1)*250 = 1000 us (enough for a 4-byte ACK payload
// at 250 kbps), 4 retries. Worst-case write() ~ 5 attempts * 1.55 ms ~ 7.8 ms,
// still inside the 10 ms packet period.
static const uint8_t RADIO_ARD = 3;
static const uint8_t RADIO_ARC = 4;

// Settings common to TX and RX. Caller then sets pipes and role.
inline void radioApplyCommonConfig(RF24& radio) {
  radio.setAddressWidth(5);
  radio.setCRCLength(RF24_CRC_16);     // hardware CRC16
  radio.setDataRate((rf24_datarate_e)RADIO_DATA_RATE);
  radio.setPALevel((rf24_pa_dbm_e)RADIO_PA_LEVEL);
  radio.setChannel(RADIO_CHANNEL);
  radio.setAutoAck(true);
  radio.enableDynamicPayloads();
  radio.enableAckPayload();
  radio.setRetries(RADIO_ARD, RADIO_ARC);
}
