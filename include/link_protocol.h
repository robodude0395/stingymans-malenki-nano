// include/link_protocol.h — shared, hardware-independent link protocol
//
// Pure C++ (no Arduino/RF24 dependency) so it builds on both AVRs and on the
// host for unit tests (pio test -e native).
//
// Frequency hopping scheme (TX-clocked, hop-per-packet, RX follows/predicts):
//   - TX sends one packet every HOP_PERIOD_US and moves to the next channel of
//     a per-pair hop table after EVERY packet, ACK or not.
//   - Each packet carries hopIdx (index of the channel it was sent on), so any
//     received packet lets the RX resync instantly.
//   - RX hops with each packet, predicts hops on missed packets, and dwells on
//     one channel while unsynced until the TX comes by.
// Based on the well-tested DIY-Multiprotocol-TX-Module Bayang protocol:
//   https://github.com/pascallanger/DIY-Multiprotocol-TX-Module
//   Multiprotocol/Bayang_nrf24l01.ino     (TX hops each packet over an ID-derived table)
//   Multiprotocol/Bayang_Rx_nrf24l01.ino  (RX hops with packets, predicts on misses)
// The ACK-payload telemetry follows the TMRh20 RF24 examples/AcknowledgementPayloads.

#pragma once

#include <stdint.h>
#include <string.h>

#ifdef __AVR__
#include <avr/pgmspace.h>
#define LP_PROGMEM PROGMEM
#define LP_READ_BYTE(p) pgm_read_byte(p)
#else
#define LP_PROGMEM
#define LP_READ_BYTE(p) (*(const uint8_t*)(p))
#endif

// ─── Hopping parameters ───
static const uint8_t  HOP_COUNT       = 16;      // channels per pair hop table
static const uint32_t HOP_PERIOD_US   = 10000;   // one packet + one hop every 10 ms (100 Hz)
static const uint8_t  MIN_HOP_SPACING = 8;       // MHz between consecutive hops (best effort)

// Channel pool (nRF channel n = 2400 + n MHz). Avoids the main energy of WiFi
// 1/6/11 (2412/2437/2462 ±8 MHz), BLE advertising (2402/2426/2480 = ch 2/26/80)
// and ch 76 (RF24 default, used by lots of hobby nRF24 gear at events).
// Known limitation: in EU/UK venues WiFi ch 13 (2472 MHz) overlaps 71-82;
// hopping still works on the other two clusters.
static const uint8_t CHANNEL_POOL_SIZE = 25;
static const uint8_t CHANNEL_POOL[CHANNEL_POOL_SIZE] LP_PROGMEM = {
  21, 22, 23, 24, 25, 27, 28,
  46, 47, 48, 49, 50, 51, 52, 53,
  71, 72, 73, 74, 75, 77, 78, 79, 81, 82
};

// ─── Per-pair link parameters (derived from TX_ID) ───
struct LinkParams {
  uint8_t  address[5];      // 40-bit nRF24 pipe address
  uint8_t  hop[HOP_COUNT];  // hop table (nRF channel numbers)
  uint16_t idTag;           // in-packet pair tag
  uint8_t  key;             // CRC-8 init value
};

// xorshift32 PRNG (Marsaglia). Integer-only, identical on AVR and host.
inline uint32_t lpXorshift32(uint32_t& x) {
  x ^= x << 13;
  x ^= x >> 17;
  x ^= x << 5;
  return x;
}

inline uint8_t lpAbsDiff(uint8_t a, uint8_t b) {
  return (a > b) ? (uint8_t)(a - b) : (uint8_t)(b - a);
}

// Deterministic: same id -> same params on both boards.
inline void deriveLinkParams(uint32_t id, LinkParams& out) {
  // --- Hop table: shuffle the pool, then pick 16 with spacing ---
  uint32_t s = (id ? id : 0x1F2E3D4CUL) ^ 0x9E3779B9UL;
  if (s == 0) s = 1;

  uint8_t pool[CHANNEL_POOL_SIZE];
  for (uint8_t i = 0; i < CHANNEL_POOL_SIZE; i++) {
    pool[i] = LP_READ_BYTE(&CHANNEL_POOL[i]);
  }
  for (uint8_t i = CHANNEL_POOL_SIZE - 1; i > 0; i--) {
    uint8_t j = (uint8_t)(lpXorshift32(s) % (uint32_t)(i + 1));
    uint8_t t = pool[i]; pool[i] = pool[j]; pool[j] = t;
  }

  out.hop[0] = pool[0];
  for (uint8_t i = 1; i < HOP_COUNT; i++) {
    int8_t pick = -1;
    int8_t fallback = -1;
    for (uint8_t k = i; k < CHANNEL_POOL_SIZE; k++) {
      if (lpAbsDiff(pool[k], out.hop[i - 1]) >= MIN_HOP_SPACING) {
        if (fallback < 0) fallback = (int8_t)k;
        if (i != HOP_COUNT - 1 || lpAbsDiff(pool[k], out.hop[0]) >= MIN_HOP_SPACING) {
          pick = (int8_t)k;
          break;
        }
      }
    }
    if (pick < 0) pick = fallback;
    if (pick >= 0) {
      uint8_t t = pool[i]; pool[i] = pool[(uint8_t)pick]; pool[(uint8_t)pick] = t;
    }
    out.hop[i] = pool[i];  // best effort if nothing qualified; both sides still agree
  }

  // --- Address, idTag, key: second independent stream ---
  uint32_t a = id ^ 0xA5C35A3CUL;
  if (a == 0) a = 1;
  for (uint8_t i = 0; i < 5; i++) {
    uint8_t b = (uint8_t)(lpXorshift32(a) >> 8);
    if (b == 0x00 || b == 0x55 || b == 0xAA || b == 0xFF) b ^= 0x3C;  // no preamble-like / flat bytes
    out.address[i] = b;
  }
  out.idTag = (uint16_t)(lpXorshift32(a) >> 16);
  if (out.idTag == 0) out.idTag = 0x5A5A;
  out.key = (uint8_t)lpXorshift32(a);
}

// ─── Packets ───
struct __attribute__((packed)) CommandPacket {   // 8 bytes, TX -> RX
  uint16_t idTag;   // [0-1] pair tag from TX_ID; RX rejects mismatches
  uint8_t  seq;     // [2]   +1 every packet (wraps)
  uint8_t  hopIdx;  // [3]   index in hop table of the channel this packet is sent on
  int8_t   speed;   // [4]   -100..100 (joystick Y)
  int8_t   turn;    // [5]   -100..100 (joystick X)
  uint8_t  flags;   // [6]   bit0 = joystick button
  uint8_t  check;   // [7]   CRC-8 (poly 0x07) over bytes 0..6, init = link key
};

struct __attribute__((packed)) AckPacket {       // 4 bytes, RX -> TX in ACK payload
  uint8_t idTagLo;      // low byte of idTag; TX ignores payloads that don't match
  uint8_t seqEcho;      // seq of last valid packet the RX accepted
  uint8_t linkQuality;  // valid packets in last 1 s window (100 Hz => percent), max 100
  uint8_t flags;        // bit0 = RX in failsafe, bit1 = RX in ACQUIRE
};

static_assert(sizeof(CommandPacket) == 8, "CommandPacket must be 8 bytes");
static_assert(sizeof(AckPacket) == 4, "AckPacket must be 4 bytes");

static const uint8_t ACK_FLAG_FAILSAFE = 0x01;
static const uint8_t ACK_FLAG_ACQUIRE  = 0x02;
static const uint8_t CMD_FLAG_BUTTON   = 0x01;

enum PacketStatus : uint8_t { PKT_OK = 0, PKT_BAD_ID, PKT_BAD_CRC, PKT_BAD_FIELD };

// CRC-8, poly 0x07, MSB first, no reflection, no xorout.
inline uint8_t crc8(const uint8_t* data, uint8_t len, uint8_t init) {
  uint8_t crc = init;
  for (uint8_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t b = 0; b < 8; b++) {
      crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
    }
  }
  return crc;
}

inline void sealCommandPacket(CommandPacket& p, const LinkParams& lp) {
  p.idTag = lp.idTag;
  p.check = crc8((const uint8_t*)&p, sizeof(CommandPacket) - 1, lp.key);
}

inline PacketStatus validateCommandPacket(const CommandPacket& p, const LinkParams& lp) {
  if (p.idTag != lp.idTag) return PKT_BAD_ID;
  if (crc8((const uint8_t*)&p, sizeof(CommandPacket) - 1, lp.key) != p.check) return PKT_BAD_CRC;
  if (p.hopIdx >= HOP_COUNT) return PKT_BAD_FIELD;
  if (p.speed < -100 || p.speed > 100 || p.turn < -100 || p.turn > 100) return PKT_BAD_FIELD;
  return PKT_OK;
}

// True when seq is 1..127 ahead of last (mod 256). Rejects duplicates/old packets.
inline bool seqIsNewer(uint8_t seq, uint8_t last) {
  uint8_t d = (uint8_t)(seq - last);
  return d >= 1 && d <= 127;
}

// RX sequence filter: decides whether an authentic packet's command is used.
//   - Not locked: accept any seq (first packet after acquisition).
//   - Locked: accept seq 1..127 ahead of lastSeq; reject duplicates/old ones.
//   - A packet far "behind" lastSeq means the TX rebooted and restarted seq.
//     It is re-anchored once a second packet confirms the new sequence
//     (restartSeq -> newer seq), instead of rejecting until sync is lost.
// Updates lastSeq on accept and the restart state in every case.
inline bool seqAccept(uint8_t seq, bool locked, uint8_t& lastSeq,
                      uint8_t& restartSeq, bool& restartPending) {
  bool fresh = !locked || seqIsNewer(seq, lastSeq);
  if (!fresh && seq != lastSeq) {
    fresh = restartPending && seqIsNewer(seq, restartSeq);
    restartSeq = seq;
    restartPending = !fresh;
  }
  if (fresh) {
    restartPending = false;
    lastSeq = seq;
  }
  return fresh;
}

// -100..100 -> 1000..2000 us RC pulse (clamped).
inline uint16_t channelToPulseUs(int8_t v) {
  int16_t us = 1500 + 5 * (int16_t)v;
  if (us < 1000) us = 1000;
  if (us > 2000) us = 2000;
  return (uint16_t)us;
}
