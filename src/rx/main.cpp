// src/rx/main.cpp — simple 2-channel PWM receiver (battle robot)
//
// Link: nRF24L01+ on one fixed channel (RADIO_CHANNEL), 250 kbps, hardware
// CRC16, per-pair 5-byte address. Every packet is also checked for the pair
// ID tag + CRC-8 (link_protocol.h), so other transmitters are ignored.
// Each received packet is auto-ACKed with a 4-byte telemetry payload.
//
// Outputs: hardware Timer1 PWM (no ISR, zero jitter), 50 Hz, 1000-2000 us.
//   CH1 (D9)  = speed (joystick Y)
//   CH2 (D10) = turn  (joystick X)
//
// Boot: no pulses (pins LOW) until the first valid packet. No motion before link.
// Failsafe: no valid packet for FAILSAFE_TIMEOUT_MS -> FAILSAFE_MODE.
//
// Status LED (D4):
//   - OFF        = radio chip not detected (retried every 1 s)
//   - Blink 1 Hz = waiting for TX / failsafe
//   - Solid ON   = linked, outputs live
//
// ─── Wiring: SparkFun Pro Micro 5V/16MHz (ATmega32U4) ───
//   nRF24L01+   CE   -> D7
//               CSN  -> D8
//               SCK  -> D15
//               MOSI -> D16
//               MISO -> D14
//               IRQ  -> not connected
//               VCC  -> 3.3V from a SEPARATE regulator (nRF24 adapter board or
//                       an LDO fed from 5V). PA+LNA modules draw up to ~115 mA.
//               GND  -> GND
//               10-100 uF capacitor directly across nRF24 VCC/GND.
//   LED         D4 -> 330 ohm -> LED -> GND
//   CH1 PWM     D9  (OC1A) -> ESC/servo 1 signal
//   CH2 PWM     D10 (OC1B) -> ESC/servo 2 signal
//   Power       ESC BEC 5V -> RAW (or VCC), common GND with the ESCs.
//   Do NOT use D17/PB0 (SS / onboard RX LED): it must stay OUTPUT for SPI master.
//
// Upload: pio run -e rx -t upload (double-tap RST if the port is not found)

#include <Arduino.h>
#include <SPI.h>
#include <RF24.h>
#include <util/atomic.h>
#include "radio_config.h"

// ─── Pin Definitions ───
const uint8_t RX_CE_PIN  = 7;
const uint8_t RX_CSN_PIN = 8;
const uint8_t LED_PIN    = 4;
const uint8_t CH1_PIN    = 9;   // OC1A
const uint8_t CH2_PIN    = 10;  // OC1B

// ─── Failsafe ───
#define FAILSAFE_NEUTRAL   0  // 1500 us, outputs stay on: bidirectional ESCs stop
#define FAILSAFE_NO_PULSES 1  // outputs off (LOW): use for a unidirectional weapon ESC,
                              // where 1500 us would be HALF throttle
#ifndef FAILSAFE_MODE
#define FAILSAFE_MODE FAILSAFE_NEUTRAL
#endif
const unsigned long FAILSAFE_TIMEOUT_MS = 200;  // 20 missed packets at 100 Hz
const uint16_t FAILSAFE_US = 1500;

// ─── PWM (Timer1, fast PWM mode 14, TOP = ICR1, prescaler 8) ───
const uint32_t PWM_FRAME_US     = 20000;               // 50 Hz frame
const uint32_t PWM_TICKS_PER_US = F_CPU / 8000000UL;   // 2 @16MHz, 1 @8MHz
static_assert(PWM_TICKS_PER_US >= 1, "F_CPU too low for Timer1 PWM");
const uint16_t PWM_TOP          = (uint16_t)(PWM_FRAME_US * PWM_TICKS_PER_US - 1);
// Output enable/disable only in this TCNT1 window (after the longest pulse,
// before TOP) so no truncated pulse ever reaches an ESC.
const uint16_t PWM_SAFE_LO      = (uint16_t)(2200 * PWM_TICKS_PER_US);
const uint16_t PWM_SAFE_HI      = (uint16_t)(PWM_TOP - 200 * PWM_TICKS_PER_US);

// ─── Timing ───
const unsigned long LQ_WINDOW_MS      = 1000;
const unsigned long HEALTH_CHECK_MS   = 1000;
const unsigned long LED_BLINK_MS      = 500;
const unsigned long DEBUG_INTERVAL_MS = 500;   // 2 Hz

// ─── Global State ───
RF24 radio(RX_CE_PIN, RX_CSN_PIN);
LinkParams link;

bool radioOk     = false;
bool radioPlus   = false;   // chip reports nRF24L01+ (supports 250 kbps)
bool everLinked  = false;
bool inFailsafe  = false;
unsigned long lastValidMs = 0;
uint8_t lastSeq  = 0;
int8_t lastSpeed = 0;
int8_t lastTurn  = 0;
uint16_t pulse1Us = FAILSAFE_US;
uint16_t pulse2Us = FAILSAFE_US;

bool outputsRequested = false;
bool outputsEnabled   = false;

uint8_t validCount  = 0;     // valid packets in the current 1 s window
uint8_t linkQuality = 0;     // valid packets in the last 1 s window (~%)
uint16_t rejected   = 0;     // packets with wrong ID / CRC / fields (other TXs, noise)

unsigned long lastLqMs = 0, lastHealthMs = 0, lastLedMs = 0, lastDebugMs = 0;
bool ledPhase = false;

// ─── Non-blocking USB debug output ───
// USB CDC Serial.print() can block up to 250 ms when the port is open but not
// read. Lines are queued and written only into free USB buffer space.
const uint8_t LOG_BUF_SIZE = 200;
char logBuf[LOG_BUF_SIZE];
uint8_t logHead = 0, logTail = 0;

void logQueue(const char* s, uint8_t n) {
  if (logTail == logHead) logTail = logHead = 0;
  if (n > (uint8_t)(LOG_BUF_SIZE - logHead)) return;   // full: drop line
  memcpy(logBuf + logHead, s, n);
  logHead += n;
}

void logLine(const __FlashStringHelper* s) {
  char line[48];
  size_t n = strlen_P((PGM_P)s);
  if (n > sizeof(line) - 2) n = sizeof(line) - 2;
  memcpy_P(line, (PGM_P)s, n);
  line[n++] = '\r';
  line[n++] = '\n';
  logQueue(line, (uint8_t)n);
}

void serviceLog() {
  if (logTail == logHead) return;
  int room = Serial.availableForWrite() - 1;  // never fill the USB bank completely
  if (room <= 0) return;
  uint8_t n = logHead - logTail;
  if (n > room) n = (uint8_t)room;
  Serial.write((const uint8_t*)logBuf + logTail, n);
  logTail += n;
}

// ─── PWM ───
void pwmSetUs(uint16_t us1, uint16_t us2) {
  pulse1Us = us1;
  pulse2Us = us2;
  // OCR1x is double-buffered in fast PWM: takes effect at BOTTOM, glitch-free
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    OCR1A = (uint16_t)(us1 * PWM_TICKS_PER_US);
    OCR1B = (uint16_t)(us2 * PWM_TICKS_PER_US);
  }
}

void pwmInit() {
  digitalWrite(CH1_PIN, LOW);
  digitalWrite(CH2_PIN, LOW);
  pinMode(CH1_PIN, OUTPUT);
  pinMode(CH2_PIN, OUTPUT);
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    TCCR1B = 0;                         // stop timer
    TCCR1A = _BV(WGM11);                // mode 14, outputs disconnected
    TCNT1  = 0;
    ICR1   = PWM_TOP;
  }
  pwmSetUs(FAILSAFE_US, FAILSAFE_US);
  TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS11);  // prescaler 8, start
}

// Connect/disconnect the PWM pins only between pulses (no truncated pulse).
void serviceOutputEnable() {
  if (outputsRequested == outputsEnabled) return;
  uint16_t t;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { t = TCNT1; }
  if (t < PWM_SAFE_LO || t > PWM_SAFE_HI) return;
  if (outputsRequested) {
    TCCR1A |= _BV(COM1A1) | _BV(COM1B1);
  } else {
    TCCR1A &= (uint8_t)~(_BV(COM1A1) | _BV(COM1B1));  // pins fall back to LOW
  }
  outputsEnabled = outputsRequested;
}

// ─── Radio ───
// Queue the telemetry sent back in the next ACK.
void loadAckPayload() {
  AckPacket ack;
  ack.idTagLo     = (uint8_t)(link.idTag & 0xFF);
  ack.seqEcho     = lastSeq;
  ack.linkQuality = linkQuality;
  ack.flags       = inFailsafe ? ACK_FLAG_FAILSAFE : 0;
  radio.flush_tx();                          // keep exactly one fresh payload queued
  radio.writeAckPayload(1, &ack, sizeof(ack));
}

bool initRadio() {
  if (!radio.begin()) return false;
  radioApplyCommonConfig(radio);
  radioPlus = radio.isPVariant();
  radio.openReadingPipe(1, link.address);
  radio.startListening();
  loadAckPayload();
  return true;
}

// Read up to 3 packets (FIFO depth). Bounded so a dead SPI bus can't hang loop().
void pollRadio() {
  for (uint8_t n = 0; n < 3 && radio.available(); n++) {
    uint8_t width = radio.getDynamicPayloadSize();
    if (width != sizeof(CommandPacket)) {   // 0 = corrupt or dead SPI bus
      radio.flush_rx();
      rejected++;
      break;
    }
    CommandPacket p;
    radio.read(&p, sizeof(p));
    loadAckPayload();                        // previous payload went out with this ACK

    if (validateCommandPacket(p, link) != PKT_OK) {
      rejected++;
      continue;
    }

    // Valid packet from our TX
    lastSeq = p.seq;
    lastValidMs = millis();
    if (validCount < 255) validCount++;
    lastSpeed = p.speed;
    lastTurn  = p.turn;
    // Straight pass-through. For tank mixing use e.g.
    //   left = constrain(speed + turn, -100, 100); right = constrain(speed - turn, -100, 100);
    pwmSetUs(channelToPulseUs(p.speed), channelToPulseUs(p.turn));
    outputsRequested = true;

    if (!everLinked) {
      everLinked = true;
      logLine(F("RX: *** LINKED to TX *** outputs on"));
    } else if (inFailsafe) {
      logLine(F("RX: Link restored"));
    }
    inFailsafe = false;
  }
}

void setup() {
  Serial.begin(115200);       // never wait for Serial: the robot runs without USB

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  pwmInit();                  // outputs LOW until the first valid packet
  deriveLinkParams(TX_ID, link);
  radioOk = initRadio();

  unsigned long nowMs = millis();
  lastLqMs = lastHealthMs = lastLedMs = lastDebugMs = nowMs;
}

void loop() {
  // (1) Receive
  if (radioOk) pollRadio();

  // Read the clock AFTER receiving. pollRadio() stamps lastValidMs with
  // millis(); if nowMs were taken before it, lastValidMs could be 1 ms newer
  // and the unsigned (nowMs - lastValidMs) would wrap to ~4e9, tripping the
  // failsafe instantly on a perfectly healthy link.
  unsigned long nowMs = millis();

  // (2) Failsafe
  if (everLinked && !inFailsafe && nowMs - lastValidMs > FAILSAFE_TIMEOUT_MS) {
    inFailsafe = true;
#if FAILSAFE_MODE == FAILSAFE_NO_PULSES
    outputsRequested = false;
#else
    pwmSetUs(FAILSAFE_US, FAILSAFE_US);
#endif
    logLine(F("RX: FAILSAFE (no packets)"));
  }

  // (3) Glitch-free output enable/disable
  serviceOutputEnable();

  // (4) Radio health check / retry every second. An nRF brownout (motor noise,
  // weak 3.3V) resets it to defaults (2 Mbps) -> full re-init.
  if (nowMs - lastHealthMs >= HEALTH_CHECK_MS) {
    lastHealthMs = nowMs;
    if (!radioOk || radio.getDataRate() != (rf24_datarate_e)RADIO_DATA_RATE ||
        radio.getChannel() != RADIO_CHANNEL) {
      bool wasOk = radioOk;
      radioOk = initRadio();
      if (wasOk) logLine(radioOk ? F("RX: Radio re-init (brownout?)") : F("RX: Radio chip LOST"));
      else if (radioOk) logLine(F("RX: Radio chip found"));
    }
  }

  // (5) Link quality window
  if (nowMs - lastLqMs >= LQ_WINDOW_MS) {
    lastLqMs = nowMs;
    linkQuality = validCount > 100 ? 100 : validCount;
    validCount = 0;
  }

  // (6) Status LED
  if (!radioOk) {
    digitalWrite(LED_PIN, LOW);
  } else if (everLinked && !inFailsafe) {
    digitalWrite(LED_PIN, HIGH);
  } else if (nowMs - lastLedMs >= LED_BLINK_MS) {
    lastLedMs = nowMs;
    ledPhase = !ledPhase;
    digitalWrite(LED_PIN, ledPhase ? HIGH : LOW);
  }

  // (7) Debug at 2 Hz
  if (nowMs - lastDebugMs >= DEBUG_INTERVAL_MS) {
    lastDebugMs = nowMs;
    char line[120];
    int n;
    if (!radioOk) {
      n = snprintf_P(line, sizeof(line),
                     PSTR("RX: RADIO=NOT DETECTED (check CE D7, CSN D8, SPI 14/15/16, 3.3V)\r\n"));
    } else {
      n = snprintf_P(line, sizeof(line),
                     PSTR("RX: RADIO=OK%S TX=%S ch=%u rate=%u pa=%u lq=%u fs=%u spd=%d trn=%d p1=%u p2=%u rej=%u\r\n"),
                     radioPlus ? PSTR("(+)") : PSTR("(non-plus/clone)"),
                     (everLinked && !inFailsafe) ? PSTR("YES") : PSTR("NO"),
                     (unsigned)RADIO_CHANNEL, (unsigned)RADIO_DATA_RATE, (unsigned)RADIO_PA_LEVEL,
                     (unsigned)linkQuality,
                     (unsigned)(inFailsafe ? 1 : 0), (int)lastSpeed, (int)lastTurn,
                     (unsigned)(outputsEnabled ? pulse1Us : 0),
                     (unsigned)(outputsEnabled ? pulse2Us : 0),
                     rejected);
    }
    if (n > 0 && n < (int)sizeof(line)) logQueue(line, (uint8_t)n);
  }

  serviceLog();
}
