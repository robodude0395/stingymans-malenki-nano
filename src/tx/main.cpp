// src/tx/main.cpp — TX Radio Communication with LED Status
// Single-channel nRF24L01 link (100 Hz on RADIO_CHANNEL, per-pair address +
// ID check from TX_ID in include/radio_config.h), ACK-payload telemetry.
//
// LED behavior:
//   - OFF             = radio chip failed to initialize (SPI issue);
//                       if it fails after a good boot it is retried every 1s
//   - Solid ON        = radio chip connected successfully via SPI
//   - Fast blink (5x) = robot connection confirmed (first ACK received)
//
// ─── Wiring: Arduino Pro Mini 5V/16MHz (ATmega328P) ───
//   nRF24L01+   CE   -> D7
//               CSN  -> D8
//               SCK  -> D13
//               MOSI -> D11
//               MISO -> D12
//               IRQ  -> not connected
//               VCC  -> 3.3V from a SEPARATE regulator (e.g. AMS1117-3.3 or an
//                       nRF24 adapter board). The 5V Pro Mini has no 3.3V rail.
//                       PA+LNA modules draw up to ~115 mA.
//               GND  -> GND
//               10-100 uF capacitor across nRF24 VCC/GND (needed at PA_MAX).
//               nRF24 SPI/CE/CSN inputs are 5V tolerant: no level shifter needed.
//   LED         D4 -> resistor -> LED -> GND
//   Joystick    X -> A0, Y -> A1, button -> A2 (to GND, INPUT_PULLUP)
//   OLED SSD1306 128x32 I2C: SDA -> A4, SCL -> A5
//   Battery     read through the internal bandgap (VCC), no divider.
//
// Upload: pio run -e tx -t upload (USB-serial adapter on the FTDI header)

#include <Arduino.h>
#include <SPI.h>
#include <RF24.h>
#include <U8g2lib.h>
#include <Wire.h>
#include "radio_config.h"

// ─── Pin Definitions ───
const uint8_t CE_PIN  = 7;
const uint8_t CSN_PIN = 8;
const uint8_t LED_PIN = 4;
const uint8_t JOY_Y_PIN = A1;   // Joystick Y-axis (forward/backward)
const uint8_t JOY_X_PIN = A0;   // Joystick X-axis (left/right)
const uint8_t JOY_BTN_PIN = A2;  // Joystick push button (active LOW)
const uint8_t BAT_PIN = A2;     // unused: battery is read via internal bandgap; note A2 is the button

// ─── Constants ───
const unsigned long LOOP_INTERVAL_US   = PACKET_PERIOD_US;  // 10ms = 100Hz transmission
const unsigned long CONNECTION_TIMEOUT = 500;  // 500ms ACK timeout
const uint8_t ROBOT_CONNECT_BLINKS    = 5;    // Number of fast blinks on robot connect
const uint16_t DISPLAY_EVERY_N_LOOPS  = 1000000UL / LOOP_INTERVAL_US;  // ~1 second
const uint8_t DEBUG_EVERY_N_LOOPS     = 2;    // debug line at 50Hz (as before)
const unsigned long RADIO_RETRY_MS    = 1000; // re-init retry period after a runtime radio failure

// OLED: 400kHz I2C and a chunked, non-blocking frame push. A full sendBuffer()
// (512 bytes) would block ~13ms at 400kHz (~50ms at 100kHz) and delay packets. Each chunk is DISPLAY_CHUNK_TILES*8 bytes (8 tiles = 64 B ~1.6ms).
// If the OLED misbehaves at 400kHz, use 100000 with DISPLAY_CHUNK_TILES 4.
const uint32_t OLED_I2C_CLOCK     = 400000;
const uint8_t DISPLAY_TILES_W     = 16;   // 128 px / 8
const uint8_t DISPLAY_TILES_H     = 4;    // 32 px / 8
const uint8_t DISPLAY_CHUNK_TILES = 8;
const uint8_t DISPLAY_CHUNKS_PER_ROW = DISPLAY_TILES_W / DISPLAY_CHUNK_TILES;
const uint8_t DISPLAY_CHUNKS      = DISPLAY_CHUNKS_PER_ROW * DISPLAY_TILES_H;

// Joystick dead-band and hysteresis
const int DEADBAND_HALF = 22;   // ±22 around center
const int HYSTERESIS    = 3;    // ADC counts
int joyYCenter = 512;           // Calibrated at startup
int joyXCenter = 512;           // Calibrated at startup

// ─── Global State ───
RF24 radio(CE_PIN, CSN_PIN);
U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C display(U8G2_R2, /* reset=*/ U8X8_PIN_NONE);

bool radioInitFailed  = false;
bool radioChipOk      = false;  // true = nRF24L01 chip detected on SPI
bool robotConnected   = false;  // true = received ACK from robot
bool robotFirstConnect = false; // true = first-time robot connection (triggers blink pattern)
bool radioRecovering  = false;  // true = radio failed at runtime (after a good boot), retrying
unsigned long lastAckTime    = 0;
unsigned long previousLoopUs = 0;
unsigned long lastRadioRetryMs = 0;

// Link state
LinkParams link;                // address + ID tag + CRC key derived from TX_ID
uint8_t txSeq    = 0;           // packet sequence number
uint8_t ackCount = 0;           // ACKs in the current 1s window
uint16_t retxCount = 0;         // retransmits in the current 1s window (diagnostic)
uint16_t retxPerSec = 0;        // retransmits in the last 1s window
uint8_t txLinkQuality = 0;      // ACKs per second (100Hz => percent)
AckPacket lastAck = {};         // last telemetry from the robot
bool lastAckValid = false;

// Display / debug state
uint8_t displayChunk = 0xFF;    // next chunk to push; >= DISPLAY_CHUNKS = idle
uint8_t debugCounter = 0;

// Potentiometer state
bool yInDeadBand  = true;
bool xInDeadBand  = true;
int8_t lastSpeed  = 0;
int8_t lastTurn   = 0;

// Battery monitoring state
uint16_t loopCounter = 0;
int batteryPercent   = 0;
int batteryVoltageMv = 0;

// Blink animation state
uint8_t blinkCount     = 0;
unsigned long lastBlinkTime = 0;
bool blinkPhase        = false;  // toggling state during blink animation
bool blinkingActive    = false;  // true while blink animation is running

// Configure radio as transmitter (channel is set in radioApplyCommonConfig)
void configureRadio() {
  radioApplyCommonConfig(radio);
  radio.openWritingPipe(link.address);
  radio.stopListening();
}

// Runtime radio recovery: write() timeout (RF24 sets failureDetected) or an
// nRF brownout reset (POR = 2Mbps). If the chip does not answer, LED goes OFF
// (radio fail) and loop() retries every RADIO_RETRY_MS until it comes back.
void reinitRadio(unsigned long nowMs) {
  radio.failureDetected = 0;
  lastRadioRetryMs = nowMs;
  if (!radio.begin()) {
    if (!radioInitFailed) {
      Serial.println(F("TX: Radio chip lost! (LED OFF, retrying)"));
      // loop() skips the 1s display block while recovering: redraw the
      // connection row once so the OLED doesn't keep showing "LINKED".
      // Blocking sendBuffer() is fine here: there is no radio link to keep.
      display.setDrawColor(0);
      display.drawBox(0, 16, 70, 16);   // row 2 text area (keeps BAT and LOW!)
      display.setDrawColor(1);
      display.setFont(u8g2_font_7x14B_tr);
      display.setCursor(0, 29);
      display.print(F("NO LINK"));
      displayChunk = 0xFF;              // cancel any partial chunked push
      display.sendBuffer();
    }
    radioInitFailed = true;
    radioRecovering = true;
    radioChipOk = false;
    robotConnected = false;
    lastAckValid = false;
    blinkingActive = false;
    digitalWrite(LED_PIN, LOW);
    return;
  }
  configureRadio();
  if (radioInitFailed) {
    radioInitFailed = false;
    radioRecovering = false;
    radioChipOk = true;
    digitalWrite(LED_PIN, HIGH);   // back to solid ON = chip OK
    previousLoopUs = micros();     // restart the slot clock, no burst
  }
  Serial.println(F("TX: Radio re-init"));
}

void setup() {
  // Initialize Serial for debug output (57600: 9600 blocked ~60ms per debug line)
  Serial.begin(57600);
  Serial.println(F("TX: Starting..."));

  // Configure LED pin
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // Configure joystick button pin (active LOW with internal pull-up)
  pinMode(JOY_BTN_PIN, INPUT_PULLUP);

  // Derive per-pair address, ID tag and CRC key
  deriveLinkParams(TX_ID, link);

  // Initialize RF24 radio
  if (!radio.begin()) {
    Serial.println(F("TX: Radio chip NOT detected on SPI!"));
    radioInitFailed = true;
    // LED stays OFF — indicates radio chip failure
    return;
  }

  // Radio chip detected — turn LED solid ON
  radioChipOk = true;
  digitalWrite(LED_PIN, HIGH);
  Serial.println(F("TX: Radio chip OK (LED solid ON)"));

  // Configure radio as transmitter
  configureRadio();

  Serial.print(F("TX: ID=0x"));
  Serial.print((uint32_t)TX_ID, HEX);
  Serial.print(F(" ch="));
  Serial.print(RADIO_CHANNEL);
  Serial.print(F(" PA="));
  Serial.print(RADIO_PA_LEVEL);
  Serial.print(F(" rate="));
  Serial.print(RADIO_DATA_RATE);
  Serial.print(radio.isPVariant() ? F(" chip=nRF24L01+") : F(" chip=NON-PLUS/CLONE"));
  Serial.print(F(" addr="));
  for (uint8_t i = 0; i < 5; i++) {
    if (link.address[i] < 0x10) Serial.print(F("0"));
    Serial.print(link.address[i], HEX);
  }
  Serial.println();

  // Calibrate joystick center (average 16 samples)
  long sumY = 0, sumX = 0;
  for (uint8_t i = 0; i < 16; i++) {
    sumY += analogRead(JOY_Y_PIN);
    sumX += analogRead(JOY_X_PIN);
    delay(2);
  }
  joyYCenter = sumY / 16;
  joyXCenter = sumX / 16;
  Serial.print(F("TX: Joy center Y="));
  Serial.print(joyYCenter);
  Serial.print(F(" X="));
  Serial.println(joyXCenter);

  // Initialize OLED display (I2C)
  display.setBusClock(OLED_I2C_CLOCK);
  display.begin();
  // A stuck I2C bus / unplugged OLED must never hang the radio loop
  Wire.setWireTimeout(3000, true);
  Serial.println(F("TX: OLED display initialized"));
  Serial.println(F("TX: Waiting for receiver..."));

  // Initialize timing
  previousLoopUs = micros();
}

void loop() {
  unsigned long currentMs = millis();
  unsigned long currentUs = micros();

  // ─── Radio Init Failure: LED stays OFF, do nothing ───
  // (a failure at boot stays final as before; a runtime failure is retried)
  if (radioInitFailed) {
    if (radioRecovering && currentMs - lastRadioRetryMs >= RADIO_RETRY_MS) {
      reinitRadio(currentMs);
    }
    return;
  }

  // ─── Handle blink animation (robot connection celebration) ───
  if (blinkingActive) {
    if (currentMs - lastBlinkTime >= 100) {  // 100ms per toggle = fast blink
      lastBlinkTime = currentMs;
      blinkPhase = !blinkPhase;
      digitalWrite(LED_PIN, blinkPhase ? HIGH : LOW);
      if (!blinkPhase) {
        blinkCount++;
      }
      if (blinkCount >= ROBOT_CONNECT_BLINKS) {
        // Done blinking — return to solid ON (radio chip still connected)
        blinkingActive = false;
        digitalWrite(LED_PIN, HIGH);
      }
    }
    // Continue with normal loop during blink animation
  }

  // ─── 10ms fixed-rate slot (100Hz) ───
  if (currentUs - previousLoopUs < LOOP_INTERVAL_US) {
    return;
  }
  previousLoopUs += LOOP_INTERVAL_US;
  if (currentUs - previousLoopUs >= LOOP_INTERVAL_US) {
    previousLoopUs = currentUs;  // overrun by more than a slot: resync, don't burst
  }

  // ─── Read joystick Y-axis (speed: forward/backward) ───
  int rawY = analogRead(JOY_Y_PIN);
  int8_t speed = 0;
  int yLow = joyYCenter - DEADBAND_HALF;
  int yHigh = joyYCenter + DEADBAND_HALF;

  if (yInDeadBand) {
    if (rawY < (yLow - HYSTERESIS)) {
      yInDeadBand = false;
    } else if (rawY > (yHigh + HYSTERESIS)) {
      yInDeadBand = false;
    }
  } else {
    if (rawY >= yLow && rawY <= yHigh) {
      yInDeadBand = true;
    }
  }

  if (yInDeadBand) {
    speed = 0;
  } else if (rawY < yLow) {
    // Map from 0..(yLow-1) → -100..-1
    speed = map(rawY, 0, yLow - 1, -100, -1);
  } else {
    // Map from (yHigh+1)..max using symmetric range from center
    // Use distance from center to 0 as reference for both sides
    int yMax = joyYCenter + (joyYCenter - 0);  // Mirror the low range
    if (yMax > 1023) yMax = 1023;
    int clamped = min(rawY, yMax);
    speed = map(clamped, yHigh + 1, yMax, 1, 100);
  }
  speed = constrain(speed, -100, 100);
  lastSpeed = speed;

  // ─── Read joystick X-axis (turn: left/right) ───
  int rawX = analogRead(JOY_X_PIN);
  int8_t turn = 0;
  int xLow = joyXCenter - DEADBAND_HALF;
  int xHigh = joyXCenter + DEADBAND_HALF;

  if (xInDeadBand) {
    if (rawX < (xLow - HYSTERESIS)) {
      xInDeadBand = false;
    } else if (rawX > (xHigh + HYSTERESIS)) {
      xInDeadBand = false;
    }
  } else {
    if (rawX >= xLow && rawX <= xHigh) {
      xInDeadBand = true;
    }
  }

  if (xInDeadBand) {
    turn = 0;
  } else if (rawX < xLow) {
    // Map from 0..(xLow-1) → -100..-1
    turn = map(rawX, 0, xLow - 1, -100, -1);
  } else {
    // Map using symmetric range from center, shrunk slightly to ensure full +100
    int xMax = joyXCenter + (joyXCenter - 0);
    xMax = xMax * 95 / 100;  // Shrink 5% to account for pot not reaching full range
    if (xMax > 1023) xMax = 1023;
    int clamped = min(rawX, xMax);
    turn = map(clamped, xHigh + 1, xMax, 1, 100);
  }
  turn = constrain(turn, -100, 100);
  lastTurn = turn;

  // ─── Read joystick button ───
  bool btnPressed = !digitalRead(JOY_BTN_PIN);  // Active LOW

  // ─── Build CommandPacket ───
  CommandPacket packet;
  packet.seq    = txSeq++;
  packet.hopIdx = 0;               // unused (single channel)
  packet.speed = speed;
  packet.turn  = turn;
  packet.flags = btnPressed ? 0x01 : 0x00;
  sealCommandPacket(packet, link);  // sets idTag + CRC-8

  // ─── Transmit packet ───
  bool ackReceived = radio.write(&packet, sizeof(packet));
  // Retransmits used by this write (0..RADIO_ARC): a high rt count means a
  // lossy link (interference, power or antenna problem).
  uint8_t arc = radio.getARC();
  if (retxCount <= 65535 - arc) retxCount += arc;

  // write() hit RF24's 95ms failure timeout (nRF brownout / SPI fault):
  // recover in this slot instead of waiting for the 1s health check.
  if (radio.failureDetected) {
    reinitRadio(currentMs);
    if (radioInitFailed) {
      return;
    }
  }

  if (ackReceived) {
    lastAckTime = currentMs;
    if (ackCount < 255) ackCount++;

    // Drain ACK payloads (telemetry from the robot). Bounded to the 3-entry
    // FIFO depth so a stuck SPI bus (reads 0x00 = "FIFO not empty") can't hang loop().
    for (uint8_t n = 0; n < 3 && radio.available(); n++) {
      uint8_t width = radio.getDynamicPayloadSize();  // 0 = corrupt (or dead SPI bus)
      if (width != sizeof(AckPacket)) {
        radio.flush_rx();
        break;
      }
      AckPacket ack;
      radio.read(&ack, sizeof(ack));
      if (ack.idTagLo == (uint8_t)(link.idTag & 0xFF)) {
        lastAck = ack;
        lastAckValid = true;
      }
    }

    // First time getting an ACK from robot — trigger blink celebration
    if (!robotConnected) {
      robotConnected = true;
      robotFirstConnect = true;
      blinkingActive = true;
      blinkCount = 0;
      blinkPhase = false;
      lastBlinkTime = currentMs;
      Serial.println(F("TX: *** CONNECTED to receiver *** (LED blinking)"));
    }
  }

  // ─── Update connection state ───
  if (currentMs - lastAckTime > CONNECTION_TIMEOUT) {
    lastAckValid = false;  // robot telemetry is stale now
    if (robotConnected) {
      robotConnected = false;
      Serial.println(F("TX: *** DISCONNECTED from receiver *** (waiting for receiver...)"));
      // LED stays solid ON — radio chip is still working
      // (only robot link is lost, not the local SPI connection)
      digitalWrite(LED_PIN, HIGH);
    }
  }

  // ─── Push one OLED chunk per slot (non-blocking frame update) ───
  if (displayChunk < DISPLAY_CHUNKS) {
    display.updateDisplayArea((displayChunk % DISPLAY_CHUNKS_PER_ROW) * DISPLAY_CHUNK_TILES,
                              displayChunk / DISPLAY_CHUNKS_PER_ROW,
                              DISPLAY_CHUNK_TILES, 1);
    displayChunk++;
  }

  // ─── Battery monitoring and OLED display (every 100th loop ~ 1 second) ───
  bool lqTick = false;
  loopCounter++;
  if (loopCounter >= DISPLAY_EVERY_N_LOOPS) {
    loopCounter = 0;
    lqTick = true;

    // Link quality over the last second
    txLinkQuality = min(ackCount, (uint8_t)100);
    ackCount = 0;
    retxPerSec = retxCount;
    retxCount = 0;

    // Radio health check: nRF brownout reset (POR = 2Mbps) that write() did not
    // catch (write() timeouts are handled right after write() every slot)
    if (radio.getDataRate() != (rf24_datarate_e)RADIO_DATA_RATE) {
      reinitRadio(currentMs);
      if (radioInitFailed) {
        return;
      }
    }

    // Read battery voltage using bandgap method (no divider needed)
    // Measures VCC against internal 1.1V reference: VCC = 1.1V * 1023 / ADC
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
    delay(2);
    ADCSRA |= _BV(ADSC);
    while (bit_is_set(ADCSRA, ADSC));
    batteryVoltageMv = (int)(1131349L / ADC);

    // Map voltage from 3000mV (0%) to 4200mV (100%), clamped
    batteryPercent = constrain(map(batteryVoltageMv, 3000, 4200, 0, 100), 0, 100);

    // Update OLED display using full frame buffer
    display.clearBuffer();
    display.setFont(u8g2_font_7x14B_tr);

    // Row 1: Battery voltage (V) for testing
    display.setCursor(0, 13);
    display.print(F("BAT: "));
    display.print(batteryVoltageMv / 1000);
    display.print(F("."));
    int decimals = (batteryVoltageMv % 1000) / 10;
    if (decimals < 10) display.print(F("0"));
    display.print(decimals);
    display.print(F("V"));

    // Row 2: Connection status
    display.setCursor(0, 29);
    if (robotConnected) {
      display.print(F("LINKED"));
    } else {
      display.print(F("NO LINK"));
    }

    // Low battery warning (replaces connection text)
    if (batteryPercent <= 10) {
      display.setCursor(70, 29);
      display.print(F("LOW!"));
    }

    // Frame is sent in chunks over the next slots (was display.sendBuffer())
    displayChunk = 0;

    // Link quality debug line (replaces the normal line on this tick)
    Serial.print(F("TX: LQ tx="));
    Serial.print(txLinkQuality);
    Serial.print(F("% rx="));
    Serial.print(lastAckValid ? lastAck.linkQuality : 0);
    Serial.print(F("% rt="));
    Serial.print(retxPerSec);
    Serial.print(F(" fs="));
    Serial.println((lastAckValid && (lastAck.flags & ACK_FLAG_FAILSAFE)) ? 1 : 0);
  }

  // ─── Debug output ───
  debugCounter++;
  if (debugCounter < DEBUG_EVERY_N_LOOPS || lqTick) {
    return;
  }
  debugCounter = 0;
  Serial.print(F("TX: ACK="));
  Serial.print(ackReceived ? "Y" : "N");
  Serial.print(F(" robot="));
  Serial.print(robotConnected ? "Y" : "N");
  Serial.print(F(" Y="));
  Serial.print(rawY);
  Serial.print(F(" spd="));
  Serial.print(speed);
  Serial.print(F(" X="));
  Serial.print(rawX);
  Serial.print(F(" trn="));
  Serial.print(turn);
  Serial.print(F(" btn="));
  Serial.println(btnPressed ? "1" : "0");
}
