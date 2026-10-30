#!/usr/bin/env bash
# Upload the RX firmware to a Pro Micro / Leonardo that only shows up in
# bootloader mode (no working sketch on it yet).
# Usage: ./upload_rx.sh   then short RST to GND (twice quickly) when asked.
set -e
cd "$(dirname "$0")"

PIO=~/.platformio/penv/bin/pio
AVRDUDE=~/.platformio/packages/tool-avrdude/avrdude
CONF=~/.platformio/packages/tool-avrdude/avrdude.conf
HEX=.pio/build/rx/firmware.hex

"$PIO" run -e rx

# Wait for any old port to go away, then for the bootloader to appear
while ls /dev/ttyACM* >/dev/null 2>&1; do
  echo "Unplug USB or wait for the board to drop off..."; sleep 1
done
echo ">>> Now short RST to GND twice quickly <<<"
until PORT=$(ls /dev/ttyACM* 2>/dev/null | head -n1) && [ -n "$PORT" ]; do sleep 0.05; done
echo "Bootloader found on $PORT, flashing..."
sleep 0.3
"$AVRDUDE" -C "$CONF" -p atmega32u4 -c avr109 -P "$PORT" -b 57600 -D -U "flash:w:$HEX:i"
echo "Done. Replug USB (without shorting RST) and check: ls /dev/ttyACM*"
