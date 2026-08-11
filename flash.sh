#!/bin/bash

TARGET=${1:-stewart}
BOARD=stm32f411e-disco
BIN=build/${BOARD}/${TARGET}/nuttx.bin

echo ">>> Flashing payload: $TARGET"
echo ">>> Board:            $BOARD"

if [ ! -f "$BIN" ]; then
  echo ">>> ERROR: $BIN not found. Build it first with: ./build.sh $TARGET"
  exit 1
fi

openocd -f interface/stlink.cfg -f target/stm32f4x.cfg \
  -c "transport select swd" \
  -c "adapter speed 100" \
  -c "init" \
  -c "reset halt" \
  -c "flash write_image erase $BIN 0x08000000" \
  -c "reset init" \
  -c "resume" \
  -c "shutdown"