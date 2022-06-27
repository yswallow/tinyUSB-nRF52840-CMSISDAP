## tinyUSB nRF52840 CMSIS-DAP example

### build

1. Place `device/hid_CMSISDAP` folder into `tinyUSB/examples/device/hid_CMSISDAP`
2. Overwrite `tinyUSB/examples/rules.mk` by `rules.mk` in this repository.
2. `cd tinyUSB/examples/device/hid_CMSISDAP`
3. `make BOARD=feather_nrf52840_express all`

### flashing

1. make Feather nRF52840 Express into bootloader mode ( by double-reset )
2. `make BOARD=feather_nrf52840_express flash SERIAL=/dev/ttyACM0`

### wireing

see & configure in `device/hid_CMSISDAP/src/board_config.h`

| feather | target |
|----|----|
| A5 | SWCLK |
| A4 | SWDIO |
| 3V | 3V |
| GND | GND |


