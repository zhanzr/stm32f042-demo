Demo For Nucleo-F042K6 Board.

0.96' 160x80 ST7735S LCD module.
Backlight(Could be left unconnected, if don't need to control the intensity of backlight, it is default on):
BL	<->	PA0

CS	<->	PB1
DC	<->	PB0
RST	<->	PA4

MOSI	<->	PA7
SCLK	<->	PA5

3V3
GND

This is a pure soft spi flavour for easy testing and porting.