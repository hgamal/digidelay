# Electric Druid Digital Delay DIGIDELAY

## Write and Release
	pk2cmd -P -F./dist/default/production/digidelay.X.production.hex -M -R

## Read and Relase

	pk2cmd -P -GFx.dat -R

## Pic Programmer Connections
- pin 1 - MCLR
- pin 4  - PGD
- pin 5  - PGC
- pin 8, 19, 27 - VSS (GND)
- pin 13, 28 - VDD
- pin 19(-), 20(+) - 10uF

![Connections](/doc/pickit2_connection.jpg)