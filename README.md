# Electric Druid Digital Delay DIGIDELAY

Changing firmware from Electric Druid Digital Delay Processor

https://electricdruid.net/product/digidelay-delay-processor-chip/

## Programming device

The follow examples uses Pickit 2 on Linux

### Write and Release Vdd

	pk2cmd -P -F./dist/default/production/digidelay.X.production.hex -M -R

### Read and Relase Vdd

	pk2cmd -P -GFx.dat -R

### Pickit2 Programmer Connections

- pin 1 - MCLR
- pin 4 - PGD
- pin 5 - PGC
- pin 8, 19, 27 - VSS (GND)
- pin 13, 28 - VDD
- pin 19(-), 20(+) - 10uF

### Pickit2 ICSP Programmer Connections

It is easy to program the pedal without unplug the Digidelay Processor attaching 
the programmer directly to the pedal pcb board.

- pin 1 - MCLR
- pin 4 - PGD
- pin 5 - PGC
- pin 8 - GND

Do not forget to set Delay (VR3) and Low (VR4) knobs to its middle position.

![Connections](/doc/pickit2_connection.jpg)