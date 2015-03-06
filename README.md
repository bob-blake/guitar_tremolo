# guitar_tremolo
Guitar tremolo pedal firmware for Microchip PIC16F684.  Uses CCS C compiler.

Firmware operation based heavily on Electric Druid's VCLFO (http://www.electricdruid.net).  Five DDS functions implemented: Sine, Ramp Up, Ramp Down, Square, and Triangle.  PWM output controls an LED which modulates a photoresistor, which adjusts the gain of an op-amp.