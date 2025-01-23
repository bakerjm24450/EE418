# EE 418 Project 1

This system is an interrupt-driven interface to a DHT22 temperature and humidity sensor.

## Implementation details
The DHT22 uses a unique 1-wire communication protocol, transmitting a 40-bit data packet
after each measurement. Each bit is sent as a 50us low pulse followed by a high pulse
of 26-28us for a bit of 0, or 70us for a bit of 1.

The data is sent as 16 bits for the humidity value, 16 bits for the temperature value, and
an 8 bit checksum.

The interrupt is triggered on the falling edge of signal, at the end of each transmitted bit. After
42 edges (2 start bits followed by 40 data bits), a flag is set to indicate the packet is complete.

## Supplied code

### dht.h
dht.h contains the prototype declarations for several C functions. The requirements of
each are:

#### dhtInit()
Configure the GPIO pin used to communicate with the DHT22 and install the ISR

#### dhtStartMeasurement()
Start a measurement on the DHT22 by pulling the GPIO pin low for 1 ms. See the project
notes and/or DHT22 datasheet for details on starting a measurement.

#### dhtGetResults()
Wait for the measurement to complete and compute the temperature in Fahrenheit and
the relative humidity. This function should return a code indicating succcess or
failure. If there is a failure, the cause should be indicated (timeout or checksum mismatch).

### dht.c
Write the implementations of the DHT22 functions (defined in dht.h) here. You should
also include the ISR function in dht.c

### main.c
The supplied main.c can be used as-is. It initializes the DHT22 interface, and then
enters an infinite loop to take a measurement every 2 seconds, displaying the results.