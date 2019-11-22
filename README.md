# B-L475E-IOT01A enhanced code for use in ECET230 

The ARM will send the following packet once each time interval. Default time interval 0.1 second. The packet will be ASCII values.   

* The start of packet is 3 # signs:					###
* The next 3 Packet Number:					000 to 999 

* The next 4 characters are the X accelerometer value:	0000 to FFFF
* The next 4 characters are the Y	accelerometer value:	0000 to FFFF
* The next 4 characters are the Z accelerometer value:	0000 to FFFF

* The next 4 characters are the X gyroscope value:	0000 to FFFF
* The next 4 characters are the Y	gyroscope value:	0000 to FFFF
* The next 4 characters are the Z gyroscope value:	0000 to FFFF

* The next 4 characters are the X magnetometer value:	0000 to FFFF
* The next 4 characters are the Y	magnetometer value:	0000 to FFFF
* The next 4 characters are the Z magnetometer value:	0000 to FFFF

* The next 1 characters is for temperature above or below zero value:	+ or -
* The next 5 characters are the temperature value:	000.0 to 999.9

* The next 5 characters are the humidity value:	000.0 to 999.9

* The next 4 characters are the pressure value:	0000 to 9999

* The next 4 characters are the analog ADC ARDA0 value:	0000 to 4095
* The next 4 characters are the analog ADC ARDA1 value:	0000 to 4095
* The next 4 characters are the analog ADC ARDA2 value:	0000 to 4095
* The next 4 characters are the analog ADC ARDA3 value:	0000 to 4095
* The next 4 characters are the analog ADC ARDA4 value:	0000 to 4095
* The next 4 characters are the analog ADC ARDA5 value:	0000 to 4095

* The next 5 characters are for the input state of the I/O	00000 to 11111 of Input pins (Arduino D4 to D0)

* The blue button (PA0)		value:		0 or 1

* The next three characters are the checksum value:		000 to 999

* The next 2 characters are \r\n		value:		\r\n

The GUI can send the following to the ARM at any time you require for you project. The packet will be ASCII values.
* The start of packet is 3 # signs:					###
* The next 4 characters are for the output state of the I/O	0000 to 1111 of Output pins
 (Arduino D13, D12, D11, D08)
 
*3 characters  d5d6prescaler  	value: 000-999
*3 characters  d5d6period		value: 000-999
*3 characters  d5pulse			value: 000-999
*3 characters  d6pulse			value: 000-999
 
*3 characters  d9d10prescaler  	value: 000-999
*3 characters  d9d10period		value: 000-999
*3 characters  d9pulse			value: 000-999
*3 characters  d10pulse			value: 000-999 
 
*3 character checksum:			value: 000-999

*2 characters /r/n

