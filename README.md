arduipi
=======

ArduiPi is a shield for Raspberry Pi that brings Arduino low level extented I/O to Raspberry Pi.

This project is currently under developement and you can participate on design of hardware and/or software.


Why another shield for Raspberry or another Arduino board ? 
-----------------------------------------------------------
Well, quite simple, Arduino is pretty cool but as soon you want to connect it to network, shield are quite expensive and web server will take lot of space into your Arduino and taking some functions off (such as SPI).
That why Raspberry come, it cost less than a Arduino official shield and can do a lot of high level thing such as web server, database, home automation, ... But the drawback is that is I/O ports are quite complex to use and not so efficient than a arduino.
That said, this is why I think Raspberry and Arduino could get married :-), let then talk together and assign task to the most competitive one.


What will be the main capabilities of this shield board ?
---------------------------------------------------------
- Use of traditional components so anyone with basic soldering experience can setup the board
- Designed to be opened and fulfill major uses, letting you to choose the fonctions you need and remove the other ones.
- Lot of option that can be setup by solder pad or by little switch
- Source code and hardware will be open and available on github this means that you will be able to change anything to your needs.
- You will be able to host lighttpd web server (or other) on Raspberry Pi with API to control arduino I/O 


Wow, I get excited ? Could you talk about the functionnalities ?
----------------------------------------------------------------
- Power the Arduino side with Raspberry power or with external power 
- Selectable Arduino side power with 5V or 3.3V (works if powered by Raspberry or by external power)
- FTDI cable connector to be able to program AVR chip independently leaving RX/TX of arduino and Raspberry free
- ICSP connector (may be removed if not used)
- Power level shifter between Pi side and Arduino side for I2C
- Power level shifter between Pi side and Arduino side for SPI
- 1-Wire (DS2482) IC setup on the board providing fully OWFS support provided by I2C bus (can be used by Pi and/or Arduino)
- Soldering pad I2C between Arduino and Raspberry (Arduino will work as a I2C slave from Raspberry)
- Soldering pad SPI between Arduino and Raspberry (Arduino will work as a SPI slave from Raspberry)
- Option to connect the Arduino to Raspberry Pi Serial to do async communication if needed
- Possibility to place needed component (Opto coupler and two resistors) to use the French dedicated telinformation to send Pi electriciy information issued the the main power meter (using serial RX)
- Grove connector on board for I2C and Serial connection from Pi and Arduino
 
Fine, when will all of this will be available ?
-----------------------------------------------
I am currently working on, the steps are :
- prototype the schematics (POC) with Eagle CAD => almost done
- designing the schematic and the PCB with Eagle CAD => currently working on
- create lot of sample code (i2c, spi, serial, on Arduino side and on Raspberry side) => i2c done on both side
- create basic WEB UI on raspberry using lighttpd => to do
- wished this project on Seeedstudio R-Duino-Pi contest page http://www.seeedstudio.com/blog/2012/10/08/meet-arduipi-a-rpi-bridge-plate-for-arduino-fans/


So that all for now, let me know if you have more ideas and if you want more specific things or even if you want to help.

Charles