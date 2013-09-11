/* ========================================================================
Program : i2c_demo.ino
Purpose : wait for i2c command, parse command and do action 
Version : 1.0
Author  : (c) Charles-Henri Hallard (http://hallard.me)
Comments: this file belong to the ArduiPi project
		you will find more information on this project on my blog and github
		http://hallard.me/arduipi
		https://github.com/hallard/arduipi
	  You can use or distribute this code unless you leave this comment
	  too see this code correctly indented, please use Tab values of 2
=========================================================================== */

#include <arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <DS2482.h>
#include <SeeedGrayOLED.h>

// ======================================================================
// Constants definition
// ======================================================================
#define  SLAVE_ADDRESS	0x2a  /* slave address,any number from 0x01 to 0x7F */
#define  CMD_MAX_SIZE   16  	/* max command size */
#define  MAX_SENT_BYTES 3
#define  IDENTIFICATION 0x0D
#define  LOOP_DELAY 	2000 		/* by default blink led every 2 seconds */
#define  BLINK_VALUE	LOOP_DELAY / 2 

//#define	 DEBUG_SERIAL

#define	 CMD_ARDUINO_PIN0		0x00
#define	 CMD_ARDUINO_PIN18	0x12
#define	 CMD_PORT_PIN0			0x00
#define	 CMD_PORT_PIN7			0x07
#define	 CMD_A0_ARDUINO			0xA0
#define	 CMD_A6_ARDUINO			0xA6
#define	 CMD_AVR_CMD_PORTB	0x1B
#define	 CMD_AVR_CMD_PORTC	0x1C
#define	 CMD_AVR_CMD_PORTD	0x1D
#define	 CMD_AVR_CMD_DDRB		0x2B
#define	 CMD_AVR_CMD_DDRC		0x2C
#define	 CMD_AVR_CMD_DDRD		0x2D
#define	 CMD_A0_AVR					0xC0
#define	 CMD_A6_AVR					0xC6
#define	 CMD_DDR0_PIN				0xD0
#define	 CMD_DDR7_PIN				0xD7
#define	 CMD_DDR_ARDUINO		0xDD
#define	 CMD_PING						0xE0
#define	 CMD_PORT_VALUE			0xF0
#define	 CMD_DDR_VALUE			0xFD
#define	 CMD_SEPARATOR			0xFF


// Define various ADC prescaler
#define PS_16  ((1 << ADPS2))
#define PS_32  ((1 << ADPS2) | (1 << ADPS0))
#define PS_64  ((1 << ADPS2) | (1 << ADPS1))
#define PS_128 ((1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0))

#define pinLed 2

// State machine for parsing received command
enum parse_cmd	{ PARSE_CMD, PARSE_DATA, PARSE_NEXT, PARSE_ALL, PARSE_ERR, PARSE_OK };

// ======================================================================
// Volatile Global vars, may be used in interrupts
// standard Global vars 
// ======================================================================
volatile byte g_i2c_rx_buf[CMD_MAX_SIZE];	// i2c command sent by the master
volatile byte g_i2c_rx_len=0;							// i2c command len sent by the master
volatile byte g_spi_rx_buf[CMD_MAX_SIZE];	// spi command sent by the master
volatile byte g_spi_rx_len=0;							// spi command len sent by the master
volatile byte g_ser_rx_buf[CMD_MAX_SIZE];	// serial command sent by the master
volatile byte g_ser_rx_len=0;							// serial command len sent by the master
volatile boolean g_i2c_new = false;			// new i2c command received to treat
volatile boolean g_spi_new = false;			// new spi command received to treat
volatile boolean g_ser_new = false;			// new serial command received to treat
volatile byte g_i2c_tx_len = 0;						// lenght of i2c data to return to master
volatile byte g_spi_tx_len = 0;						// lenght of spi data to return to master
volatile byte g_ser_tx_len = 0;						// lenght of serial data to return to master
//volatile byte g_spi_pos = 0;						// SPI pointer on receive buffer
volatile byte g_cmd_err = 0;							// global command error

volatile long g_vcc = 0;									// vcc value (read from ADC)

byte g_i2c_tx_buf[CMD_MAX_SIZE]; 					// i2c buffer of returned data to master
byte g_spi_tx_buf[CMD_MAX_SIZE]; 					// spi buffer of returned data to master
byte g_ser_tx_buf[CMD_MAX_SIZE]; 					// serial buffer of returned data to master
byte g_cmd_size;													// new command size (can identify quickly simple command)
byte g_cmd_send= false;									// new data to send to master
byte g_ping = 0x2a;												// default ping value data to respond
byte g_ds18b20[8] ;
char buff[17];


DS2482 ds(0);

// Test passed variable flags
boolean g_i2c_tested = false;			// indicate that I2C test passed
boolean g_spi_tested = false;			// indicate that SPI test passed
boolean g_ser_tested = false;			// indicate that serial test passed
boolean g_1w_tested  = false;			// indicate that 1-Wire test passed
boolean g_analog_tested  = false;	// indicate that analog input test passed



/* ======================================================================
Function: readADC
Purpose : Read Analog Avlue
Input 	: -
Output	: value readed
Comments: Channel needed to read has been selected before
====================================================================== */
uint16_t readADC()
{
	uint8_t low, high;

	// Start Convertion
	ADCSRA |= _BV(ADSC); 
	
	// wait end
	while (bit_is_set(ADCSRA,ADSC));
	
	// read low first
	low  = ADCL;
	high = ADCH;

	return ((high << 8) | low);
}

/* ======================================================================
Function: setup
Purpose : initialize arduino board
Input 	: -
Output	: -
Comments: 
====================================================================== */
void setup()
{
	long a, vcc;
	uint8_t i;
	

	#ifdef DEBUG_SERIAL
		Serial.begin(57600);
		Serial.println("Starting ArduiPi Test Program");
	#else
		Serial.begin(9600);
	#endif

  pinMode(pinLed,OUTPUT);
  pinMode(2,OUTPUT);
  pinMode(3,OUTPUT);
  pinMode(4,OUTPUT);
  pinMode(5,OUTPUT);
  pinMode(6,OUTPUT);
  pinMode(7,OUTPUT);
  pinMode(8,OUTPUT);
  pinMode(9,OUTPUT);

  // clear the ADC prescaler defined by arduino env
	// enable ADC and set prescaler to 64 (250Khz)
  //ADCSRA &= ~PS_128;  
  //ADCSRA |= PS_64;    
	ADCSRA = _BV(ADEN) | _BV(ADPS2) | _BV(ADPS1) ;	
	
	// REFS1 REFS0          --> 0 1, AVcc internal ref. -Selects AVcc external reference
	// MUX3 MUX2 MUX1 MUX0  --> 1110 1.1V (VBG)         -Selects channel 14, bandgap voltage, to measure
  ADMUX = (0<<REFS1) | (1<<REFS0) | (0<<ADLAR) | (1<<MUX3) | (1<<MUX2) | (1<<MUX1) | (0<<MUX0);
	
  // Setup Analog Pin as input
	pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  
	// we are ready, start i2c slave mode
  Wire.begin(SLAVE_ADDRESS); 
	
	// Init DS2482 1-Wire i2c controller
	ds.reset();
	
	// Clear 1-Wire device Address
	for (i=0;i<8;i++)
		g_ds18b20[i] = 0 ;
	
	#ifdef DEBUG_SERIAL
		Serial.println("Searching 1-Wire devices."); 
	#endif 
	
	if ( !ds.wireSearch(g_ds18b20)) 
	{ 
		#ifdef DEBUG_SERIAL
			Serial.println("No more addresses."); 
		#endif 
		
		ds.wireResetSearch(); 
	}
	else
	{
		#ifdef DEBUG_SERIAL
			Serial.print("Found Device : 0x"); 

			for (i=0;i<8;i++)
				sprintf(&buff[i*2], "%02X", g_ds18b20[i]);

			Serial.println(buff); 
		#endif

		// Got 1 wire device okay
		g_1w_tested = true;
	}

	//initialize SEEED Gray OLED display
  SeeedGrayOled.init();  								
	
	// Set i2c to 400Khz to improve display speed
	// TWBR = 10 ;

	// Set i2c to 100Khz to improve compatibility
	TWBR = 10 ;

	//clear the screen and set start position to top left corner
  SeeedGrayOled.clearDisplay();           

	// Set display to Normal mode
  SeeedGrayOled.setNormalDisplay();       
	
	// Set to vertical mode for displaying text
  SeeedGrayOled.setVerticalMode();        
	
	//Set the cursor to 0th line, 0th Column  
  SeeedGrayOled.setTextXY(0,0);          
	
  SeeedGrayOled.putString("ArduiPi Test");
	
	// register ISR Interrupt for I2C
  Wire.onRequest(requesti2cEvent);
  Wire.onReceive(receivei2cEvent);

  // Set SPI to 1MHz
  SPI.setClockDivider(SPI_CLOCK_DIV16);
  
	// have to send on master in, slave out
	pinMode(MISO, OUTPUT);    

	// turn on SPI in slave mode 
	//pinMode(SS, INPUT);    
	SPCR |= _BV(SPE);    

  // turn on interrupts
  SPCR |= _BV(SPIE);
	
	// SPI First response should be ping response
	SPDR = g_ping;
}

/* ======================================================================
Function: loop
Purpose : main loop, blink led and wait for received command, then do it
Input 	: -
Output	: -
Comments: 
====================================================================== */
void loop()
{

	static byte nblink = 2 ;			// number of blink (/2) to do (default 1 so we set 2)
	static byte blinkdelay =100;	// duration of the blink (default 100 ms)
	static int  ldelay = LOOP_DELAY; // every second loop
	static byte c;
	static uint8_t pin = pinLed;
	static uint16_t _a0,_a1,_a2,_a3;
	static long _millis;
	
  //light=analogRead(0);  // reading photoresistor

  //Serial.print("light: " );
  //Serial.println(light );

  // Loop until delay expired or command received
  while ( (ldelay != 0) && (g_i2c_new == false) && (g_spi_new == false) && (g_ser_new == false))
  {
		// take care, if we need to answer to device or
		// other devices this delay should not be too long 
		// to keep timing safe and to anwser quickly
		
		// Read 1.1V reference against AVcc
		ADMUX &= 0b11110000;
		ADMUX |= 0b00001110;
		delay(2);
		g_vcc = readADC();
		
		// Back-calculate Vcc in mV
		g_vcc = 1126400L / g_vcc ; 
		
		// Read A0
		ADMUX &= 0b11110000 ;
		delay(2);
		_a0 = (g_vcc * readADC()) / 1023 ;
		
		// Read A1
		ADMUX &= 0b11110000 ;
		ADMUX |= 0b00000001;
		delay(2);
		_a1 = (g_vcc * readADC()) / 1023 ;

		// Read A2
		ADMUX &= 0b11110000 ;
		ADMUX |= 0b00000010;
		delay(2);
		_a2 = (g_vcc * readADC()) / 1023 ;

		// Read A3
		ADMUX &= 0b11110000 ;
		ADMUX |= 0b00000011;
		delay(2);
		_a3 = (g_vcc * readADC()) * 11 / 1023 ;
		
		
		// Now check the values are correct
		// A0 must be between 3.1V and 3.5V
		// A1 must be between 1.4V and 1.8V
		// 3V3 (a2) must be between 3.2V and 3.4V
		// VIN (a3) should be > 6V
		if ((_a0 > 3100 && _a0 < 3500 ) &&
			 (_a1 > 1400 && _a1 < 1800 ) &&
			 (_a2 > 3200 && _a2 < 3400 ) &&
			 (_a3 > 6000 ) )
		{
			g_analog_tested = true ;
		}
		else
		{
			g_analog_tested = false ;
		}
		
		// Check if we received Serial Data
		if (Serial.available() > 0)
		{
			c = Serial.read();

			// check not overflowing, our buffer is enought ?
			if ( g_ser_rx_len < CMD_MAX_SIZE - 1 ) /* keep \0 of the serial string */
			{
				// discard \r
				if (c != '\r' )
				{
					// End of command
					if ( c == '\n' )
					{
						// We received a string, end it without \r or \n
						g_ser_rx_buf[g_ser_rx_len] = 0x00;
						
						// Time to treat this command
						g_ser_new = true;
					}
					else
					{
						// Put char in buffer 
						g_ser_rx_buf[g_ser_rx_len++] = c;
					}
				}
			}
			else
			{
				// overflow 
				// force treating buffer
				g_ser_rx_buf[CMD_MAX_SIZE] = 0x00;
						
				// Time to treat this command
				g_ser_new = true;
			}
		}

    // wait 10 ms done waiting ADC to settle
    // delay(2);  
    ldelay -= 10 ;
		
		// each 100 ms if we need to blink
		if ( (ldelay % 100) == 0 )
		{
			if ( nblink > 0 )
			{
				if ( (nblink % 2) == 0  )
				{
					// light on the led
					digitalWrite(pin,1);  
					
				}
				else
				{
					// light off the led
					digitalWrite(pin,0);  
					
					// next led
					if (++pin > 9 )
						pin = pinLed;
				}

				// we finished this blink
				if ( --nblink <= 0 )
				{
					// restart loop delay without blink
					ldelay = LOOP_DELAY ;
				}
			}
		}
  }

	// ok we exited waiting delay for us
  // so, is there something to do for I2C ?
  if (g_i2c_new )
  {
		// parse command and setup the blink
		nblink = parse_cmd( false ) * 2;

		// Reset buffer len;
		g_i2c_rx_len = 0;
		
		// we done what ne needed to on our received command
		g_i2c_new = false;
	}
	
  // so, is there something to do for SPI ?
	// for now do nothing except ACK for test firmware
  if ( g_spi_new )
	{
		// parse command and setup the blink
		//nblink = parse_cmd( false ) * 2;
		
		// Spi is working
		g_spi_tested = true;
		
		// time to go, we are ready to send
		// g_spi_cmd_send = true ;
	
		// Reset buffer len;
		g_spi_rx_len = 0;

		// ack our received command
		g_spi_new = false;
	}

  // so, is there something to do for Serial ?
  if ( g_ser_new )
	{
		// parse command and setup the blink
		nblink = parse_cmd( false ) * 2;
	
		// Reset buffer len;
		g_ser_rx_len = 0;
		
		// ack our received command
		g_ser_new = false;
	}
	
  // main loop delay expired, time to refresh screen
  if (ldelay == 0) 
	{
	
<<<<<<< HEAD
		// Display only when received 1st i2c command from PI
		// this avoid I2C bus corruption
		if ( g_i2c_tested )
		{
			// Set i2c to 400Khz
			// TWBR = 10 ;
			
			_millis = millis();
=======
		// Set i2c to 400Khz
		//TWBR = 10 ;
		
		_millis = millis();
>>>>>>> 844ab5995264b348bbf195b0210b292249921bf4

			SeeedGrayOled.setTextXY(1,0);           
			SeeedGrayOled.putString("1Wire  : ");
			SeeedGrayOled.putString(g_1w_tested ? "OK":"--");
		
			SeeedGrayOled.setTextXY(2,0);           
			SeeedGrayOled.putString("Analog : ");
			SeeedGrayOled.putString(g_analog_tested ? "OK":"--");
		
			SeeedGrayOled.setTextXY(3,0);           
			SeeedGrayOled.putString("I2C    : ");
			SeeedGrayOled.putString(g_i2c_tested ? "OK":"--");

			SeeedGrayOled.setTextXY(4,0);           
			SeeedGrayOled.putString("SPI    : ");
			SeeedGrayOled.putString(g_spi_tested ? "OK":"--");

			SeeedGrayOled.setTextXY(5,0);           
			SeeedGrayOled.putString("Serial : ");
			SeeedGrayOled.putString(g_ser_tested ? "OK":"--");


<<<<<<< HEAD
	/*		
			SeeedGrayOled.setTextXY(2,0);           
			SeeedGrayOled.putString("Vcc ");
			dtostrf(g_vcc/1000.0,4,2,buff);
			SeeedGrayOled.putString(buff);
			SeeedGrayOled.putString(" V");
			
			SeeedGrayOled.setTextXY(3,0);
			SeeedGrayOled.putString("A0  ");
			dtostrf(_a0/1000.0,4,2,buff);
			SeeedGrayOled.putString(buff);
			SeeedGrayOled.putString(" V");

			SeeedGrayOled.setTextXY(4,0);
			SeeedGrayOled.putString("A1  ");
			dtostrf(_a1/1000.0,4,2,buff);
			SeeedGrayOled.putString(buff);
			SeeedGrayOled.putString(" V");

			SeeedGrayOled.setTextXY(5,0);
			SeeedGrayOled.putString("3V3 ");
			dtostrf(_a2/1000.0,4,2,buff);
			SeeedGrayOled.putString(buff);
			SeeedGrayOled.putString(" V");

			SeeedGrayOled.setTextXY(6,0);
			SeeedGrayOled.putString("Vin ");
			dtostrf(_a3/1000.0,4,2,buff);
			SeeedGrayOled.putString(buff);
			SeeedGrayOled.putString(" V");
	*/		
			_millis = millis() - _millis;

			SeeedGrayOled.setTextXY(8,0);
			SeeedGrayOled.putString("took ");
			SeeedGrayOled.putNumber(_millis);
			SeeedGrayOled.putString(" ms");

			// Set i2c to 100Khz
			// TWBR = 72 ;
		}
=======
/*		
		SeeedGrayOled.setTextXY(2,0);           
		SeeedGrayOled.putString("Vcc ");
		dtostrf(g_vcc/1000.0,4,2,buff);
		SeeedGrayOled.putString(buff);
		SeeedGrayOled.putString(" V");
		
		SeeedGrayOled.setTextXY(3,0);
		SeeedGrayOled.putString("A0  ");
		dtostrf(_a0/1000.0,4,2,buff);
		SeeedGrayOled.putString(buff);
		SeeedGrayOled.putString(" V");

		SeeedGrayOled.setTextXY(4,0);
		SeeedGrayOled.putString("A1  ");
		dtostrf(_a1/1000.0,4,2,buff);
		SeeedGrayOled.putString(buff);
		SeeedGrayOled.putString(" V");

		SeeedGrayOled.setTextXY(5,0);
		SeeedGrayOled.putString("3V3 ");
		dtostrf(_a2/1000.0,4,2,buff);
		SeeedGrayOled.putString(buff);
		SeeedGrayOled.putString(" V");

		SeeedGrayOled.setTextXY(6,0);
		SeeedGrayOled.putString("Vin ");
		dtostrf(_a3/1000.0,4,2,buff);
		SeeedGrayOled.putString(buff);
		SeeedGrayOled.putString(" V");
*/		
		_millis = millis() - _millis;

		SeeedGrayOled.setTextXY(8,0);
		SeeedGrayOled.putString("took ");
		SeeedGrayOled.putNumber(_millis);
		SeeedGrayOled.putString(" ms");

		// Set i2c to 100Khz
		//TWBR = 72 ;
>>>>>>> 844ab5995264b348bbf195b0210b292249921bf4

		// restart new loop 
		ldelay = LOOP_DELAY ;
		
		// setup a new blink
		nblink = 2;
	}
}




/* ======================================================================
Function: parse_cmd
Purpose : parse command received
Input 	: flag indicating if we need to send return value (ie i2cget command)
					this flag is set when called from i2c interrupt 
Output	: number of blink the alive led should blink
Comments: 
====================================================================== */
int parse_cmd( boolean is_get_command )
{
	volatile byte * prx ;		// pointer on received buffer
	volatile byte * ptx;	// pointer ou transmit buffer 
	volatile byte * ptx_len;	// pointer ou transmit buffer len
	volatile byte * prx_len;	// pointer ou received buffer len

	static volatile uint8_t *pport	;	// pointer to the port we will work on
	static volatile uint8_t *pddr	;	// pointer to the port DDR we will work on
	static int i;
	static byte cmd;

	// pointer to the correct buffer
	if ( g_i2c_new )
	{
		// i2c command
		prx = &g_i2c_rx_buf[0] ;
		ptx = &g_i2c_tx_buf[0] ;
		ptx_len = &g_i2c_tx_len ;
		prx_len = &g_i2c_rx_len ;
	}
	else if (g_spi_new)
	{
		// spi command
		prx = &g_spi_rx_buf[0] ;
		ptx = &g_spi_tx_buf[0] ;
		ptx_len = &g_spi_tx_len ;
		prx_len = &g_i2c_rx_len ;
	}
	else
	{
		// serial command
		prx = &g_ser_rx_buf[0] ;
		ptx = &g_ser_tx_buf[0] ;
		ptx_len = &g_ser_tx_len ;
		prx_len = &g_ser_rx_len ;
	}
	

	if ( ! is_get_command)
	{
		#ifdef DEBUG_SERIAL
			Serial.print(g_i2c_new ? "I2C":g_spi_new ? "SPI":"Serial");
			Serial.print(" Command (");
			Serial.print(*prx_len);
			Serial.print(") : ");
		
			// print all buffer received bytes
			for (i = 0; i <*prx_len ; i++)
			{
				Serial.print(*(prx+i), HEX);
				Serial.print(" ");
			}
			Serial.println("");
		#endif
	}
		
	// get command received and point on next value
	cmd = *prx++;
	
	// Ping set return value command 
	if ( cmd == CMD_PING )
	{
		// Ping Get command
		if ( is_get_command )
		{
			*ptx = g_ping ;
			*ptx_len = 1;

			// send response buffer quickly before debug
			if ( g_i2c_new)
			{
				Wire.write( (uint8_t *) ptx, *ptx_len); 
				g_i2c_tested = true;
			}
			
			#ifdef DEBUG_SERIAL
				Serial.print("Ping = 0x");
				Serial.println(*ptx, HEX);
			#endif
		}
		// Ping Set command
		else if ( *prx_len == 2 )
		{
			// next ping will return the new data received
			g_ping = *prx;
			
			#ifdef DEBUG_SERIAL
				Serial.print("Set Ping return value to 0x");
				Serial.println(g_ping, HEX);
			#endif
		}
	} // if ping command
	
	
	// Specific to test firmware, just send back what we get on serial
	// followed by :OK, then Pi should send back ACK
	if ( g_ser_new)
	{
			#ifdef DEBUG_SERIAL
				Serial.print("Serial received(");
				Serial.print(*prx_len);
				Serial.print(") : ");
			#endif
			
			// If we received ACK from PI, all is fine
			if (*(prx-1) == 'A' && *(prx+0) == 'C' && *(prx+1) == 'K' )
			{
				g_ser_tested = true;
			}
			else
			{
				// print all buffer received bytes followed by OK
				for (i = 0; i <*prx_len ; i++)
					Serial.write(*(prx+i-1));

				// Add the :OK at the end to tell Pi it's OKAY
				// After that, PI should send US ACK response
				Serial.println(":OK");
				Serial.flush();
			}

			// Nothing more to do
			return 0;
	}
	
	// Analog command
	else if ( (cmd >= CMD_A0_ARDUINO && cmd <= CMD_A6_ARDUINO) || (cmd >= CMD_A0_AVR && cmd <= CMD_A6_AVR) )
	{
		if (cmd >= CMD_A0_AVR )
			// convert avr command to arduino
			cmd -= CMD_A0_AVR;
		else
			// convert arduino command to analog port (0..5)
			cmd -= CMD_A0_ARDUINO;

		// Analog Get command
		if ( is_get_command )
		{
			// A6 is not existing but we will use this command to return vcc value in mV
			if (cmd == CMD_A0_ARDUINO - CMD_A6_ARDUINO) 
			{
				i = g_vcc;
			}
			else
			{
					// read data (could be optimized with direct avr register control)
				i = analogRead ( cmd );
			}
			
			// LSB first 
			*ptx = (byte) ( i & 0xFF);
			*(ptx+1) = (byte) ( ( i & 0xFF00)  >> 8 );
			*ptx_len = 2;

			// send I2C response buffer quickly before debug
			Wire.write( (uint8_t *) ptx, *ptx_len); 
				
			// SPI response
			if ( g_spi_new) 	
			{
				// To DO SPI response
			}
			
			#ifdef DEBUG_SERIAL
				Serial.print("AnalogRead(");
				Serial.print( cmd );
				Serial.print(") = ");
				Serial.println(i, HEX);
			#endif

		}
	} // if Analog Read command
	
	// Arduino pin command 
	else if ( (cmd >= CMD_ARDUINO_PIN0 && cmd <= CMD_ARDUINO_PIN18) )
	{
		// Arduino Get pin command
		if ( is_get_command )
		{
			*ptx = digitalRead ( cmd);
			*ptx_len = 1;

			// send response buffer quickly before debug
			Wire.write( (uint8_t *) ptx, *ptx_len); 
				
			#ifdef DEBUG_SERIAL
				Serial.print("DigitalRead(");
				Serial.print( cmd );
				Serial.print(")=");
				Serial.println( *ptx );
			#endif
		}
		// Arduino Set pin command (byte value)
		else if ( *prx_len == 2 )
		{
			// 1st byte is DDR command 
			if (*prx == INPUT || *prx==OUTPUT )
			{
				digitalWrite ( cmd, *prx);
				#ifdef DEBUG_SERIAL
					Serial.print("DigitalWrite(");
					Serial.print( cmd );
					Serial.print(", 0x");
					Serial.print( *prx, HEX );
					Serial.println(")");
				#endif
			}
		}
		// Arduino Set pin direction command (word value)
		else if ( *prx_len == 3 )
		{
			// 1st byte is DDR command 
			if (*prx == CMD_DDR_ARDUINO)
			{
				// 2nd byte is pin mode value
				if (*(prx+1) >= INPUT || *(prx+1) <= INPUT_PULLUP)
				{
					pinMode ( cmd, *(prx+1) );
					#ifdef DEBUG_SERIAL
						Serial.print("pinMode(");
						Serial.print( cmd );
						Serial.print(", 0x");
						Serial.print( *prx, HEX );
						Serial.print( "," );
						Serial.print( *(prx+1) );
						Serial.println(")");
					#endif
				}
			}
		}
	} // if Arduino pin command 
	
	// AVR port value
	else if ( cmd >= CMD_AVR_CMD_PORTB && cmd <= CMD_AVR_CMD_PORTD )
	{
		// on which port and DDR we will work
		if ( cmd == CMD_AVR_CMD_PORTB ) 
		{
			pport = &PORTB;
		}
		else if ( cmd == CMD_AVR_CMD_PORTC ) 
		{
			pport = &PORTC;
		}
		else
		{
			pport = &PORTD;
		}
		
		// port Get command
		if ( is_get_command)
		{
			// now we know the port, get port value
			*ptx = *pport ;
			*ptx_len = 1;

			// send response buffer quickly before debug
			Wire.write( (uint8_t *) ptx, *ptx_len); 

		}
		// port Set command
		else
		{
			*pport = *prx;
		}
		
		#ifdef DEBUG_SERIAL
			Serial.print("Port ");
			Serial.write( 'B' + (cmd - CMD_AVR_CMD_PORTB));
			if (is_get_command)
			{
				Serial.print(" value is 0x");
				Serial.println(*ptx, HEX);
			}
			else
			{
				Serial.print(" Set to 0x");
				Serial.println(*prx, HEX);
			}
		#endif
		
	}	// if AVR port value

	// AVR DDR port value
	else if ( cmd >= CMD_AVR_CMD_DDRB && cmd <= CMD_AVR_CMD_DDRD )
	{
		// on which port and DDR we will work
		if ( cmd == CMD_AVR_CMD_DDRB ) 
		{
			pddr = &DDRB;
		}
		else if ( cmd == CMD_AVR_CMD_DDRC ) 
		{
			pddr = &DDRC;
		}
		else
		{
			pddr = &DDRD;
		}
		// port Get command
		if ( is_get_command)
		{
			// now we know the port, get port value
			*ptx = *pddr ;
			*ptx_len = 1;

			// send response buffer quickly before debug
			Wire.write( (uint8_t *) ptx, *ptx_len); 
			
			#ifdef DEBUG_SERIAL
				Serial.print("DDR ");
				Serial.write( 'B' + (cmd - CMD_AVR_CMD_DDRB) );
				Serial.print(" value is 0x");
				Serial.println(*ptx, HEX);
			#endif

		}
		// AVR Set port direction (byte value)
		else if ( *prx_len == 2 )
		{
			*pddr = *prx;
			
			#ifdef DEBUG_SERIAL
				Serial.print("DDR ");
				Serial.write( 'B' + (cmd - CMD_AVR_CMD_DDRB) );
				Serial.print(" Set to 0x");
				Serial.println(*prx, HEX);
			#endif
	
		}
		// AVR Set port pin direction command (word value)
		else if ( *prx_len == 3 )
		{
			// 1st byte is port pin 
			if (*prx >= CMD_PORT_PIN0 && *prx <= CMD_PORT_PIN7)
			{
				// get bit asked
				i = ( 1 << (*prx) );

				// if 2nd byte is DDR value 1
				if (*(prx+1) == 0x01)
				{
					*pddr |= i;
					#ifdef DEBUG_SERIAL
						Serial.print("Set Pin DDR");
						Serial.print(cmd);
						Serial.print(" (|=0x");
						Serial.print( i, HEX );
						Serial.println(")");
					#endif
				}
				// reset pin direction
				// if 2nd byte is DDR value 1
				else if (*(prx+1) == 0x00)
				{
					*pport &= ~i;
					#ifdef DEBUG_SERIAL
						Serial.print("Clear Pin DDR ");
						Serial.print(cmd);
						Serial.print(" (&=0x");
						Serial.print( ~i, HEX );
						Serial.println(")");
					#endif
				}
			}
		}
	} // if AVR DDR port value

	// PWM command
	else if ( (cmd >= CMD_A0_ARDUINO && cmd <= CMD_A6_ARDUINO) || (cmd >= CMD_A0_AVR && cmd <= CMD_A6_AVR) )
	{
		/* TO DO
		*/
	}

	
}


/* ======================================================================
Function: requesti2cEvent
Purpose : i2c ISR called, master want some data from us, just send it
Input 	: -
Output	: -
Comments: ISR code, should be as small as possible, avoid print, println
					or consuming code. If you need heavy treatment, put a flag and 
					do it async in the main loop
					you can also send response as fast as possible and do print after
					this oode is called when we issue a i2cget command from Raspberry
					so we need to answer in this routine, we can't do it async
					!!! here we treat only response to send back to master !!!
====================================================================== */
void requesti2cEvent()
{
	static int i;
	volatile static byte cmd ;
	volatile uint8_t *pport	;	// pointer to the port we will work on
	volatile uint8_t *pddr	;	// pointer to the port DDR we will work on


	// we received new data and it is a read command (1 byte data)
	if ( g_i2c_new && g_i2c_rx_len == 1)
	{
		parse_cmd( true) ;
		
		// we done what ne needed to on our received command
		g_i2c_new = false;
	}

	// we validated the response
  if( g_i2c_tx_len >0)
  {
    // send response buffer 
    //Wire.write(g_i2c_tx_buf, g_i2c_tx_len); 
  }
  else
  {
		#ifdef SERIAL_DEBUG
			Serial.println("requesti2cEvent No data");
		#endif
  }
}

// 
/* ======================================================================
Function: receivei2cEvent
Purpose : i2c ISR called, master sended some data to us, just grab it
Input 	: number of byte received
Output	: -
Comments: ISR code, should be as small as possible, avoid print, println
					or consuming code. If you need heavy treatment, put a flag and 
					do it in the main loop
====================================================================== */
void receivei2cEvent(int nbyte)
{
  static byte p;
  
  // check not overflowing, our buffer is enought ?
  if ( nbyte < CMD_MAX_SIZE )
  {
    // Grab all the command bytes into the receive buffer
    for (p = 0; p < nbyte; p++)
    {
      g_i2c_rx_buf[p] = Wire.read();
    }
		
		// len of data received
		g_i2c_rx_len = nbyte;
		
		// init response len
		g_i2c_tx_len = 0;

    // get out quickly from isr
    // we will do some long time instruction 
		// such as display in the mail loop
    g_i2c_new = true;

  }
  else
  {
    // flush the receive buffer
    while(Wire.available()) 
      Wire.read(); 

		// indicate a error
    g_cmd_err++;
  }
}

/* ======================================================================
Function: spi interrupt vector
Purpose : called when received spi data byte
Input 	: -
Output	: -
Comments: ISR code, should be as small as possible, avoid print, println
					or consuming code. If you need heavy treatment, put a flag and 
					do it in the main loop
					
					!!!!!!!!!!!!!!!!! SPI SPECIFIC !!!!!!!!!!!!!
					The slave cannot respond to an incoming byte at the same moment
					The response has to be in the next exchange. 
					This is because the bits which are being sent, and the bits 
					which are being received, are being sent simultaneously. 
					Thus master has to talk to the slave two times, what we decided here
					  - Master send the string command according API
						- Slave ACK the command with 0xAA return value
						- Master wait slave to be ok to send back response
						- Master send the 0x55 to tell the slave now he needs answer
						- Slave send back the string response
					
 ====================================================================== */
ISR (SPI_STC_vect)
{
	static byte p;

	// get value from SPI Data Register    
	p = SPDR;  
	
	// SPI next response should always be ping response
	SPDR = g_ping;

	// indicate main loop we received a byte 
	g_spi_new = true;          

/*
	// To be sure we have finished processing our prevoius command
	if ( g_spi_new == false);
	{
		//Serial.print(p, HEX);

		// do we want a response from our previous command ?
		if (g_spi_rx_len==0 )    
		{    
			SPDR = 0x00 ;
		}
		
		// if buffer is not full
		if (g_spi_rx_len < CMD_MAX_SIZE)    
		{    
			// New command to process
			if (p != CMD_SEPARATOR) 
			{
				// put received data into buffer
				g_spi_rx_buf[g_spi_rx_len++] = p;        
				
				if ( g_spi_rx_len == 1 && p==CMD_PING )
				{
						SPDR = g_ping ;
				}

				
			}
			else
			{
				// Send magic code to master indicating we will work
				//SPDR = 0xAA;
				
				// Ping set return value command 
	//			if ( g_spi_rx_buf[0]==CMD_PING )
	//			{
	//					SPDR = g_ping ;
	//			}

				g_spi_new = true;          
			}  
		}
	}
	*/
} 

/* ======================================================================
Function: ISR issued from ADC
Purpose : Interrupt routine trigerred when ADC complete
Input 	: - 
Output	: -
Comments: ISR code, should be as small as possible, avoid print, println
					or consuming code. If you need heavy treatment, put a flag and 
					do it in the main loop
====================================================================== */
/*
ISR(ADC_vect)
{
  // Done reading
  readFlag = 1;
  
  // Must read low first
  analogVal = ADCL | (ADCH << 8);
  
  // Not needed because free-running mode is enabled.
  // Set ADSC in ADCSRA (0x7A) to start another ADC conversion
  // ADCSRA |= B01000000;
}
*/
