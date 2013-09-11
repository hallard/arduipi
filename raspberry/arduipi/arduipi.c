/* ======================================================================
Program : arduipi.c
Purpose : arduipi demo program for ArduiPi project
Version : 1.0
Author  : (c) Charles-Henri Hallard
Comments: This program is written for the open source project ArduiPi 
					you can find documentation and all code on my github located at
					https://github.com/hallard/arduipi

					You can use or distribute this code unless you leave this comment
					too see this code correctly indented, please use tab values of 2
 
====================================================================== */
#include <stdio.h>
#include <stdint.h>
#include <sys/stat.h>
#include <unistd.h>
#include <stdio.h>
#include <linux/i2c-dev.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdarg.h>
#include <errno.h>
#include <string.h>
#include <signal.h>
#include <syslog.h>
#include <getopt.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>
#include <fcntl.h>


// ----------------
// Constants
// ----------------
#define true 1
#define false 0

// Program name and version
#define PRG_NAME    "arduipi"
#define PRG_VERSION	"1.0"

// Define i2c default device & address
#define I2C_DEVICE_0 	"/dev/i2c-0"
#define I2C_DEVICE_1 	"/dev/i2c-1"
#define I2C_ADDRESS	0x2A

// Define spi default device 
#define SPI_DEVICE_1	"/dev/spidev0.1"
#define SPI_DEVICE_0	"/dev/spidev0.0"
#define SPI_MODE			0
#define SPI_BITS_WORD	8	
#define SPI_SPEED			1000000
#define SPI_DELAY			0

// Arduipi defined command
#define ARDUIPI_CMD_PING 0xe0

// receive buffer size
#define BUFFER_SIZE	32

// Program mode function
enum mode_e 	{ MODE_QUICK_ACK, MODE_READ_ACK, MODE_SET, MODE_GET, MODE_GET_WORD };

// Program protocol 
enum proto_e 	{ PROTO_I2C, PROTO_SPI, PROTO_SERIAL };

// Config Option structure parameters
struct 
{
	char port[128];	// device name ex:/dev/i2c-0
	char data[BUFFER_SIZE];	// Data buffer
	int datasize;
	int address;					// i2c slave address
	int mode;							// program mode functionnality
	char *mode_str;				// program mode functionnality human readable
	int proto;						// protocol used
	char *proto_str;			// protocol mode functionnality human readable
	uint8_t spi_mode;
	uint8_t spi_bits;			// spi bits per word
	uint32_t spi_speed ;	// spi frequency max
	uint16_t spi_delay;		// spi delay
	int verbose;					// verbose mode, speak more to user
	int hexout;

} opts = {
	.port = "",
	.data = "",
	.datasize =0,
	.address = I2C_ADDRESS,
	.mode = MODE_QUICK_ACK,
	.mode_str = "ack",
	.proto = PROTO_I2C,
	.proto_str = "i2c",
	.spi_mode = SPI_MODE,
	.spi_bits = SPI_BITS_WORD,
	.spi_speed = SPI_SPEED,
	.spi_delay = SPI_DELAY,
	.verbose = false,
	.hexout = false
};


// ======================================================================
// Global vars 
// ======================================================================
int 	g_fd_device; 	// handle
int		g_exit_pgm;		// indicate end of the program
int		g_pi_rev;			// Rasberry Pi Board Revision

/* ======================================================================
Function: log_syslog
Purpose : write event to syslog
Input 	: stream to write if needed
					string to write in printf format
					printf other arguments
Output	: -
Comments: 
====================================================================== */
void log_syslog( FILE * stream, const char *format, ...)
{
	static char tmpbuff[512]="";
	va_list args;

	// do a style printf style in ou buffer
	va_start (args, format);
	vsnprintf (tmpbuff, sizeof(tmpbuff), format, args);
	tmpbuff[sizeof(tmpbuff) - 1] = '\0';
	va_end (args);

	// Write to logfile
	openlog( PRG_NAME, LOG_PID|LOG_CONS, LOG_USER);
 	syslog(LOG_INFO, tmpbuff);
 	closelog();
 	
 	// stream passed ? write also to it
 	if ((stream && opts.verbose) || stream == stdout ) 
 	{
 		fprintf(stream, tmpbuff);
 		fflush(stream);
 	}
}


/* ======================================================================
Function: clean_exit
Purpose : exit program 
Input 	: exit code
Output	: -
Comments: 
====================================================================== */
void clean_exit (int exit_code)
{
	// close i2c if opened 
  if (g_fd_device)
  {
		// then close
  	close(g_fd_device);
  }

	if ( exit_code != EXIT_SUCCESS)
		log_syslog(stdout, "Closing %s due to error\n", PRG_NAME);
	
	exit(exit_code);
}

/* ======================================================================
Function: fatal
Purpose : exit program due to a fatal error
Input 	: string to write in printf format
					printf other arguments
Output	: -
Comments: 
====================================================================== */
void fatal (const char *format, ...)
{
	char tmpbuff[512] = "";
	va_list args;

	va_start(args, format);
	vsnprintf(tmpbuff, sizeof(tmpbuff), format, args);
	tmpbuff[sizeof(tmpbuff) - 1] = '\0';
	va_end(args);

	// Write to logfile
	openlog( PRG_NAME, LOG_PID | LOG_CONS, LOG_USER);
 	syslog(LOG_INFO, tmpbuff);
 	closelog();

	fprintf(stderr,"\r\nFATAL: %s \r\n", tmpbuff );
	fflush(stderr);

	clean_exit(EXIT_FAILURE);
}


/* ======================================================================
Function: isr_handler
Purpose : Interrupt routine Code for signal
Input 	: -
Output	: -
Comments: 
====================================================================== */
void isr_handler (int signum)
{
	// Does we received CTRL-C ?
	if ( signum==SIGINT || signum==SIGTERM)
	{
		// Indicate we want to quit
		g_exit_pgm = true;
		
		log_syslog(NULL, "Received SIGINT/SIGTERM");
	}
	// Our receive buffer is full
	else if (signum == SIGIO)
	{
		log_syslog(NULL, "Received SIGIO");
	
	}
	
}

/* ======================================================================
Function: i2c_init
Purpose : initialize i2c port for communication
Input 	: -
Output	: i2c Port Handle
Comments: -
====================================================================== */
int i2c_init(void)
{
	int fd ;

  // Open i2c bus
  if ( (fd = open(opts.port, O_RDWR)) < 0 ) 
  	fatal( "i2c_init device %s: %s", opts.port, strerror(errno));

	// set slave address
	if ( ioctl(fd, I2C_SLAVE, opts.address) < 0)
		fatal( "i2c_init error setting slave address 0x%02X : %s", opts.address, strerror(errno));

 	return fd ;
}

/* ======================================================================
Function: spi_init
Purpose : initialize spi port for communication
Input 	: -
Output	: spi Port Handle
Comments: -
====================================================================== */
int spi_init(void)
{
	int fd ; 
	int ret;
	uint8_t mode;			// spi mode
	uint8_t bits;			// spi bits per word

  // Open spi bus
  if ( (fd = open(opts.port, O_RDWR)) < 0 ) 
  	fatal( "spi_init %s: %s", opts.port, strerror(errno));

	// set spi mode
	ret = ioctl(fd, SPI_IOC_WR_MODE, &opts.spi_mode);
	if (ret == -1)
		fatal( "spi_init %s : error writing mode %02X : %s",  opts.port, opts.spi_mode, strerror(errno));

	// read the value we set and check it is the same
	ret = ioctl(fd, SPI_IOC_RD_MODE, &mode);
	if (ret == -1 || mode !=opts.spi_mode )
		fatal( "spi_init %s : error checking mode %02X, found %02X : %s",  opts.port, opts.spi_mode, mode, strerror(errno));

	// set spi bits per word
	ret = ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &opts.spi_bits);
	if (ret == -1)
		fatal( "spi_init %s : error setting write bits per word %d : %s",  opts.port, opts.spi_bits, strerror(errno));

	// read the value we set and check it is the same
	ret = ioctl(fd, SPI_IOC_RD_BITS_PER_WORD, &bits);
	if (ret == -1 || bits != opts.spi_bits )
		fatal( "spi_init %s : error checking bits per word %d, found %d : %s",  opts.port, opts.spi_bits, bits, strerror(errno));

	// set spi max speed hz
	ret = ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &opts.spi_speed);
	if (ret == -1)
		fatal( "spi_init %s : error setting write max speed %d Hz: %s",  opts.port, opts.spi_speed, strerror(errno));

	// read the value we set and check it is the same
	ret = ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &opts.spi_speed);
	if (ret == -1 )
		fatal( "spi_init %s : error setting read max speed %d KHz, %s",  opts.port, opts.spi_speed, strerror(errno));

 	return fd ;
}

/* ======================================================================
Function: charToHexDigit
Purpose : convert char to hex value
Input 	: char
Output	: byte value
Comments: 
====================================================================== */
unsigned char charToHexDigit(char c) 
{
	unsigned char ret ;
	
	// parse to lower case
	if (c >= 'A' && c <= 'F')
		c= c - 'A' + 'a';
 
  if (c >= 'a') 
    ret = c - 'a' + 10; 
  else 
    ret = c - '0'; 

	return (ret);
} 

/* ======================================================================
Function: get_pi_version
Purpose : Get Raspberry Pi Revision Board Version
Input 	: -
Output	: Raspberry Pi revision (1 or 2)
Comments: 
====================================================================== */
// Get raspberry PI model version
int get_pi_version( void ) 
{ 
	int rev = 0;
	char buff[512];
	char * p;
	char * pend;
	
	FILE * fd ;

	// do some clean up
	memset(buff,0,sizeof(buff));

	fd = fopen("/proc/cpuinfo","r");

	// Opened successfully
	if( fd )
	{
		//printf("File opened successfully through fopen()\n");
		
		// parse each line until we the end or we find the good one
		while( fgets(buff, sizeof(buff), fd) != NULL && rev ==0 ) 
		{
			// search 
			if( (strstr(buff, "Revision" )) != NULL )  
			{
				// point to the separator ":" format is has follow
				// Revision        : 000f
				if ( (p = strtok( buff, ":")) != NULL )
				{
					// Ok get value
					if ( (p = strtok( NULL, ":")) != NULL )
					{
						// Revision Version is in hex format so put 0x before the number
						*p   = 'x';
						*--p = '0';
						
						// convert to number
						rev = strtol(p, &pend, 16);
						
						//printf("rev=%d 0x%04x\n", rev, rev);
						
						// not Okay ?
						if ( !*pend )
						{
							rev= 0;
						}
						else
						{
							// Revision 1 or 2 ?
							rev = (rev < 4 ) ? 1 : 2 ;
						}
					}
				}
			}
		}

		// Close the file.
		if(fd) 
		{
			fclose(fd);
		}
	}

	return rev;
} 



/* ======================================================================
Function: usage
Purpose : display usage
Input 	: program name
Output	: -
Comments: 
====================================================================== */
void usage( char * name)
{

	printf("%s\n", PRG_NAME );
	printf("Usage is: %s [options] [protocol] [mode] [-d device] [-a address] [-t data]\n", PRG_NAME);
	printf("  --<D>evice   : device name, i2c or spi\n");
	printf("  --<a>ddress  : i2c device address (default 0x2A)\n");
	printf("  --<d>data    : data to send\n");
	printf("protocol is:\n");
	printf("  --<I>2c      : set protocol to i2c (default)\n");
	printf("  --<S>pi      : set protocol to spi\n");
	printf("mode is:\n");
	printf("  --<s>et value: set value (byte or word type determined by data size)\n");
	printf("  --<g>etbyte  : get byte value\n");
	printf("  --<G>getword : get word value\n");
	printf("  --<q>uick    : i2c quick check device\n");
	printf("  --ac<k>      : i2c check if device sent ack\n");
	printf("Options are:\n");
	printf("  --ma<x>speed : max spi speed (in KHz)\n");
	printf("  --dela<y>    : spi delay (usec)\n");
	printf("  --<b>its     : spi bits per word\n");
	printf("  --<l>oop     : spi loopback\n");
	printf("  --cp<H>a     : spi clock phase\n");
	printf("  --cp<O>l     : spi clock polarity\n");
	printf("  --<L>sb      : spi least significant bit first\n");
	printf("  --<C>s-high  : spi chip select active high\n");
	printf("  --<3>wire    : spi SI/SO signals shared\n");
	printf("  --<N>o-cs    : spi no chip select\n");
	printf("  --<R>eady    : spi Ready\n");
	printf("  --<v>erbose  : speak more to user\n");
	printf("  --he<X>      : show return values in hexadecimal format\n");
	printf("  --<V>ersion  : show program version and Raspberry Pi revision\n");
	printf("  --<h>elp\n");
	printf("<?> indicates the equivalent short option.\n");
	printf("    long options always in lowercase\n");
	printf("Short options are prefixed by \"-\" instead of by \"--\".\n");
	printf("Example :\n");
	printf( "%s --i2c --getbyte --hex --data 0xe0\nSend a ping command and return ping value in hex format\n", PRG_NAME);
//	printf( "%s -m r -v\nstart %s to wait for a value, then display it and exit\n", PRG_NAME, PRG_NAME);
}


/* ======================================================================
Function: parse_args
Purpose : parse argument passed to the program
Input 	: -
Output	: -
Comments: 
====================================================================== */
void parse_args(int argc, char *argv[])
{

	static struct option longOptions[] =
	{
		{"device" 	,required_argument, 0, 'D' },
		{"data"   	,required_argument, 0, 'd' },
		{"verbose"	,no_argument			,	0, 'v' },
		{"version"	,no_argument			,	0, 'V' },
		{"address"	,required_argument, 0, 'a' },
		{"i2c"    	,no_argument			,	0, 'I' },
		{"spi"    	,no_argument			,	0, 'S' },
		{"set"    	,no_argument			,	0, 's' },
		{"getbyte"	,no_argument			,	0, 'g' },
		{"getword"	,no_argument			, 0, 'G' },
		{"quick"  	,no_argument			,	0, 'q' },
		{"ack"    	,no_argument			, 0, 'k' },
		{"help"			,no_argument			,	0, 'h' },
		{"maxspeed"	,required_argument, 0, 'x' },
		{"delay"  	,required_argument, 0, 'y' },
		{"bits"   	,required_argument, 0, 'b' },
		{"loop"		  ,no_argument			, 0, 'l' },
		{"cpha"		  ,no_argument			, 0, 'H' },
		{"cpol"		  ,no_argument			, 0, 'O' },
		{"lsb"			,no_argument			, 0, 'L' },
		{"cs-high"	,no_argument			, 0, 'C' },
		{"3wire"		,no_argument			, 0, '3' },
		{"no-cs"		,no_argument			, 0, 'N' },
		{"ready"		,no_argument			, 0, 'R' },
		{"hex"			,no_argument			, 0, 'X' },
		
		{0, 0, 0, 0}
	};

	int optionIndex = 0;
	int c;
	char * pEnd;
	
	// By default set the correct I2C device function of Pi version

	while (1) 
	{
		/* no default error messages printed. */
		opterr = 0;

		c = getopt_long(argc, argv, "D:d:vVa:x:y:b:ISsgGqkhlHOLC3NRX", longOptions, &optionIndex);

		if (c < 0)
			break;

		switch (c) 
		{
			case 'v':	opts.verbose = true;	break;

			
			case 's': opts.mode = MODE_SET				; 	opts.mode_str = "set"				; break;
			case 'k': opts.mode = MODE_READ_ACK	; 	opts.mode_str = "read ack"	; break;
			case 'q': opts.mode = MODE_QUICK_ACK	; 	opts.mode_str = "quick ack"	; break;
			case 'g': opts.mode = MODE_GET				; 	opts.mode_str = "get byte"	; break;
			case 'G': opts.mode = MODE_GET_WORD	; 	opts.mode_str = "get word"	; break;
			case 'I': opts.proto= PROTO_I2C    	; 	opts.proto_str= "i2c"     	; break;
			case 'l': opts.spi_mode |= SPI_LOOP			; break;
			case 'H': opts.spi_mode |= SPI_CPHA			; break;
			case 'O': opts.spi_mode |= SPI_CPOL			; break;
			case 'L': opts.spi_mode |= SPI_LSB_FIRST	;	break;
			case 'C': opts.spi_mode |= SPI_CS_HIGH		;	break;
			case '3': opts.spi_mode |= SPI_3WIRE			;	break;
			case 'N': opts.spi_mode |= SPI_NO_CS			;	break;
			case 'R': opts.spi_mode |= SPI_READY			;	break;
			case 'X': opts.hexout = true		;	break;
			
			
			// SPI init default bus
			case 'S': 
				opts.proto= PROTO_SPI ; 
				opts.proto_str= "spi";  

				// if bus still i2c, default to spi0
				// /dev/i2c-x -> i is th 5th char
				if ( opts.port[5] == 'i' )
					strcpy(opts.port, SPI_DEVICE_0);
					
			break;

			
			// i2c slave address
			case 'a':
				opts.address = strtol(optarg,&pEnd,0);
				
				if ( !pEnd || opts.address < 1 || opts.address > 0x7F )
				{
						fprintf(stderr, "--address %d (0x%02x) ignored.\n", opts.address, opts.address);
						fprintf(stderr, "--address must be between 1 and 255 or 0x01 and 0xff\n");
						opts.address = I2C_SLAVE;
						fprintf(stderr, "--setting slave to default 0x%02x\n", opts.address);
				}
			break;

			// spi max speed
			case 'x':
				opts.spi_speed = strtol(optarg,&pEnd,0) ;
				
				if ( !pEnd || opts.spi_speed < 1 || opts.spi_speed > 10000 )
				{
						fprintf(stderr, "--maxspeed %d Khz ignored.\n", opts.spi_speed);
						fprintf(stderr, "--maxspeed must be between 1 and 10000 (KHz)\n");
						opts.spi_speed = SPI_SPEED;
						fprintf(stderr, "--setting max speed to default %d Khz\n", opts.spi_speed/1000);
				}
				else
				{
					opts.spi_speed *= 1000;
				}
			break;

			// spi delay 
			case 'y':
				opts.spi_delay = strtol(optarg,&pEnd,0) ;
				
				if ( !pEnd || opts.spi_delay < 0 )
				{
						fprintf(stderr, "--delay %dus ignored.\n", opts.spi_delay);
						opts.spi_delay = SPI_DELAY;
						fprintf(stderr, "--setting delay to default %d us\n", opts.spi_delay);
				}
			break;

			// spi bits per word 
			case 'b':
				opts.spi_bits = strtol(optarg,&pEnd,0) ;
				
				if ( !pEnd || opts.spi_bits < 0 || opts.spi_bits > 64 )
				{
						fprintf(stderr, "--bits %d ignored.\n", opts.spi_bits);
						opts.spi_delay = SPI_BITS_WORD;
						fprintf(stderr, "--setting bits per word to default %d us\n", opts.spi_bits);
				}
			break;

			// Device name
			case 'D':
				strncpy(opts.port, optarg, sizeof(opts.port) - 1);
				opts.port[sizeof(opts.port) - 1] = '\0';
			break;

			// Data
			case 'd':
			{
				int ishex = false;
				char * p = opts.data;

			
				// Check correct size
				if ( strlen (optarg) >= 2)
				{
					// is it a hex string 
					if (optarg[0] == '0' && optarg[1] == 'x' )
						ishex = true;
				}
				
				if (ishex)
				{
					// put hex value into buffer data
					for (c = 2 ; optarg[c] != '\0' ; c += 2, p++)    
					{
						// even number of hex char
						if ( !optarg[c+1] )
						{
							*p = charToHexDigit(optarg[c]) ; 
							c+=2;
							break;
						}
						else
						{
							*p = charToHexDigit(optarg[c]) * 16 + charToHexDigit(optarg[c+1]); 
						}
					}
					
					opts.datasize = (c/2)- 1;
				}
				else
				{
					// copy the string
					opts.datasize = strlen(optarg);
					strncpy (opts.data, optarg, opts.datasize+1);
					strncat (opts.data, "\n", 1);
					opts.datasize++;
				}
			}
			break;

			// version
			case 'V':
			{
				printf("%s v%s\n", PRG_NAME, PRG_VERSION);
				printf("Raspberry Board Revision : %04x\n", g_pi_rev );
				exit(EXIT_SUCCESS);
			}
			break;

			// help
			case 'h':
				usage(argv[0]);
				exit(EXIT_SUCCESS);
			break;
			
			case '?':
			default:
				fprintf(stderr, "Unrecognized option.\n");
				fprintf(stderr, "Run with '--help'.\n");
				exit(EXIT_FAILURE);
			break;
		}
	} /* while */

	
	if (opts.verbose)
	{
		
		if ( opts.proto == PROTO_I2C )
		{
			printf("-- i2c Stuff -- \n");
			printf("i2c bus       : %s\n", opts.port);
			printf("slave address : 0x%02X\n", opts.address);
		}
		if ( opts.proto == PROTO_SPI )
		{
			printf("-- spi Stuff -- \n");
			printf("spi bus       : %s\n", opts.port);
			printf("spi mode      : %d\n", opts.spi_mode);
			printf("bits per word : %d\n", opts.spi_bits);
			printf("max speed     : %d Hz (%d KHz)\n", opts.spi_speed, opts.spi_speed/1000);
		}
				
		printf("mode          : %s\n", opts.mode_str);
		printf("protocol      : %s\n", opts.proto_str);
		printf("verbose       : %s\n", opts.verbose? "yes" : "no");
		printf("data (%02i)     : ", opts.datasize);
		
		if (opts.datasize >0 )
		{
			for (c = 0; c <opts.datasize ; c++)    
				printf("0x%02X ", opts.data[c]);
							
			printf("\n");
		}
		else
		{
			printf("NULL\n");
		}

	}	
}

/* ======================================================================
Function: do_i2c
Purpose : do i2c stuff
Input 	: -
Output	: -
Comments: documentation on i2c smbus API can be found at
					http://www.mjmwired.net/kernel/Documentation/i2c/smbus-protocol
					http://www.mjmwired.net/kernel/Documentation/i2c/dev-interface
====================================================================== */
void do_i2c(void)
{
  int r=0;

	g_fd_device = i2c_init();
		
	if (opts.verbose)
		 	log_syslog(stdout, "i2c Init succeded\n");

	// Mode : Check device
	if ( opts.mode == MODE_QUICK_ACK || opts.mode == MODE_READ_ACK )
	{
		if (opts.mode == MODE_QUICK_ACK)	
			r = i2c_smbus_write_quick(g_fd_device, I2C_SMBUS_WRITE);
		else
			r = i2c_smbus_read_byte(g_fd_device);
		
		// If not found 
		if ( r < 0  )
		{
			log_syslog(stdout, "i2c device 0x%02x was not found\n", opts.address);
			clean_exit( EXIT_SUCCESS );
		}
		else
		{
			log_syslog(stdout, "i2c device 0x%02x is detected\n", opts.address);
			clean_exit( EXIT_SUCCESS );
		}
			
		// sure, we have finished our job
		g_exit_pgm = true;
	}
	else
	{
		// Get Byte command
		if (opts.mode == MODE_GET )
		{
			// Set the command we wand to read
			r = i2c_smbus_write_byte(g_fd_device, opts.data[0]);

			// If OK
			if ( r >= 0 )
			{
				// Read the return value
				r = i2c_smbus_read_byte(g_fd_device); 
			}
			else
			{
				log_syslog(stdout, "Error from device 0x%02x on i2c_smbus_write_byte : %d %s\n", opts.address, r, strerror(errno));
				clean_exit( EXIT_FAILURE );
			}
			
		}
		// Get word command
		else if (opts.mode == MODE_GET_WORD )
		{
				r = i2c_smbus_read_word_data(g_fd_device, opts.data[0]);
				//log_syslog(stderr, "Get Word 0x%02x : %d\n", opts.address, r);
		}
		else if (opts.mode == MODE_SET )
		{
			// Set Byte command
			if (opts.datasize==1)
			{
				r = i2c_smbus_write_byte(g_fd_device, opts.data[0]);
			}
			// Set Word Byte command
			else if ( opts.datasize==2)
			{
				r = i2c_smbus_write_byte_data(g_fd_device, opts.data[0], opts.data[1]);
			}
		}

		// had a error ?
		if (r<0)
		{
			log_syslog(stdout, "Error from device 0x%02x : %d %s\n", opts.address, r, strerror(errno));
			clean_exit( EXIT_FAILURE );
		}
		else
		{
			//log_syslog(stdout, "Successfully done action on device 0x%02x\n", opts.address);
			if (opts.hexout)
				log_syslog(stdout, opts.mode == MODE_GET_WORD ? "0x%04X\n":"0x%02X\n", r);
			else
				log_syslog(stdout, "%d\n", r);

			clean_exit( EXIT_SUCCESS );
		}
	}
}


/* ======================================================================
Function: spi_write
Purpose : send spi datado spi stuff
Input 	: spi Port Handle
					pointer to send buffer (buffer will be erased by device response
					size of buffer
Output	: -1 if error
Comments: 
====================================================================== */
int spi_transfer(int fd, unsigned char * buf, int n)
{
	struct spi_ioc_transfer tr ;
	
	tr.rx_buf = (unsigned long) buf,
	tr.tx_buf = (unsigned long) buf;	
	tr.len = n;
	tr.speed_hz 		= opts.spi_speed;
	tr.delay_usecs 	= opts.spi_delay;
	tr.bits_per_word= opts.spi_bits;
		
	return (ioctl(fd, SPI_IOC_MESSAGE(1), &tr) );
}

/* ======================================================================
Function: do_spi
Purpose : do spi stuff
Input 	: -
Output	: -
Comments: 
====================================================================== */
void do_spi(void)
{
  int r=0;

	g_fd_device = spi_init();
		
	if (opts.verbose)
		 	log_syslog(stdout, "spi Init succeded\n");

	// Mode : Check device, 
	// arbitray, just check device response there is no such mode in SPI
	if ( opts.mode == MODE_QUICK_ACK || opts.mode == MODE_READ_ACK )
	{
		// send a ping command
		opts.data[0] = ARDUIPI_CMD_PING ;
		
		// transfert one byte with the ping command
		// test firmware always response 2a 
		spi_transfer( g_fd_device, (unsigned char *) opts.data, 1);
	
		// If error 
		if ( r < 0 )
		{
			log_syslog(stdout, "Error from spi device %s : %s\n", opts.port, strerror(errno));
			clean_exit( EXIT_FAILURE );
		}
		else
		{
			// get real response
			r = (int) opts.data[0];
			
			// not expected response ?
			//if ( r != ARDUIPI_CMD_PING)
			//	log_syslog(stdout, "Warning, expected %02X response and received %02X from spi device %s\n", ARDUIPI_CMD_PING, r, opts.port);
			//else
				log_syslog(stdout, opts.hexout?"0x%02X\n":"%d\n", r);
		}
		
		// sure, we have finished our job
		g_exit_pgm = true;
	}
	else
	{
		// Get Byte command
		if (opts.mode == MODE_GET )
		{
			// Set the command we wand to read send 2 
			// Dummy 2nd byte this is where we will have our response
			opts.data[1] = 0xff ;
			r = spi_transfer( g_fd_device, (unsigned char *) opts.data, 2);

			// If OK Read the return value
			if ( r >= 0 )
				r = opts.data[1]; 
			
		}
		// Get word command
		else if (opts.mode == MODE_GET_WORD )
		{
				// Dummy 2nd and 3rd bytes this is where we will have our word response
				opts.data[1] = 0xff ;
				opts.data[2] = 0xff ;
				r = spi_transfer( g_fd_device, (unsigned char *) opts.data, 3);

				// If OK // Read the return value
				if ( r >= 0 )
					r = opts.data[1] | (opts.data[2] << 8 );
		}
		else if (opts.mode == MODE_SET )
		{
			// send bulk data data
			r = spi_transfer( g_fd_device, (unsigned char *) opts.data, opts.datasize);
		}

		// had a error ?
		if ( r < 0 )
		{
			log_syslog(stdout, "Error from spi_transfer on device %s : %d %s\n", opts.port,  r, strerror(errno));
			clean_exit( EXIT_FAILURE );
		}
		else
		{
			if (opts.hexout)
				log_syslog(stdout, opts.mode == MODE_GET_WORD ? "0x%04X\n":"0x%02X\n", r);
			else
				log_syslog(stdout, "%d\n", r);
			
			clean_exit( EXIT_SUCCESS );
		}
	}
}



/* ======================================================================
Function: main
Purpose : Main entry Point
Input 	: -
Output	: -
Comments: 
====================================================================== */
int main(int argc, char **argv)
{
	struct sigaction exit_action;
  int n;
	char	buff[BUFFER_SIZE];

	g_fd_device = 0;
	g_exit_pgm = false;

	// clean up our buffer size
	bzero(buff, BUFFER_SIZE);

	// Get Raspberry Board Revision
	g_pi_rev = get_pi_version() ;
		
	// No Raspberry Board ?
	if ( g_pi_rev == 0 ) 
	{
		printf("Error Unable to find Raspberry Board Revision\n");
		clean_exit(EXIT_FAILURE);
	}
	
	// Set i2c device default bus depending on PI version
	strcpy(opts.port, g_pi_rev >= 2 ? I2C_DEVICE_1 : I2C_DEVICE_0 );
	
	// get command line args
	parse_args(argc, argv);

	// Set up the structure to specify the exit action.
	exit_action.sa_handler = isr_handler;
	sigemptyset (&exit_action.sa_mask);
	exit_action.sa_flags = 0;
	sigaction (SIGTERM, &exit_action, NULL);
	sigaction (SIGINT,  &exit_action, NULL); 

	// do i2c job
	if ( opts.proto == PROTO_I2C )
		do_i2c();

	// do spi job
	if ( opts.proto == PROTO_SPI )
		do_spi();
		
	// Do while not end 
	// this is for demo purpose, you can do whatever you like in this loop
	while ( ! g_exit_pgm ) 
	{
		// example Read adc value from arduino
		n = i2c_smbus_read_byte_data(g_fd_device, 0x01);

		// We want to display results on stdout even if we are deamon ?
		if (opts.verbose)
		{
			log_syslog(stdout, "Received 0x%02x from slave\n", n);
		}

		// Do it again in 5 seconds
		sleep(5);

	}

  log_syslog(stderr, "Program terminated\n");

  clean_exit(EXIT_SUCCESS);

  // avoid compiler warning
  return (0);
}

