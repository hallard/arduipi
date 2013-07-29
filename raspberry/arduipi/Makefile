#*********************************************************************
# This is the makefile for the ArduiPi program
#
#	02/18/2013 	Charles-Henri Hallard (http://hallard.me)
#							Modified for compiling and use on Raspberry ArduiPi Board
#							ArduiPi project documentation http://hallard.me/arduipi
# 
# *********************************************************************

# Where you want it installed when you do 'make install'
PREFIX=/usr/local

# The recommended compiler flags for the Raspberry Pi
CCFLAGS=-Ofast -mfpu=vfp -mfloat-abi=hard -march=armv6zk -mtune=arm1176jzf-s

# Program to compile
PROGRAM=arduipi

# make all
all: ${PROGRAM} 

${PROGRAM}: ${SOURCE}
	gcc ${CCFLAGS} -Wall $@.c -o $@

clean:
	rm -rf $(PROGRAM)

# Install the executable
install: 
	@if ( test ! -d $(PREFIX) ) ; then mkdir -p $(PREFIX) ; fi
	@if ( test ! -d $(PREFIX)/bin ) ; then mkdir -p $(PREFIX)/bin ; fi
	@echo "[Install $(PROGRAM)]"; 
	@install -m 0755 $(PROGRAM) $(PREFIX)/bin
	
# Uninstall the executable
uninstall: 
	@echo "[Uninstall $(PROGRAM)]"; 
	@rm -rf $(PREFIX)/bin/$(PROGRAM) ; 
	
.PHONY: install






