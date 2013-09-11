#!/bin/bash
# quick and dirty code, I know, will arrange this later


echo "Testing ArduiPi Serial communication"
echo "------------------------------------"
# configure serial port to Arduino compatible mode
stty -F /dev/ttyAMA0 cs8 -cstopb -parenb 9600 ignbrk -brkint -icrnl -imaxbel -opost -onlcr -isig -icanon -iexten -echo -echoe -echok -echoctl -echoke noflsh -ixon -crtscts 
echo -n "  Sending serial port command."
echo "FLUSH" > /dev/ttyAMA0
sleep 1

# send I_AM_PI string
echo "I_AM_PI" > /dev/ttyAMA0
echo -n "."
# get I_AM_PI:OK response string from Arduino
read RESPONSE < /dev/ttyAMA0 
echo -n "."
if `echo ${RESPONSE} | grep "I_AM_PI:OK" 1>/dev/null 2>&1`
then
	# response OK send back ACK
  echo "Arduino Serial OK"
	echo "ACK" > /dev/ttyAMA0
else
  echo "Error No Serial response from Arduino"
fi
echo


echo "Testing AruiPi I2C communication"
echo "-----------------------------------"

echo -n "  Checking Arduino presence."
# list I2C devices on bus and check Arduino is here, Arduino I2C address is 0x2a
if `i2cdetect -y 1 | grep 2a 1>/dev/null 2>&1`
then
  echo "Arduino I2C OK"
else
  echo "Error No I2C response from Arduino"
fi

#echo -n "  Checking Arduino Ping."
# send ping command to arduino, Arduino should respond 0x2a in ping response
#if `i2cget -y 1 0x2a 0xe0 | grep "2a" 1>/dev/null 2>&1`
#if `arduipi --i2c --getbyte --hex --data 0xe0 | grep 0x2A 1>/dev/null 2>&1`
#then
#  echo "Arduino I2C Ping OK"
#else
#  echo "Error in Ping response from Arduino"
#fi
echo


echo "Testing AruiPi SPI communication"
echo "-----------------------------------"

echo -n "  Checking Arduino Ping."
# send ping command to arduino, Arduino should respond 0x2a in ping response
#if `i2cget -y 1 0x2a 0xe0 | grep "2a" 1>/dev/null 2>&1`
if `arduipi --maxspeed 1000 --spi --ack --hex | grep 0x2A 1>/dev/null 2>&1`
then
  echo "Arduino SPI Ping OK"
else
  echo "Error in Ping response from Arduino"
fi
echo


function setandclear
{
	gpio -g mode $1 out
	gpio -g write  $1 1
}

function seton
{
	gpio -g write $1 0
}

	
function setio
{
	echo "--- Led on  GPIO$1"
	gpio -g write  $1 0
	sleep 1
	echo "--- Led off GPIO$1"
	gpio -g write  $1 1
	sleep 1
}

echo "--- All Led off"
for io in 7 4 17 18 27 22 23 24 25 28 29 30 31
do
   setandclear $io
done
sleep 2

echo "--- All Led On"
for io in 7 4 17 18 27 22 23 24 25 28 29 30 31
do
   seton $io
done
sleep 2

echo "--- All Led off"
for io in 7 4 17 18 27 22 23 24 25 28 29 30 31
do
   setandclear $io
done

echo "--- Cycling"
#21 for Pi Rev 1
for io in 7 4 17 18 27 22 23 24 25 28 29 30 31
do
   setio $io
done




