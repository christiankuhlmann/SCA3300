/******************************************************************************
SCA3300.h
SCA3300 Arduino Library Header File
Christian Kuhlmann
Version 1.0 - May 05 2023
https://github.com/christiankuhlmann/SCA3300

This library is based off SCL3300 by David Armstrong
https://github.com/DavidArmstrong/SCA3300

Changes from SCL3300:
- Fixes to SCL3300::available()
    # Incorrect data assignment to data object
- Altered acceleration calculations
    # SCA3300 has different ratio for calculating acceleration
- Moved from doubles to floats 
    # 32bit architecture and 32 bit data, doubles ineffcicient
- Changed WHOAMI
    # SCL3300 is 0xc1, SCA3300 is 0x51s


Resources:
Uses SPI.h for SPI operation

This code is released under the [MIT License](http://opensource.org/licenses/MIT).
Please review the LICENSE.md file included with this example.
Distributed as-is; no warranty is given.

This file prototypes the SCA3300 class, as implemented in SCA3300.cpp

******************************************************************************/

// include this library's description file
#include "SCA3300.h"

// Public Methods //////////////////////////////////////////////////////////
// Set the sensor mode to the number provided as modeNum.
boolean SCA3300::setMode(int modeNum) {
  // Set Sensor mode - If not called, the default is mode 4, as set in header file
  // Only allowed values are: 1,2,3,4
  if (modeNum > 0 && modeNum < 5) {
    SCA3300_mode = modeNum;
    if (!setFastRead) beginTransmission(); //Set up this SPI port/bus
    transfer(modeCMD[SCA3300_mode]); //Set mode on hardware
    if (!setFastRead) endTransmission(); //Let go of SPI port/bus
    if (crcerr || statuserr) {
	  reset(); //Reset chip to fix the error state
      return false;  //Let the caller know something went wrong
    } else return true; // Valid value, and chip was set to that mode
  } else
    return false; // Invalid value
}

// Current Version of begin() to initialize the library and the SCA3300
boolean SCA3300::begin(void) {
  //This is the updated Version 3 begin function
  // Determine if we need to set up to use the default SPI interface, or some other one
  if (_spiPort == nullptr) _spiPort = &SPI;
  
  //Wait the required 1 ms before initializing the SCA3300 inclinomenter
  unsigned long startmillis = millis();
  while (millis() - startmillis < 1) ;
  
  initSPI();	// Initialize SPI Library
  if (!setFastRead) beginTransmission(); //Set up this SPI port/bus
  //Write SW Reset command
  transfer(SwtchBnk0);
  transfer(SWreset);
  startmillis = millis();
  while (millis() - startmillis < 1) ;
  //Set measurement mode
  transfer(modeCMD[SCA3300_mode]); //Set mode on hardware
  //The first response after reset is undefined and shall be discarded
  //wait 5 ms to stablize
  startmillis = millis();
  while (millis() - startmillis < 100) ;

  //Read Status to clear the status summary
  transfer(RdStatSum);
  transfer(RdStatSum); //Again, due to off-response protocol used
  transfer(RdStatSum); //And now we can get the real status
  
  //Read the WHOAMI register
  transfer(RdWHOAMI);
  //And again
  transfer(RdWHOAMI);
  if (!setFastRead) endTransmission(); //Let go of SPI port/bus
  //We now wait until the end of begin() to report if an error occurred
  if (crcerr || statuserr) return false;
  // Once everything is initialized, return a known expected value
  // The WHOAMI command should give an 8 bit value of 0x51
  return (SCA3300_DATA == 0x51); //Let the caller know if this worked
}

// Set up the SPI communication with the SCA3300 with provided Chip Select pin number, and provided SPI port
boolean SCA3300::begin(SPIClass &spiPort, uint8_t csPin) {
  SCA3300_csPin = csPin;
  _spiPort = &spiPort; //Grab the port the user wants us to use
  return begin();
} // begin

// Set up the SPI communication with the SCA3300 with provided Chip Select pin number
boolean SCA3300::begin(uint8_t csPin) {
  SCA3300_csPin = csPin;
  _spiPort = &SPI; // With this call, we do the default SPI interface
  return begin();
} // begin

//Check to validate that the sensor is still reachable and ready to provide data
boolean SCA3300::isConnected() {
  if (!setFastRead) beginTransmission(); //Set up this SPI port/bus
  transfer(SwtchBnk0);
  //Read the WHOAMI register
  transfer(RdWHOAMI);
  //And again
  transfer(RdWHOAMI);
  if (!setFastRead) endTransmission(); //Let go of SPI port/bus
  if (crcerr || statuserr) return false;
  // Once everything is initialized, return a known expected value
  // The WHOAMI command should give an 8 bit value of 0x51
  return (SCA3300_DATA == 0x51); //Let the caller know if this worked
}

//Read all the sensor data together to keep it consistent
//This is required according to the datasheet
boolean SCA3300::available(void) {
  //Version 3 of this function
  boolean errorflag = false;
  //Read all Sensor Data, as per Datasheet requirements
  if (!setFastRead) beginTransmission(); //Set up this SPI port/bus
  transfer(SwtchBnk0);

  transfer(RdAccX);
  if (crcerr || statuserr) errorflag = true;

  transfer(RdAccY);
  if (crcerr || statuserr) errorflag = true;
  SCAData.AccX = SCA3300_DATA;

  transfer(RdAccZ);
  if (crcerr || statuserr) errorflag = true;
  SCAData.AccZ = SCA3300_DATA;

  transfer(RdTemp);
  if (crcerr || statuserr) errorflag = true;
  SCAData.TEMP = SCA3300_DATA;

  transfer(RdStatSum);
  if (crcerr || statuserr) errorflag = true;
  SCAData.StatusSum = SCA3300_DATA;

  transfer(RdSTO);
  if (crcerr || statuserr) errorflag = true;
  SCAData.STO = SCA3300_DATA;

  transfer(RdWHOAMI);
  if (crcerr || statuserr) errorflag = true;
  SCAData.WHOAMI = SCA3300_DATA;

  if (!setFastRead) endTransmission(); //Let go of SPI port/bus
  if (errorflag) return false; //Inform caller that something went wrong
  // The WHOAMI command should give an 8 bit value of 0x51
  return (SCA3300_DATA == 0x51); //Let the caller know this worked
}

/* Set SCA3300 library into Fast Read Mode
 * Warning: Using Fast Read Mode in the library works by keeping the
 *          SPI connection continuously open.  This may or may not affect
 *          the behavior of other hardware interactions, depending on the
 *          sketch design.  Fast Read Mode is considered an advanced use case,
 *          and not recommended for the beginner.
*/
void SCA3300::setFastReadMode() {
  setFastRead = true;
  beginTransmission(); //Set up this SPI port/bus
  begin(); //Re-init chip
}

/* Stop Fast Read Mode
 * Warning: Using Fast Read Mode in the library works by keeping the
 *          SPI connection continuously open.  This may or may not affect
 *          the behavior of other hardware interactions, depending on the
 *          sketch design.  Fast Read Mode is considered an advanced use case,
 *          and not recommended for the beginner.
*/
void SCA3300::stopFastReadMode() {
  setFastRead = false;
  endTransmission();  //Close connection to SPI port/bus
  begin(); //Re-init chip
}

//Return the calculated X axis accelerometer value in units of 'g'
float SCA3300::getCalculatedAccelerometerX(void) {
  return acceleration(SCAData.AccX);
}

//Return the calculated Y axis accelerometer value in units of 'g'
float SCA3300::getCalculatedAccelerometerY(void) {
  return acceleration(SCAData.AccY);
}

//Return the calculated Z axis accelerometer value in units of 'g'
float SCA3300::getCalculatedAccelerometerZ(void) {
  return acceleration(SCAData.AccZ);
}

// Read the sensor Serial Number as created by the manufacturer
unsigned long SCA3300::getSerialNumber(void) {
  //Return Device Serial number
  boolean errorflag = false;
  unsigned long serialNum = 0;
  if (!setFastRead) beginTransmission(); //Set up this SPI port/bus
  transfer(SwtchBnk1);
  if (crcerr || statuserr) errorflag = true;
  transfer(RdSer1);
  if (crcerr || statuserr) errorflag = true;
  transfer(RdSer2);
  serialNum = SCA3300_DATA;
  if (crcerr || statuserr) errorflag = true;
  transfer(SwtchBnk0);
  serialNum = ((unsigned long)SCA3300_DATA << 16) | serialNum;
  if (!setFastRead) endTransmission(); //Let go of SPI port/bus
  //We wait until now to return an error code
  //In this case we send a 0 since a real serial number will never be 0
  if (crcerr || statuserr || errorflag) return 0;
  return serialNum;
}

// Place the sensor in a Powered Down mode to save power
uint16_t SCA3300::powerDownMode(void) {
  //Software power down of sensor
  if (!setFastRead) beginTransmission(); //Set up this SPI port/bus
  transfer(SwtchBnk0);
  transfer(SetPwrDwn);
  endTransmission(); //Let go of SPI port/bus
  //Since an error is non-zero, we will return 0 if there was no error
  if (crcerr || statuserr) return (uint16_t)(SCA3300_CMD & 0xff); //check CRC and RS bits
  return 0;
}

// Revive the sensor from a power down mode so we can start getting data again
uint16_t SCA3300::WakeMeUp(void) {
  //Software Wake Up of sensor
  beginTransmission(); //Set up this SPI port/bus
  transfer(WakeUp);
  if (!setFastRead) endTransmission(); //Let go of SPI port/bus
  //Since an error is non-zero, we will return 0 if there was no error
  if (crcerr || statuserr) return (uint16_t)(SCA3300_CMD & 0xff); //check CRC and RS bits
  return 0;
}

// Hardware reset of the sensor electronics
uint16_t SCA3300::reset(void) {
  //Software reset of sensor
  //beginTransmission(); //Set up this SPI port/bus
  //transfer(SwtchBnk0);
  //transfer(SWreset);
  //endTransmission(); //Let go of SPI port/bus
  //we have to call begin() to set up the SCA3300 to the same state as before it was reset
  begin(); //Re-init chip
  //Since an error is non-zero, we will return 0 if there was no error
  if (crcerr || statuserr) return (uint16_t)(SCA3300_CMD & 0xff); //check CRC and RS bits
  return 0;
}

// Routine to get temperature in degrees Celsius
float SCA3300::getCalculatedTemperatureCelsius(void) {
  // Return calculated temperature in degrees C
  float Temperature = -273. + (SCAData.TEMP / 18.9);
  return Temperature;
}

// Routine to get temperature in degrees Farenheit
float SCA3300::getCalculatedTemperatureFarenheit(void) {
  // Return calculated temperature in degrees F
  float Temperature = -273. + (SCAData.TEMP / 18.9);
  Temperature = (Temperature * 9./5.) + 32.;
  return Temperature;
}
 
 //Convert raw accelerometer value to g's of acceleration
float SCA3300::acceleration(int16_t SCA3300_ACC) { //two's complement value expected
  // Return acceleration in g
  if (SCA3300_mode == 1) return (float)SCA3300_ACC / 2700.;
  if (SCA3300_mode == 2) return (float)SCA3300_ACC / 1350.;
  if (SCA3300_mode == 3) return (float)SCA3300_ACC / 5400.;
  if (SCA3300_mode == 4) return (float)SCA3300_ACC / 5400.;
  return (float)SCA3300_ACC / 5400.; //Default should never be reached
}

//private functions for serial transmission
// Begin SPI bus transmission to the device
void SCA3300::beginTransmission() {
  _spiPort->beginTransaction(spiSettings);
} //beginTransmission

// End SPI bus transmission to the device
void SCA3300::endTransmission() {
  // take the chip/slave select high to de-select:
  digitalWrite(SCA3300_csPin, HIGH);
  _spiPort->endTransaction();
  unsigned long startmillis = millis();
  while (millis() - startmillis < 1) ; //wait a bit
} //endTransmission

//Initialize the Arduino SPI library for the SCA3300 hardware
void SCA3300::initSPI() {
  //Initialize the Arduino SPI library for the SCA3300 hardware
  _spiPort->begin();
  // Maximum SPI frequency is 2 MHz - 4 MHz to achieve the best performance
  // initialize the chip select pin:
  pinMode(SCA3300_csPin, OUTPUT);
  digitalWrite(SCA3300_csPin, HIGH);
  // Data is read and written MSb first.
  // Data is captured on rising edge of clock (CPHA = 0)
  // Data is propagated on the falling edge (MISO line) of the SCK. (CPOL = 0)
}

// The following is taken directly from the Murata SCA3300 datasheet
// Calculate CRC for 24 MSB's of the 32 bit dword
// (8 LSB's are the CRC field and are not included in CRC calculation)
uint8_t SCA3300::CalculateCRC(uint32_t Data)
{
uint8_t BitIndex;
uint8_t BitValue;
uint8_t SCA3300_CRC;

SCA3300_CRC = 0xFF;
for (BitIndex = 31; BitIndex > 7; BitIndex--) {
  BitValue = (uint8_t)((Data >> BitIndex) & 0x01);
  SCA3300_CRC = CRC8(BitValue, SCA3300_CRC);
}
SCA3300_CRC = (uint8_t)~SCA3300_CRC;
return SCA3300_CRC;
}
uint8_t SCA3300::CRC8(uint8_t BitValue, uint8_t SCA3300_CRC)
{
  uint8_t Temp;
  Temp = (uint8_t)(SCA3300_CRC & 0x80);
  if (BitValue == 0x01) {
    Temp ^= 0x80;
  }
  SCA3300_CRC <<= 1;
  if (Temp > 0) {
    SCA3300_CRC ^= 0x1D;
  }
  return SCA3300_CRC;
}

// Routine to transfer a 32-bit integer to the SCA3300, and return the 32-bit data read
unsigned long SCA3300::transfer(unsigned long value) {
  FourByte dataorig;
  unsigned long startmicros;
  
  dataorig.bit32 = value; //Allow 32 bit value to be sent 8 bits at a time
  #ifdef debug_SCA3300
  Serial_SCA.print(dataorig.bit32, HEX);
  Serial_SCA.print(" ");
  for (int j = 3; j >= 0; j--) {
    Serial_SCA.print(dataorig.bit8[j], HEX);
    Serial_SCA.print(" ");
  }
  #endif
  //Must allow at least 10 uSec between SPI transfers
  //The datasheet shows the CS line must be high during this time
  if (!setFastRead) startmicros = micros();
  //while ((micros() - startmicros < 10) && (micros() > 10)) ;
  if (!setFastRead) while ((micros() - startmicros < 10)) ;
  
  digitalWrite(SCA3300_csPin, LOW); //Now chip select can be enabled for the full 32 bit xfer
  SCA3300_DATA = 0;
  for (int i = 3; i >= 0; i--) { //Xfers are done MSB first
    dataorig.bit8[i] = _spiPort->transfer(dataorig.bit8[i]);
  }
  SCA3300_DATA = dataorig.bit8[1] + (dataorig.bit8[2] << 8);
  SCA3300_CRC = dataorig.bit8[0];
  SCA3300_CMD = dataorig.bit8[3];
  digitalWrite(SCA3300_csPin, HIGH); //And we are done
  #ifdef debug_SCA3300
  for (int i = 3; i >= 0; i--) {
    Serial_SCA.print(" ");
    Serial_SCA.print(dataorig.bit8[i], HEX);
  }
  Serial_SCA.print("  ");
  #endif
  if (SCA3300_CRC == CalculateCRC(dataorig.bit32))
    crcerr = false;
  else
    crcerr = true;
  //check RS bits
  if ((SCA3300_CMD & 0x03) == 0x01)
    statuserr = false;
  else
    statuserr = true;
  #ifdef debug_SCA3300
  Serial_SCA.print((SCA3300_CMD & 0x03));
  Serial_SCA.print(" ");
  Serial_SCA.print(SCA3300_DATA, HEX);
  Serial_SCA.print(" ");
  Serial_SCA.print(SCA3300_CRC, HEX);
  Serial_SCA.print(" ");
  Serial_SCA.print(CalculateCRC(dataorig.bit32), HEX);
  Serial_SCA.print(" ");
  Serial_SCA.println(crcerr);
  #endif
  return dataorig.bit32;
}
