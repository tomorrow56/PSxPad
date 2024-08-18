/*  PSX Controller Decoder Library (PsxPad.h)
	Based on PSX Library
		http://playground.arduino.cc/Main/PSXLibrary)
		Written by: Kevin Ahrendt June 22nd, 2008

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

/****************
#define psxLeft		0x0001 
#define psxDown		0x0002
#define psxRight	0x0004
#define psxUp		  0x0008
#define psxStrt		0x0010
#define psxSlct		0x0080

#define psxSqu		0x0100
#define psxX		  0x0200
#define psxO		  0x0400
#define psxTri		0x0800
#define psxR1		  0x1000
#define psxL1		  0x2000
#define psxR2		  0x4000
#define psxL2		  0x8000
***************/

// Includes the Psx Library 
#include <PsxPad.h>

/***** Pin info *****
 M5AtomS3
 3v3
 G5     G39
 G6     G38
 G7     5V
 G8     GND

 M5Atom
 3v3
 G22    G21
 G19    G25
 G23    5V
 G33    GND
*********************/

// For M5AtomS3
#define dataPin  5  // brown, pull-up
#define cmndPin  6  // orange
#define attPin   7  // yellow
#define clockPin 8  // blue

//#define LEDPin 13

// Initializes the library
PsxPad PsxPad(dataPin, cmndPin, attPin, clockPin, 10);

// data stores the controller response
uint16_t data = 0;

void setup()
{
  // Defines what each pin is used
  // (Data Pin #, Cmnd Pin #, Att Pin #, Clk Pin #, Delay)
  // Delay measures how long the clock remains at each state, measured in microseconds.
  // too small delay may not work (under 5)
  PsxPad.begin();

  // Establishes LEDPin as an output so the LED can be seen
  //pinMode(LEDPin, OUTPUT);
  Serial.begin(115200);
}

void loop()
{
  // Psx.read() initiates the PSX controller and returns the button data
  data = PsxPad.read();
  Serial.println(data);

  if (data & psxR2)
  {
    //digitalWrite(LEDPin, HIGH);
  }
  else
  {
    //digitalWrite(LEDPin, LOW);
  }
  delay(20);
}
