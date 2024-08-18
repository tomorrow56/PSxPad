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


#ifndef PsxPad_h
#define PsxPad_h

#include "Arduino.h"

// Button Hex Representations:
#define psxLeft		0x0001 
#define psxDown		0x0002
#define psxRight	0x0004
#define psxUp		0x0008
#define psxStrt		0x0010
#define psxSlct		0x0080

#define psxSqu		0x0100
#define psxX		0x0200
#define psxO		0x0400
#define psxTri		0x0800
#define psxR1		0x1000
#define psxL1		0x2000
#define psxR2		0x4000
#define psxL2		0x8000


class PsxPad
{
  public:
	PsxPad();
	PsxPad(byte dataPin, byte cmndPin, byte attPin, byte clockPin, byte delay);
	void begin();
	uint16_t read();

  private:
	byte shift(byte _dataOut);

	byte _dataPin;
	byte _cmndPin;
	byte _attPin;
	byte _clockPin;

	byte _delay;
	byte _i;
	boolean _temp;
	byte _dataIn;

	byte _data1;
	byte _data2;
	unsigned int _dataOut;
};

#endif
