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

#include "PsxPad.h"

PsxPad::PsxPad(byte dataPin, byte cmndPin, byte attPin, byte clockPin, byte delay)
: _dataPin (dataPin), _cmndPin (cmndPin), _attPin (attPin), _clockPin (clockPin), _delay (delay)
{
}

void PsxPad::begin()
{
	pinMode(_dataPin, INPUT_PULLUP);
//	digitalWrite(_dataPin, HIGH);	// Turn on internal pull-up

	pinMode(_cmndPin, OUTPUT);

	pinMode(_attPin, OUTPUT);
	digitalWrite(_attPin, HIGH);

	pinMode(_clockPin, OUTPUT);
	digitalWrite(_clockPin, HIGH);
}


// Does the actual shifting, both in and out simultaneously
byte PsxPad::shift(byte _dataOut)
{
	_temp = 0;
	_dataIn = 0;

	for (_i = 0; _i <= 7; _i++){
		if ( _dataOut & (1 << _i) ){
			digitalWrite(_cmndPin, HIGH);	// Writes out the _dataOut bits
		}else{
			digitalWrite(_cmndPin, LOW);
		}
		digitalWrite(_clockPin, LOW);
		delayMicroseconds(_delay);
		_temp = digitalRead(_dataPin);		// Reads the data pin
		if (_temp){
			_dataIn = _dataIn | (B10000000 >> _i);		// Shifts the read data into _dataIn
		}
		digitalWrite(_clockPin, HIGH);
		delayMicroseconds(_delay);
	}
	return _dataIn;
}


uint16_t PsxPad::read()
{
	digitalWrite(_attPin, LOW);

	shift(0x01);
	shift(0x42);
	shift(0xFF);

	_data1 = ~shift(0xFF);
	_data2 = ~shift(0xFF);

	digitalWrite(_attPin, HIGH);

	_dataOut = (_data2 << 8) | _data1;

	return _dataOut;
}
