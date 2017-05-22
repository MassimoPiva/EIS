#include "Arduino.h"
#include "LcdBarGraphX.h"


// -- initializing bar segment characters
#ifndef USE_BUILDIN_FILLED_CHAR
// -- filled character
byte LcdBarGraphX::_level0[8] = {
    B11111,
    B11111,
    B11111,
    B11111,
    B11111,
    B11111,
    B11111,
    B11111
};
#endif

// -- character with one bar
byte LcdBarGraphX::_level1[8] = {
    B10000,
    B10000,
    B10000,
    B10000,
    B10000,
    B10000,
    B10000,
    B10000
};
// -- character with two bars
byte LcdBarGraphX::_level2[8] = {
    B11000,
    B11000,
    B11000,
    B11000,
    B11000,
    B11000,
    B11000,
    B11000
};
// -- character with three bars
byte LcdBarGraphX::_level3[8] = {
    B11100,
    B11100,
    B11100,
    B11100,
    B11100,
    B11100,
    B11100,
    B11100
};
// -- character with four bars
byte LcdBarGraphX::_level4[8] = {
    B11110,
    B11110,
    B11110,
    B11110,
    B11110,
    B11110,
    B11110,
    B11110
};

// -- constructor
LcdBarGraphX::LcdBarGraphX(LCD* lcd, byte numCols, byte startX, byte startY)
{
    // -- setting fields
    _lcd = lcd;
    _numCols = numCols;
    _startX = startX;
	_startY = startY;
}

void LcdBarGraphX::begin()
{
    // -- creating characters
#ifndef USE_BUILDIN_FILLED_CHAR
    _lcd->createChar(0, this->_level0);
#endif
    _lcd->createChar(1, this->_level1);
    _lcd->createChar(2, this->_level2);
    _lcd->createChar(3, this->_level3);
    _lcd->createChar(4, this->_level4);
    // -- setting initial values
    this->_prevValue = 0; // -- cached value
    this->_lastFullChars = 0; // -- cached value
	this->_initialized = true;
}

// -- the draw function
void LcdBarGraphX::drawValue(int value, int maxValue) {
	if(!this->_initialized) {
		this->begin();
	}
    // -- calculate full (filled) character count
    byte fullChars = (long)value * _numCols / maxValue;
    // -- calculate partial character bar count
    byte mod = ((long)value * _numCols * 5 / maxValue) % 5;

    // -- if value does not change, do not draw anything
    int normalizedValue = (int)fullChars * 5 + mod;
    if(this->_prevValue != normalizedValue) {
        // -- do not clear the display to eliminate flickers
        _lcd->setCursor(_startX, _startY);

        // -- write filled characters
        for(byte i=0; i<fullChars; i++) {
#ifdef USE_BUILDIN_FILLED_CHAR
            _lcd->write((byte)USE_BUILDIN_FILLED_CHAR);  // -- use build in filled char
#else
            _lcd->write((byte)0);
#endif
        }

        // -- write the partial character
        if(mod > 0) {
            _lcd->write(mod); // -- index the right partial character
            ++fullChars;
        }

        // -- clear characters left over the previous draw
        for(byte i=fullChars;i<this->_lastFullChars;i++) {
            _lcd->write(' ');
        }

        // -- save cache
        this->_lastFullChars = fullChars;
        this->_prevValue = normalizedValue;
    }
    /*
    // debug info
    _lcd->setCursor(0,1);
    _lcd->write('[');
    _lcd->print((int)value);
    _lcd->write(' ');
    _lcd->print(normalizedValue);
    _lcd->write(' ');
    _lcd->print((int)mod);
    _lcd->write(']');
    */
}
