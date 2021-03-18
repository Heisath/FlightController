// Rotary.h

#ifndef _ROTARY_h
#define _ROTARY_h
#include "arduino.h"

class Rotary {
	bool left, right, button;

public:
	volatile bool _int_left, _int_right, _int_button;

	inline void Update() {
		left = _int_left;
		right = _int_right;
		button = _int_button;
		_int_left = 0;
		_int_right = 0;
		_int_button = 0;
	}

	inline bool Left() {
		bool temp = left;
		left = false;
		return temp;
	}
	inline bool Right() {
		bool temp = right;
		right = false;
		return temp;
	}
	inline bool Button() {
		bool temp = button;
		button = false;
		return temp;
	}

	inline void Flush() {
		left = false;
		right = false;
		button = false;
	}

} rotary_left, rotary_right;



#endif

