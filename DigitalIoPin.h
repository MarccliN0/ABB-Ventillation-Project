/*
 * DigitalIoPin.h
 *
 *  Created on: 31 Jan 2017
 *      Author: Hege
 */

#ifndef DIGITALIOPIN_H_
#define DIGITALIOPIN_H_

class DigitalIoPin {
public:
	DigitalIoPin(int port, int pin, bool input = true, bool pullup = true, bool invert = false);
	virtual ~DigitalIoPin();
	bool read();
	void write(bool value);
private:
	int port;
	int pin;
	bool input;
	bool pullup;
	bool invert;
};

#endif /* DIGITALIOPIN_H_ */
