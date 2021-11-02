#ifndef DIGITALIOPIN_H_
#define DIGITALIOPIN_H_

class DigitalIoPin {
public:
	DigitalIoPin(int newPort, int newPin, bool newInput = true, bool newPullup = true, bool newInvert = false);
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
