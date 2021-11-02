
#include "DigitalIoPin.h"
#include <chip.h>

DigitalIoPin::DigitalIoPin(int newPort, int newPin, bool newInput, bool newPullup, bool newInvert) {
	port = newPort;
	pin = newPin;
	input = newInput;
	pullup = newPullup;
	invert = newInvert;
	if (newInput){
		Chip_IOCON_PinMuxSet(LPC_IOCON, newPort,newPin, (IOCON_MODE_PULLUP | IOCON_DIGMODE_EN|IOCON_INV_EN));
		Chip_GPIO_SetPinDIRInput(LPC_GPIO,newPort,newPin);
	} else {
		Chip_IOCON_PinMuxSet(LPC_IOCON, newPort,newPin,(IOCON_MODE_INACT | IOCON_DIGMODE_EN));
		Chip_GPIO_SetPinDIROutput(LPC_GPIO,newPort,newPin);
		Chip_GPIO_SetPinState(LPC_GPIO,newPort,newPin,false);

	}

}

DigitalIoPin::~DigitalIoPin() {
	// TODO Auto-generated destructor stub
}

bool DigitalIoPin::read(){
	return Chip_GPIO_GetPinState(LPC_GPIO,port,pin);
}

void DigitalIoPin::write(bool st){
	Chip_GPIO_SetPinState(LPC_GPIO,port,pin,st);
}

