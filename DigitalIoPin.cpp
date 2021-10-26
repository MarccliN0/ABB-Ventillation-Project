/*
 * DigitalIoPin.cpp
 *
 *  Created on: 31 Jan 2017
 *      Author: Hege
 */

#include "DigitalIoPin.h"
#include <chip.h>

DigitalIoPin::DigitalIoPin(int po, int pi, bool inp, bool pull, bool inv) {
	port = po;
	pin = pi;
	input = inp;
	pullup = pull;
	invert = inv;
	if (inp){
		Chip_IOCON_PinMuxSet(LPC_IOCON, po,pi, (IOCON_MODE_PULLUP | IOCON_DIGMODE_EN|IOCON_INV_EN));
		Chip_GPIO_SetPinDIRInput(LPC_GPIO,po,pi);
	} else {
		Chip_IOCON_PinMuxSet(LPC_IOCON, po,pi,(IOCON_MODE_INACT | IOCON_DIGMODE_EN));
		Chip_GPIO_SetPinDIROutput(LPC_GPIO,po,pi);
		Chip_GPIO_SetPinState(LPC_GPIO,po,pi,false);

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

