/*
 * ABBcontroller.cpp
 *
 * Created on: 10.3.2017
 * Authors: Tommi Pälviö, Henri Riisalo
 * Description: Class for controlling a ventilation system via user interface.
 *				Methods that need to be looped for the class to function as intended:
 *				"readUserinput" and "autoMeasure" or "manualMeasure" depending on which mode is active.
 */

#include "ABBcontroller.h"

/**
 * Required by 'Sleep()'
 */
static volatile int counter;
/**
 * Required by 'millis()'
 */
static volatile uint32_t systicks;


static volatile int counterAutoMeasure; //Counts when to raise the flagAutoMeasure
static volatile int counterAvg; //Counts when to raise the flagAvg
static volatile bool flagAutoMeasure; //If true the frequency is changed and the UI values are updated.
static volatile bool flagAvg; // If true, the average pressure is calculated
static volatile bool flagMeasure; // If true, the pressure is measured and added to the sum counter.
static volatile bool flagSlowAlert; // If true, finding the correct pressure is too slow.
static volatile bool flagPreventAlert = false; // If true, the alert is not shown.
static volatile int counterSlow; // Counts when to raise flagSlowAlert
/**
 * Maximum pressure available for automatic mode
 */
static uint16_t pascLimit = 120;

#ifdef __cplusplus
extern "C" {
#endif
/**
 * @brief	Handle interrupt from SysTick timer
 * @return	Nothing
 */
void SysTick_Handler(void)
{
	systicks++;
	if(counter > 0) counter--;

	counterAutoMeasure++;
	if (counterAutoMeasure >= 5000) {
		counterAutoMeasure = 0;
		flagAutoMeasure = true;
		flagAvg = true;
	}

	counterAvg++;
	if (counterAvg >= 500){
		counterAvg = 0;
		flagMeasure = true;
	}
	counterSlow++;
	if (counterSlow >= 60000){
		counterSlow = 0;
		if (!flagPreventAlert){
			flagSlowAlert = true;
		}

	}
}
#ifdef __cplusplus
}
#endif

/**
 * @brief	Delays execution
 * @param	int ms - number of milliseconds the program will sleep
 * @return	Nothing
 */
void Sleep(int ms)
{
	counter = ms;
	while(counter > 0) {
		__WFI();
	}
}

/**
 * @brief	This function is required by the modbus library
 * @return	Nothing
 */
uint32_t millis() {
	return systicks;
}

/**
 * @brief	Initializes object of class ABBcontroller.
 * 			Does not automatically begin operations with the connected ABB.
 */
ABBcontroller::ABBcontroller() {
	autoMode = true;
	node = new ModbusMaster(2);
	//I2C i2c = new I2C(0, 100000);
	frequency = 3000;
	pasc = 20;
	tickLimit = 300;
	oneStep = 250;

	pressureAvg = 0;
	pressureCount = 0;
	pressureCurrent = 0;

	// USER INTRERFACE INITIALIZATION
	// Setting up liquidCrystal object to control the LCD
	DigitalIoPin* pin1 = new DigitalIoPin(0, 8, false);
	DigitalIoPin* pin2 = new DigitalIoPin(1, 6, false);
	DigitalIoPin* pin3 = new DigitalIoPin(1, 8, false);
	DigitalIoPin* pin4 = new DigitalIoPin(0, 5, false);
	DigitalIoPin* pin5 = new DigitalIoPin(0, 6, false);
	DigitalIoPin* pin6 = new DigitalIoPin(0, 7, false);
	lcd = new LiquidCrystal(pin1, pin2, pin3, pin4, pin5, pin6);

	// Setting default state for user interface
	userInterfaceState = menu;
	selection = automaticMode;

	// Initializing switches
	switch1Ok = new DigitalIoPin(0, 17, true, true, true);
	switch2Left = new DigitalIoPin(1, 11, true, true, true);
	switch3Right = new DigitalIoPin(1, 9, true, true, true);

	// Initially draw the user interface on the LCD
	drawUserInterface();

	// Initializing variables for setting up new fan speed and pressure level
	frequencyTemp = 50; pascTemp = 50;
}

/**
 * @brief	Initializes communications with the connected ABB.
 * @return 	Nothing
 */
bool ABBcontroller::startAbb(){
	node->begin(9600); // set transmission rate - other parameters are set inside the object and can't be changed here

	printRegister( 3); // for debugging

	node->writeSingleRegister(0, 0x0406); // prepare for starting

	printRegister(3); // for debugging

	Sleep(1000); // give converter some time to set up
	// note: we should have a startup state machine that check converter status and acts per current status
	//       but we take the easy way out and just wait a while and hope that everything goes well

	printRegister(3); // for debugging

	node->writeSingleRegister(0, 0x047F); // set drive to start mode

	printRegister(3); // for debugging

	Sleep(1000); // give converter some time to set up
	// note: we should have a startup state machine that check converter status and acts per current status
	//       but we take the easy way out and just wait a while and hope that everything goes well

	printRegister(3); // for debugging
	return true;
}

/**
 * @brief	slave: read 16-bit registers starting at reg.
 * 			For debugging purposes.
 * @param	uint16_t req
 * @return 	Nothing
 */
void ABBcontroller::printRegister(uint16_t reg){
	uint8_t result;
	// slave: read 16-bit registers starting at reg to RX buffer
	result = node->readHoldingRegisters(reg, 1);

	// do something with data if read is successful
	if (result == node->ku8MBSuccess)
	{
		printf("R%d=%04X\n", reg, node->getResponseBuffer(0));
	}
	else {
		printf("R%d=???\n", reg);
	}
}

/**
 * @brief 	Sets new motor frequency for the connected ABB
 * @param 	uint16_t freq - Scale: 0 - 20000
 * @return 	bool atSetpoint
 */
bool ABBcontroller::setFrequency(uint16_t freq){
	uint8_t result;
	int ctr;
	bool atSetpoint;
	const int delay = 500;

	node->writeSingleRegister(1, freq); // set motor frequency

	printf("Set freq = %d\n", freq/40); // for debugging

	// wait until we reach set point or timeout occurs
	ctr = 0;
	atSetpoint = false;
	do {
		Sleep(delay);
		// read status word
		result = node->readHoldingRegisters(3, 1);
		// check if we are at setpoint
		if (result == node->ku8MBSuccess) {
			if(node->getResponseBuffer(0) & 0x0100) atSetpoint = true;
		}
		ctr++;
	} while(ctr < 20 && !atSetpoint);

	printf("Elapsed: %d\n", ctr * delay); // for debugging
	frequency = freq;
	return atSetpoint;
}

/**
 * @brief Returns the current mode of the controller
 * @return bool autoMode
 */
bool ABBcontroller::getMode(){
	return ABBcontroller::autoMode;
}

/**
 * @brief	Main method for manual mode
 * 			Makes ABB to maintain a specified fan speed
 * 			Needs to be called in the main loop
 * @return	bool ???
 */
bool ABBcontroller::manualMeasure(){
	if (!flagAutoMeasure) {
		if (flagMeasure){
			ABBcontroller::measure();
		}
		return false;
	} else {
		flagAutoMeasure = false;
		ABBcontroller::drawUserInterface();
		ABBcontroller::setFrequency(frequency);
		return true;
	}
}

/**
 * @brief	Main method for automatic mode
 * 			Makes ABB to maintain a specified pressure level
 * 			Needs to be called in the main loop
 * @return	???
 */
bool ABBcontroller::autoMeasure(){
	if (!flagAutoMeasure) {
		if (flagMeasure){
			ABBcontroller::measure();
		}
		return false;

	} else {
		flagAutoMeasure = false;
		//uint16_t inc;
		int i = 0;
		int j = 0;
		uint8_t result;

		if (ABBcontroller::compare() == 1 && (frequency -oneStep) >= 1000){
			// slave: read (2) 16-bit registers starting at register 102 to RX buffer
			j = 0;
			do {
				result = node->readHoldingRegisters(102, 2);
				j++;
			} while(j < 3 && result != node->ku8MBSuccess);
			// note: sometimes we don't succeed on first read so we try up to threee times
			// if read is successful print frequency and current (scaled values)
			if (result == node->ku8MBSuccess) {
				printf("F=%4d, I=%4d  (ctr=%d)\n", node->getResponseBuffer(0), node->getResponseBuffer(1),j);
			}
			else {
				printf("ctr=%d\n",j);
			}
			// lippu tähän

			//Sleep(3000);
			i++;
			if(i >= 20) {
				i=0;
			}
			frequency = frequency - oneStep;
			ABBcontroller::setFrequency(frequency);

		} else if (ABBcontroller::compare() == -1&& (frequency +oneStep) <= 20000) {

			//while(ABBcontroller::measureAndCompare() == -1){
			// slave: read (2) 16-bit registers starting at register 102 to RX buffer
			j = 0;
			do {
				result = node->readHoldingRegisters(102, 2);
				j++;
			} while(j < 3 && result != node->ku8MBSuccess);
			// note: sometimes we don't succeed on first read so we try up to threee times
			// if read is successful print frequency and current (scaled values)
			if (result == node->ku8MBSuccess) {
				printf("F=%4d, I=%4d  (ctr=%d)\n", node->getResponseBuffer(0), node->getResponseBuffer(1),j);
			}
			else {
				printf("ctr=%d\n",j);
			}

			//Sleep(3000);
			i++;
			if(i >= 20) {
				i=0;
			}
			frequency+=oneStep;
			ABBcontroller::setFrequency(frequency);
			//}
		} else {
		}

		return true;
	}
}

/**
 * @brief	Reads values from pressure sensor via I2C and counts the average when the flag is true.
 * @return 	Nothing
 */
void ABBcontroller::measure(){
	I2C i2c(0, 100000);

	uint8_t pressureData[3];
	uint8_t readPressureCmd = 0xF1;
	int16_t pressure = 0;

	if (i2c.transaction(0x40, &readPressureCmd, 1, pressureData, 3)) {
		/* Output temperature. */
		pressure = (pressureData[0] << 8) | pressureData[1];

	}
	else {
		DEBUGOUT("Error reading pressure.\r\n");
	}
	//Sleep(100);
	pressure = pressure/240.0;

	if (!flagAvg){
		pressureCount += pressure;
		flagMeasure = false;
	} else {
		pressureAvg = pressureCount/5.0;
		pressureCurrent = pressureAvg;
		pressureCount = 0;
		flagAvg = false;
		DEBUGOUT("Pressure read over I2C is %.1f Pa\r\n",	pressureCurrent);
		ABBcontroller::drawUserInterface();
	}
}

/**
 * @brief	Compares readings from pressure sensor and the
 * 			wanted pressure level and adjusts settings for the automatic mode
 * @return	int
 * 			1: current pressure > wanted pressure
 * 		   -1: current pressure < wanted pressure
 * 			0: current pressure == wanted pressure
 */
int ABBcontroller::compare(){
	int comparison;

	if(pressureCurrent >= pasc){
		comparison = pressureCurrent - pasc;
	} else {
		comparison = pasc - pressureCurrent;
	}

	if (comparison < 2){
		oneStep = 25;
		flagPreventAlert = true;
	} else if (comparison < 5){
		oneStep = 50;
	} else if (comparison > 30){
		oneStep = 1000;
	}else if (comparison > 10){
		oneStep = 500;
	} else {
		oneStep = 250;
	}

	if (pressureCurrent == pasc){
		return 0;
	} else if (pressureCurrent > pasc){
		return 1;
	} else {
		return -1;
	}
}

/**
 * @brief	Draws the user interface on the LCD
 * @return 	Nothing
 */
void ABBcontroller::drawUserInterface() {
	if (flagSlowAlert) { // set UI state to show an alert
		flagSlowAlert = false;
		userInterfaceState = warningUnreachablePressure;
	}

	lcd->clear();
	lcd->setCursor(0,0);

	char buffer [33];
	std::string tempString = "";

	switch(userInterfaceState) {
	case menu:	// MENU STATE ACTIVE
		if (selection == automaticMode) { // Automatic mode selected
			// First row
			lcd->print("Automatic Mode");
			// Second row
			lcd->setCursor(0,1);
			if (autoMode) { // Prints wanted pressure and current pressure
				itoa(pasc, buffer, 10);
				tempString = "WP:" + std::string(buffer) + " CP:";
				itoa(pressureCurrent, buffer, 10);
				tempString += std::string(buffer);
				lcd->print(tempString);

			} else lcd->print("Activate");

		} else if (selection == manualMode) { // Manual mode selected
			// First row
			lcd->print("Manual Mode");
			// Second row
			lcd->setCursor(0,1);
			if (!autoMode) { // Prints current frequency(fan speed) and pressure
				itoa(frequencyTemp, buffer, 10);
				tempString = "F:" + std::string(buffer) + "% P:";

				itoa(pressureCurrent, buffer, 10);
				tempString += std::string(buffer) + " Pa";
				lcd->print(tempString);

			} else lcd->print("Activate");
		}
		break;

	case automaticMode: // AUTOMATIC STATE ACTIVE
		// First row
		lcd->print("Set pressure:");
		// Second row
		lcd->setCursor(0,1);
		itoa(pascTemp, buffer, 10);
		tempString = std::string(buffer) + " Pa";
		lcd->print(tempString);
		break;

	case manualMode: // MANUAL STATE ACTIVE
		// First row
		lcd->print("Set fan speed:");
		// Second row
		lcd->setCursor(0,1);
		itoa(frequencyTemp, buffer, 10);
		tempString = std::string(buffer) + "%";
		lcd->print(tempString);
		break;

	case warningUnreachablePressure: // WARNING STATE ACTIVE
		lcd->print("Taking too long");
		lcd->setCursor(0,1);
		lcd->print("to reach pressure!");
		break;

	default: // we should never go here
		ITM_write("drawUserInterface: Error!\n");
		break;
	}
}

/**
 * @brief	Reads user input from UART
 * 			Needs to be called in the main loop
 * @return	Nothing
 */
void ABBcontroller::readUserinput() {
	int userInput = -1;

	/*
	// ohjailu switcheillä pois käytöstä.
	if (switch1Ok->read()) {
		userInput = ok;
	} else if (switch2Left->read()) {
		userInput = left;
	} else if (switch3Right->read()) {
		userInput = right;
	} else {
		return;
	}*/

	int c;
	c = Board_UARTGetChar(); // read input from UART
	switch(c) {
	case 49: // 1
		userInput = left;
		break;
	case 50: // 2
		userInput = right;
		break;
	case 51: // 3
		userInput = ok;
		break;
	default: // nothing was pressed
		return;
	}
	Board_UARTPutChar(c); // echo back the character - for debugging purposes
	Board_UARTPutChar('\r');
	Board_UARTPutChar('\n');

	if (userInterfaceState == menu) { // MENU - toggle between modes
		if (userInput == ok) {
			userInterfaceState = selection;
			// temppien settaus  tähän
		} else if (userInput == left || userInput == right) {
			if (selection == automaticMode) {
				selection = manualMode;
			} else selection = automaticMode;
		}

	} else if (userInterfaceState == manualMode) { // MANUAL MODE - set new frequency (fan speed) for manual mode
		switch (userInput) {
		case ok: // set the current value as new frequency and return to menu state
			frequency = (20000/100)*frequencyTemp;
			userInterfaceState = menu;
			autoMode = false; // set manual mode active
			break;

		case left: // decrement frequency value: scale 10-100%
			frequencyTemp -= 10;
			if (frequencyTemp <= 0) frequencyTemp = 100;
			break;

		case right: // increment frequency value
			frequencyTemp += 10;
			if (frequencyTemp >= 100) frequencyTemp = 10;
			break;

		default:
			ITM_write("error'\n");
			break;
		}
	} else if (userInterfaceState == automaticMode) { // AUTOMATIC MODE - set new pressure level for the automatic mode to maintain
		switch (userInput) {
		case ok: // set the current value as wanted pressure level and return to menu state
			pasc = pascTemp;
			userInterfaceState = menu;
			autoMode = true; 			// set auto mode active
			flagPreventAlert = false; 	// enable showing alert
			counterSlow = 0; 			// disable alert flag
			break;

		case left: // decrement pressure level: scale 5-120 Pa
			pascTemp -= 5;
			if (pascTemp <= 1) pascTemp = pascLimit;
			break;

		case right: // increment pressure level
			pascTemp += 5;
			if (pascTemp >= pascLimit) pascTemp = 5;
			break;

		default:
			ITM_write("error'\n");
			break;
		}
	} else if(userInterfaceState == warningUnreachablePressure) { // ALERT - notify user that given pressure level might not be reachable
		if (userInput == ok) { // acknowledge the alert
			flagPreventAlert = true;
			userInterfaceState = menu;
		}
	} else {
		ITM_write("readUserInput: Something went wrong!\n");
	}
	drawUserInterface();	// redraw the UI to reflect the changes
	Sleep(200); 			// prevent the program from immediately reading another input
}

ABBcontroller::~ABBcontroller() {
	// TODO Auto-generated destructor stub
}

