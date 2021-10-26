/*
 * ABBcontroller.h
 *
 *  Created on: 10.3.2017
 *  Authors: Tommi Pälviö, Henri Riisalo
 */

#ifndef ABBCONTROLLER_H_
#define ABBCONTROLLER_H_

// Libraries
#include <cstring>
#include <cstdio>
#include <stdlib.h> // itoa
// Own files
#include "ModbusMaster.h"
#include "I2C.h"
#include "ITM_write.h"
#include "LiquidCrystal.h"
#include "DigitalIoPin.h"

class ABBcontroller {
public:
	ABBcontroller();
	bool autoMeasure();
	bool manualMeasure();
	virtual ~ABBcontroller();
	bool getMode();
	void printRegister(uint16_t reg);
	bool setFrequency(uint16_t freq);
	void measure();
	int compare();
	bool startAbb();
	bool stopAbb();
	void printData();
	// USER INTERFACE FUNCTIONS
	void drawUserInterface(); 	// draws the user interface on the LCD
	void readUserinput();		// reads user input and controls UI

private:
	/**
	 * current mode of the program and dictates the way the ABB is controlled.
	 */
	bool autoMode;
	/**
	 * ModbusMaster object for communicating with the ABB ??
	 */
	ModbusMaster *node;
	/**
	 * Current frequency of the ABB.
	 */
	uint16_t frequency;
	/**
	 * Pressure level for the automatic mode to maintain.
	 * Value can be changed through the user interface.
	 */
	uint16_t pasc;
	/**
	 * One tick is 100ms
	 */
	int tickLimit;
	/**
	 * Frequency value added or removed from the current frequency when
	 * automatic mode is trying to acquire the specified pressure level.
	 * Adjusted by ABBcontroller::compare()
	 */
	uint16_t oneStep;
	/**
	 * Current pressure
	 */
	float pressureCurrent;
	/**
	 * The result of the average calculation of the pressure.
	 */
	float pressureAvg;
	/**
	 * Variable to hold the sum of pressures used in average calculation
	 */
	int pressureCount;

	// USER INTERFACE RELATED VARIABLES
	/**
	 * LiquidCrystal object. Displays the user interface on LCD screen
	 */
	LiquidCrystal* lcd;
	/**
	 * Different states of the user interface
	 */
	enum userInterfaceStates { menu,
		automaticMode, manualMode, warningUnreachablePressure };
	/**
	 * Current state of the user interface. Dictates what is drawn on the LCD
	 */
	int userInterfaceState;
	/**
	 * Describes which option is currently selected in "menu" state
	 */
	int selection;
	/**
	 *	Different controls for user interface
	 */
	enum interfaceControls { ok, left, right };
	/**
	 * DigitalIoPin objects used when the button controls are enabled.
	 */
	DigitalIoPin *switch1Ok, *switch2Left, *switch3Right;
	/**
	 * Variables for setting up new pressure level and fan speed through the user interface.
	 */
	uint16_t frequencyTemp, pascTemp;

};

#endif /* ABBCONTROLLER_H_ */
