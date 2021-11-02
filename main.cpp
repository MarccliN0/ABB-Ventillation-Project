#if defined (__USE_LPCOPEN)
#if defined(NO_BOARD_LIB)
#include "chip.h"
#else
#include "board.h"
#endif
#endif

#include <cr_section_macros.h>
#include <cstring>
#include <cstdio>

#include "ABBcontroller.h"

int main(void)
{

#if defined (__USE_LPCOPEN)
	// Read clock settings and update SystemCoreClock variable
	SystemCoreClockUpdate();
#if !defined(NO_BOARD_LIB)
	// Set up and initialize all required blocks and
	// functions related to the board hardware
	Board_Init();
	// Set the LED to the state of "On"
	Board_LED_Set(0, true);
#endif
#endif

	/* Enable and setup SysTick Timer at a periodic rate */
	SysTick_Config(SystemCoreClock / 1000);

	/* Enable RIT */
	Chip_RIT_Init(LPC_RITIMER);

	/* Enable ITM printing */
	ITM_init();

	ABBcontroller abbController;	// create controller object
	abbController.startAbb();		// initialize connection with ABB

	// Main loop that keeps controlling both UI and ABB
	while (1) {
		// UI
		abbController.readUserinput();

		// ABB
		if (abbController.getMode()) {
			abbController.autoMeasure();
		} else{
			abbController.manualMeasure();
		}
	}

	return 1;
}

