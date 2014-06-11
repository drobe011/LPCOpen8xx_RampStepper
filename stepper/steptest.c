#include "workingramp_defs.h"
#include <chip.h>

#include <cross_studio_io.h>

volatile uint32_t ticker;

int main(void)
{
	STEP_setupSystem();
	STEP_setupMRT(0, 2000);
	STEP_setupSCT(SCT_PPS(START_SPEED_PPS));
	
	uint32_t ch = 0, ticksave = 0;
	uint8_t tstFlag = false;

	STEP_FORWARD();
	ticksave = ticker;

	while(1)
	{
		if (ticker - ticksave >= 10)
		{
			if (tstFlag == true)
			{
				if (m_Vals.m_STATE == STOP)
				{
					tstFlag = false;
					debug_puts("test done");
				}
			}			
			if (debug_kbhit())
			{
				ch = debug_getch();
				if(ch == 'e') {STEP_ENABLE_DRIVER(); debug_puts("Enable");}
				if(ch == 'd') {STEP_DISABLE_DRIVER(); debug_puts("Disable");}
				if(ch == 'c' && m_Vals.m_STATE == STOP) STEP_CHANGE_DIR();
				if(ch == 't')
				{
					debug_puts("test start");
					tstFlag = true;
					STEP_start(STEPS_PER_REV*5);
				}
			}
			//ticksave = ticker;
		}
	}
}

void SysTick_Handler(void)
{
	ticker++;
}