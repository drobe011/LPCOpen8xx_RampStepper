#include "workingramp_defs.h"

//TODO: Change DIR in struct
//TODO: Change EN in struct

#define DDD

#ifdef DDD
#include <cross_studio_io.h>
volatile uint32_t saMilli, eaMilli, sdMilli, edMilli, fSpeed;
#endif

int main(void)
{
typedef struct UARTD_API {
 // index of all the uart driver functions
uint32_t (*uart_get_mem_size)(void);
UART_HANDLE_T (*uart_setup)(uint32_t base_addr, uint8_t *ram);
uint32_t (*uart_init)(UART_HANDLE_T handle, UART_CONFIG_T *set);
//--polling functions--//
uint8_t (*uart_get_char)(UART_HANDLE_T handle);
void (*uart_put_char)(UART_HANDLE_T handle, uint8_t data);
uint32_t (*uart_get_line)(UART_HANDLE_T handle, UART_PARAM_T * param);
uint32_t (*uart_put_line)(UART_HANDLE_T handle, UART_PARAM_T * param);
//--interrupt functions--//
void (*uart_isr)(UART_HANDLE_T handle);
} UARTD_API_T ;
 // end of structure



	setupSystem();
	setupMRT(0, 1000);
	setupSCT(SCT_PPS(START_SPEED_PPS));
	
	uint32_t ch = 0, cnts = 1;

	#ifdef DDD
	debug_puts("Hello");
	#endif
	
	FORWARD();
	ENABLE_DRIVER();
	start(STEPS_PER_REV*5);
	while(1)
	{
		if (ticker > 5000)
		{
			ticker = 0;
		
			#ifdef DDD
			debug_printf("Steps %d\n\r",steps+rampsteps*2);
			debug_printf("  RU-%d, RUN-%d, RD-%d\n\r",rampsteps,steps,rampsteps);
			debug_printf("  Accel time :%d\n\r", eaMilli-saMilli);
			debug_printf("  Decel time :%d\n\r", edMilli-sdMilli);
			debug_printf("  Runspeed :%d\n\r", fSpeed);
			#endif
			
			start(STEPS_PER_REV*cnts);
			cnts = (cnts < 7) ? cnts + 1 : 1;
		}
		#ifdef DDD
		if (debug_kbhit())
		{
			ch = debug_getch();
			if(ch == 'e') ENABLE_DRIVER();
			if(ch == 'd') DISABLE_DRIVER();
			if(ch == 'c' && m_Vals.m_STATE == STOP) CHANGE_DIR();
		}
		#endif
	}
}
void SysTick_Handler(void)
{
	ticker++;
}

void MRT_IRQHandler(void)
{
	switch (m_Vals.m_STATE)
	{
		case ACCEL:
		m_Vals.m_SPEED = (m_Vals.m_SPEED < FULL_SPEED_PPS) ? m_Vals.m_SPEED + 2 : FULL_SPEED_PPS;	
		if (rampsteps >= m_Vals.m_MAXRAMPSTEPS-1)
		{
			m_Vals.m_STATE = RUN;
			#ifdef DDD
			eaMilli=ticker;
			#endif
		}

		break;
		case DECEL:
		m_Vals.m_SPEED = (m_Vals.m_SPEED > START_SPEED_PPS) ? m_Vals.m_SPEED - 2 : START_SPEED_PPS;
		if (rampsteps >= m_Vals.m_MAXRAMPSTEPS)
		{
			m_Vals.m_STATE = STOP;
			DISABLE_RUN_CLK();
			#ifdef DDD
			edMilli=ticker;
			#endif
		}
		break;
		default:
		DISABLE_RAMP_CLK();
	}
	RUN_CLK_PPS(m_Vals.m_SPEED);
	CLEAR_MRT_IRQ(1);
}

void SCT_IRQHandler(void)
{
	if (m_Vals.m_STATE == RUN)
	{
		if (steps >= m_Vals.m_MAXRUNSTEPS)
		{
			#ifdef DDD
			fSpeed = m_Vals.m_SPEED;
			sdMilli=ticker;
			#endif
			m_Vals.m_STATE = DECEL;
			ENABLE_RAMP_CLK();
			rampsteps = 0;
		}
		else steps++;
	}
	else if (m_Vals.m_STATE == ACCEL || m_Vals.m_STATE == DECEL) rampsteps++;
	
	CLEAR_SCT_IRQ(1);
}

void start(uint32_t moves)
{
	m_Vals.m_DIR = FORWARD;
	m_Vals.m_EN = ENABLE;
	m_Vals.m_STATE = ACCEL;
	m_Vals.m_SPEED = START_SPEED_PPS;
	m_Vals.m_MOVESTEPS = moves;
	m_Vals.m_MAXRAMPSTEPS = (moves / 10)* 2;
	m_Vals.m_MAXRUNSTEPS = (moves / 10) * 6;
	steps = 0;
	rampsteps = 0;
	RUN_CLK_PPS(START_SPEED_PPS);
	ENABLE_RAMP_CLK();
	ENABLE_RUN_CLK();
	#ifdef DDD
	saMilli=ticker;
	#endif
}

void setupSystem()
{
	SystemCoreClockUpdate();
	SysTick_Config(SystemCoreClock/1000);
	
	LPC_SYSCON->SYSAHBCLKCTRL |= (1 << GPIO) | (1 << SCT) | (1 << SWM) | (1 << MRT);
	LPC_SWM->PINASSIGN6 &= (STEP_PIN << CTOUT_0_O);
	LPC_SYSCON->SYSAHBCLKCTRL &= ~(1 << SWM);
	LPC_SYSCON->PRESETCTRL |= (1 << MRT_RST) | (1 << SCT_RST);

	LPC_GPIO_PORT->DIR0 = (1 << DIR) | (1 << EN);
	LPC_GPIO_PORT->SET0 = (1 << DIR) | (1 << EN);

	NVIC_EnableIRQ(SysTick_IRQn);
	NVIC_EnableIRQ(MRT_IRQn);
	NVIC_EnableIRQ(SCT_IRQn);
}

void setupMRT(uint32_t channel, uint32_t rate)
{
	SET_RAMP_CLKRATE(uS(rate));
	LPC_MRT->Channel[channel].CTRL = 0;
	LPC_MRT->Channel[channel].STAT |= (1 << INTFLAG);
}

void setupSCT(uint32_t rate)
{
	LPC_SCT->OUTPUT = 1;
	LPC_SCT->CONFIG |= (1 << UNIFY) | (1 << AUTOLIMIT_L);
	LPC_SCT->CTRL_U |= (1 << BIDIR_L);
	LPC_SCT->MATCH[0].U = rate;
	LPC_SCT->MATCH[1].U = 1;
	LPC_SCT->MATCHREL[0].U = rate;
	LPC_SCT->MATCHREL[1].U = 1;
	LPC_SCT->EVENT[0].STATE = 0xFFFFFFFF;
	LPC_SCT->EVENT[1].STATE = 0xFFFFFFFF;
	LPC_SCT->EVENT[0].CTRL = (1 << COMBMODE) | (1 << DIRECTION) | (1 << OUTSEL);
	LPC_SCT->EVENT[1].CTRL = (1 << MATCHSEL) | (1 << COMBMODE) | (2 << DIRECTION) | (1 << OUTSEL);
	LPC_SCT->OUT[0].SET = 2;
    LPC_SCT->OUT[0].CLR = 1;
	LPC_SCT->EVEN = 1;
	LPC_SCT->CTRL_U |= (1 << CLRCTR);
}