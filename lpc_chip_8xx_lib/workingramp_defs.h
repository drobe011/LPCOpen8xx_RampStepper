#ifndef WORKINGRAMP_DEFS_H
#define WORKINGRAMP_DEFS_H

#include "LPC8xx.h"
//#include <math.h>
//#include "PID_v1.h"

///SYSCON
#define GPIO 6
#define SWM 7
#define SCT 8
#define MRT 10
#define MRT_RST 7
#define SCT_RST 8

///GPIO
#define DIR 7
#define STEP_PIN 16
#define EN 17

///SWM
#define CTOUT_0_O 24

///MRT
#define INTEN 0
#define INTFLAG 0

///SCT
#define UNIFY 0
#define AUTOLIMIT_L 17
#define BIDIR_L 4
#define COMBMODE 12
#define DIRECTION 21
#define OUTSEL 5
#define MATCHSEL 0
#define CLRCTR 3
#define HALT_L 2

#define F_CPU 24000000UL

#define FORWARD() LPC_GPIO_PORT->CLR0 = (1 << DIR)
#define REVERSE() LPC_GPIO_PORT->SET0 = (1 << DIR)
#define CHANGE_DIR() LPC_GPIO_PORT->NOT0 = (1 << DIR)
#define ENABLE_DRIVER() LPC_GPIO_PORT->CLR0 = (1 << EN)
#define DISABLE_DRIVER() LPC_GPIO_PORT->SET0 = (1 << EN)
#define uS(x) ((F_CPU / 1000000) * x)
#define PPS(x) (F_CPU / x)
#define SET_RAMP_CLKRATE(x) LPC_MRT->Channel[0].INTVAL = (x)
#define ENABLE_RAMP_CLK() LPC_MRT->Channel[0].CTRL |= (1 << INTEN)
#define DISABLE_RAMP_CLK() LPC_MRT->Channel[0].CTRL &= ~(1 << INTEN)
#define RUN_CLK_PPS(x) LPC_SCT->MATCHREL[0].U = SCT_PPS(x)
#define DISABLE_RUN_CLK() LPC_SCT->CTRL_U |= (1 << HALT_L)
#define ENABLE_RUN_CLK() LPC_SCT->CTRL_U &= ~(1 << HALT_L)
#define CLEAR_MRT_IRQ(x) LPC_MRT->IRQ_FLAG |= x
#define CLEAR_SCT_IRQ(x) LPC_SCT->EVFLAG = x

#define STEPS_PER_REV 400
#define START_SPEED_PPS 200
#define FULL_SPEED_PPS 2000

enum {ENABLE, DISABLE, FORWARD, REVERSE};
enum {ACCEL, RUN, DECEL, STOP};

struct
{
	uint32_t m_EN;
	uint32_t m_DIR;
	volatile uint32_t m_STATE;
	volatile uint32_t m_MOVESTEPS;
	volatile uint32_t m_MAXRUNSTEPS;
	volatile uint32_t m_MAXRAMPSTEPS;
	volatile uint32_t m_SPEED;
}m_Vals;

volatile uint32_t ticker, steps, rampsteps;
void setupSystem();
void setupMRT(uint32_t channel, uint32_t rate);
void setupSCT(uint32_t rate);
void start(uint32_t moves);
static __inline__ uint32_t SCT_uS(uint32_t us) {return (SystemCoreClock / 1000000) * us;}
static __inline__ uint32_t SCT_PPS(uint32_t pps) {return ((SystemCoreClock / pps) / 2);}


#endif