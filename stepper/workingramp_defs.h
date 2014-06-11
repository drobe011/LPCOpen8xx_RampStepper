#ifndef WORKINGRAMP_DEFS_H
#define WORKINGRAMP_DEFS_H

#include <chip.h>

///GPIO
extern const uint32_t DIR;
extern const uint32_t STEP_PIN;
extern const uint32_t EN;

#define F_CPU 24000000UL

#define uS(x) ((F_CPU / 1000000) * x)
#define PPS(x) (F_CPU / x)
#define SET_RAMP_CLKRATE(c, x) Chip_MRT_SetInterval(LPC_MRT_CH(c), (x))
#define ENABLE_RAMP_CLK() Chip_MRT_SetEnabled(LPC_MRT_CH0) //LPC_MRT->Channel[0].CTRL |= (1 << INTEN)
#define DISABLE_RAMP_CLK() Chip_MRT_SetDisabled(LPC_MRT_CH0) //LPC_MRT->Channel[0].CTRL &= ~(1 << INTEN)
#define RUN_CLK_PPS(x) Chip_SCT_SetMatchReload(LPC_SCT, SCT_MATCH_0, SCT_PPS(x))
#define DISABLE_RUN_CLK() Chip_SCT_SetControl(LPC_SCT, SCT_CTRL_HALT_L) //->CTRL_U |= (1 << HALT_L)
#define ENABLE_RUN_CLK() Chip_SCT_ClearControl(LPC_SCT, SCT_CTRL_HALT_L) //->CTRL_U &= ~(1 << HALT_L)
#define CLEAR_MRT_IRQ(x) LPC_MRT->IRQ_FLAG |= x

extern const uint32_t STEPS_PER_REV;

extern const uint32_t START_SPEED_PPS;

enum {mENABLE, mDISABLE, mFORWARD, mREVERSE};
enum {ACCEL, RUN, DECEL, STOP};

typedef struct
{
	uint32_t m_EN;
	uint32_t m_DIR;
	volatile uint32_t m_STATE;
	volatile uint32_t m_MOVESTEPS;
	volatile uint32_t m_MAXRUNSTEPS;
	volatile uint32_t m_MAXRAMPSTEPS;
	volatile uint32_t m_DECELSTEPS;
	volatile uint32_t m_SPEED;
}t_Vals;

extern t_Vals m_Vals;

void STEP_setupSystem();
void STEP_setupMRT(uint32_t channel, uint32_t rate);
void STEP_setupSCT(uint32_t rate);
void STEP_start(uint32_t moves);
static __inline__ uint32_t SCT_uS(uint32_t us) {return (SystemCoreClock / 1000000) * us;}
static __inline__ uint32_t SCT_PPS(uint32_t pps) {return ((SystemCoreClock / pps) / 2);}
static __inline__ void STEP_ENABLE_DRIVER() {Chip_GPIO_SetPinOutLow(LPC_GPIO_PORT, 0, EN); m_Vals.m_EN = mENABLE;}
static __inline__ void STEP_DISABLE_DRIVER(){Chip_GPIO_SetPinOutHigh(LPC_GPIO_PORT, 0, EN); ; m_Vals.m_EN = mDISABLE;}
static __inline__ void STEP_FORWARD() {Chip_GPIO_SetPinOutLow(LPC_GPIO_PORT, 0, DIR); m_Vals.m_DIR = mFORWARD;}
static __inline__ void STEP_REVERSE() {Chip_GPIO_SetPinOutHigh(LPC_GPIO_PORT, 0, DIR); m_Vals.m_DIR = mREVERSE;}
static __inline__ void STEP_CHANGE_DIR() {Chip_GPIO_SetPinToggle(LPC_GPIO_PORT, 0, DIR); m_Vals.m_DIR = (m_Vals.m_DIR == mREVERSE) ? mFORWARD : mREVERSE;}
				
#endif