#ifndef WTIMER_H
#define WTIMER_H

#include "libmfwtimer.h"

#define SECONDS(x) 		((uint32_t)(x) * 1000)
#define MILLISECONDS(x) (SECONDS(x) / 1000)

#define ABSOLUTE    0
#define RELATIVE    1

uint8_t CheckTask(struct wtimer_desc *desc);
void ScheduleTask(struct wtimer_desc *desc, wtimer_desc_handler_t handler, uint8_t relative, uint32_t time);

struct wtimer_state {
	struct wtimer_state_time {
		uint32_t cur;
		uint16_t ref;
	} time;
	struct wtimer_desc *queue;
};

extern struct wtimer_state wtimer_state[2];
extern struct wtimer_callback *wtimer_pending;

#define WTIMER_NULL      (0x0)
#define WTIMER_NULLDESC  ((struct wtimer_desc *)WTIMER_NULL)
#define WTIMER_NULLCB    ((struct wtimer_callback *)WTIMER_NULL)
#define WTIMER_PTR(x)    ((struct wtimer_desc *)((char *)&(x) - (char *)&((struct wtimer_desc *)0)->next))
#define WTIMER_CBPTR(x)  ((struct wtimer_callback *)((char *)&(x) - (char *)&((struct wtimer_callback *)0)->next))

// the following routines must be called with (wtimer) interrupts disabled
extern void wtimer_addcb_core(struct wtimer_callback *desc);
extern uint8_t wtimer_removecb_core(struct wtimer_callback *desc);
extern void wtimer0_schedq(void);
extern void wtimer0_update(void);
extern void wtimer0_addcore(struct wtimer_desc *desc);
extern uint8_t wtimer0_removecb_core(struct wtimer_desc *desc);
uint8_t wtimer_idle(uint8_t flags);

enum wtimer_func_t
{
	WTIMER_GLOBAL_IRQ_ENABLE,
	WTIMER_GLOBAL_IRQ_DISABLE,
	WTIMER_CC_IRQ_ENABLE,
	WTIMER_CC_IRQ_DISABLE,
	WTIMER_SET_CC,
	WTIMER_GET_CC,
	WTIMER_GET_CNT,
	WTIMER_CHECK_CC_IRQ,
};

void wtimer_cc0_irq(void);
void wtimer_reg_func(uint8_t name, void *fn);

#endif /* WTIMER_H */
