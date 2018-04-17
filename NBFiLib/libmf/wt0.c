#include "wtimer.h"

extern void (* __wtimer_globla_irq_enable)(void);
extern void (* __wtimer_globla_irq_disable)(void);

uint8_t wtimer0_removecb_core(struct wtimer_desc *desc)
{
	struct wtimer_desc *d;
	uint8_t ret = 0;
	d = WTIMER_PTR(wtimer_state[0].queue);
	for (;;) {
		struct wtimer_desc *dn = d->next;
		if (dn == WTIMER_NULLDESC)
			break;
		if (dn == desc) {
			dn = dn->next;
			d->next = dn;
			++ret;
			if (dn == WTIMER_NULLDESC)
				break;
		}
		d = dn;
	}
	return ret;
}

void wtimer0_addabsolute(struct wtimer_desc *desc)
{
	__wtimer_globla_irq_disable();
	wtimer0_update();
	wtimer0_addcore(desc);
	wtimer0_schedq();
	__wtimer_globla_irq_enable();
}

void wtimer0_addrelative(struct wtimer_desc *desc)
{
	__wtimer_globla_irq_disable();
	wtimer0_update();
	desc->time += wtimer_state[0].time.cur;
	wtimer0_addcore(desc);
	wtimer0_schedq();
	__wtimer_globla_irq_enable();
}

uint32_t wtimer0_curtime(void)
{
	uint32_t r;

	__wtimer_globla_irq_disable();
	wtimer0_update();
	r = wtimer_state[0].time.cur;
	__wtimer_globla_irq_enable();
	return r;
}

uint8_t wtimer0_remove(struct wtimer_desc *desc)
{
	uint8_t ret;

	__wtimer_globla_irq_disable();
	ret = wtimer_removecb_core((struct wtimer_callback *)desc);
	ret += wtimer0_removecb_core(desc);
	__wtimer_globla_irq_enable();
	return ret;
}
