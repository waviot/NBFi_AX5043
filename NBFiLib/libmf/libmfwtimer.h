#ifndef LIBMFWTIMER_H
#define LIBMFWTIMER_H

struct wtimer_callback;
struct wtimer_desc;

typedef void (*wtimer_callback_handler_t)(struct wtimer_callback *desc);
typedef void (*wtimer_desc_handler_t)(struct wtimer_desc *desc);

struct wtimer_callback {
	// do not change the order
	// must be compatible with wtimer_desc
	struct wtimer_callback *next;
	wtimer_callback_handler_t handler;
};

struct wtimer_desc {
	// do not change the order
	struct wtimer_desc *next;
	wtimer_desc_handler_t handler;
	uint32_t time;
};

#define WTFLAG_CANSLEEP                 0x01
#define WTFLAG_CANSTANDBY               0x02
#define WTFLAG_CANSLEEPCONT             0x04

#define WTIDLE_WORK                     0x01
#define WTIDLE_SLEEP                    0x02

extern void wtimer_init(void);
extern void wtimer_init_deepsleep(void);
extern uint8_t wtimer_idle(uint8_t flags);
extern uint8_t wtimer_runcallbacks(void);

extern uint32_t wtimer0_curtime(void);
extern void wtimer0_addabsolute(struct wtimer_desc *desc);
extern void wtimer0_addrelative(struct wtimer_desc *desc);
extern uint8_t wtimer_remove(struct wtimer_desc *desc);
extern uint8_t wtimer0_remove(struct wtimer_desc *desc);

extern void wtimer_add_callback(struct wtimer_callback *desc);
extern uint8_t wtimer_remove_callback(struct wtimer_callback *desc);

extern uint8_t wtimer_cansleep(void);

void wtimer_irq(void);

#endif /* LIBMFWTIMER_H */
