/*
 * ax5043.c
 *
 *  Created on: 28 мар. 2018 г.
 *      Author: lema
 */

#include "ax5043.h"
#include "easyax5043.h"

#ifdef FORMAT_CODE
//#pragma default_function_attributes = @ "AXRADIO_FUNC"
#endif
     
     
void (*__ax5043_enable_global_irq)(void);
void (*__ax5043_disable_global_irq)(void);
void (*__ax5043_enable_pin_irq)(void);
void (*__ax5043_disable_pin_irq)(void);
uint8_t (*__ax5043_get_irq_pin_state)(void);
void (*__spi_rx)(uint8_t *, uint16_t);
void (*__spi_tx)(uint8_t *, uint16_t);
void (*__spi_tx_rx)(uint8_t *, uint8_t *, uint16_t);
void (*__spi_cs_set)(uint8_t);
void (*__axradio_statuschange)(struct axradio_status *st);
void (*__ax5043_on_off_pwr)(uint8_t) = 0;

void ax5043_reg_func(uint8_t name, void*  fn)
{
	switch(name)
	{
	case AXRADIO_ENABLE_GLOBAL_IRQ:
		__ax5043_enable_global_irq = (void(*)(void))fn;
		break;
	case AXRADIO_DISABLE_GLOBAL_IRQ:
		__ax5043_disable_global_irq = (void(*)(void))fn;
		break;
	case AXRADIO_ENABLE_IRQ_PIN:
		__ax5043_enable_pin_irq = (void(*)(void))fn;
		break;
	case AXRADIO_DISABLE_IRQ_PIN:
		__ax5043_disable_pin_irq = (void(*)(void))fn;
		break;
	case AXRADIO_GET_IRQ_PIN:
		__ax5043_get_irq_pin_state = (uint8_t(*)(void))fn;
		break;
	case AXRADIO_SPI_RX:
		__spi_rx = (void(*)(uint8_t*,uint16_t))fn;
		break;
	case AXRADIO_SPI_TX:
		__spi_tx = (void(*)(uint8_t*,uint16_t))fn;
		break;
	case AXRADIO_SPI_TX_RX:
		__spi_tx_rx = (void(*)(uint8_t*,uint8_t*,uint16_t))fn;
		break;
	case AXRADIO_SPI_CS_WRITE:
		__spi_cs_set = (void(*)(uint8_t))fn;
		break;
	case AXRADIO_STATUSCHANGE:
		__axradio_statuschange = (void(*)(struct axradio_status*))fn;
		break;
    case AXRADIO_ON_OFF_PWR:
        __ax5043_on_off_pwr = (void(*)(uint8_t))fn;
        break;
	default:
		break;
	}
}

void ax5043_enter_deepsleep(void)
{
	ax5043_spi_write(AX5043_PWRMODE, AX5043_PWRSTATE_POWERDOWN);
	ax5043_spi_write(AX5043_PWRMODE, AX5043_PWRSTATE_DEEPSLEEP);
}

uint8_t ax5043_wakeup_deepsleep_core(void)
{
	return 0;
}

uint8_t ax5043_wakeup_deepsleep(void)
{
	{
		uint8_t i = ax5043_wakeup_deepsleep_core();
		if (i)
			return i;
	}
	if (ax5043_probeirq())
		return RADIO_ERR_IRQ;
	return 0;
}

uint8_t ax5043_reset(void)
{
	uint8_t i;
	// Reset Device
	ax5043_spi_write(AX5043_PWRMODE, 0x80);
	ax5043_spi_write(AX5043_PWRMODE, AX5043_PWRSTATE_POWERDOWN);
	// Check Scratch
	i = ax5043_spi_read(AX5043_REVISION);
	if (i != SILICONREV1)
		return RADIO_ERR_REVISION;
	ax5043_spi_write(AX5043_SCRATCH, 0x55);
	i = ax5043_spi_read(AX5043_SCRATCH);
	if (i != 0x55)
		return RADIO_ERR_COMM;
	ax5043_spi_write(AX5043_SCRATCH, 0xAA);
	i = ax5043_spi_read(AX5043_SCRATCH);
	if (i != 0xAA)
		return RADIO_ERR_COMM;
	if (ax5043_probeirq())
		return RADIO_ERR_IRQ;
	return RADIO_OK;
}

uint8_t ax5043_probeirq(void)
{
	uint8_t i[2] = {0, 0};
	__ax5043_disable_pin_irq();
	ax5043_spi_write(AX5043_PINFUNCIRQ, 0x01);
	i[0] = __ax5043_get_irq_pin_state();
	ax5043_spi_write(AX5043_PINFUNCIRQ, 0x00);
	i[1] = __ax5043_get_irq_pin_state();
	ax5043_spi_write(AX5043_PINFUNCIRQ, 0x03);

	if (!i[0] || i[1])
	{
		/* Error */
		ax5043_spi_write(AX5043_PINFUNCIRQ, 0x02);
		__ax5043_enable_pin_irq();
		return 1;
	}

	__ax5043_enable_pin_irq();
	return 0;
}

void ax5043_spi_read_fifo(uint8_t *ptr, uint8_t len)
{
	uint8_t spi_tx_buf[10];
	__ax5043_disable_pin_irq();
	__spi_cs_set(0);

	spi_tx_buf[0] = AX5043_FIFODATA;
	__spi_tx(spi_tx_buf, 1);
	while(len--)
		__spi_rx(ptr++, 1);

	__spi_cs_set(1);
	__ax5043_enable_pin_irq();
}


void ax5043_spi_write_fifo(uint8_t *ptr, uint8_t len)
{
	uint8_t spi_tx_buf[10];
	__ax5043_disable_pin_irq();
	__spi_cs_set(0);

	spi_tx_buf[0] = AX5043_FIFODATA | 0x80;
	__spi_tx(spi_tx_buf, 1);
	while(len--)
		__spi_tx(ptr++, 1);

	__spi_cs_set(1);
	__ax5043_enable_pin_irq();
}


uint8_t ax5043_spi_write(uint32_t add, uint8_t data)
{
	uint8_t spi_tx_buf[10], spi_rx_buf[10];
	__ax5043_disable_pin_irq();

	if (add >= 0x70)
	{
		add |= 0x7000;
		__spi_cs_set(0);
		spi_tx_buf[0] = (add >> 8) | 0x80;
		spi_tx_buf[1] = add & 0xFF;
		spi_tx_buf[2] = data;
		__spi_tx_rx(spi_tx_buf, spi_rx_buf, 3);
		__spi_cs_set(1);
	}
	else
	{
		__spi_cs_set(0);
		spi_tx_buf[0] = add | 0x80;
		spi_tx_buf[1] = data;
		__spi_tx_rx(spi_tx_buf, spi_rx_buf, 2);
		__spi_cs_set(1);
	}
	__ax5043_enable_pin_irq();
	return 1;
}

uint8_t ax5043_spi_read(uint32_t add)
{
	uint8_t spi_tx_buf[10], spi_rx_buf[10];
	__ax5043_disable_pin_irq();
	if (add >= 0x70)
	{
		add |= 0x7000;
		__spi_cs_set(0);
		spi_tx_buf[0] = add >> 8;
		spi_tx_buf[1] = add & 0xFF;
		spi_tx_buf[2] = 0x00;
		__spi_tx_rx(spi_tx_buf, spi_rx_buf, 3);
		__spi_cs_set(1);
		__ax5043_enable_pin_irq();
		return spi_rx_buf[2];
	}
	else
	{
		__spi_cs_set(0);
		spi_tx_buf[0] = add;
		spi_tx_buf[1] = 0x00;
		__spi_tx_rx(spi_tx_buf, spi_rx_buf, 2);
		__spi_cs_set(1);
		__ax5043_enable_pin_irq();
		return spi_rx_buf[1];
	}
}


uint16_t ax5043_spi_read16(uint32_t add)
{
	uint8_t spi_tx_buf[10], spi_rx_buf[10];
	__ax5043_disable_pin_irq();
	if (add >= 0x70)
	{
		add |= 0x7000;
		__spi_cs_set(0);
		spi_tx_buf[0] = add >> 8;
		spi_tx_buf[1] = add & 0xFF;
		spi_tx_buf[2] = 0x00;
		spi_tx_buf[3] = 0x00;
		__spi_tx_rx(spi_tx_buf, spi_rx_buf, 4);
		__spi_cs_set(1);
		__ax5043_enable_pin_irq();
		return (uint16_t)spi_rx_buf[2] << 8 | (uint16_t)spi_rx_buf[3];
	}
	else
	{
		__spi_cs_set(0);
		spi_tx_buf[0] = add;
		spi_tx_buf[1] = 0x00;
		spi_tx_buf[2] = 0x00;
		__spi_tx_rx(spi_tx_buf, spi_rx_buf, 3);
		__spi_cs_set(1);
		__ax5043_enable_pin_irq();
		return (uint16_t)spi_rx_buf[1] << 8 | (uint16_t)spi_rx_buf[2];
	}
}

uint32_t ax5043_spi_read24(uint32_t add)
{
	uint8_t spi_tx_buf[10], spi_rx_buf[10];
	__ax5043_disable_pin_irq();
	if (add >= 0x70)
	{
		add |= 0x7000;
		__spi_cs_set(0);
		spi_tx_buf[0] = add >> 8;
		spi_tx_buf[1] = add & 0xFF;
		spi_tx_buf[2] = 0x00;
		spi_tx_buf[3] = 0x00;
		spi_tx_buf[4] = 0x00;
		__spi_tx_rx(spi_tx_buf, spi_rx_buf, 5);
		__spi_cs_set(1);
		__ax5043_enable_pin_irq();
		return (uint32_t)spi_rx_buf[2] << 16 | (uint32_t)spi_rx_buf[3]  << 8 | (uint32_t)spi_rx_buf[4];
	}
	else
	{
		__spi_cs_set(0);
		spi_tx_buf[0] = add;
		spi_tx_buf[1] = 0x00;
		spi_tx_buf[2] = 0x00;
		spi_tx_buf[3] = 0x00;
		__spi_tx_rx(spi_tx_buf, spi_rx_buf, 4);
		__spi_cs_set(1);
		__ax5043_enable_pin_irq();
		return (uint32_t)spi_rx_buf[1] << 16 | (uint32_t)spi_rx_buf[2]  << 8 | (uint32_t)spi_rx_buf[3];
	}
}


void ax5043_tcxo_set_reset(uint8_t set)
{
	ax5043_spi_write(AX5043_PINFUNCPWRAMP, set);
}

void ax5043_hard_reset()
{
    if(__ax5043_on_off_pwr)
    {
        __ax5043_on_off_pwr(0);
        __ax5043_on_off_pwr(1);
    }
}

#ifdef FORMAT_CODE
//#pragma default_function_attributes = 
#endif
