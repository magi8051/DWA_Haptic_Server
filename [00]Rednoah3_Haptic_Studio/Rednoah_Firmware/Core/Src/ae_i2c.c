/**
 ******************************************************************************
 * @file		: ae_i2c.c
 * @version	: 220712A
 * @creaor		: magi8051
 ******************************************************************************
 * @attention	: Design for Haptic RedNoah3 Board
 * 2022.07.12	: i2c code change for time margin
 ******************************************************************************
 * Do Not Change!! *
 */

#include "stm32f4xx_hal.h"
#include "ae_i2c.h"

/*user free define code*/
GPIO_TypeDef *I2C_BASS;
uint16_t SCL_PIN;
uint16_t SDA_PIN;

/*can not redefine!!*/
#define SDA(in) I2C_BASS->BSRR = (in) ? SDA_PIN : SDA_PIN << 16
#define SCL(in) I2C_BASS->BSRR = (in) ? SCL_PIN : SCL_PIN << 16

/*ACK:"0", NoACK:"1"*/
#define I2C_NACK (I2C_BASS->IDR & SDA_PIN) ? 1 : 0
#define I2C_DAT (I2C_BASS->IDR & SDA_PIN) ? 1 : 0

/*Select i2c for the channel you want for board.*/
void init_i2c(uint32_t type, uint32_t ch)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	I2C_BASS = GPIOB;

	if (ch == 1)
	{
		if (type == 0x00)
		{ // rednoah3
			SCL_PIN = GPIO_PIN_8;
			SDA_PIN = GPIO_PIN_7;
		}
	}
	else
	{
		I2C_BASS = GPIOD;
		SCL_PIN = GPIO_PIN_8;
		SDA_PIN = GPIO_PIN_9;
	}

	/*Configure I2Cx Port */
	GPIO_InitStruct.Pin = SCL_PIN | SDA_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(I2C_BASS, &GPIO_InitStruct);
	HAL_GPIO_WritePin(I2C_BASS, SDA_PIN | SCL_PIN, GPIO_PIN_SET);
}

/* design for i2c delay (5cycle) */
static void asm_delay(void)
{
	__asm volatile("NOP");
}

static void idelay(volatile uint32_t tmout)
{

	for (volatile int i = 0; i < tmout; i++)
		;
}

uint32_t init_hs_i2c(uint8_t dat, uint32_t tmout)
{
	uint8_t tmp;
	uint8_t ack;

	// Start
	SDA(0);
	idelay(tmout);
	tmp = 0;
	ack = 0;
	SCL(0);
	tmp = dat;

	for (int i = 0; i < 8; i++)
	{
		SDA((tmp & 0x80) ? 1 : 0);
		idelay(tmout);
		SCL(1);
		idelay(tmout);
		SCL(0);
		tmp <<= 1;
	}

	// No ACK Event
	idelay(tmout);
	SDA(1); // Set High-z
	idelay(tmout);
	SCL(1);
	idelay(tmout);
	SCL(0);
	idelay(tmout);

	// no stop function
	idelay(tmout);
	SCL(1);
	idelay(tmout);
	SDA(1);

	return ack;
}

uint32_t new_i2c_write_task(uint8_t id, uint8_t *dat, uint16_t size, uint16_t tmout)
{
	uint8_t tmp, ack;

	/*START*/
	SDA(0);
	idelay(tmout); // 100ns ?
	SCL(0);
	ack = 0;
	tmp = id;

	/*Start ID*/
	for (int i = 0; i < 8; i++)
	{
		SDA((tmp & 0x80) ? 1 : 0);
		idelay(tmout);
		SCL(1);
		idelay(tmout);
		SCL(0);
		tmp <<= 1;
	}

	/*ACK*/
	SDA(1);
	idelay(tmout + 1); /* margin check */
	SCL(1);
	idelay(tmout);
	ack = I2C_NACK;
	SCL(0);

	while (size--)
	{
		tmp = (*dat++);
		for (int i = 0; i < 8; i++)
		{
			SDA((tmp & 0x80) ? 1 : 0);
			idelay(tmout);
			SCL(1);
			idelay(tmout);
			SCL(0);
			tmp <<= 1;
		}
		/*ACK*/
		SDA(1);
		idelay(tmout + 1); /* margin check */
		SCL(1);
		// ack = I2C_NACK;
		idelay(tmout);
		SCL(0);
	}

	/*STOP*/
	SDA(0);
	idelay(tmout + 1); /* margin check */
	SCL(1);
	idelay(tmout);
	SDA(1);

	return ack;
}

uint32_t new_i2c_read_task(uint8_t id, uint8_t *dat, uint16_t size, uint16_t tmout)
{
	uint8_t tmp, ack;

	/*START*/
	SDA(0);
	idelay(tmout);
	SCL(0);
	ack = 0;
	tmp = id + 1;

	/*R-ID*/
	for (int i = 0; i < 8; i++)
	{
		SDA((tmp & 0x80) ? 1 : 0);
		idelay(tmout);
		SCL(1);
		idelay(tmout);
		SCL(0);
		tmp <<= 1;
	}
	/*ACK*/
	SDA(1);
	idelay(tmout + 1); /* margin check */
	SCL(1);
	idelay(tmout);
	ack = I2C_NACK;
	SCL(0);
	tmp = 0;
	idelay(tmout);

	/*R-DATA*/
	while (size--)
	{ /*read sda pin*/
		for (int i = 0; i < 8; i++)
		{
			SCL(1);
			idelay(tmout);
			tmp |= I2C_DAT;
			SCL(0);
			//idelay(tmout + 1); /* margin check */

			if (i < 7)
			{
				tmp <<= 1;
				idelay(tmout + 1); /* margin check */
			}
		}
		(*dat++) = tmp;
		tmp = 0;

		if (size)
		{
			/*Force NoACK*/
			SDA(0);
			idelay(tmout); 
			SCL(1);
			idelay(tmout);
			SCL(0);
			//idelay(tmout);
			SDA(1);
			idelay(tmout); 
		}
		else
		{
			/*NoAck*/
			idelay(tmout);
			SCL(1);
			idelay(tmout);
			SCL(0);
		}
	}
	/*STOP*/
	SDA(0);
	idelay(tmout + 1); /* margin check */
	SCL(1);
	idelay(tmout);
	SDA(1);

	return 0;
}

uint8_t i2c_pin_state(void)
{
	uint8_t sda, scl;
	sda = (I2C_BASS->IDR & SDA_PIN) ? 1 : 0;
	scl = (I2C_BASS->IDR & SCL_PIN) ? 1 : 0;
	return (scl << 1) | sda;
}

uint32_t i2c_write_task(uint8_t id, uint16_t addr, uint16_t type, uint8_t *dat, uint16_t size, uint16_t tmout)
{
	uint8_t tmp, ack;

	/*START*/
	SDA(0);
	idelay(tmout); // 100ns ?
	SCL(0);
	ack = 0;
	tmp = id;

	/*Start ID*/
	for (int i = 0; i < 8; i++)
	{
		SDA((tmp & 0x80) ? 1 : 0);
		idelay(tmout);
		SCL(1);
		idelay(tmout);
		SCL(0);
		tmp <<= 1;
	}

	/*ACK*/
	SDA(1);
	idelay(tmout + 1); /* margin check */
	SCL(1);
	idelay(tmout);
	ack = I2C_NACK;
	SCL(0);

	/* Write Address */
	while (type)
	{
		tmp = addr >> ((1 - type) * 8);
		for (int i = 0; i < 8; i++)
		{
			SDA((tmp & 0x80) ? 1 : 0);
			idelay(tmout);
			SCL(1);
			idelay(tmout);
			SCL(0);
			tmp <<= 1;
		}
		/*ACK*/
		SDA(1);
		idelay(tmout + 1); /* margin check */
		SCL(1);
		// ack = I2C_NACK;
		idelay(tmout);
		SCL(0);
		type--;
	}

	/* Write Data */
	while (size--)
	{
		tmp = (*dat++);
		for (int i = 0; i < 8; i++)
		{
			SDA((tmp & 0x80) ? 1 : 0);
			idelay(tmout);
			SCL(1);
			idelay(tmout);
			SCL(0);
			tmp <<= 1;
		}
		/*ACK*/
		SDA(1);
		idelay(tmout + 1); /* margin check */
		SCL(1);
		// ack = I2C_NACK;
		idelay(tmout);
		SCL(0);
	}

	/*STOP*/
	SDA(0);
	idelay(tmout + 1); /* margin check */
	SCL(1);
	idelay(tmout);
	SDA(1);

	return ack;
}

uint32_t i2c_read_task(uint8_t id, uint16_t addr, uint16_t type, uint8_t *dat, uint16_t size, uint16_t tmout)
{
	uint8_t tmp, ack;

	/* Write Address */
	i2c_write_task(id, addr, type, dat, 0, tmout);

	/*START*/
	SDA(0);
	idelay(tmout);
	SCL(0);
	ack = 0;
	tmp = id + 1;

	/*R-ID*/
	for (int i = 0; i < 8; i++)
	{
		SDA((tmp & 0x80) ? 1 : 0);
		idelay(tmout);
		SCL(1);
		idelay(tmout);
		SCL(0);
		tmp <<= 1;
	}
	/*ACK*/
	SDA(1);
	idelay(tmout + 1); /* margin check */
	SCL(1);
	idelay(tmout);
	ack = I2C_NACK;
	SCL(0);
	tmp = 0;
	idelay(tmout);

	/*R-DATA*/
	while (size--)
	{ /*read sda pin*/
		for (int i = 0; i < 8; i++)
		{
			SCL(1);
			idelay(tmout);
			tmp |= I2C_DAT;
			SCL(0);
			//idelay(tmout + 1); /* margin check */

			if (i < 7)
			{
				tmp <<= 1;
				idelay(tmout + 1); /* margin check */
			}
		}
		(*dat++) = tmp;
		tmp = 0;

		if (size)
		{
			/*Force NoACK*/
			SDA(0);
			idelay(tmout);
			SCL(1);
			idelay(tmout);
			SCL(0);
			//idelay(tmout);
			SDA(1);
			idelay(tmout);
		}
		else
		{
			/*NoAck*/
			idelay(tmout);
			SCL(1);
			idelay(tmout);
			SCL(0);
		}
	}
	/*STOP*/
	SDA(0);
	idelay(tmout + 1); /* margin check */
	SCL(1);
	idelay(tmout);
	SDA(1);

	return 0;
}
