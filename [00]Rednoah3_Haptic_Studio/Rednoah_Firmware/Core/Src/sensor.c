/**
  ******************************************************************************
  * @file		: sensor.c
  * @version	: 23.01.13
  * @creaor		: magi8051

  ******************************************************************************
  * i2c update with ae_i2c
 ******************************************************************************
* Do Not Change!! *
*/

#include "stm32f4xx_hal.h"
#include "sensor.h"

/*user free define code*/
GPIO_TypeDef *I2C_BASS_SCL;
GPIO_TypeDef *I2C_BASS_SDA;

uint16_t GSCL;
uint16_t GSDA;

/*can not redefine!!*/
#define SDA(in) I2C_BASS_SDA->BSRR = (in) ? GSDA : GSDA << 16
#define SCL(in) I2C_BASS_SCL->BSRR = (in) ? GSCL : GSCL << 16

/*ACK:"0", NoACK:"1"*/
#define I2C_NACK (I2C_BASS_SDA->IDR & GSDA) ? 1 : 0
#define I2C_DAT (I2C_BASS_SDA->IDR & GSDA) ? 1 : 0

/*Select i2c for the channel you want for board.*/
void si2c_init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;

	/*
	I2C_SDA1 = PC10
	I2C_SCL1 = PC12
	I2C_SDA2 = PC11
	I2C_SCL2 = PA15
	I2C_SDA3 = PD0
	I2C_SCL3 = PD1
	*/

	/* Set I2C GPIO*/
	I2C_BASS_SDA = GPIOC;
	/*Configure I2Cx Port */
	GPIO_InitStruct.Pin = GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(I2C_BASS_SDA, &GPIO_InitStruct);
	HAL_GPIO_WritePin(I2C_BASS_SDA, GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12, GPIO_PIN_SET);

	I2C_BASS_SCL = GPIOA;
	/*Configure I2Cx Port */
	GPIO_InitStruct.Pin = GPIO_PIN_15;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(I2C_BASS_SCL, &GPIO_InitStruct);
	HAL_GPIO_WritePin(I2C_BASS_SCL, GPIO_PIN_15, GPIO_PIN_SET);

	I2C_BASS_SDA = GPIOD;
	/*Configure I2Cx Port */
	GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(I2C_BASS_SDA, &GPIO_InitStruct);
	HAL_GPIO_WritePin(I2C_BASS_SDA, GPIO_PIN_0 | GPIO_PIN_1, GPIO_PIN_SET);
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

uint32_t si2c_write_task(uint8_t ch, uint8_t id, uint8_t *dat, uint16_t size, uint16_t tmout)
{
	uint8_t tmp, ack;

	if (ch == 0)
	{
		I2C_BASS_SDA = GPIOC;
		I2C_BASS_SCL = GPIOC;
		GSDA = GPIO_PIN_10;
		GSCL = GPIO_PIN_12;
	}
	else if (ch == 1)
	{
		I2C_BASS_SDA = GPIOC;
		I2C_BASS_SCL = GPIOA;
		GSDA = GPIO_PIN_11;
		GSCL = GPIO_PIN_15;
	}
	else
	{
		I2C_BASS_SDA = GPIOD;
		I2C_BASS_SCL = GPIOD;
		GSDA = GPIO_PIN_0;
		GSCL = GPIO_PIN_1;
	}

	/*START*/
	SDA(0);
	idelay(tmout); // 100ns ?
	ack = 0;
	tmp = id;
	SCL(0);

	/*Start ID*/
	for (int i = 0; i < 8; i++)
	{
		SDA((tmp & 0x80) ? 1 : 0);
		idelay(tmout);
		SCL(1);
		idelay(tmout);
		tmp <<= 1;
		SCL(0);
	}

	/*ACK*/
	SDA(1);
	idelay(tmout);
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
			tmp <<= 1;
			SCL(0);
		}
		/*ACK*/
		SDA(1);
		idelay(tmout);
		SCL(1);
		idelay(tmout);
		SCL(0);
	}

	/*STOP*/
	SDA(0);
	idelay(tmout);
	SCL(1);
	idelay(tmout);
	SDA(1);

	return ack;
}

uint32_t si2c_read_task(uint8_t ch, uint8_t id, uint8_t *dat, uint16_t size, uint16_t tmout)
{
	uint8_t tmp, ack;

	if (ch == 0)
	{
		I2C_BASS_SDA = GPIOC;
		I2C_BASS_SCL = GPIOC;
		GSDA = GPIO_PIN_10;
		GSCL = GPIO_PIN_12;
	}
	else if (ch == 1)
	{
		I2C_BASS_SDA = GPIOC;
		I2C_BASS_SCL = GPIOA;
		GSDA = GPIO_PIN_11;
		GSCL = GPIO_PIN_15;
	}
	else
	{
		I2C_BASS_SDA = GPIOD;
		I2C_BASS_SCL = GPIOD;
		GSDA = GPIO_PIN_0;
		GSCL = GPIO_PIN_1;
	}

	/*START*/
	SDA(0);
	idelay(tmout);
	tmp = id + 1;
	SCL(0);

	/*R-ID*/
	for (int i = 0; i < 8; i++)
	{
		SDA((tmp & 0x80) ? 1 : 0);
		idelay(tmout);
		SCL(1);
		idelay(tmout);
		tmp <<= 1;
		SCL(0);
	}
	/*ACK*/
	SDA(1);
	idelay(tmout);
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

			if (i < 7)
			{
				tmp <<= 1;
				idelay(tmout);
			}
		}
		(*dat++) = tmp;
		tmp = 0;

		if (size)
		{
			SDA(0); /*Force NoACK*/
			idelay(tmout);
			SCL(1);
			idelay(tmout);
			SCL(0);
			SDA(1);
			idelay(tmout);
		}
		else
		{ /*Noack*/
			idelay(tmout);
			SCL(1);
			idelay(tmout);
			SCL(0);
		}
	}
	/*STOP*/
	SDA(0);
	idelay(tmout);
	SCL(1);
	idelay(tmout);
	SDA(1);

	return 0;
}

uint32_t si2c_pin_state(void)
{
	uint8_t sda, scl;
	sda = (I2C_BASS_SDA->IDR & GSDA) ? 1 : 0;
	scl = (I2C_BASS_SCL->IDR & GSCL) ? 1 : 0;
	return (scl << 1) | sda;
}

uint8_t i2c_detect(uint8_t ch, uint8_t *dev)
{
	uint8_t id_cnt = 0;

	for (uint8_t i = 2; i < 200; i += 2)
	{
		if (!si2c_write_task(ch, i, &i, 1, 20))
		{
			dev[id_cnt] = i;
			id_cnt++;
		}
	}

	return id_cnt;
}
