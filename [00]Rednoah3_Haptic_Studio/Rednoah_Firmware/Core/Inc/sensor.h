/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file   : Sensor_i2c.h
  * @brief  : Header for i2c_gpio.c file.
  This file contains the common defines of the application.
  * @version  : 0.0.1
  * @creaor   : magi8051
  * @update	  : 21.08.12
  ******************************************************************************
  * @attention  : Design for BlueBerry Board
  *
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __SI2C_GPIO_H
#define __SI2C_GPIO_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "stm32f4xx_hal.h"

  static void idelay(uint32_t tmout);
  static void asm_delay(void);
  void si2c_init(void);
  uint32_t si2c_write_task(uint8_t ch, uint8_t id, uint8_t *dat, uint16_t size, uint16_t tmout);
  uint32_t si2c_read_task(uint8_t ch, uint8_t id, uint8_t *dat, uint16_t size, uint16_t tmout);
  uint32_t si2c_pin_state(void);
  uint8_t i2c_detect(uint8_t ch, uint8_t *dev);

#ifdef __cplusplus
}
#endif

#endif /* __SI2C_GPIO_H */
