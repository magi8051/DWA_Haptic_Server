/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file   : rednoah_i2c.h
  * @brief  : Header for i2c_gpio.c file.
  This file contains the common defines of the application.
  * @version  : 0.0.1
  * @creaor   : magi8051
  * @update	  : 23.04.18
  ******************************************************************************
  * @attention  : Design for BlueBerry Board
  *
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __REDNOAH_I2C_H
#define __REDNOAH_I2C_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "stm32f4xx_hal.h"

  void init_i2c(uint32_t type, uint32_t ch);
  static void idelay(uint32_t tmout);
  uint32_t init_hs_i2c(uint8_t dat, uint32_t tmout);
  uint32_t i2c_write_task(uint8_t id, uint16_t addr, uint16_t type, uint8_t *dat, uint16_t size, uint16_t tmout);
  uint32_t i2c_read_task(uint8_t id, uint16_t addr, uint16_t type, uint8_t *dat, uint16_t size, uint16_t tmout);
  uint32_t new_i2c_write_task(uint8_t id, uint8_t *dat, uint16_t size, uint16_t tmout);
  uint32_t new_i2c_read_task(uint8_t id, uint8_t *dat, uint16_t size, uint16_t tmout);
  uint8_t i2c_pin_state(void);

#ifdef __cplusplus
}
#endif

#endif /* __REDNOAH_I2C_H */
