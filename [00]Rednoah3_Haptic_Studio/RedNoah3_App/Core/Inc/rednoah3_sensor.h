/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file   : rednoah_sensor.h
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

#ifndef __REDNOAH_SENSOR_H
#define __REDNOAH_SENSOR_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "stm32f4xx_hal.h"

  void si2c_init(void);
  static void idelay(uint32_t tmout);
  static void asm_delay(void);
  uint32_t si2c_write_task(uint8_t ch, uint8_t id, uint8_t *dat, uint16_t size, uint16_t tmout);
  uint32_t si2c_read_task(uint8_t ch, uint8_t id, uint8_t *dat, uint16_t size, uint16_t tmout);
  uint32_t si2c_pin_state(void);
  uint8_t i2c_detect(uint8_t ch, uint8_t *dev);

#ifdef __cplusplus
}
#endif

#endif /* __REDNOAH_SENSOR_H */
