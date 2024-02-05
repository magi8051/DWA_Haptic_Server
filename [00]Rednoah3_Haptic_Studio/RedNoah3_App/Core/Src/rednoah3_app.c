/**
  ******************************************************************************
  * @file		: sensor.c
  * @version	: 23.01.04
  * @creaor		: magi8051

  ******************************************************************************
  * i2c update with ae_i2c
 ******************************************************************************
* Do Not Change!! *
*/

#include "stm32f4xx_hal.h"
#include "rednoah3_app.h"
#include "rednoah3_i2c.h"
#include "rednoah3_sensor.h"
#include "rednoah3_boot.h"
#include "hpatic_wave.h"
#include "main.h"

/*Base Struct for BBB*/
volatile struct bit_reg s_bit; // bit reigster
struct uart_packet s_upk;      // uart buffer
struct rtp_packet s_rtp;       // rtp buffer
uint8_t LDO_Value = 0x1F;

/*board info*/
u8 g_board_info;
u8 g_ic_info;

/* probe */
u8 g_dev[2];

/*boot pv*/
u32 g_flash_jmp;

/*i2c pv*/
u8 g_i2c_info[] = {I2C_50KHZ, I2C_100KHZ, I2C_400KHZ, I2C_1MHZ, I2C_1_2MHZ, I2C_2MHZ, I2C_3MHZ};
u8 g_i2c_clk;
u8 g_i2c_id;

/* spi pv */
u8 g_spi_mode;
u8 g_spi_clk;

/*timer pv*/
u16 g_tmr_psc;
u16 g_tmr_period;
u32 g_tmr_cnt_1ms;

/*comm mode*/
u16 comm_value = 0;

enum board_type
{
    REDNOAH2 = 0,
    REDNOAH3,
    REDNOAH4
};

enum fsm
{
    SYS_FSM = 0,
    I2C_FSM,
    RTP_FSM,
    REV_FSM,
    SCP_FSM,
    COMM_FSM // ��� ��� ����
};

enum dev
{
    DW7800 = 0,
    DW7802,
    DW7912,
    DW7914
};

/* Macro Data Conversion Function */
u16 _8u16(u8 *in) { return (u16)(((u16)in[0] << 8) | in[1]); }

u32 _8u32(u8 *in) { return (u32)((((u32)in[0] << 24) | ((u32)in[1] << 16) | ((u32)in[2] << 8) | ((u32)in[3] << 0))); }

s16 _twos(u16 in)
{ // two's complement
    s16 ret;
    if (in > 32767)
        ret = (s16)(in - 65536);
    else
        ret = in;
    return ret;
}

float _fp(u32 in, u32 num)
{
    return (float)in / (float)num;
}

void _16u8(u16 *_u16, u8 *_u8)
{
    *(_u8 + 0) = *(_u16 + 0) >> 8;
    *(_u8 + 1) = *(_u16 + 0) >> 0;
}

#if 0
static void delay_us(int t)
{
    for (volatile int k = 0; k < 10; k++)
    {
        for (volatile int i = 0; i < t - 1; i++)
        {
            __asm volatile("NOP");
            __asm volatile("NOP");
            __asm volatile("NOP");
            __asm volatile("NOP");
            __asm volatile("NOP");
            __asm volatile("NOP");
        }
    }
}
#endif

#if 0
static void MX_SPI5_Init_odw(void)
{

    /* USER CODE BEGIN SPI5_Init 0 */

    /* USER CODE END SPI5_Init 0 */

    /* USER CODE BEGIN SPI5_Init 1 */

    /* USER CODE END SPI5_Init 1 */
    /* SPI5 parameter configuration*/
    hspi5.Instance = SPI5;
    hspi5.Init.Mode = SPI_MODE_MASTER;
    hspi5.Init.Direction = SPI_DIRECTION_2LINES;
    hspi5.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi5.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi5.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi5.Init.NSS = SPI_NSS_SOFT;
    hspi5.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
    hspi5.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi5.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi5.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi5.Init.CRCPolynomial = 10;

    if (HAL_SPI_Init(&hspi5) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN SPI5_Init 2 */
    hspi5.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
    hspi5.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi5.Init.CLKPhase = SPI_PHASE_2EDGE;

    if (HAL_SPI_Init(&hspi5) != HAL_OK)
    {
        Error_Handler();
    }

    /* USER CODE END SPI5_Init 2 */
}
#endif

static void idelay_spi(volatile uint32_t tmout)
{

    for (volatile int i = 0; i < tmout; i++)
        ;
}

/***************************************************************************
Function	: uart_transfer_task
Version		: 1.0
Descript 	:
***************************************************************************/
void uart_transfer_task(u32 tx_size)
{
    /*Default Header. Do not change!*/
    s_upk.buf[0] = 0xAE;
    s_upk.buf[1] = s_upk.hw;
    s_upk.buf[2] = tx_size >> 8;
    s_upk.buf[3] = tx_size >> 0;
    s_upk.buf[4] = 0;
    s_upk.buf[5] = s_upk.echo;

    for (int i = 0; i < tx_size; i++)
    {
        s_upk.buf[4] += s_upk.buf[6 + i];
    }

    LED5_T;
    HAL_UART_Transmit_DMA(&huart1, (u8 *)s_upk.buf, tx_size + BHD6);
    // HAL_UART_Transmit_IT(&huart1,(u8*)s_upk.buf, tx_size + BHD6);
}

static void delay_us(int t)
{
    for (volatile int k = 0; k < 10; k++)
    {
        for (volatile int i = 0; i < t; i++)
        {
            __asm volatile("NOP");
            __asm volatile("NOP");
            __asm volatile("NOP");
            __asm volatile("NOP");
            __asm volatile("NOP");
            __asm volatile("NOP");
        }
    }
}

static void device_info_request(void)
{
    volatile u8 pin, size, i;
    u32 pid[3];

    i = 0;
    pin = i2c_pin_state();

    /*step: sw version*/
    s_upk.buf[6] = 0;
    s_upk.buf[7] = 0;
    s_upk.buf[8] = (u8)(REDNOAH_FW_INFO >> 24);
    s_upk.buf[9] = (u8)(REDNOAH_FW_INFO >> 16);
    s_upk.buf[10] = (u8)(REDNOAH_FW_INFO >> 8);
    s_upk.buf[11] = (u8)(REDNOAH_FW_INFO >> 0);

    /*step: serial number*/
    pid[0] = HAL_GetUIDw0();
    pid[1] = HAL_GetUIDw1();
    pid[2] = HAL_GetUIDw2();

    for (int i = 0; i < 3; i++)
    {
        s_upk.buf[12 + i * 4] = (u8)(pid[i] >> 24);
        s_upk.buf[13 + i * 4] = (u8)(pid[i] >> 16);
        s_upk.buf[14 + i * 4] = (u8)(pid[i] >> 8);
        s_upk.buf[15 + i * 4] = (u8)(pid[i] >> 0);
    }

    /* i2c pin setting error */
    if (pin != 3)
    {
        s_upk.buf[24] = 0xff;
        s_upk.buf[25] = pin;
        s_upk.buf[26] = 0;
        s_upk.buf[27] = 0;
        size = 19;
    }
    else
    {
        s_upk.buf[24] = 0;
        for (g_i2c_id = 0; g_i2c_id < 200; g_i2c_id += 2)
        {
            if (i2c_write_task(g_i2c_id, 0, 1, &g_i2c_id, 1, I2C_400KHZ) == I2C_ACK)
            {
                s_upk.buf[24]++;
                s_upk.buf[25 + i] = g_i2c_id;
                s_upk.buf[26 + i] = i2c_8bit_r(0x00);
                i += 2;
            }
        }
        size = 19 + s_upk.buf[24] * 2;
    }

    uart_transfer_task(size);
}

/***************************************************************************
Function	: trig pin control
Version		: 1.0
Descript 	: io 1 ~ 4
***************************************************************************/
static void LDO_controlup(int count) // odw MCP4011
{

    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET); // UD PIN HIGH
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_SET); // CS PIN HIGH
    delay_us(10);                                       // DELAY

    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_RESET); // CS PIN LOW
    delay_us(3);                                          // DELAY

    for (int i = 0; i < count; i++)
    {
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET); // UD PIN LOW
        delay_us(2);                                          // DELAY
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);   // UD PIN HIGH
        delay_us(2);                                          // DELAY
    }
    delay_us(10);                                       // DELAY
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_SET); // CS PIN HIGH
}

static void LDO_controldown(int count) // odw MCP4011
{

    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_SET);   // CS PIN HI
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET); // UD PIN LOW
    delay_us(10);                                         // DELAY

    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_RESET); // CS PIN LOW
    delay_us(3);                                          // DELAY
    for (int i = 0; i < count; i++)
    {
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET); // UD PIN HIGH
        delay_us(2);                                        // DELAY

        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET); // UD PIN LOW
        delay_us(2);                                          // DELAY
    }

    delay_us(10);                                       // DELAY
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_SET); // CS PIN HIGH
}

static void LDO_control() // odw MCP4011
{

    if (s_upk.buf[8] == 0x00) // write
    {
        LDO_Value = s_upk.buf[9];
        LDO_controldown(64);
        LDO_controlup((int)(s_upk.buf[9]));
    }

    else // read
    {
        s_upk.buf[9] = LDO_Value;
    }
    uart_transfer_task(4);
}

static void trig_ctrl_task(int setup)
{
    u32 pin_temp;
    u8 temp, pin_num;
    static u8 tn, time_num;
    static u32 pin[8]; //, pin_cnt;
    static u16 delay_cnt, time_elapsed, time[8], time_temp;

    if (!setup)
    {
        /*init reg*/
        tn = 0;
        delay_cnt = 0;
        time_elapsed = 0;
        memset(time, 0, sizeof(time));
        memset(pin, 0, sizeof(pin));

        pin_num = s_upk.buf[8];
        time_num = pin_num * 2;
        for (int i = 0; i < pin_num; i++)
        {
            pin[i] = GPIO_PIN_0 << s_upk.buf[9 + i * 5];
            pin[i + pin_num] = (uint32_t)GPIO_PIN_0 << (16 + s_upk.buf[9 + i * 5]);
            time[i] = _8u16(s_upk.buf + (10 + i * 5));
            time[i + pin_num] = _8u16(s_upk.buf + (12 + i * 5)) + time[i];
        }

        /* sort array */
        for (int i = 0; i < time_num; i++)
        {
            for (int j = 0; j < time_num - 1; j++)
            {
                if (time[j] > time[j + 1])
                {
                    pin_temp = pin[j];
                    pin[j] = pin[j + 1];
                    pin[j + 1] = pin_temp;

                    time_temp = time[j];
                    time[j] = time[j + 1];
                    time[j + 1] = time_temp;
                }
            }
        }

        /* remove duplicats time */
        for (int i = 0; i < time_num - 1; i++)
        {
            temp = time_num;
            for (int j = i + 1; j < temp; j++)
            {
                if (time[i] == time[i + 1])
                {
                    time_num--;
                    pin[i] |= pin[i + 1];
                    for (int k = i + 1; k < time_num; k++)
                    {
                        pin[k] = pin[k + 1];
                        time[k] = time[k + 1];
                    }
                }
                else
                {
                    break;
                }
            }
        }

        delay_cnt = time[tn] - time_elapsed;
        time_elapsed = time[tn];
        s_bit.trig_act = 1;
    }
    else
    {
        if (s_bit.trig_act)
        {
            if (delay_cnt > 1)
            {
                delay_cnt--;
            }
            else
            {
                GPIOA->BSRR = pin[tn];
                tn++;
                delay_cnt = time[tn] - time_elapsed;
                time_elapsed = time[tn];

                if (tn >= time_num) /* finish */
                {
                    s_bit.trig_act = 0;
                }
            }
            // HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_3);
        }
    }
}

/***************************************************************************
Function	: gpio_ctrl_task
Version		: 1.0
Descript 	: chip info, reset, who am i
***************************************************************************/
static void gpio_ctrl_task(int rw)
{
    GPIO_TypeDef *PORT;
    uint16_t pin[] = {GPIO_PIN_0, GPIO_PIN_1, GPIO_PIN_2, GPIO_PIN_3,
                      GPIO_PIN_4, GPIO_PIN_5, GPIO_PIN_7, GPIO_PIN_9,
                      GPIO_PIN_8, GPIO_PIN_6, GPIO_PIN_8, GPIO_PIN_9};

    if (rw == 0)
    {
        if (s_upk.buf[9] <= 5)
        {
            PORT = GPIOA;
            s_upk.buf[10] = HAL_GPIO_ReadPin(PORT, pin[s_upk.buf[9]]);
        }
        else if (s_upk.buf[9] <= 9)
        {
            PORT = GPIOF;
            s_upk.buf[10] = HAL_GPIO_ReadPin(PORT, pin[s_upk.buf[9]]);
        }
        else if (s_upk.buf[9] <= 11)
        {
            PORT = GPIOD;
            s_upk.buf[10] = HAL_GPIO_ReadPin(PORT, pin[s_upk.buf[9]]);
        }
    }
    else
    {
        if (s_upk.buf[9] <= 5)
        {
            PORT = GPIOA;
            HAL_GPIO_WritePin(PORT, pin[s_upk.buf[9]], (GPIO_PinState)s_upk.buf[10]);
        }
        else if (s_upk.buf[9] <= 9)
        {
            PORT = GPIOF;
            HAL_GPIO_WritePin(PORT, pin[s_upk.buf[9]], (GPIO_PinState)s_upk.buf[10]);
        }
        else if (s_upk.buf[9] <= 11)
        {
            PORT = GPIOD;
            HAL_GPIO_WritePin(PORT, pin[s_upk.buf[9]], (GPIO_PinState)s_upk.buf[10]);
        }
    }
    uart_transfer_task(5);
}

/***************************************************************************
Function	: board_set_task
Version		: 1.0
Descript 	: chip info, reset, who am i
***************************************************************************/
static void board_set_task(void)
{
    switch (s_upk.buf[7])
    {
    case 0x00: /*board info */
        device_info_request();
        break;

    case 0x01: /*device info*/
        g_ic_info = s_upk.buf[8];
        g_i2c_id = s_upk.buf[9];
        if (s_upk.buf[5])
            uart_transfer_task(1); // echo
        break;

    case 0x02: /*board reset */
        REDNOAH_RESET;
        break;

    case 0x03: /*ldo control */
        ADJ_LDO_EN(s_upk.buf[8]);
        break;

    case 0x04: /* find auto i2c id */
        s_upk.buf[8] = 0;
        for (u8 id = 1; id < 119; id++)
        {
            if (i2c_write_task(id << 1, 0, 1, &id, 1, I2C_400KHZ) == I2C_ACK)
            {
                s_upk.buf[9 + s_upk.buf[8]] = id << 1;
                s_upk.buf[8]++;
            }
        }
        uart_transfer_task(3 + s_upk.buf[8]); // do not change!
        break;

    case 0x05: /* Trigger pin control */
        trig_ctrl_task(0);
        break;

    case 0x06: /* GPIO Control */
        gpio_ctrl_task(s_upk.buf[8]);
        break;

    case 0x07: /* LDO control */
        LDO_control();
        break;

    case 0xFA: /*firmware update*/
        if (s_upk.buf[8] == 0x01)
        {
            bootloader_update_task();
            jump_vector_table();
        }
        break;
    }
}

/***************************************************************************
Function  : sys_timer_set
Version		: 1.0
Descript 	: timer control tick base is usec
***************************************************************************/
static void sys_timer_set(u32 timer, u32 mode, u32 usec)
{
    switch (timer)
    {
    case TIMER_10:
        if (mode == PLAY)
        {
            /*APB2_180Mhz*/
            s_bit.timer10_act = 1;
            htim10.Instance = TIM10;
            htim10.Init.Prescaler = 180 - 1;
            htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
            htim10.Init.Period = usec - 1;
            htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
            htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
            HAL_TIM_Base_Init(&htim10);
            HAL_TIM_Base_Start_IT(&htim10);
        }
        else
        {
            s_bit.timer10_act = 0;
            HAL_TIM_Base_Stop_IT(&htim10);
        }
        break;

    case TIMER_7:
        if (mode == PLAY)
        {
            s_bit.timer7_act = 1;
            TIM_MasterConfigTypeDef sMasterConfig = {0};
            htim7.Instance = TIM7;
            htim7.Init.Prescaler = 90 - 1;
            htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
            htim7.Init.Period = usec - 1;
            htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
            HAL_TIM_Base_Init(&htim7);
            sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
            sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
            HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig);
            HAL_TIM_Base_Start_IT(&htim7);
        }
        else
        {
            s_bit.timer7_act = 0;
            HAL_TIM_Base_Stop_IT(&htim7);
        }
        break;
    }
}

/***************************************************************************
Function	: init_rednoah_platform_board
Version		: 1.0
Descript 	: init function
***************************************************************************/
void init_redhoah3_system(void)
{
    int n;
    /*LDO power on*/
    ADJ_LDO_EN(1); /* ADJ LDO Enable */

    /*Auto board check*/
    s_upk.hw = (BOOT_ID2 << 2) | (BOOT_ID1 << 1) | (BOOT_ID0 << 0);

    /*gpio i2c init start*/
    g_i2c_clk = g_i2c_info[3]; /* default 1mhz */
    init_i2c(0, 1);            /* rednoah default, i2c ch 1 */

    /* Accel sensor i2c init */
    si2c_init();

    /*AXL Sensor*/
    SPI3_NSS0(1);
    SPI3_NSS1(1);
    SPI3_NSS2(1);

    /*uart get start*/
    HAL_UART_Receive_IT(&huart1, &s_upk.get, 1);
    // HAL_UART_Receive_DMA(&huart1, &s_upk.get, 1);

    /* DW791x Module Board Init */
    IO10(1); /* DW7912 / DW7802 Enable */
    IO11(0); /* DW7914 / DW7917 Disable */

    /* for handshake */
    // device_info_request();

    /* probe haptic ic */
    g_i2c_id = 0xB2;
    u8 tx[2];
    if (i2c_8bit_r(0x00) == 0x32)
    {
        /* DW7912 set*/
        g_dev[0] = DW7912;
        i2c_8bit_w(0x03, 0x01); // mem mode
        i2c_8bit_w(0x04, 0x05); // user vd-clamp
        i2c_8bit_w(0x08, 0x53); // vd-clamp 3.3v
        i2c_8bit_w(0x0C, 0x01); // mem call number 1
        i2c_8bit_w(0x14, 0x04); // loop x 5

        /* write header*/
        tx[0] = 0x00;
        tx[1] = 0x01;
        i2c_write_task(g_i2c_id, 0x1B, I2C_8BIT, tx, 2, I2C_1MHZ);
        i2c_write_task(g_i2c_id, 0x1D, I2C_8BIT, (u8 *)g_haptic_hd, sizeof(g_haptic_hd), I2C_1MHZ);
        /* write body */
        tx[0] = 0x02;
        tx[1] = 0x7C;
        i2c_write_task(g_i2c_id, 0x1B, I2C_8BIT, tx, 2, I2C_1MHZ);
        i2c_write_task(g_i2c_id, 0x1D, I2C_8BIT, (u8 *)g_haptic_body, sizeof(g_haptic_body), I2C_1MHZ);
    }
    else if (i2c_8bit_r(0x00) == 0x40)
    {
        /* DW7914 */
        g_dev[0] = DW7914;
        i2c_8bit_w(0x0B, 0x01); // mem mode
        i2c_8bit_w(0x04, 0x05); // vd-clamp mode
        i2c_8bit_w(0x0A, 0x53); // vd 3.3V
        i2c_8bit_w(0x0F, 0x01); // mem num 1
        i2c_8bit_w(0x17, 0x04); // loop x 5

        /* write header*/
        tx[0] = 0x00;
        tx[1] = 0x01;
        i2c_write_task(g_i2c_id, 0x46, I2C_8BIT, tx, 2, I2C_1MHZ);
        i2c_write_task(g_i2c_id, 0x48, I2C_8BIT, (u8 *)g_haptic_hd, sizeof(g_haptic_hd), I2C_1MHZ);
        /* write body */
        tx[0] = 0x02;
        tx[1] = 0x7C;
        i2c_write_task(g_i2c_id, 0x46, I2C_8BIT, tx, 2, I2C_1MHZ);
        i2c_write_task(g_i2c_id, 0x48, I2C_8BIT, (u8 *)g_haptic_body, sizeof(g_haptic_body), I2C_1MHZ);
    }

    /* SYS LED display*/
    for (int i = 0; i < 4; i++)
    {
        n = i % 2;
        LED1(n);
        HAL_Delay(100);
        LED2(n);
        HAL_Delay(100);
        LED3(n);
        HAL_Delay(100);
    }

    /*boot messsage to PC*/
    sys_timer_set(TIMER_10, PLAY, 1000); /* systick 1ms */

    /*LDO Reset*/
    LDO_controldown(65);
    LDO_controlup(31);
}

/***************************************************************************
Function	: tick_led_task
Version		: 1.0
Descript 	: led blink tick 1ms
***************************************************************************/
static void sys_led_task(void)
{
    volatile static u32 t1, t2, n;

    t1++;
    g_tmr_cnt_1ms++;

    if (t1 > 200)
    {
        t1 = 0;
        LED4_T;
    }

    if (s_bit.rtp_act)
    {
        if (t2 > 100)
        {
            n++;
            t2 = 0;
            if (n == 1)
            {
                LED1(LEDON);
                LED2(LEDOF);
                LED3(LEDOF);
            }
            else if (n == 2)
            {
                LED1(LEDOF);
                LED2(LEDON);
                LED3(LEDOF);
            }
            else if (n == 3)
            {
                n = 0;
                LED1(LEDOF);
                LED2(LEDOF);
                LED3(LEDON);
            }
        }
        else
        {
            t2++;
        }
    }
}

/***************************************************************************
Function	: sys_key_task
Version		: 1.0
Descript 	: polling mode 10ms
***************************************************************************/
static void sys_key_task(void)
{
    volatile static u8 t1, t2, t3;
    volatile static u32 lock;

    if (!KEY1 || !KEY2 || !KEY3)
    {
        if (!KEY1 && !lock)
        {
            t1 = 20;
            lock = 1;
            /* user code here!! */
            if (g_dev[0] == DW7912)
            {
                i2c_8bit_w(0x09, 0x01);
            }
            else if (g_dev[0] == DW7914)
            {
                i2c_8bit_w(0x0C, 0x01);
            }
        }
        else if (!KEY2 && !lock)
        {
            t2 = 20;
            lock = 1;
            /* user code here!! */
        }
        else if (!KEY3 && !lock)
        {
            t3 = 20;
            lock = 1;
            /* user code here!! */
        }
    }
    else
    {
        if (t1)
        {
            t1--;
            (t1 == 0) ? LED1(LEDOF) : LED1(LEDON);
        }
        else if (t2)
        {
            t2--;
            (t2 == 0) ? LED2(LEDOF) : LED2(LEDON);
        }
        else if (t3)
        {
            t3--;
            (t3 == 0) ? LED3(LEDOF) : LED3(LEDON);
        }
        lock = 0;
    }
}

static void comm_set_task(void)
{
    if (s_upk.buf[7] == 0) // read
    {
        s_upk.buf[7] = comm_value;
        uart_transfer_task(2);
    }
    else // write
    {
        if (s_upk.buf[8] == 0)
            comm_value = 0; // i2c mode
        else
            comm_value = 1; // SPI mode

        s_upk.buf[7] = comm_value;
        uart_transfer_task(2);
    }
}

/***************************************************************************
Function	: i2c task
Version		: 1.2
Descript 	: i2c r/w event
***************************************************************************/
static void i2c_task(void)
{
    u16 size;
    static u8 scope;
    u16 type, dsize, asize;

    if (comm_value == 0)
    { // i2c mode

        if (s_bit.scope_act)
        {
            scope = 1;
            s_bit.scope_act = 0;
        }

        // id = g_i2c_id;
        s_bit.i2c_act = 1;
        type = s_upk.buf[7] & 0x10;
        asize = s_upk.buf[7] & 0x0f;
        /* clk set */
        g_i2c_clk = g_i2c_info[s_upk.buf[8]];
        // clk = g_i2c_clk;

        dsize = _8u16(s_upk.buf + 9);
        g_i2c_id = s_upk.buf[11];

        if (type == 0x10) /* i2c read mode */
        {
            if (new_i2c_write_task(g_i2c_id, (u8 *)s_upk.buf + 12, asize, g_i2c_clk) != I2C_ACK)
            {
                s_upk.buf[7] = 0xff;
                uart_transfer_task(2);
            }
            else
            {
                new_i2c_read_task(g_i2c_id, (u8 *)s_upk.buf + (12 + asize), dsize, g_i2c_clk);
                uart_transfer_task(BHD6 + asize + dsize);
            }
        }
        else /* i2c write mode */
        {
            if (new_i2c_write_task(g_i2c_id, (u8 *)s_upk.buf + 12, dsize + asize, g_i2c_clk) != I2C_ACK)
            {
                s_upk.buf[7] = 0xff;
            }
            else
            {
                s_upk.buf[7] = 0x00;
            }
            uart_transfer_task(2);
        }

        // g_i2c_clk = clk;
        // g_i2c_id = id;
        s_bit.i2c_act = 0;
        s_bit.scope_act = scope;
        scope = 0;
    }
    else
    {
        // SPI mode

        type = s_upk.buf[7] & 0x10;
        //u16 size;
        size = (*(s_upk.buf + 9) << 8) + *(s_upk.buf + 10) + 1;
        u16 response_size;

        IO9(0);
        idelay_spi(20);

        /* data mode */

        if (!(type == 0x10)) /* write mode */
        {
            s_upk.buf[12] = s_upk.buf[12] & 0x7f;
            HAL_SPI_Transmit(&hspi5, s_upk.buf + 12, size, 1000);
        }

        else /* read mode */
        {

            s_upk.buf[12] = s_upk.buf[12] | 0x80;
            response_size = (*(s_upk.buf + 9) << 8) + *(s_upk.buf + 10) + 2;

            HAL_SPI_TransmitReceive(&hspi5, s_upk.buf + 12, s_upk.buf + (12 + size - 1), response_size, 1000);
            size = response_size + size - 1;
        }

        idelay_spi(20);
        IO9(1);
        uart_transfer_task(6 + size);
    }
}

/***************************************************************************
Function	: i2c byte write
Version		: 1.0
Descript 	: 8bit addr, data format
***************************************************************************/
u8 i2c_8bit_w(u8 addr, u8 data)
{
    return i2c_write_task(g_i2c_id, addr, I2C_8BIT, &data, 1, g_i2c_clk);
}

/***************************************************************************
Function	: i2c byte read
Version		: 1.0
Descript 	: 8bit addr, data format
***************************************************************************/
u8 i2c_8bit_r(u8 addr)
{
    u8 data;
    i2c_read_task(g_i2c_id, addr, I2C_8BIT, &data, 1, g_i2c_clk);
    return data;
}

/***************************************************************************
Function	: init rtp task
Version		: 1.0
Descript 	: DW7800, DW7912, DW7914 rtp setup
***************************************************************************/
int init_rtp_task(void)
{
    static u8 p;
    u16 time;
	   u16 fifo, play, div;

    /* basic setup */
    if (s_upk.buf[8] == PLAY)			
    {
        /* first time register clear*/
        if (s_bit.rtp_act == 0)
        {
            p = 0;
            s_rtp.play = 0;
            s_rtp.cnt[0] = 0;
            s_rtp.cnt[1] = 0;
        }

        s_rtp.play++;
        s_rtp.dual = 0;
        s_rtp.cnt[p] = 0;
        s_rtp.pid[0] = g_i2c_id;
        s_rtp.dev = s_upk.buf[7];

        if (s_rtp.dev & 0xF0)
        {
            if (s_rtp.dev == 0x12)
            {
                s_rtp.pid[0] = 0x90;
                s_rtp.pid[1] = 0xB2;
            }
            else
            {
                s_rtp.pid[0] = 0xB2;
                s_rtp.pid[1] = 0x90;
            }
            s_rtp.dual = 1;
        }

        s_rtp.size[p] = _8u16(s_upk.buf + 9);

        for (int i = 0; i < s_rtp.size[p]; i++)
        {
            s_rtp.buf[p][i] = s_upk.buf[11 + i];
        }
        p ^= 1;

        /* is first time run?*/
        if (s_bit.rtp_act == 0)
        {
            /* device setup */
            if (s_rtp.dev == DW7800)
            {
                time = 5000;
            }
            else if (s_rtp.dev == DW7802)
            {
                time = 5000;
            }
            else if ((s_rtp.dev & 0X0F) == DW7912)
            {
                /* for test set value */
                time = 5000;	
            /* step : check fifo */
            fifo = i2c_8bit_r(0x0A);
            /* dual play size check */
            div = s_rtp.size[p] >> s_rtp.dual;
            /* what if fifo is empty?*/
           
								play = 2048 - (fifo * 64 + 64);

                i2c_write_task(s_rtp.pid[0], 0x0A, I2C_8BIT, ((u8 *)s_rtp.buf[p] + s_rtp.cnt[p]), play, I2C_3MHZ);

                if (s_rtp.dual == 1)
                {
                    i2c_write_task(s_rtp.pid[1], 0x0A, I2C_8BIT, ((u8 *)s_rtp.buf[p] + (div + s_rtp.cnt[p])), play, I2C_3MHZ);
                }
								s_rtp.cnt[p] += play;
							
            }
            else if (s_rtp.dev == DW7914)
            {
                time = 5000;
                // i2c_8bit_w(0x0B, 0x00); /* RTP */
            }

            s_bit.rtp_act = 1;
            s_bit.rtp_stop = 0;
            sys_timer_set(TIMER_7, PLAY, time); /* start timer */
        }
    }
    else /* stop */
    {
        if (s_rtp.dev == DW7800)
        {
            /* dw7800 sw reset */
            // i2c_8bit_w(0x05, 0x01);
        }
        else if (s_rtp.dev == DW7802)
        {
            i2c_8bit_w(0x0C, 0x00);
        }
        else if ((s_rtp.dev & 0X0F) == DW7912)
        {
            /* play stop & fifo fulsh */
            i2c_8bit_w(0x09, 0x00);
        }
        else if (s_rtp.dev == DW7914)
        {
            i2c_8bit_w(0x0C, 0x00);
        }

        s_bit.rtp_stop = 1;
    }

    return 0;
}

/***************************************************************************
Function	: play rtp task
Version		: 1.0
Descript 	: stream data for real time play
***************************************************************************/
int play_rtp_task(void)
{
    u8 buf[2];
    u16 fifo, play, div;
    static u8 echo, p;

    if (s_rtp.play && (s_bit.rtp_stop == 0))
    {
        /* DW7800 */
        if (s_rtp.dev == DW7800)
        {
            /* step : check fifo */
            fifo = i2c_8bit_r(0x03);

            if (fifo < 127)
            {
                /* what if fifo is empty?*/
                if (s_rtp.size[p] > s_rtp.cnt[p])
                {
                    play = 127 - fifo;

                    if (play > (s_rtp.size[p] - s_rtp.cnt[p]))
                    {
                        play = s_rtp.size[p] - s_rtp.cnt[p];
                    }

                    i2c_write_task(g_i2c_id, 0x04, I2C_8BIT, ((u8 *)s_rtp.buf[p] + s_rtp.cnt[p]), play, I2C_1MHZ);
                    s_rtp.cnt[p] += play;

                    /* hand shake */
                    if (echo == 0)
                    {
                        echo = 1;
                        _16u8(&s_rtp.cnt[p], s_upk.buf + 9);
                        uart_transfer_task(5);
                    }

                    /* swap buffer */
                    if (s_rtp.size[p] == s_rtp.cnt[p])
                    {
                        p ^= 1;
                        echo = 0;
                        s_rtp.play--;
                        s_rtp.cnt[p] = 0;
                    }
                }
            }
        }

        /* DW7802 */
        else if (s_rtp.dev == DW7802)
        {
            /* step : check fifo */
            i2c_read_task(g_i2c_id, 0x31, I2C_8BIT, buf, 2, I2C_1MHZ);
            fifo = _8u16(buf);

            /* what if fifo is empty?*/
            if (s_rtp.size[p] > s_rtp.cnt[p])
            {
                play = 2048 - fifo;

                if (play > (s_rtp.size[p] - s_rtp.cnt[p]))
                {
                    play = s_rtp.size[p] - s_rtp.cnt[p];
                }

                i2c_write_task(g_i2c_id, 0x0D, I2C_8BIT, ((u8 *)s_rtp.buf[p] + s_rtp.cnt[p]), play, I2C_1MHZ);
                // i2c_8bit_w(0x0C, 0x01);
                buf[0] = 0x01;
                i2c_write_task(g_i2c_id, 0x0C, I2C_8BIT, buf, 1, I2C_1MHZ);
                s_rtp.cnt[p] += play;

                /* hand shake */
                if (echo == 0)
                {
                    echo = 1;
                    _16u8(&s_rtp.cnt[p], s_upk.buf + 9);
                    uart_transfer_task(5);
                }

                /* swap buffer */
                if (s_rtp.size[p] == s_rtp.cnt[p])
                {
                    p ^= 1;
                    echo = 0;
                    s_rtp.play--;
                    s_rtp.cnt[p] = 0;
                }
            }
        }

        /* DW7912 */
        else if ((s_rtp.dev & 0X0F) == DW7912)
        {
            /* step : check fifo */
            fifo = i2c_8bit_r(0x0A);
            /* dual play size check */
            div = s_rtp.size[p] >> s_rtp.dual;
            /* what if fifo is empty?*/
            if (div > s_rtp.cnt[p])
            {
								play = 2048 - (fifo * 64 + 64);
                if (play > (div - s_rtp.cnt[p]))
                {
                    play = div - s_rtp.cnt[p];
                }

                i2c_write_task(s_rtp.pid[0], 0x0A, I2C_8BIT, ((u8 *)s_rtp.buf[p] + s_rtp.cnt[p]), play, I2C_3MHZ);

                if (s_rtp.dual == 1)
                {
                    i2c_write_task(s_rtp.pid[1], 0x0A, I2C_8BIT, ((u8 *)s_rtp.buf[p] + (div + s_rtp.cnt[p])), play, I2C_3MHZ);
                }

                buf[0] = 0x01;
                i2c_write_task(s_rtp.pid[0], 0x09, I2C_8BIT, buf, 1, I2C_3MHZ);

                if (s_rtp.dual == 1)
                {
                    i2c_write_task(s_rtp.pid[1], 0x09, I2C_8BIT, buf, 1, I2C_3MHZ);
                }

                s_rtp.cnt[p] += play;

                /* hand shake */
                if (echo == 0)
                {
                    echo = 1;
                    _16u8(&s_rtp.cnt[p], s_upk.buf + 9);
                    uart_transfer_task(5);
                }

                /* swap buffer */
                if (div <= s_rtp.cnt[p])
                {
                    p ^= 1;
                    echo = 0;
                    s_rtp.play--;
                    s_rtp.cnt[p] = 0;
                }
            }
        }

        /* DW7914 */
        else if (s_rtp.dev == DW7914)
        {
            /* step : check fifo */
            i2c_read_task(g_i2c_id, 0x4D, I2C_8BIT, buf, 2, I2C_1MHZ);
            fifo = _8u16(buf);

            /* what if fifo is empty?*/
            if (s_rtp.size[p] > s_rtp.cnt[p])
            {
                play = 2048 - fifo;

                if (play > (s_rtp.size[p] - s_rtp.cnt[p]))
                {
                    play = s_rtp.size[p] - s_rtp.cnt[p];
                }

                i2c_write_task(g_i2c_id, 0x0D, I2C_8BIT, ((u8 *)s_rtp.buf[p] + s_rtp.cnt[p]), play, I2C_1MHZ);
                // i2c_8bit_w(0x0C, 0x01);
                buf[0] = 0x01;
                i2c_write_task(g_i2c_id, 0x0C, I2C_8BIT, buf, 1, I2C_1MHZ);
                s_rtp.cnt[p] += play;

                /* hand shake */
                if (echo == 0)
                {
                    echo = 1;
                    _16u8(&s_rtp.cnt[p], s_upk.buf + 9);
                    uart_transfer_task(5);
                }

                /* swap buffer */
                if (s_rtp.size[p] == s_rtp.cnt[p])
                {
                    p ^= 1;
                    echo = 0;
                    s_rtp.play--;
                    s_rtp.cnt[p] = 0;
                }
            }
        }
    }
    else
    {
        /* play end or stop*/
        p = 0;
        echo = 0;
        s_bit.rtp_act = 0;
        sys_timer_set(TIMER_7, STOP, 0);

        /* for hand shake */
        s_upk.buf[6] = 0x02;
        s_upk.buf[7] = s_rtp.dev;
        s_upk.buf[8] = 0xff;
				s_upk.buf[9] = s_rtp.play;
				s_upk.buf[10] = s_bit.rtp_stop;
        uart_transfer_task(5);

        LED1(LEDON);
        LED2(LEDON);
        LED3(LEDON);
    }

    return 0;
}

/***************************************************************************
Function	: ADXL343 Sensor
Version		: 1.0
Descript 	: Data read out (200us)
***************************************************************************/
void g_sensor_read_out(void)
{
    u16 p, n;
    u8 dat[6], fifo;
    static u8 cks;

    // HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_3);

    /* Auto trigger mode */
    if (s_rtp.mode == 0x01)
    {
        if (s_rtp.loop[0] > s_rtp.cnt[0])
        {
            /* all sensor read */
            dat[0] = 0x39;
            si2c_write_task(0, s_rtp.id, dat, 1, I2C_2MHZ);
            si2c_read_task(0, s_rtp.id, &fifo, 1, I2C_2MHZ);

            if (fifo + s_rtp.cnt[0] > s_rtp.loop[0])
            {
                fifo = s_rtp.loop[0] - s_rtp.cnt[0];
            }

            for (int i = 0; i < fifo; i++)
            {
                for (int ch = 0; ch < 3; ch++)
                {
                    dat[0] = 0x32;
                    si2c_write_task(ch, s_rtp.id, dat, 1, I2C_2MHZ);
                    si2c_read_task(ch, s_rtp.id, dat, 6, I2C_2MHZ);

                    p = s_rtp.cnt[0] * 18;
                    n = (ch * 6) + p;
                    s_upk.buf[12 + n] = dat[0];
                    s_upk.buf[13 + n] = dat[1];
                    s_upk.buf[14 + n] = dat[2];
                    s_upk.buf[15 + n] = dat[3];
                    s_upk.buf[16 + n] = dat[4];
                    s_upk.buf[17 + n] = dat[5];
                }
                s_rtp.cnt[0]++;
            }
        }
        else
        {
            /* kill timer */
            s_rtp.cnt[0] = 0;
            s_bit.scope_act = 0;
            sys_timer_set(TIMER_7, STOP, 0);

            /* for hand shake */
            s_upk.buf[8] = 0xFF;
            s_upk.buf[9] = 0x00;
            s_upk.buf[10] = s_rtp.size[0] >> 8;
            s_upk.buf[11] = s_rtp.size[0] >> 0;
            uart_transfer_task(6 + s_rtp.size[0]);

            /* flush FIFO and set to BYPASS Mode */
            for (int ch = 0; ch < 3; ch++)
            {
                dat[0] = 0x38;
                dat[1] = 0x00;
                si2c_write_task(ch, s_rtp.id, dat, 2, I2C_2MHZ);
            }
        }
    }

    /* continuous mode */
    else if (s_rtp.mode == 0x02)
    {
        if (s_rtp.cnt[0] == 0)
        {
            cks = 0;
        }

        /* sensor read for fifo size*/
        dat[0] = 0x39;
        for (int i = 0; i < 3; i++)
        {
            if (si2c_write_task(i, s_rtp.id, dat, 1, I2C_2MHZ) == I2C_ACK)
            {
                si2c_read_task(i, s_rtp.id, &fifo, 1, I2C_2MHZ);
                break;
            }
        }

        if (fifo + s_rtp.cnt[0] > s_rtp.loop[0])
        {
            fifo = s_rtp.loop[0] - s_rtp.cnt[0];
        }

        /* 18byte x fifo */
        for (int i = 0; i < fifo; i++)
        {
            for (int ch = 0; ch < 3; ch++)
            {
                /* sensor readout */
                dat[0] = 0x32;
                si2c_write_task(ch, s_rtp.id, dat, 1, I2C_2MHZ);
                si2c_read_task(ch, s_rtp.id, dat, 6, I2C_2MHZ);

                if (s_rtp.cnt[1] < s_rtp.loop[0])
                {
                    s_rtp.pk = 0;
                }
                else
                {
                    s_rtp.pk = 1;
                }
                /* sensor data copy to uart buffer  */
                for (int k = 0; k < 6; k++)
                {
                    n = (ch * 6) + (s_rtp.cnt[0] * 18);
                    s_rtp.buf[s_rtp.pk][(12 + k) + n] = dat[k];
                    cks += dat[k];
                }
            }
            s_rtp.cnt[0]++;
            s_rtp.cnt[1]++;
            // max size = fifo x (3ch x 6)
        }

        if (s_rtp.cnt[1] == s_rtp.loop[0])
        {
            s_rtp.cnt[0] = 0;

            for (int i = 0; i < 6; i++)
            {
                cks += s_rtp.buf[0][6 + i];
            }
            s_rtp.buf[0][4] = cks;
            HAL_UART_Transmit_DMA(&huart1, s_rtp.buf[0], s_rtp.psize + BHD6);
            LED5_T;
        }

        else if (s_rtp.cnt[1] == s_rtp.loop[1])
        {
            s_rtp.cnt[0] = 0;
            s_rtp.cnt[1] = 0;

            for (int i = 0; i < 6; i++)
            {
                cks += s_rtp.buf[1][6 + i];
            }
            s_rtp.buf[1][4] = cks;
            HAL_UART_Transmit_DMA(&huart1, s_rtp.buf[1], s_rtp.psize + BHD6);
            LED5_T;
        }
    }
}

/***************************************************************************
Function	: ADXL343 Sensor
Version		: 1.0
Descript 	: Data set up
***************************************************************************/
void g_sensor_set_task(void)
{
    u16 time;
    u8 dat[2], get, rsize;

    rsize = 18;
    time = 1000;
    s_rtp.id = 0x3A;

    if (s_upk.buf[7] == 0x00)
    {
        /* sensor setting & detect */
        if (s_upk.buf[8] == 0x00)
        {
            /* wrtie dummy data for i2c line activate */
            dat[0] = 0x00;
            si2c_write_task(0, s_rtp.id, dat, 1, I2C_400KHZ);
            si2c_write_task(1, s_rtp.id, dat, 1, I2C_400KHZ);
            si2c_write_task(2, s_rtp.id, dat, 1, I2C_400KHZ);

            /* auto contect detect */
            for (int ch = 0; ch < 3; ch++)
            {
                dat[0] = 0x00;
                si2c_write_task(ch, s_rtp.id, dat, 1, I2C_400KHZ);
                si2c_read_task(ch, s_rtp.id, &get, 1, I2C_400KHZ);
                (get == 0xE5) ? (s_upk.buf[9 + ch] = get) : (s_upk.buf[9 + ch] = 0xFF);

                if (get == 0xE5)
                {
                    /* Default Sensor setup */
                    dat[0] = 0x2E; /* sensor setup */
                    dat[1] = 0x00;
                    si2c_write_task(ch, s_rtp.id, dat, 2, I2C_400KHZ);

                    dat[0] = 0x1D; /* Tab threshold */
                    dat[1] = 20;
                    si2c_write_task(ch, s_rtp.id, dat, 2, I2C_400KHZ);

                    dat[0] = 0x21; /* DUR register */
                    dat[1] = 50;
                    si2c_write_task(ch, s_rtp.id, dat, 2, I2C_400KHZ);

                    dat[0] = 0x22;
                    dat[1] = 0;
                    si2c_write_task(ch, s_rtp.id, dat, 2, I2C_400KHZ);

                    dat[0] = 0x23;
                    dat[1] = 0;
                    si2c_write_task(ch, s_rtp.id, dat, 2, I2C_400KHZ);

                    dat[0] = 0x2A; /* Tab Axes */
                    dat[1] = 0x07;
                    si2c_write_task(ch, s_rtp.id, dat, 2, I2C_400KHZ);

                    dat[0] = 0x2C; /* Data Rate 3200Hz */
                    dat[1] = 0x0F;
                    si2c_write_task(ch, s_rtp.id, dat, 2, I2C_400KHZ);

                    dat[0] = 0x2D; /* Power Control */
                    dat[1] = 0x08;
                    si2c_write_task(ch, s_rtp.id, dat, 2, I2C_400KHZ);
                }
            }
            /* for handshake */
            uart_transfer_task(6);
        }
        /* g scale setting */
        else if (s_upk.buf[8] == 0x01)
        {
            for (int ch = 0; ch < 3; ch++)
            {
                if (s_upk.buf[9 + (ch * 2)] & 0x01)
                {
                    /* sensor data format */
                    dat[0] = 0x31;
                    /* set to full resolution */
                    dat[1] = 0x08 | s_upk.buf[10 + (ch * 2)];
                    si2c_write_task(ch, s_rtp.id, dat, 2, I2C_400KHZ);
                }

                /* for handshake */
                uart_transfer_task(4);
            }
        }
        /* Sensor Offset Calibration */
        else if (s_upk.buf[8] == 0x02)
        {
            sys_timer_set(TIMER_7, PLAY, 300);
            s_bit.scope_cal = 1;
        }
    }

    /* auto trigger setting*/
    else if (s_upk.buf[7] == 0x01)
    {
        s_rtp.mode = s_upk.buf[7];
        // s_rtp.loop[0] = _8u16(s_upk.buf + 11) * 1000 / time;
        s_rtp.loop[0] = _8u16(s_upk.buf + 11) * 3200 / 1000;
        s_rtp.size[0] = s_rtp.loop[0] * rsize; /* 3 Sensor x 6byte */
        s_rtp.cnt[0] = 0;

        /* Set to FIFO Mode */
        for (int ch = 0; ch < 3; ch++)
        {
            dat[0] = 0x38;
            dat[1] = 0x40;
            si2c_write_task(ch, s_rtp.id, dat, 2, I2C_400KHZ);
        }

        /* timer set & measure started*/
        sys_timer_set(TIMER_7, s_upk.buf[8], time);

        if (s_upk.buf[8] == PLAY)
        {
            s_bit.scope_act = 1;
        }
        else
        {
            s_bit.scope_act = 0;
        }

        /* device setup */
        if (s_upk.buf[9] == DW7800)
        {
            /* not supported  */
        }
        else if (s_upk.buf[9] == DW7802)
        {
            /*  */
            i2c_8bit_w(0x0C, 0x01); /* play on */
        }
        else if (s_upk.buf[9] == DW7912)
        {
            /*  */
            i2c_8bit_w(0x09, 0x01); /* play on */
        }
        else if (s_upk.buf[9] == DW7914)
        {
            /*  */
            i2c_8bit_w(0x0C, 0x01); /* play on */
        }
    }

    /* continuous setting*/
    else if (s_upk.buf[7] == 0x02)
    {
        s_rtp.mode = s_upk.buf[7];
        // s_rtp.loop[0] = _8u16(s_upk.buf + 11) * 1000 / time;
        s_rtp.loop[0] = _8u16(s_upk.buf + 11) * 3200 / 1000;
        s_rtp.loop[1] = s_rtp.loop[0] << 1;
        s_rtp.psize = s_rtp.loop[0] * rsize + 6;
        s_rtp.cnt[0] = 0;
        s_rtp.cnt[1] = 0;

        /* stream dummy data */
        for (int i = 0; i < 10; i++)
        {
            s_rtp.buf[0][i] = s_upk.buf[i];
            s_rtp.buf[1][i] = s_upk.buf[i];
        }
        /* packet size redefine */
        s_rtp.buf[0][2] = s_rtp.psize >> 8;
        s_rtp.buf[0][3] = s_rtp.psize >> 0;
        s_rtp.buf[1][2] = s_rtp.psize >> 8;
        s_rtp.buf[1][3] = s_rtp.psize >> 0;

        s_rtp.buf[0][10] = (s_rtp.psize - 6) >> 8;
        s_rtp.buf[0][11] = (s_rtp.psize - 6) >> 0;
        s_rtp.buf[1][10] = (s_rtp.psize - 6) >> 8;
        s_rtp.buf[1][11] = (s_rtp.psize - 6) >> 0;

        /* Set to FIFO Mode */
        for (int ch = 0; ch < 3; ch++)
        {
            dat[0] = 0x38;
            dat[1] = 0x40;
            si2c_write_task(ch, s_rtp.id, dat, 2, I2C_400KHZ);
        }

        /* timer set & measure started*/
        sys_timer_set(TIMER_7, s_upk.buf[8], time);

        if (s_upk.buf[8] == PLAY)
        {
            s_bit.scope_act = 1;
        }
        else
        {
            /* flush FIFO and set to BYPASS Mode */
            for (int ch = 0; ch < 3; ch++)
            {
                dat[0] = 0x38;
                dat[1] = 0x00;
                si2c_write_task(ch, s_rtp.id, dat, 2, I2C_400KHZ);
            }

            uart_transfer_task(3);
            s_bit.scope_act = 0;
        }
    }
}

/***************************************************************************
Function	: ADXL343 Sensor
Version		: 1.0
Descript 	: Offset Calibration
***************************************************************************/
void g_sensor_ofs_cal(void)
{
    static s32 sum[9];
    static u16 cnt;
    u16 avg = 320;
    u8 dat[6], buf[6], fifo;
    s8 ofs[3];

    /* initialize offset registers */
    if (cnt == 0)
    {
        dat[0] = 0x1E;
        dat[1] = 0x00;
        dat[2] = 0x00;
        dat[3] = 0x00;
        for (int ch = 0; ch < 3; ch++)
        {
            si2c_write_task(ch, s_rtp.id, dat, 4, I2C_2MHZ);
        }

        /* Set to FIFO Mode */
        for (int ch = 0; ch < 3; ch++)
        {
            dat[0] = 0x38;
            dat[1] = 0x40;
            si2c_write_task(ch, s_rtp.id, dat, 2, I2C_2MHZ);
        }
    }

    /* read all sensors */
    dat[0] = 0x39;
    si2c_write_task(0, s_rtp.id, dat, 1, I2C_2MHZ);
    si2c_read_task(0, s_rtp.id, &fifo, 1, I2C_2MHZ);
    if (fifo + cnt > avg)
    {
        fifo = avg - cnt;
    }
    for (int i = 0; i < fifo; i++)
    {
        for (int ch = 0; ch < 3; ch++)
        {
            dat[0] = 0x32;
            si2c_write_task(ch, s_rtp.id, dat, 1, I2C_2MHZ);
            si2c_read_task(ch, s_rtp.id, dat, 6, I2C_2MHZ);

            /* swap bytes */
            buf[0] = dat[1];
            buf[1] = dat[0];
            buf[2] = dat[3];
            buf[3] = dat[2];
            buf[4] = dat[5];
            buf[5] = dat[4];

            /* summation of 0.1sec read data */
            sum[0 + (ch * 3)] += _twos(_8u16(&buf[0]));
            sum[1 + (ch * 3)] += _twos(_8u16(&buf[2]));
            sum[2 + (ch * 3)] += _twos(_8u16(&buf[4]));
        }
        cnt++;
    }

    /* write offset */
    if (cnt == avg)
    {
        for (int ch = 0; ch < 3; ch++)
        {
            ofs[0] = -sum[0 + (ch * 3)] / avg / 4; /* average and convert g values */
            ofs[1] = -sum[1 + (ch * 3)] / avg / 4; /* read data :  3.9mg/LSB in full resolution mode */
            ofs[2] = -sum[2 + (ch * 3)] / avg / 4; /* ofs reg   : 15.6mg/LSB */

            dat[0] = 0x1E;
            dat[1] = ofs[0];
            dat[2] = ofs[1];
            dat[3] = ofs[2];

            si2c_write_task(ch, s_rtp.id, dat, 4, I2C_2MHZ);
        }
        /* kill timer */
        sys_timer_set(TIMER_7, STOP, 0);
        s_bit.scope_cal = 0;

        /* for hand shake */
        s_upk.buf[9] = 0xFF;
        uart_transfer_task(4);

        /* flush FIFO and set to BYPASS Mode */
        for (int ch = 0; ch < 3; ch++)
        {
            dat[0] = 0x38;
            dat[1] = 0x00;
            si2c_write_task(ch, s_rtp.id, dat, 2, I2C_2MHZ);
        }

        /* initialize variables */
        cnt = 0;
        for (int i = 0; i < 9; i++)
        {
            sum[i] = 0;
        }
    }
}

/***************************************************************************
Function	: rednoah3 main task fucntion
Date		: 2023.04.18
***************************************************************************/
void rednoah3_main_task(void)
{
    if (s_bit.timer10_act)
    {
        sys_led_task();
        sys_key_task();
        trig_ctrl_task(RUN);
        s_bit.timer10_act = 0;
    }

    if (s_bit.timer7_act)
    {
        if (s_bit.rtp_act)
        {
            play_rtp_task();
        }
        else if (s_bit.scope_act)
        {
            g_sensor_read_out();
        }
        else if (s_bit.scope_cal)
        {
            g_sensor_ofs_cal();
        }
        s_bit.timer7_act = 0;
    }

    if (s_bit.rx_done == 1)
    {
        switch (s_upk.buf[6])
        {
        case SYS_FSM:
            board_set_task();
            break;

        case I2C_FSM:
            i2c_task();
            break;

        case RTP_FSM:
            init_rtp_task();
            break;

        case SCP_FSM:
            g_sensor_set_task();
            break;
        case COMM_FSM:
            comm_set_task();
            break;
        }
        s_bit.rx_done = 0;

        LED5_T;
    }
}

/***************************************************************************
Function	: HAL_GPIO_EXTI_Callback
Date		: 2021.08.12
Descript 	: sw1, sw2, sw3
***************************************************************************/
void HAL_GPIO_EXTI_Callback(u16 GPIO_Pin)
{
}

/***************************************************************************
Function	: HAL_UART_RxCpltCallback
Date		: 2021.08.12
Editor		: JKS
Version		: 1.0
Descript 	: degign for speed
***************************************************************************/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    u8 cks;
    static u32 num, step, get_size;

    s_upk.buf[num++] = s_upk.get;

    switch (step)
    {
    case 0:
        if (s_upk.buf[0] != 0xAE)
            num = 0;
        else
            step = 1;
        break;
    case 1:
        if (num >= BHD6)
        {
            s_upk.get_size = _8u16(s_upk.buf + 2);
            get_size = s_upk.get_size + BHD6;
            step = 2;
        }
        break;
    case 2:
        if (num >= get_size)
        {
            cks = 0;
            for (int i = 0; i < s_upk.get_size; i++)
            {
                cks += s_upk.buf[6 + i];
            }

            /*checksum fail*/
            if (cks != s_upk.buf[4])
            {
                s_upk.buf[6] = 0x00;
                s_upk.buf[7] = 0xEC;
                s_upk.buf[8] = 0xE1;
                uart_transfer_task(5);
            }
            else
            {
                s_bit.rx_done = 1;
            }
            num = 0;
            step = 0;
        }
        break;
    }

    HAL_UART_Receive_IT(&huart1, &s_upk.get, 1);
#if 0
  HAL_UART_Receive_DMA(&huart1, &s_upk.get, 1);
#endif
}

/***************************************************************************
Function	: HAL_UART_TxCpltCallback
Date		: 2021.08.12
Editor		: JKS
Version		: 1.0
Descript 	: Reserve
***************************************************************************/
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandle)
{
    /* Set transmission flag: transfer complete*/
    // P0_T;
    s_bit.tx_done = 1;
}

/***************************************************************************
Function	: HAL_TIM_PeriodElapsedCallback
Date		: 2021.08.12
Editor		: JKS
Version		: 1.0
Descript 	: degign for speed
***************************************************************************/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    /* USER CODE BEGIN Callback 0 */

    if (htim->Instance == TIM10)
    {
        s_bit.timer10_act = 1;
    }
    else if (htim->Instance == TIM7)
    {
        s_bit.timer7_act = 1;
    }
}
