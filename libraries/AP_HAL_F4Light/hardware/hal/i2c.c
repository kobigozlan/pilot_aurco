/*
    (c) 2017 night_ghost@ykoctpa.ru
 
based on: datasheet

*/

#pragma GCC optimize ("O2")

#include "i2c.h"
#include "dma.h"
#include "systick.h"
#include "stm32f4xx_i2c.h"
#include "stm32f4xx_dma.h"



static i2c_state i2c1_state IN_CCM;
static i2c_state i2c2_state IN_CCM;

static const i2c_dev i2c_dev1 = {
    .I2Cx         = I2C1,
    .gpio_port    = &gpiob,
    .sda_pin      = 9,
    .scl_pin      = 8,
    .clk       	  = RCC_APB1Periph_I2C1,
    .gpio_af	  = GPIO_AF_I2C1,
    .ev_nvic_line = I2C1_EV_IRQn,
    .er_nvic_line = I2C1_ER_IRQn,
//    .dma          = { DMA_CR_CH1, DMA1_STREAM0, DMA1_STREAM6 }, // I2C1
    .state        = &i2c1_state,
};
/** I2C1 device */
const i2c_dev* const _I2C1 = &i2c_dev1;


static const i2c_dev i2c_dev2 = {
    .I2Cx         = I2C2,
    .gpio_port    = &gpiob,
    .sda_pin      = 11,
    .scl_pin      = 10,
    .clk       	  = RCC_APB1Periph_I2C2,
    .gpio_af	  = GPIO_AF_I2C2,
    .ev_nvic_line = I2C2_EV_IRQn,
    .er_nvic_line = I2C2_ER_IRQn,
//    .dma          = { DMA_CR_CH7, DMA1_STREAM3 /* intersects with spi2_tx */ , DMA1_STREAM7 }, // I2C2
    .state        = &i2c2_state,
};

/** I2C2 device */
const i2c_dev* const _I2C2 = &i2c_dev2;


typedef enum {TX = 0, RX = 1, TXREG = 2} I2C_Dir;

static void delay_10us(){
    hal_delay_microseconds(10);
}

uint32_t i2c_bit_time=4;

/**
 * @brief  DeInitializes peripherals used by the I2C driver.
 * @param  None
 * @retval None
 */
void i2c_lowLevel_deinit(const i2c_dev *dev){
    GPIO_InitTypeDef GPIO_InitStructure;

    /* I2C Peripheral Disable */
    I2C_Cmd(dev->I2Cx, DISABLE);

    /* I2C DeInit */
    I2C_DeInit(dev->I2Cx);

    /*!< GPIO configuration */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz; // low speed to prevent glitches

    /*!< Configure I2C pins: SCL */
    GPIO_InitStructure.GPIO_Pin = BIT(dev->scl_pin);
    GPIO_Init(dev->gpio_port->GPIOx, &GPIO_InitStructure);

    /*!< Configure I2C pins: SDA */
    GPIO_InitStructure.GPIO_Pin = BIT(dev->sda_pin);
    GPIO_Init(dev->gpio_port->GPIOx, &GPIO_InitStructure);
}

/**
 * @brief  Initializes peripherals used by the I2C driver.
 */
static inline void i2c_lowLevel_init(const i2c_dev *dev)  {
    memset(dev->state,0,sizeof(i2c_state));

    GPIO_InitTypeDef GPIO_InitStructure;

    /* Enable the i2c */
    RCC_APB1PeriphClockCmd(dev->clk, ENABLE);

    /* Reset the Peripheral */
    RCC_APB1PeriphResetCmd(dev->clk, ENABLE);
    RCC_APB1PeriphResetCmd(dev->clk, DISABLE);

    /* Enable the GPIOs for the SCL/SDA Pins */
    RCC_AHB1PeriphClockCmd(dev->gpio_port->clk, ENABLE);

    memset(dev->state,0,sizeof(i2c_state));

// common configuration
    /* common GPIO configuration */
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz; // GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;

    /* Configure SCL */
    GPIO_InitStructure.GPIO_Pin = BIT(dev->scl_pin);
    GPIO_Init(dev->gpio_port->GPIOx, &GPIO_InitStructure);

    /* Configure SDA */
    GPIO_InitStructure.GPIO_Pin = BIT(dev->sda_pin);
    GPIO_Init(dev->gpio_port->GPIOx, &GPIO_InitStructure);

    /* Connect GPIO pins to peripheral, SCL must be first! */
    gpio_set_af_mode(dev->gpio_port, dev->scl_pin, dev->gpio_af);
    gpio_set_af_mode(dev->gpio_port, dev->sda_pin, dev->gpio_af);
}

void i2c_init(const i2c_dev *dev, uint16_t address, uint32_t speed)
{

    i2c_lowLevel_init(dev); // init GPIO hardware

    i2c_bit_time = 1000000l / speed;

    I2C_InitTypeDef I2C_InitStructure;     /* I2C configuration */
    I2C_StructInit(&I2C_InitStructure);

    I2C_InitStructure.I2C_Mode                = I2C_Mode_I2C;
    I2C_InitStructure.I2C_DutyCycle           = I2C_DutyCycle_2;
    I2C_InitStructure.I2C_OwnAddress1         = address;
    I2C_InitStructure.I2C_Ack                 = I2C_Ack_Enable;
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_InitStructure.I2C_ClockSpeed          = speed;


    I2C_ITConfig(dev->I2Cx, I2C_IT_EVT | I2C_IT_BUF | I2C_IT_ERR, DISABLE);

    (void)dev->I2Cx->DR; 

    /* I2C Peripheral Enable */
    I2C_Cmd(dev->I2Cx, ENABLE);
    /* Apply I2C configuration after enabling it */
    I2C_Init(dev->I2Cx, &I2C_InitStructure);
}

/**
 * @brief  DeInitializes peripherals used by the I2C driver.
 * @param  None
 * @retval None
 */
void i2c_deinit(const i2c_dev *dev)
{
    i2c_lowLevel_deinit(dev);
}

static void ev_handler(const i2c_dev *dev, bool err){
    if(dev->state->handler) revo_call_handler(dev->state->handler, err);
    else { // disable interrupts
        dev->I2Cx->CR2 &= ~(I2C_CR2_ITBUFEN|I2C_CR2_ITEVTEN|I2C_CR2_ITERREN);    // Disable interrupts
    }
}

void I2C1_EV_IRQHandler(); // to avoid warnings
void I2C1_ER_IRQHandler();
void I2C2_EV_IRQHandler();
void I2C2_ER_IRQHandler();

void I2C1_EV_IRQHandler(){                // I2C1 Event                   
    ev_handler(_I2C1, false);
}
  
void I2C1_ER_IRQHandler(){                // I2C1 Error                   
    ev_handler(_I2C1, true);
}

void I2C2_EV_IRQHandler(){                // I2C2 Event                 
    ev_handler(_I2C2, false);
}

void I2C2_ER_IRQHandler(){                // I2C2 Error
    ev_handler(_I2C2, true);
}



void i2c_master_release_bus(const i2c_dev *dev) {
    gpio_write_bit(dev->gpio_port, dev->scl_pin, 1);
    gpio_write_bit(dev->gpio_port, dev->sda_pin, 1);
    gpio_set_mode(dev->gpio_port, dev->scl_pin, GPIO_OUTPUT_OD_PU);
    gpio_set_mode(dev->gpio_port, dev->sda_pin, GPIO_OUTPUT_OD_PU);
}


/**
 * @brief Reset an I2C bus.
 *
 * Reset is accomplished by clocking out pulses until any hung slaves
 * release SDA and SCL, then generating a START condition, then a STOP
 * condition.
 *
 * @param dev I2C device
 */
 
#define MAX_I2C_TIME 300 // 300ms before device turn off

bool i2c_bus_reset(const i2c_dev *dev) {

    /* Release both lines */
    i2c_master_release_bus(dev);

    uint32_t t=systick_uptime();

    /*
     * Make sure the bus is free by clocking it until any slaves release the
     * bus.
     */

again:
    /* Wait for any clock stretching to finish */
    while (!gpio_read_bit(dev->gpio_port, dev->scl_pin)) {// device can output 1 so check clock first
        if(systick_uptime()-t > MAX_I2C_TIME) return false;
        hal_yield(10);
    }
    delay_10us();	// 50kHz

    while (!gpio_read_bit(dev->gpio_port, dev->sda_pin)) {
        /* Wait for any clock stretching to finish */
        while (!gpio_read_bit(dev->gpio_port, dev->scl_pin)){
            if(systick_uptime()-t > MAX_I2C_TIME) return false;
            hal_yield(10);
        }
        delay_10us();	// 50kHz

        /* Pull low */
        gpio_write_bit(dev->gpio_port, dev->scl_pin, 0);
        delay_10us();

        /* Release high again */
        gpio_write_bit(dev->gpio_port, dev->scl_pin, 1);
        delay_10us();
    }

    /* Generate start then stop condition */
    gpio_write_bit(dev->gpio_port, dev->sda_pin, 0);
    delay_10us();
    gpio_write_bit(dev->gpio_port, dev->scl_pin, 0);
    delay_10us();
    gpio_write_bit(dev->gpio_port, dev->scl_pin, 1);
    delay_10us();
    gpio_write_bit(dev->gpio_port, dev->sda_pin, 1);
    
    uint32_t rtime = stopwatch_getticks();
    uint32_t dt    = us_ticks * 50; // 50uS

    while ((stopwatch_getticks() - rtime) < dt) {
        if (!gpio_read_bit(dev->gpio_port, dev->scl_pin))  goto again; // any SCL activity after STOP
    }

// we was generating signals on I2C bus, but BUSY flag senses it even when hardware is off
// datasheet: It indicates a communication in progress on the bus. This information is still updated when the interface is disabled (PE=0).

    dev->I2Cx->CR1 |= I2C_CR1_SWRST;           // set SoftReset for some time 
    hal_yield(0);
    dev->I2Cx->CR1 &= (uint16_t)(~I2C_CR1_SWRST); // clear SoftReset flag 
    return true;
}



