#ifndef CONFIG_STM32F3_DISCOVERY_H
#define CONFIG_STM32F3_DISCOVERY_H

#define BOARD_STM32F3_DISCOVERY

/* Stm32f3 has a 8MHz internal clock and 64MHz pll. */

#define EXT_CLK 8000000
#define AHB_CLK 64000000

/*-----------------------------------------------------------------------------*/

/*
 * Onboard LEDs
 */

/* BLUE, on PE08 */
#ifndef USE_LED_1
#define USE_LED_1 1
#endif
#define LED_1_GPIO GPIOE
#define LED_1_GPIO_CLK RCC_AHBENR_IOPEEN
#define LED_1_GPIO_PIN GPIO8
#define LED_1_GPIO_ON gpio_clear
#define LED_1_GPIO_OFF gpio_set
#define LED_1_AFIO_REMAP ((void)0)

/* RED, on PE09 */
#ifndef USE_LED_2
#define USE_LED_2 1
#endif
#define LED_2_GPIO GPIOE
#define LED_2_GPIO_CLK RCC_AHBENR_IOPEEN
#define LED_2_GPIO_PIN GPIO9
#define LED_2_GPIO_ON gpio_clear
#define LED_2_GPIO_OFF gpio_set
#define LED_2_AFIO_REMAP ((void)0)

/* ORANGE , on PE10 */
#ifndef USE_LED_3
#define USE_LED_3 1
#endif
#define LED_3_GPIO GPIOE
#define LED_3_GPIO_CLK RCC_AHBENR_IOPEEN
#define LED_3_GPIO_PIN GPIO10
#define LED_3_GPIO_ON gpio_clear
#define LED_3_GPIO_OFF gpio_set
#define LED_3_AFIO_REMAP ((void)0)

/* GREEN, on PE11 */
#ifndef USE_LED_4
#define USE_LED_4 1
#endif
#define LED_4_GPIO GPIOE
#define LED_4_GPIO_CLK RCC_AHBENR_IOPEEN
#define LED_4_GPIO_PIN GPIO11
#define LED_4_GPIO_ON gpio_clear
#define LED_4_GPIO_OFF gpio_set
#define LED_4_AFIO_REMAP ((void)0)

/* BLUE, on PE12 */
#ifndef USE_LED_5
#define USE_LED_5 1
#endif
#define LED_5_GPIO GPIOE
#define LED_5_GPIO_CLK RCC_AHBENR_IOPEEN
#define LED_5_GPIO_PIN GPIO12
#define LED_5_GPIO_ON gpio_clear
#define LED_5_GPIO_OFF gpio_set
#define LED_5_AFIO_REMAP ((void)0)

/* RED, on PE13 */
#ifndef USE_LED_6
#define USE_LED_6 1
#endif
#define LED_6_GPIO GPIOE
#define LED_6_GPIO_CLK RCC_AHBENR_IOPEEN
#define LED_6_GPIO_PIN GPIO13
#define LED_6_GPIO_ON gpio_clear
#define LED_6_GPIO_OFF gpio_set
#define LED_6_AFIO_REMAP ((void)0)

/* ORANGE, on PE14 */
#ifndef USE_LED_7
#define USE_LED_7 1
#endif
#define LED_7_GPIO GPIOE
#define LED_7_GPIO_CLK RCC_AHBENR_IOPEEN
#define LED_7_GPIO_PIN GPIO14
#define LED_7_GPIO_ON gpio_clear
#define LED_7_GPIO_OFF gpio_set
#define LED_7_AFIO_REMAP ((void)0)

/* GREEN, on PE15 */
#ifndef USE_LED_8
#define USE_LED_8 1
#endif
#define LED_8_GPIO GPIOE
#define LED_8_GPIO_CLK RCC_AHBENR_IOPEEN
#define LED_8_GPIO_PIN GPIO15
#define LED_8_GPIO_ON gpio_clear
#define LED_8_GPIO_OFF gpio_set
#define LED_8_AFIO_REMAP ((void)0)

/*-----------------------------------------------------------------------------*/


/***************************************************************************************************/
/**************************************    UART    *************************************************/
/***************************************************************************************************/

// If you don't need all those uarts comment a usart block out to free pins.
#define UART1_GPIO_AF GPIO_AF7
#define UART1_GPIO_PORT_TX GPIOC
#define UART1_GPIO_TX GPIO4
#define UART1_GPIO_PORT_RX GPIOC
#define UART1_GPIO_RX GPIO5

// UART2 is used for the Spectrum rx input also
#define UART2_GPIO_AF GPIO_AF7
#define UART2_GPIO_PORT_TX GPIOA
#define UART2_GPIO_TX GPIO2
#define UART2_GPIO_PORT_RX GPIOA
#define UART2_GPIO_RX GPIO3

#define UART3_GPIO_AF GPIO_AF7
#define UART3_GPIO_PORT_TX GPIOB
#define UART3_GPIO_TX GPIO10
#define UART3_GPIO_PORT_RX GPIOB
#define UART3_GPIO_RX GPIO11

// UARTS 4 AND 5 CANT BE USED IF SPI3 IS NEEDED
#if !USE_SPI3
#define UART4_GPIO_AF GPIO_AF5
#define UART4_GPIO_PORT_TX GPIOC
#define UART4_GPIO_TX GPIO10
#define UART4_GPIO_PORT_RX GPIOC
#define UART4_GPIO_RX GPIO11

#define UART5_GPIO_AF GPIO_AF5
#define UART5_GPIO_PORT_TX GPIOC
#define UART5_GPIO_TX GPIO12
#define UART5_GPIO_PORT_RX GPIOD
#define UART5_GPIO_RX GPIO2
#endif


/***************************************************************************************************/
/**************************************    SPI     *************************************************/
/***************************************************************************************************/

#if STM32F3_DISCOVERY_SPI1_FOR_L3GD20
#define SPI1_GPIO_AF GPIO_AF5
#define SPI1_GPIO_PORT_SCK GPIOA
#define SPI1_GPIO_SCK GPIO5
#define SPI1_GPIO_PORT_MISO GPIOA
#define SPI1_GPIO_MISO GPIO6
#define SPI1_GPIO_PORT_MOSI GPIOA
#define SPI1_GPIO_MOSI GPIO7
#else
#define SPI1_GPIO_AF GPIO_AF5
#define SPI1_GPIO_PORT_SCK GPIOB
#define SPI1_GPIO_SCK GPIO3
#define SPI1_GPIO_PORT_MISO GPIOB
#define SPI1_GPIO_MISO GPIO4
#define SPI1_GPIO_PORT_MOSI GPIOB
#define SPI1_GPIO_MOSI GPIO5
#endif

/* CANNOT BE USED IF PWM CHANNELS 10 & 11 ARE ACTIVE !!! */
#define SPI2_GPIO_AF GPIO_AF5
#define SPI2_GPIO_PORT_SCK GPIOB
#define SPI2_GPIO_SCK GPIO13
#define SPI2_GPIO_PORT_MISO GPIOB
#define SPI2_GPIO_MISO GPIO14
#define SPI2_GPIO_PORT_MOSI GPIOB
#define SPI2_GPIO_MOSI GPIO15

/* CANNOT BE USED IF UARTS 4 & 5 ARE ACTIVE !!! */
#define SPI3_GPIO_AF GPIO_AF6
#define SPI3_GPIO_PORT_SCK GPIOC
#define SPI3_GPIO_SCK GPIO10
#define SPI3_GPIO_PORT_MISO GPIOC
#define SPI3_GPIO_MISO GPIO11
#define SPI3_GPIO_PORT_MOSI GPIOC
#define SPI3_GPIO_MOSI GPIO12

#define SPI_SELECT_SLAVE0_PORT GPIOE /*CAMBIAR*/
#define SPI_SELECT_SLAVE0_PIN GPIO2
#define SPI_SELECT_SLAVE1_PORT GPIOE
#define SPI_SELECT_SLAVE1_PIN GPIO7
#define SPI_SELECT_SLAVE2_PORT GPIOE
#define SPI_SELECT_SLAVE2_PIN GPIO3


/***************************************************************************************************/
/**************************************    I2C     *************************************************/
/***************************************************************************************************/
#define I2C1_GPIO_PORT GPIOA
#define I2C1_GPIO_SCL GPI015
#define I2C1_GPIO_SDA GPIO14

#define I2C2_GPIO_PORT GPIOA
#define I2C2_GPIO_SCL GPIO9
#define I2C2_GPIO_SDA GPIO10

//#define I2C3_GPIO_PORT_SCL GPIOA
//#define I2C3_GPIO_SCL GPIO8
//#define I2C3_GPIO_PORT_SDA GPIOC
//#define I2C3_GPIO_SDA GPIO9


/***************************************************************************************************/
/**************************************    ADC     *************************************************/
/***************************************************************************************************/

#define USE_AD_TIM4 1

#define BOARD_ADC_CHANNEL_1 9 /*REVISAR*/
#define BOARD_ADC_CHANNEL_2 15
#define BOARD_ADC_CHANNEL_3 14
#define BOARD_ADC_CHANNEL_4 4

#ifndef USE_AD1
#define USE_AD1 1
#endif

/* provide defines that can be used to access the ADC_x in the code or airframe file
 * these directly map to the index number of the 4 adc channels defined above
 * 4th (index 3) is used for bat monitoring by default
 */

// AUX 1
#define ADC_1 ADC1_C1
#ifdef USE_ADC_1
#ifndef ADC_1_GPIO_CLOCK_PORT
#define ADC_1_GPIO_CLOCK_PORT RCC_AHBENR_IOPBEN
#define ADC_1_INIT() gpio_mode_setup(GPIOB, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO1)
#endif
#define USE_AD1_1 1
#else
#define ADC_1_GPIO_CLOCK_PORT 0
#define ADC_1_INIT() {}
#endif

// AUX 2
#define ADC_2 ADC1_C2
#ifdef USE_ADC_2
#ifndef ADC_2_GPIO_CLOCK_PORT
#define ADC_2_GPIO_CLOCK_PORT RCC_AHBENR_IOPCEN
#define ADC_2_INIT() gpio_mode_setup(GPIOC, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO1)
#endif
#define USE_AD1_2 1
#else
#define ADC_2_GPIO_CLOCK_PORT 0
#define ADC_2_INIT() {}
#endif

// AUX 3
#define ADC_3 ADC1_C3
#ifdef USE_ADC_3
#ifndef ADC_3_GPIO_CLOCK_PORT
#define ADC_3_GPIO_CLOCK_PORT RCC_AHBENR_IOPCEN
#define ADC_3_INIT() gpio_mode_setup(GPIOC, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO0)
#endif
#define USE_AD1_3 1
#else
#define ADC_3_GPIO_CLOCK_PORT 0
#define ADC_3_INIT() {}
#endif

// BAT
#define ADC_4 ADC1_C4
#ifndef ADC_4_GPIO_CLOCK_PORT
#define ADC_4_GPIO_CLOCK_PORT RCC_AHBENR_IOPAEN
#define ADC_4_INIT() gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO3)
#endif
#define USE_AD1_4 1

/* allow to define ADC_CHANNEL_VSUPPLY in the airframe file */
#ifndef ADC_CHANNEL_VSUPPLY
#define ADC_CHANNEL_VSUPPLY ADC_4
#endif

#define ADC_GPIO_CLOCK_PORT (ADC_1_GPIO_CLOCK_PORT | ADC_2_GPIO_CLOCK_PORT | ADC_3_GPIO_CLOCK_PORT | ADC_4_GPIO_CLOCK_PORT)

/* GPIO mapping for ADC1 pins, overwrites the default in arch/stm32/mcu_periph/adc_arch.c */
#ifdef USE_AD1
#define ADC1_GPIO_INIT(gpio) {                  \
    ADC_1_INIT();                               \
    ADC_2_INIT();                               \
    ADC_3_INIT();                               \
    ADC_4_INIT();                               \
  }
#endif // USE_AD1

#define DefaultVoltageOfAdc(adc) (0.00485*adc)


/***************************************************************************************************/
/************************************   ACTUATORS   ************************************************/
/***************************************************************************************************/

/* Default actuators driver */
#define DEFAULT_ACTUATORS "subsystems/actuators/actuators_pwm.h"
#define ActuatorDefaultSet(_x,_y) ActuatorPwmSet(_x,_y)
#define ActuatorsDefaultInit() ActuatorsPwmInit()
#define ActuatorsDefaultCommit() ActuatorsPwmCommit()


/***************************************************************************************************/
/**********************************   SERVO PWM    *************************************************/
/***************************************************************************************************/

#define PWM_USE_TIM1 1
//#define PWM_USE_TIM3 1
#define PWM_USE_TIM5 1
#define PWM_USE_TIM9 1
#define PWM_USE_TIM12 1

#define USE_PWM0  1
#define USE_PWM1  1
#define USE_PWM2  1
#define USE_PWM3  1
#define USE_PWM4  1
#define USE_PWM5  1
#define USE_PWM6  1
#define USE_PWM7  1
#define USE_PWM8  1
#define USE_PWM9  1
#if USE_SPI2
#define USE_PWM10 0
#define USE_PWM11 0
#else
#define USE_PWM10 1
#define USE_PWM11 1
#endif

#define ACTUATORS_PWM_NB 12

// PWM_SERVO_x is the index of the servo in the actuators_pwm_values array
#if USE_PWM0
#define PWM_SERVO_0 0
#define PWM_SERVO_0_TIMER TIM1
#define PWM_SERVO_0_RCC_IOP RCC_AHBENR_IOPEEN
#define PWM_SERVO_0_GPIO GPIOE
#define PWM_SERVO_0_PIN GPIO9
#define PWM_SERVO_0_AF GPIO_AF2
#define PWM_SERVO_0_OC TIM_OC1
#define PWM_SERVO_0_OC_BIT (1<<0)
#else
#define PWM_SERVO_0_OC_BIT 0
#endif

#if USE_PWM1
#define PWM_SERVO_1 1
#define PWM_SERVO_1_TIMER TIM1
#define PWM_SERVO_1_RCC_IOP RCC_AHBENR_IOPEEN
#define PWM_SERVO_1_GPIO GPIOE
#define PWM_SERVO_1_PIN GPIO11
#define PWM_SERVO_1_AF GPIO_AF2
#define PWM_SERVO_1_OC TIM_OC2
#define PWM_SERVO_1_OC_BIT (1<<1)
#else
#define PWM_SERVO_1_OC_BIT 0
#endif

#if USE_PWM2
#define PWM_SERVO_2 2
#define PWM_SERVO_2_TIMER TIM1
#define PWM_SERVO_2_RCC_IOP RCC_AHBENR_IOPEEN
#define PWM_SERVO_2_GPIO GPIOE
#define PWM_SERVO_2_PIN GPIO13
#define PWM_SERVO_2_AF GPIO_AF2
#define PWM_SERVO_2_OC TIM_OC3
#define PWM_SERVO_2_OC_BIT (1<<2)
#else
#define PWM_SERVO_2_OC_BIT 0
#endif

#if USE_PWM3
#define PWM_SERVO_3 3
#define PWM_SERVO_3_TIMER TIM1
#define PWM_SERVO_3_RCC_IOP RCC_AHB1ENR_IOPEEN
#define PWM_SERVO_3_GPIO GPIOE
#define PWM_SERVO_3_PIN GPIO14
#define PWM_SERVO_3_AF GPIO_AF2
#define PWM_SERVO_3_OC TIM_OC4
#define PWM_SERVO_3_OC_BIT (1<<3)
#else
#define PWM_SERVO_3_OC_BIT 0
#endif

#if USE_PWM4
#define PWM_SERVO_4 4
#define PWM_SERVO_4_TIMER TIM8
#define PWM_SERVO_4_RCC_IOP RCC_AHBENR_IOPCEN
#define PWM_SERVO_4_GPIO GPIOC
#define PWM_SERVO_4_PIN GPIO6
#define PWM_SERVO_4_AF GPIO_AF4
#define PWM_SERVO_4_OC TIM_OC1
#define PWM_SERVO_4_OC_BIT (1<<0)
#else
#define PWM_SERVO_4_OC_BIT 0
#endif

#if USE_PWM5
#define PWM_SERVO_5 5
#define PWM_SERVO_5_TIMER TIM8
#define PWM_SERVO_5_RCC_IOP RCC_AHBENR_IOPCEN
#define PWM_SERVO_5_GPIO GPIOC
#define PWM_SERVO_5_PIN GPIO7
#define PWM_SERVO_5_AF GPIO_AF4
#define PWM_SERVO_5_OC TIM_OC2
#define PWM_SERVO_5_OC_BIT (1<<1)
#else
#define PWM_SERVO_5_OC_BIT 0
#endif


#if USE_PWM6
#define PWM_SERVO_6 6
#define PWM_SERVO_6_TIMER TIM8
#define PWM_SERVO_6_RCC_IOP RCC_AHB1ENR_IOPCEN
#define PWM_SERVO_6_GPIO GPIOC
#define PWM_SERVO_6_PIN GPIO8
#define PWM_SERVO_6_AF GPIO_AF4
#define PWM_SERVO_6_OC TIM_OC4
#define PWM_SERVO_6_OC_BIT (1<<2)
#else
#define PWM_SERVO_6_OC_BIT 0
#endif

#if USE_PWM7
#define PWM_SERVO_7 7
#define PWM_SERVO_7_TIMER TIM15
#define PWM_SERVO_7_RCC_IOP RCC_AHBENR_IOPCEN
#define PWM_SERVO_7_GPIO GPIOf
#define PWM_SERVO_7_PIN GPIO10
#define PWM_SERVO_7_AF GPIO_AF3
#define PWM_SERVO_7_OC TIM_OC3
#define PWM_SERVO_7_OC_BIT (1<<1)
#else
#define PWM_SERVO_7_OC_BIT 0
#endif

#if USE_PWM8
#define PWM_SERVO_8 8
#define PWM_SERVO_8_TIMER TIM2
#define PWM_SERVO_8_RCC_IOP RCC_AHBENR_IOPDEN
#define PWM_SERVO_8_GPIO GPIOD
#define PWM_SERVO_8_PIN GPIO4
#define PWM_SERVO_8_AF GPIO_AF2
#define PWM_SERVO_8_OC TIM_OC2
#define PWM_SERVO_8_OC_BIT (1<<1)
#else
#define PWM_SERVO_8_OC_BIT 0
#endif

#if USE_PWM9
#define PWM_SERVO_9 9
#define PWM_SERVO_9_TIMER TIM2
#define PWM_SERVO_9_RCC_IOP RCC_AHB1ENR_IOPDEN
#define PWM_SERVO_9_GPIO GPIOD
#define PWM_SERVO_9_PIN GPIO7
#define PWM_SERVO_9_AF GPIO_AF2
#define PWM_SERVO_9_OC TIM_OC1
#define PWM_SERVO_9_OC_BIT (1<<2)
#else
#define PWM_SERVO_9_OC_BIT 0
#endif

/* PWM10 AND PWM11 cannot be used if SPI2 is active. */
#if USE_PWM10
#define PWM_SERVO_10 10
#define PWM_SERVO_10_TIMER TIM2
#define PWM_SERVO_10_RCC_IOP RCC_AHBENR_IOPDEN
#define PWM_SERVO_10_GPIO GPIOD
#define PWM_SERVO_10_PIN GPIO6
#define PWM_SERVO_10_AF GPIO_AF2
#define PWM_SERVO_10_OC TIM_OC1
#define PWM_SERVO_10_OC_BIT (1<<3)
#else
#define PWM_SERVO_10_OC_BIT 0
#endif

#if USE_PWM11
#define PWM_SERVO_11 11
#define PWM_SERVO_11_TIMER TIM15
#define PWM_SERVO_11_RCC_IOP RCC_AHBENR_IOPFEN
#define PWM_SERVO_11_GPIO GPIOF
#define PWM_SERVO_11_PIN GPIO9
#define PWM_SERVO_11_AF GPIO_AF3
#define PWM_SERVO_11_OC TIM_OC2
#define PWM_SERVO_11_OC_BIT (1<<0)
#else
#define PWM_SERVO_11_OC_BIT 0
#endif

#define PWM_TIM1_CHAN_MASK (PWM_SERVO_0_OC_BIT|PWM_SERVO_1_OC_BIT|PWM_SERVO_2_OC_BIT|PWM_SERVO_3_OC_BIT)
#define PWM_TIM9_CHAN_MASK (PWM_SERVO_4_OC_BIT|PWM_SERVO_5_OC_BIT)
#define PWM_TIM5_CHAN_MASK (PWM_SERVO_6_OC_BIT|PWM_SERVO_7_OC_BIT|PWM_SERVO_8_OC_BIT|PWM_SERVO_9_OC_BIT)
#define PWM_TIM12_CHAN_MASK (PWM_SERVO_10_OC_BIT|PWM_SERVO_11_OC_BIT)

/***************************************************************************************************/
/***********************************   PPM INPUT   *************************************************/
/***************************************************************************************************/

/*
#define USE_PPM_TIM1 1

#define PPM_CHANNEL         TIM_IC1
#define PPM_TIMER_INPUT     TIM_IC_IN_TI1
#define PPM_IRQ             NVIC_TIM1_CC_IRQ
#define PPM_IRQ2            NVIC_TIM1_UP_TIM10_IRQ
// Capture/Compare InteruptEnable and InterruptFlag
#define PPM_CC_IE           TIM_DIER_CC1IE
#define PPM_CC_IF           TIM_SR_CC1IF
#define PPM_GPIO_PORT       GPIOA
#define PPM_GPIO_PIN        GPIO8
#define PPM_GPIO_AF         GPIO_AF1
*/

#define USE_PPM_TIM8 1

#define PPM_CHANNEL         TIM_IC4
#define PPM_TIMER_INPUT     TIM_IC_IN_TI4
#define PPM_IRQ             NVIC_TIM8_CC_IRQ
#define PPM_IRQ2            NVIC_TIM8_UP_TIM13_IRQ
// Capture/Compare InteruptEnable and InterruptFlag
#define PPM_CC_IE           TIM_DIER_CC4IE
#define PPM_CC_IF           TIM_SR_CC4IF
#define PPM_GPIO_PORT       GPIOC
#define PPM_GPIO_PIN        GPIO9
#define PPM_GPIO_AF         GPIO_AF4

/***************************************************************************************************/
/*********************************  SPECTRUM UART  *************************************************/
/***************************************************************************************************/

/* The line that is pulled low at power up to initiate the bind process */
#define SPEKTRUM_BIND_PIN GPIO8
#define SPEKTRUM_BIND_PIN_PORT GPIOA

#define SPEKTRUM_UART2_RCC_REG &RCC_APB2ENR
#define SPEKTRUM_UART2_RCC_DEV RCC_APB2ENR_USART1EN
#define SPEKTRUM_UART2_BANK GPIOA
#define SPEKTRUM_UART2_PIN GPIO10
#define SPEKTRUM_UART2_AF GPIO_AF7
#define SPEKTRUM_UART2_IRQ NVIC_USART1_IRQ
#define SPEKTRUM_UART2_ISR usart1_isr
#define SPEKTRUM_UART2_DEV USART1


#endif /* CONFIG_STM32F43_DISCOVERY_H */
