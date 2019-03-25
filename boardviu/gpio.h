#ifdef STM32F4
  #include "stm32f4xx_hal_gpio_ex.h"
#else
  #include "stm32f2xx_hal_gpio_ex.h"
#endif

// ********************* dynamic configuration detection *********************

#define PANDA_REV_AB 0
#define PANDA_REV_C 1

#define PULL_EFFECTIVE_DELAY 10

int has_external_debug_serial = 0;
int is_giant_panda = 0;
int is_entering_bootmode = 0;
int revision = PANDA_REV_AB;
int is_grey_panda = 0;

int detect_with_pull(GPIO_TypeDef *GPIO, int pin, int mode) {
  set_gpio_mode(GPIO, pin, MODE_INPUT);
  set_gpio_pullup(GPIO, pin, mode);
  for (volatile int i=0; i<PULL_EFFECTIVE_DELAY; i++);
  int ret = get_gpio_input(GPIO, pin);
  set_gpio_pullup(GPIO, pin, PULL_NONE);
  return ret;
}

// must call again from main because BSS is zeroed
void detect() {

  has_external_debug_serial = 1;
  is_giant_panda = 0;
  is_grey_panda = 0;
  revision = PANDA_REV_C;
  is_entering_bootmode = 0;
  return;

  // detect has_external_debug_serial
  has_external_debug_serial = detect_with_pull(GPIOA, 3, PULL_DOWN);

#ifdef PANDA
  // detect is_giant_panda
  is_giant_panda = detect_with_pull(GPIOB, 1, PULL_DOWN);

  // detect panda REV C.
  // A13 floats in REV AB. In REV C, A13 is pulled up to 5V with a 10K
  // resistor and attached to the USB power control chip CTRL
  // line. Pulling A13 down with an internal 50k resistor in REV C
  // will produce a voltage divider that results in a high logic
  // level. Checking if this pin reads high with a pull down should
  // differentiate REV AB from C.
  revision = detect_with_pull(GPIOA, 13, PULL_DOWN) ? PANDA_REV_C : PANDA_REV_AB;

  // check if the ESP is trying to put me in boot mode
  is_entering_bootmode = !detect_with_pull(GPIOB, 0, PULL_UP);

  // check if it's a grey panda by seeing if the SPI lines are floating
  // TODO: is this reliable?
  is_grey_panda = !(detect_with_pull(GPIOA, 4, PULL_DOWN) | detect_with_pull(GPIOA, 5, PULL_DOWN) | detect_with_pull(GPIOA, 6, PULL_DOWN) | detect_with_pull(GPIOA, 7, PULL_DOWN));
#else
  // need to do this for early detect
  is_giant_panda = 0;
  is_grey_panda = 0;
  revision = PANDA_REV_AB;
  is_entering_bootmode = 0;
#endif

}

// ********************* bringup *********************
#define HSE_VALUE 8000000 //8Mhz
#define PLL_M	  (HSE_VALUE/1000000)
#define PLL_N	  336
#define PLL_P	  2       // it means is zero
#define PLL_Q	  7		

/* 
   PLL_VCO = (HSE_VALUE /PLL_M )*PLL_N = 336MHz
   SYSCLK  = PLL_VCO/PLL_P = 168MHz
   USB OTG FS = PLL_VCO/PLLQ = 48MHz

*/
void clock_init() {
  // enable external oscillator
  RCC->CR |= RCC_CR_HSEON;
  while ((RCC->CR & RCC_CR_HSERDY) == 0);

  // divide shit
  RCC->CFGR = RCC_CFGR_HPRE_DIV1 | RCC_CFGR_PPRE2_DIV2 | RCC_CFGR_PPRE1_DIV4;
  /* Configure the main PLL */
  RCC->PLLCFGR = PLL_M | (PLL_N << 6) | (((PLL_P >> 1) -1) << 16) |
                   (RCC_PLLCFGR_PLLSRC_HSE) | (PLL_Q << 24);

  // start PLL
  RCC->CR |= RCC_CR_PLLON;
  while ((RCC->CR & RCC_CR_PLLRDY) == 0);

  // Configure Flash prefetch, Instruction cache, Data cache and wait state
  // *** without this, it breaks ***
  FLASH->ACR = FLASH_ACR_ICEN | FLASH_ACR_DCEN | FLASH_ACR_LATENCY_5WS;

  // switch to PLL
  RCC->CFGR |= RCC_CFGR_SW_PLL;
  while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);

  // *** running on PLL ***
}

void periph_init() {
  // enable GPIOB, UART2, CAN, USB clock
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;

  RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;
 // RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
 // RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
  #ifdef PANDA
 // RCC->APB1ENR |= RCC_APB1ENR_UART5EN;
  #endif

  RCC->APB2ENR |= RCC_APB2ENR_USART6EN;

  RCC->APB1ENR |= RCC_APB1ENR_CAN1EN;
  RCC->APB1ENR |= RCC_APB1ENR_CAN2EN;
  
 // RCC->APB1ENR |= RCC_APB1ENR_DACEN;
  RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
  RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
  RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
//  RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
  RCC->AHB2ENR |= RCC_AHB2ENR_OTGFSEN;
  RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
// RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
// RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

  // needed?
  RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
}

// ********************* setters *********************

void set_can_enable(CAN_TypeDef *CAN, int enabled) {
  // enable CAN busses
  if (CAN == CAN1) {
    #ifdef PANDA
      // CAN1_EN
     // set_gpio_output(GPIOC, 1, !enabled);
    #endif
  } else if (CAN == CAN2) {
    #ifdef PANDA
      // CAN2_EN
     // set_gpio_output(GPIOC, 13, !enabled);
    #endif
  }

}

#ifdef PANDA
  #define LED_RED   13 
  #define LED_GREEN 14
  #define LED_BLUE  15
#else
  #define LED_RED 10
  #define LED_GREEN 11
  #define LED_BLUE -1
#endif

void set_led(int led_num, int on) {

  if (led_num == -1) return;

  #ifdef PANDA
    set_gpio_output(GPIOD, led_num, on);
  #endif
}

void set_can_mode(int can, int use_gmlan) {
    if (can == 1) {
      // B5,B13: normal mode
      set_gpio_alternate(GPIOB, 5, GPIO_AF9_CAN2);
      set_gpio_alternate(GPIOB, 13, GPIO_AF9_CAN2);
    }
}

#define USB_POWER_NONE 0
#define USB_POWER_CLIENT 1
#define USB_POWER_CDP 2
#define USB_POWER_DCP 3

int usb_power_mode = USB_POWER_NONE;

void set_usb_power_mode(int mode) {

  return;

  switch (mode) {
    case USB_POWER_CLIENT:
      // B2,A13: set client mode
      set_gpio_output(GPIOB, 2, 0);
      set_gpio_output(GPIOA, 13, 1);
      break;
    case USB_POWER_CDP:
      // B2,A13: set CDP mode
      set_gpio_output(GPIOB, 2, 1);
      set_gpio_output(GPIOA, 13, 1);
      break;
    case USB_POWER_DCP:
      // B2,A13: set DCP mode on the charger (breaks USB!)
      set_gpio_output(GPIOB, 2, 0);
      set_gpio_output(GPIOA, 13, 0);
      break;
  }
  usb_power_mode = mode;
}

#define ESP_DISABLED 0
#define ESP_ENABLED 1
#define ESP_BOOTMODE 2

void set_esp_mode(int mode) {
  switch (mode) {
    case ESP_DISABLED:
      // ESP OFF
      set_gpio_output(GPIOC, 14, 0);
      set_gpio_output(GPIOC, 5, 0);
      break;
    case ESP_ENABLED:
      // ESP ON
      set_gpio_output(GPIOC, 14, 1);
      set_gpio_output(GPIOC, 5, 1);
      break;
    case ESP_BOOTMODE:
      set_gpio_output(GPIOC, 14, 1);
      set_gpio_output(GPIOC, 5, 0);
      break;
  }
}

// ********************* big init function *********************

// board specific
void gpio_init() {
  // pull low to hold ESP in reset??
  // enable OTG out tied to ground
  GPIOA->ODR = 0;
  GPIOB->ODR = 0;
  GPIOA->PUPDR = 0;
  //GPIOC->ODR = 0;
  GPIOB->AFR[0] = 0;
  GPIOB->AFR[1] = 0;

  // C2,C3: analog mode, voltage and current sense
  set_gpio_mode(GPIOC, 2, MODE_ANALOG);
  set_gpio_mode(GPIOC, 3, MODE_ANALOG);

#ifdef PEDAL
  // comma pedal has inputs on C0 and C1
  set_gpio_mode(GPIOC, 0, MODE_ANALOG);
  set_gpio_mode(GPIOC, 1, MODE_ANALOG);
  // DAC outputs on A4 and A5
  //   apparently they don't need GPIO setup
#endif

  // C8: FAN aka TIM3_CH4
  set_gpio_alternate(GPIOC, 8, GPIO_AF2_TIM3);

  // turn off LEDs and set mode
  set_led(LED_RED, 0);
  set_led(LED_GREEN, 0);
  set_led(LED_BLUE, 0);

  // A11,A12: USB
  set_gpio_alternate(GPIOA, 11, GPIO_AF10_OTG_FS);
  set_gpio_alternate(GPIOA, 12, GPIO_AF10_OTG_FS);
  GPIOA->OSPEEDR = GPIO_OSPEEDER_OSPEEDR11 | GPIO_OSPEEDER_OSPEEDR12;

#ifdef PANDA
  // enable started_alt on the panda
  set_gpio_pullup(GPIOA, 1, PULL_UP);

  // C6,C7 : USART 6 for debugging
  set_gpio_alternate(GPIOC, 6, GPIO_AF8_USART6);
  set_gpio_alternate(GPIOC, 7, GPIO_AF8_USART6);

  // A9,A10: USART 1 for talking to the ESP
  //set_gpio_alternate(GPIOA, 9, GPIO_AF7_USART1);
  //set_gpio_alternate(GPIOA, 10, GPIO_AF7_USART1);

  // B12: GMLAN, ignition sense, pull up
  //set_gpio_pullup(GPIOB, 12, PULL_UP);

  // A4,A5,A6,A7: setup SPI
  //set_gpio_alternate(GPIOA, 4, GPIO_AF5_SPI1);
  //set_gpio_alternate(GPIOA, 5, GPIO_AF5_SPI1);
  //set_gpio_alternate(GPIOA, 6, GPIO_AF5_SPI1);
  //set_gpio_alternate(GPIOA, 7, GPIO_AF5_SPI1);
#endif

  // B8,B9: CAN 1
#ifdef STM32F4
  set_gpio_alternate(GPIOB, 8, GPIO_AF9_CAN1);
  set_gpio_alternate(GPIOB, 9, GPIO_AF9_CAN1);
#else
  set_gpio_alternate(GPIOB, 8, GPIO_AF9_CAN1);
  set_gpio_alternate(GPIOB, 9, GPIO_AF9_CAN1);
#endif
  set_can_enable(CAN1, 1);

  // B5,B13: CAN 2
  set_can_mode(1, 0);
  set_can_enable(CAN2, 1);

  if (revision == PANDA_REV_C) {
    set_usb_power_mode(USB_POWER_CLIENT);
  }
}

// ********************* early bringup *********************

#define ENTER_BOOTLOADER_MAGIC 0xdeadbeef
#define ENTER_SOFTLOADER_MAGIC 0xdeadc0de
#define BOOT_NORMAL 0xdeadb111

extern void *g_pfnVectors;
extern uint32_t enter_bootloader_mode;

void jump_to_bootloader() {
  // do enter bootloader
  enter_bootloader_mode = 0;
  void (*bootloader)(void) = (void (*)(void)) (*((uint32_t *)0x1fff0004));

  // jump to bootloader
  bootloader();

  // reset on exit
  enter_bootloader_mode = BOOT_NORMAL;
  NVIC_SystemReset();
}

void early() {
  // after it's been in the bootloader, things are initted differently, so we reset
  if (enter_bootloader_mode != BOOT_NORMAL &&
      enter_bootloader_mode != ENTER_BOOTLOADER_MAGIC &&
      enter_bootloader_mode != ENTER_SOFTLOADER_MAGIC) {
    enter_bootloader_mode = BOOT_NORMAL;
    NVIC_SystemReset();
  }

  // if wrong chip, reboot
  volatile unsigned int id = DBGMCU->IDCODE;
  #ifdef STM32F4
    if ((id&0xFFF) != 0x463) enter_bootloader_mode = ENTER_BOOTLOADER_MAGIC;
  #else
    if ((id&0xFFF) != 0x411) enter_bootloader_mode = ENTER_BOOTLOADER_MAGIC;
  #endif

  // setup interrupt table
  SCB->VTOR = (uint32_t)&g_pfnVectors;

  // early GPIOs float everything
  RCC->AHB1ENR = RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN;

  GPIOA->MODER = 0; GPIOB->MODER = 0; GPIOC->MODER = 0;
  GPIOA->ODR = 0; GPIOB->ODR = 0; GPIOC->ODR = 0;
  GPIOA->PUPDR = 0; GPIOB->PUPDR = 0; GPIOC->PUPDR = 0;

  detect();

  if (enter_bootloader_mode == ENTER_BOOTLOADER_MAGIC) {
    set_led(LED_GREEN, 1);
    jump_to_bootloader();
  }

  if (is_entering_bootmode) {
    enter_bootloader_mode = ENTER_SOFTLOADER_MAGIC;
  }
}

