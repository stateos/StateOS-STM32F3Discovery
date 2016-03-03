/*******************************************************************************
@file     startup.c
@author   Rajmund Szymanski
@date     03.03.2016
@brief    STM32F3xx startup file.
          After reset the Cortex-M4 processor is in thread mode,
          priority is privileged, and the stack is set to main.
*******************************************************************************/

#ifdef  __GNUC__

#include <stm32f3xx.h>

/*******************************************************************************
 Symbols defined in linker script
*******************************************************************************/

extern unsigned  __data_init_start[];
extern unsigned       __data_start[];
extern unsigned       __data_end  [];
extern unsigned       __data_size [];
extern unsigned        __bss_start[];
extern unsigned        __bss_end  [];
extern unsigned        __bss_size [];

extern void(*__preinit_array_start[])();
extern void(*__preinit_array_end  [])();
extern void(*   __init_array_start[])();
extern void(*   __init_array_end  [])();
extern void(*   __fini_array_start[])();
extern void(*   __fini_array_end  [])();

/*******************************************************************************
 Configuration of stacks
*******************************************************************************/

#ifndef main_stack_size
#define main_stack_size    0 // <- default size of main stack
#endif
#define main_stack (((main_stack_size)+7)&(~7))

#if     main_stack_size > 0
char  __main_stack[main_stack] __attribute__ ((used, section(".main_stack")));
#endif

#ifndef proc_stack_size
#define proc_stack_size 1024 // <- default size of process stack
#endif
#define proc_stack (((proc_stack_size)+7)&(~7))

#if     proc_stack_size > 0
char  __proc_stack[proc_stack] __attribute__ ((used, section(".proc_stack")));
#endif

extern  char  __initial_msp[];
extern  char  __initial_psp[];

/*******************************************************************************
 Default fault handler
*******************************************************************************/

__attribute__ ((weak, noreturn, naked)) void Fault_Handler( void )
{
	/* Go into an infinite loop */
	for (;;);
}

/*******************************************************************************
 Default exit handler
*******************************************************************************/

void _exit( int ) __attribute__ ((weak, alias("Fault_Handler")));

/*******************************************************************************
 Prototypes of external functions
*******************************************************************************/

int   main( void );

/*******************************************************************************
 Default reset procedures
*******************************************************************************/

static inline
void MemCpy( unsigned *dst_, unsigned *end_, unsigned *src_ )
{
	while (dst_ < end_) *dst_++ = *src_++;
}

static inline
void MemSet( unsigned *dst_, unsigned *end_, unsigned val_ )
{
	while (dst_ < end_) *dst_++ = val_;
}

static inline
void DataInit( void )
{
	/* Initialize the data segment */
	MemCpy(__data_start, __data_end, __data_init_start);
	/* Zero fill the bss segment */
	MemSet(__bss_start, __bss_end, 0);
}

static inline
void CallArray( void(**dst_)(), void(**end_)() )
{
	while (dst_ < end_)(*dst_++)();
}

#ifndef __NOSTARTFILES

void __libc_init_array( void );
void __libc_fini_array( void );

#else //__NOSTARTFILES

static inline
void __libc_init_array( void )
{
#ifndef __NOSTARTFILES
	CallArray(__preinit_array_start, __preinit_array_end);
	_init();
#endif
	CallArray(__init_array_start, __init_array_end);
}

static inline
void __libc_fini_array( void )
{
	CallArray(__fini_array_start, __fini_array_end);
#ifndef __NOSTARTFILES
	_fini();
#endif
}

#endif//__NOSTARTFILES

/*******************************************************************************
 Default reset handler
*******************************************************************************/

__attribute__ ((weak, noreturn, naked)) void Reset_Handler( void )
{
#if proc_stack_size > 0
	/* Initialize the process stack pointer */
	__set_PSP((unsigned)__initial_psp);
	__set_CONTROL(CONTROL_SPSEL_Msk);
#endif
#if __FPU_USED
    /* Set CP10 and CP11 Full Access */
	SCB->CPACR = 0x00F00000U;
#endif
#ifndef __NO_SYSTEM_INIT
	/* Call the system clock intitialization function */
	SystemInit();
#endif
	/* Initialize data segments */
	DataInit();
	/* Call global & static constructors */
	__libc_init_array();
	/* Call the application's entry point */
	main();
	/* Call global & static destructors */
	__libc_fini_array();
	/* Go into an infinite loop */
	_exit(0);
}

/*******************************************************************************
 Declaration of exception handlers
*******************************************************************************/

/* Core exceptions */
void NMI_Handler                  (void) __attribute__ ((weak, alias("Fault_Handler")));
void HardFault_Handler            (void) __attribute__ ((weak, alias("Fault_Handler")));
void MemManage_Handler            (void) __attribute__ ((weak, alias("Fault_Handler")));
void BusFault_Handler             (void) __attribute__ ((weak, alias("Fault_Handler")));
void UsageFault_Handler           (void) __attribute__ ((weak, alias("Fault_Handler")));
void SVC_Handler                  (void) __attribute__ ((weak, alias("Fault_Handler")));
void DebugMon_Handler             (void) __attribute__ ((weak, alias("Fault_Handler")));
void PendSV_Handler               (void) __attribute__ ((weak, alias("Fault_Handler")));
void SysTick_Handler              (void) __attribute__ ((weak, alias("Fault_Handler")));

/* External interrupts */
void WWDG_IRQHandler              (void) __attribute__ ((weak, alias("Fault_Handler")));
void PVD_IRQHandler               (void) __attribute__ ((weak, alias("Fault_Handler")));
void TAMP_STAMP_IRQHandler        (void) __attribute__ ((weak, alias("Fault_Handler")));
void RTC_WKUP_IRQHandler          (void) __attribute__ ((weak, alias("Fault_Handler")));
void FLASH_IRQHandler             (void) __attribute__ ((weak, alias("Fault_Handler")));
void RCC_IRQHandler               (void) __attribute__ ((weak, alias("Fault_Handler")));
void EXTI0_IRQHandler             (void) __attribute__ ((weak, alias("Fault_Handler")));
void EXTI1_IRQHandler             (void) __attribute__ ((weak, alias("Fault_Handler")));
void EXTI2_TSC_IRQHandler         (void) __attribute__ ((weak, alias("Fault_Handler")));
void EXTI3_IRQHandler             (void) __attribute__ ((weak, alias("Fault_Handler")));
void EXTI4_IRQHandler             (void) __attribute__ ((weak, alias("Fault_Handler")));
void DMA1_Channel1_IRQHandler     (void) __attribute__ ((weak, alias("Fault_Handler")));
void DMA1_Channel2_IRQHandler     (void) __attribute__ ((weak, alias("Fault_Handler")));
void DMA1_Channel3_IRQHandler     (void) __attribute__ ((weak, alias("Fault_Handler")));
void DMA1_Channel4_IRQHandler     (void) __attribute__ ((weak, alias("Fault_Handler")));
void DMA1_Channel5_IRQHandler     (void) __attribute__ ((weak, alias("Fault_Handler")));
void DMA1_Channel6_IRQHandler     (void) __attribute__ ((weak, alias("Fault_Handler")));
void DMA1_Channel7_IRQHandler     (void) __attribute__ ((weak, alias("Fault_Handler")));
void ADC1_2_IRQHandler            (void) __attribute__ ((weak, alias("Fault_Handler")));
void USB_HP_CAN_TX_IRQHandler     (void) __attribute__ ((weak, alias("Fault_Handler")));
void USB_LP_CAN_RX0_IRQHandler    (void) __attribute__ ((weak, alias("Fault_Handler")));
void CAN_RX1_IRQHandler           (void) __attribute__ ((weak, alias("Fault_Handler")));
void CAN_SCE_IRQHandler           (void) __attribute__ ((weak, alias("Fault_Handler")));
void EXTI9_5_IRQHandler           (void) __attribute__ ((weak, alias("Fault_Handler")));
void TIM1_BRK_TIM15_IRQHandler    (void) __attribute__ ((weak, alias("Fault_Handler")));
void TIM1_UP_TIM16_IRQHandler     (void) __attribute__ ((weak, alias("Fault_Handler")));
void TIM1_TRG_COM_TIM17_IRQHandler(void) __attribute__ ((weak, alias("Fault_Handler")));
void TIM1_CC_IRQHandler           (void) __attribute__ ((weak, alias("Fault_Handler")));
void TIM2_IRQHandler              (void) __attribute__ ((weak, alias("Fault_Handler")));
void TIM3_IRQHandler              (void) __attribute__ ((weak, alias("Fault_Handler")));
void TIM4_IRQHandler              (void) __attribute__ ((weak, alias("Fault_Handler")));
void I2C1_EV_IRQHandler           (void) __attribute__ ((weak, alias("Fault_Handler")));
void I2C1_ER_IRQHandler           (void) __attribute__ ((weak, alias("Fault_Handler")));
void I2C2_EV_IRQHandler           (void) __attribute__ ((weak, alias("Fault_Handler")));
void I2C2_ER_IRQHandler           (void) __attribute__ ((weak, alias("Fault_Handler")));
void SPI1_IRQHandler              (void) __attribute__ ((weak, alias("Fault_Handler")));
void SPI2_IRQHandler              (void) __attribute__ ((weak, alias("Fault_Handler")));
void USART1_IRQHandler            (void) __attribute__ ((weak, alias("Fault_Handler")));
void USART2_IRQHandler            (void) __attribute__ ((weak, alias("Fault_Handler")));
void USART3_IRQHandler            (void) __attribute__ ((weak, alias("Fault_Handler")));
void EXTI15_10_IRQHandler         (void) __attribute__ ((weak, alias("Fault_Handler")));
void RTC_Alarm_IRQHandler         (void) __attribute__ ((weak, alias("Fault_Handler")));
void USBWakeUp_IRQHandler         (void) __attribute__ ((weak, alias("Fault_Handler")));
void TIM8_BRK_IRQHandler          (void) __attribute__ ((weak, alias("Fault_Handler")));
void TIM8_UP_IRQHandler           (void) __attribute__ ((weak, alias("Fault_Handler")));
void TIM8_TRG_COM_IRQHandler      (void) __attribute__ ((weak, alias("Fault_Handler")));
void TIM8_CC_IRQHandler           (void) __attribute__ ((weak, alias("Fault_Handler")));
void ADC3_IRQHandler              (void) __attribute__ ((weak, alias("Fault_Handler")));
void FMC_IRQHandler               (void) __attribute__ ((weak, alias("Fault_Handler")));
void TIM5_IRQHandler              (void) __attribute__ ((weak, alias("Fault_Handler")));
void SPI3_IRQHandler              (void) __attribute__ ((weak, alias("Fault_Handler")));
void UART4_IRQHandler             (void) __attribute__ ((weak, alias("Fault_Handler")));
void UART5_IRQHandler             (void) __attribute__ ((weak, alias("Fault_Handler")));
void TIM6_DAC_IRQHandler          (void) __attribute__ ((weak, alias("Fault_Handler")));
void TIM7_DAC2_IRQHandler         (void) __attribute__ ((weak, alias("Fault_Handler")));
void DMA2_Channel1_IRQHandler     (void) __attribute__ ((weak, alias("Fault_Handler")));
void DMA2_Channel2_IRQHandler     (void) __attribute__ ((weak, alias("Fault_Handler")));
void DMA2_Channel3_IRQHandler     (void) __attribute__ ((weak, alias("Fault_Handler")));
void DMA2_Channel4_IRQHandler     (void) __attribute__ ((weak, alias("Fault_Handler")));
void DMA2_Channel5_IRQHandler     (void) __attribute__ ((weak, alias("Fault_Handler")));
void ADC4_IRQHandler              (void) __attribute__ ((weak, alias("Fault_Handler")));
void SDADC2_IRQHandler            (void) __attribute__ ((weak, alias("Fault_Handler")));
void SDADC3_IRQHandler            (void) __attribute__ ((weak, alias("Fault_Handler")));
void COMP1_2_IRQHandler           (void) __attribute__ ((weak, alias("Fault_Handler")));
void COMP4_6_IRQHandler           (void) __attribute__ ((weak, alias("Fault_Handler")));
void COMP7_IRQHandler             (void) __attribute__ ((weak, alias("Fault_Handler")));
void HRTIM1_Master_IRQHandler     (void) __attribute__ ((weak, alias("Fault_Handler")));
void HRTIM1_TIMA_IRQHandler       (void) __attribute__ ((weak, alias("Fault_Handler")));
void HRTIM1_TIMB_IRQHandler       (void) __attribute__ ((weak, alias("Fault_Handler")));
void HRTIM1_TIMC_IRQHandler       (void) __attribute__ ((weak, alias("Fault_Handler")));
void HRTIM1_TIMD_IRQHandler       (void) __attribute__ ((weak, alias("Fault_Handler")));
void I2C3_EV_IRQHandler           (void) __attribute__ ((weak, alias("Fault_Handler")));
void I2C3_ER_IRQHandler           (void) __attribute__ ((weak, alias("Fault_Handler")));
void USB_HP_IRQHandler            (void) __attribute__ ((weak, alias("Fault_Handler")));
void USB_LP_IRQHandler            (void) __attribute__ ((weak, alias("Fault_Handler")));
void USBWakeUp_RMP_IRQHandler     (void) __attribute__ ((weak, alias("Fault_Handler")));
void TIM20_BRK_IRQHandler         (void) __attribute__ ((weak, alias("Fault_Handler")));
void TIM20_UP_IRQHandler          (void) __attribute__ ((weak, alias("Fault_Handler")));
void TIM20_TRG_COM_IRQHandler     (void) __attribute__ ((weak, alias("Fault_Handler")));
void TIM20_CC_IRQHandler          (void) __attribute__ ((weak, alias("Fault_Handler")));
void FPU_IRQHandler               (void) __attribute__ ((weak, alias("Fault_Handler")));
void SPI4_IRQHandler              (void) __attribute__ ((weak, alias("Fault_Handler")));                    

/*******************************************************************************
 Vector table for STM32F3xx (Cortex-M3)
*******************************************************************************/

void (* const vectors[])(void) __attribute__ ((used, section(".vectors"))) =
{
	/* Initial stack pointer */
	(void(*)(void))__initial_msp,

	/* Core exceptions */
	Reset_Handler,      /* Reset                                   */
	NMI_Handler,        /* Non-maskable interrupt                  */
	HardFault_Handler,  /* All classes of faults                   */
	MemManage_Handler,  /* Memory management                       */
	BusFault_Handler,   /* Pre-fetch fault, memory access fault    */
	UsageFault_Handler, /* Undefined instruction or illegal state  */
	0, 0, 0, 0,         /* Reserved                                */
	SVC_Handler,        /* System service call via SWI instruction */
	DebugMon_Handler,   /* Debug Monitor                           */
	0,                  /* Reserved                                */
	PendSV_Handler,     /* Pendable request for system service     */
	SysTick_Handler,    /* System tick timer                       */

#ifndef __NO_EXTERNAL_INTERRUPTS

	/* External interrupts */
	WWDG_IRQHandler,
	PVD_IRQHandler,
	TAMP_STAMP_IRQHandler,
	RTC_WKUP_IRQHandler,
	FLASH_IRQHandler,
	RCC_IRQHandler,
	EXTI0_IRQHandler,
	EXTI1_IRQHandler,
	EXTI2_TSC_IRQHandler,
	EXTI3_IRQHandler,
	EXTI4_IRQHandler,
	DMA1_Channel1_IRQHandler,
	DMA1_Channel2_IRQHandler,
	DMA1_Channel3_IRQHandler,
	DMA1_Channel4_IRQHandler,
	DMA1_Channel5_IRQHandler,
	DMA1_Channel6_IRQHandler,
	DMA1_Channel7_IRQHandler,
	ADC1_2_IRQHandler,
	USB_HP_CAN_TX_IRQHandler,
	USB_LP_CAN_RX0_IRQHandler,
	CAN_RX1_IRQHandler,
	CAN_SCE_IRQHandler,
	EXTI9_5_IRQHandler,
	TIM1_BRK_TIM15_IRQHandler,
	TIM1_UP_TIM16_IRQHandler,
	TIM1_TRG_COM_TIM17_IRQHandler,
	TIM1_CC_IRQHandler,
	TIM2_IRQHandler,
	TIM3_IRQHandler,
	TIM4_IRQHandler,
	I2C1_EV_IRQHandler,
	I2C1_ER_IRQHandler,
	I2C2_EV_IRQHandler,
	I2C2_ER_IRQHandler,
	SPI1_IRQHandler,
	SPI2_IRQHandler,
	USART1_IRQHandler,
	USART2_IRQHandler,
	USART3_IRQHandler,
	EXTI15_10_IRQHandler,
	RTC_Alarm_IRQHandler,
	USBWakeUp_IRQHandler,
	TIM8_BRK_IRQHandler,
	TIM8_UP_IRQHandler,
	TIM8_TRG_COM_IRQHandler,
	TIM8_CC_IRQHandler,
	ADC3_IRQHandler,
	FMC_IRQHandler,
	0,
	TIM5_IRQHandler,
	SPI3_IRQHandler,
	UART4_IRQHandler,
	UART5_IRQHandler,
	TIM6_DAC_IRQHandler,
	TIM7_DAC2_IRQHandler,
	DMA2_Channel1_IRQHandler,
	DMA2_Channel2_IRQHandler,
	DMA2_Channel3_IRQHandler,
	DMA2_Channel4_IRQHandler,
	DMA2_Channel5_IRQHandler,
	ADC4_IRQHandler,
	SDADC2_IRQHandler,
	SDADC3_IRQHandler,
	COMP1_2_IRQHandler,
	COMP4_6_IRQHandler,
	COMP7_IRQHandler,
	HRTIM1_Master_IRQHandler,
	HRTIM1_TIMA_IRQHandler,
	HRTIM1_TIMB_IRQHandler,
	HRTIM1_TIMC_IRQHandler,
	HRTIM1_TIMD_IRQHandler,
	I2C3_EV_IRQHandler,
	I2C3_ER_IRQHandler,
	USB_HP_IRQHandler,
	USB_LP_IRQHandler,
	USBWakeUp_RMP_IRQHandler,
	TIM20_BRK_IRQHandler,
	TIM20_UP_IRQHandler,
	TIM20_TRG_COM_IRQHandler,
	TIM20_CC_IRQHandler,
#if defined(SPI4_IRQn)||defined(FPU_IRQn)
	FPU_IRQHandler,
#endif
#if defined(SPI4_IRQn)
	0, 0,
	SPI4_IRQHandler,
#endif

#endif//__NO_EXTERNAL_INTERRUPTS
};

/******************************************************************************/

#endif//__GNUC__
