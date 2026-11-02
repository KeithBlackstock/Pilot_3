/* Cortex-M4 startup for GD32F303CCT6. Core exception vectors (0-15) plus
 * device IRQ vectors 0-39, using the standard STM32F1/GD32F30x IRQ
 * numbering -- needed to reach IRQ39 (USART3). The firmware's two enabled
 * peripheral interrupts are IRQ37 (USART1, see gps.cpp) and IRQ39 (USART3,
 * see ibus.cpp). The hardware indexes this
 * table directly by IRQ number, so every slot up to the one we use must
 * be present (Default_Handler) even though nothing else is enabled -- the
 * table can't just stop after the core vectors anymore. */

.syntax unified
.cpu cortex-m4
.fpu fpv4-sp-d16
.thumb

.global Reset_Handler
.global g_pfnVectors
.global USART3_IRQHandler
.global USART1_IRQHandler

.section .isr_vector, "a", %progbits
.type g_pfnVectors, %object
g_pfnVectors:
    .word _estack
    .word Reset_Handler
    .word Default_Handler   /* NMI */
    .word Default_Handler   /* HardFault */
    .word Default_Handler   /* MemManage */
    .word Default_Handler   /* BusFault */
    .word Default_Handler   /* UsageFault */
    .word 0
    .word 0
    .word 0
    .word 0
    .word Default_Handler   /* SVCall */
    .word Default_Handler   /* DebugMon */
    .word 0
    .word Default_Handler   /* PendSV */
    .word Default_Handler   /* SysTick (unused: TICKINT never set) */
    /* IRQ0-31 */
    .word Default_Handler   /* 0  WWDG */
    .word Default_Handler   /* 1  PVD */
    .word Default_Handler   /* 2  TAMPER */
    .word Default_Handler   /* 3  RTC */
    .word Default_Handler   /* 4  FLASH */
    .word Default_Handler   /* 5  RCC */
    .word Default_Handler   /* 6  EXTI0 */
    .word Default_Handler   /* 7  EXTI1 */
    .word Default_Handler   /* 8  EXTI2 */
    .word Default_Handler   /* 9  EXTI3 */
    .word Default_Handler   /* 10 EXTI4 */
    .word Default_Handler   /* 11 DMA1_Channel1 */
    .word Default_Handler   /* 12 DMA1_Channel2 */
    .word Default_Handler   /* 13 DMA1_Channel3 */
    .word Default_Handler   /* 14 DMA1_Channel4 */
    .word Default_Handler   /* 15 DMA1_Channel5 */
    .word Default_Handler   /* 16 DMA1_Channel6 */
    .word Default_Handler   /* 17 DMA1_Channel7 */
    .word Default_Handler   /* 18 ADC1_2 */
    .word Default_Handler   /* 19 USB_HP_CAN1_TX */
    .word Default_Handler   /* 20 USB_LP_CAN1_RX0 */
    .word Default_Handler   /* 21 CAN1_RX1 */
    .word Default_Handler   /* 22 CAN1_SCE */
    .word Default_Handler   /* 23 EXTI9_5 */
    .word Default_Handler   /* 24 TIM1_BRK */
    .word Default_Handler   /* 25 TIM1_UP */
    .word Default_Handler   /* 26 TIM1_TRG_COM */
    .word Default_Handler   /* 27 TIM1_CC */
    .word Default_Handler   /* 28 TIM2 */
    .word Default_Handler   /* 29 TIM3 */
    .word Default_Handler   /* 30 TIM4 */
    .word Default_Handler   /* 31 I2C1_EV */
    /* IRQ32-39 */
    .word Default_Handler   /* 32 I2C1_ER */
    .word Default_Handler   /* 33 I2C2_EV */
    .word Default_Handler   /* 34 I2C2_ER */
    .word Default_Handler   /* 35 SPI1 */
    .word Default_Handler   /* 36 SPI2 */
    .word USART1_IRQHandler /* 37 USART1 -- gps.cpp */
    .word Default_Handler   /* 38 USART2 */
    .word USART3_IRQHandler /* 39 USART3 -- ibus.cpp */
.size g_pfnVectors, . - g_pfnVectors

.section .text.Reset_Handler
.type Reset_Handler, %function
Reset_Handler:
    /* Enable the FPU (CPACR CP10/CP11 full access) before any code runs.
     * This build targets -mfloat-abi=hard, so even early float ops (int-to-
     * float conversions in icm42688.cpp/main.cpp) fault as NOCP UsageFault
     * otherwise -- the FPU is disabled by default at reset. */
    ldr r0, =0xE000ED88
    ldr r1, [r0]
    ldr r2, =0x00F00000
    orr r1, r1, r2
    str r1, [r0]
    dsb
    isb

    ldr r0, =_sidata
    ldr r1, =_sdata
    ldr r2, =_edata
copy_data:
    cmp r1, r2
    bhs copy_data_done
    ldr r3, [r0], #4
    str r3, [r1], #4
    b copy_data
copy_data_done:

    movs r0, #0
    ldr r1, =_sbss
    ldr r2, =_ebss
zero_bss:
    cmp r1, r2
    bhs zero_bss_done
    str r0, [r1], #4
    b zero_bss
zero_bss_done:

    /* Telemetry lands in its own section (see linker script) but is
     * still a zero-initialized global -- clear it the same way as .bss. */
    ldr r1, =_stelemetry
    ldr r2, =_etelemetry
zero_telemetry:
    cmp r1, r2
    bhs zero_telemetry_done
    str r0, [r1], #4
    b zero_telemetry
zero_telemetry_done:

    bl main
    b .
.size Reset_Handler, . - Reset_Handler

.section .text.Default_Handler
.type Default_Handler, %function
Default_Handler:
    b .
.size Default_Handler, . - Default_Handler
