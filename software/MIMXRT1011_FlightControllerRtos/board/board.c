/*
 * Copyright 2019 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "fsl_common.h"
#include "fsl_debug_console.h"
#include "board.h"
#include "fsl_iomuxc.h"

/* MPU configuration. */
void BOARD_ConfigMPU(void)
{
    /* Disable I cache and D cache */
    if (SCB_CCR_IC_Msk == (SCB_CCR_IC_Msk & SCB->CCR))
    {
        SCB_DisableICache();
    }
    if (SCB_CCR_DC_Msk == (SCB_CCR_DC_Msk & SCB->CCR))
    {
        SCB_DisableDCache();
    }

    /* Disable MPU */
    ARM_MPU_Disable();

    /* MPU configure:
     * Use ARM_MPU_RASR(DisableExec, AccessPermission, TypeExtField, IsShareable, IsCacheable, IsBufferable,
     * SubRegionDisable, Size)
     * API in mpu_armv7.h.
     * param DisableExec       Instruction access (XN) disable bit,0=instruction fetches enabled, 1=instruction fetches
     * disabled.
     * param AccessPermission  Data access permissions, allows you to configure read/write access for User and
     * Privileged mode.
     *      Use MACROS defined in mpu_armv7.h:
     * ARM_MPU_AP_NONE/ARM_MPU_AP_PRIV/ARM_MPU_AP_URO/ARM_MPU_AP_FULL/ARM_MPU_AP_PRO/ARM_MPU_AP_RO
     * Combine TypeExtField/IsShareable/IsCacheable/IsBufferable to configure MPU memory access attributes.
     *  TypeExtField  IsShareable  IsCacheable  IsBufferable   Memory Attribute    Shareability        Cache
     *     0             x           0           0             Strongly Ordered    shareable
     *     0             x           0           1              Device             shareable
     *     0             0           1           0              Normal             not shareable   Outer and inner write
     * through no write allocate
     *     0             0           1           1              Normal             not shareable   Outer and inner write
     * back no write allocate
     *     0             1           1           0              Normal             shareable       Outer and inner write
     * through no write allocate
     *     0             1           1           1              Normal             shareable       Outer and inner write
     * back no write allocate
     *     1             0           0           0              Normal             not shareable   outer and inner
     * noncache
     *     1             1           0           0              Normal             shareable       outer and inner
     * noncache
     *     1             0           1           1              Normal             not shareable   outer and inner write
     * back write/read acllocate
     *     1             1           1           1              Normal             shareable       outer and inner write
     * back write/read acllocate
     *     2             x           0           0              Device              not shareable
     *  Above are normal use settings, if your want to see more details or want to config different inner/outter cache
     * policy.
     *  please refer to Table 4-55 /4-56 in arm cortex-M7 generic user guide <dui0646b_cortex_m7_dgug.pdf>
     * param SubRegionDisable  Sub-region disable field. 0=sub-region is enabled, 1=sub-region is disabled.
     * param Size              Region size of the region to be configured. use ARM_MPU_REGION_SIZE_xxx MACRO in
     * mpu_armv7.h.
     */

    /*
     * Add default region to deny access to whole address space to workaround speculative prefetch.
     * Refer to Arm errata 1013783-B for more details.
     *
     */
    /* Region 0 setting: Instruction access disabled, No data access permission. */
    MPU->RBAR = ARM_MPU_RBAR(0, 0x00000000U);
    MPU->RASR = ARM_MPU_RASR(1, ARM_MPU_AP_NONE, 0, 0, 0, 0, 0, ARM_MPU_REGION_SIZE_4GB);

    /* Region 1 setting: Memory with Device type, not shareable, non-cacheable. */
    MPU->RBAR = ARM_MPU_RBAR(1, 0x80000000U);
    MPU->RASR = ARM_MPU_RASR(0, ARM_MPU_AP_FULL, 2, 0, 0, 0, 0, ARM_MPU_REGION_SIZE_512MB);

    /* Region 2 setting: Memory with Device type, not shareable,  non-cacheable. */
    MPU->RBAR = ARM_MPU_RBAR(2, 0x60000000U);
    MPU->RASR = ARM_MPU_RASR(0, ARM_MPU_AP_FULL, 2, 0, 0, 0, 0, ARM_MPU_REGION_SIZE_512MB);

#if defined(XIP_EXTERNAL_FLASH) && (XIP_EXTERNAL_FLASH == 1)
    /* Region 3 setting: Memory with Normal type, not shareable, outer/inner write back. */
    MPU->RBAR = ARM_MPU_RBAR(3, 0x60000000U);
    MPU->RASR = ARM_MPU_RASR(0, ARM_MPU_AP_RO, 0, 0, 1, 1, 0, ARM_MPU_REGION_SIZE_16MB);
#endif

    /* Region 4 setting: Memory with Device type, not shareable, non-cacheable. */
    MPU->RBAR = ARM_MPU_RBAR(4, 0x00000000U);
    MPU->RASR = ARM_MPU_RASR(0, ARM_MPU_AP_FULL, 2, 0, 0, 0, 0, ARM_MPU_REGION_SIZE_1GB);

    /* Region 5 setting: Memory with Normal type, not shareable, outer/inner write back */
    MPU->RBAR = ARM_MPU_RBAR(5, 0x00000000U);
    MPU->RASR = ARM_MPU_RASR(0, ARM_MPU_AP_FULL, 0, 0, 1, 1, 0, ARM_MPU_REGION_SIZE_32KB);

    /* Region 6 setting: Memory with Normal type, not shareable, outer/inner write back */
    MPU->RBAR = ARM_MPU_RBAR(6, 0x20000000U);
    MPU->RASR = ARM_MPU_RASR(0, ARM_MPU_AP_FULL, 0, 0, 1, 1, 0, ARM_MPU_REGION_SIZE_32KB);

    /* Region 7 setting: Memory with Normal type, not shareable, outer/inner write back */
    MPU->RBAR = ARM_MPU_RBAR(7, 0x20200000U);
    MPU->RASR = ARM_MPU_RASR(0, ARM_MPU_AP_FULL, 0, 0, 1, 1, 0, ARM_MPU_REGION_SIZE_64KB);

    /* Region 9 setting: Memory with Device type, not shareable, non-cacheable */
    MPU->RBAR = ARM_MPU_RBAR(9, 0x40000000);
    MPU->RASR = ARM_MPU_RASR(0, ARM_MPU_AP_FULL, 2, 0, 0, 0, 0, ARM_MPU_REGION_SIZE_2MB);

    /* Region 10 setting: Memory with Device type, not shareable, non-cacheable */
    MPU->RBAR = ARM_MPU_RBAR(10, 0x42000000);
    MPU->RASR = ARM_MPU_RASR(0, ARM_MPU_AP_FULL, 2, 0, 0, 0, 0, ARM_MPU_REGION_SIZE_32MB);

    /* Enable MPU */
    ARM_MPU_Enable(MPU_CTRL_PRIVDEFENA_Msk);

    /* Enable I cache and D cache */
    SCB_EnableDCache();
    SCB_EnableICache();
}

#if defined(XIP_EXTERNAL_FLASH) && (XIP_EXTERNAL_FLASH == 1)
/* SystemInitHook */
void SystemInitHook(void)
{
    /* When set this bit, FlexSPI will fetch more data than AHB burst required to meet the alignment requirement. */
    FLEXSPI->AHBCR |= FLEXSPI_AHBCR_READADDROPT_MASK;
}
#endif

#include "fsl_lpuart.h"
#include "fsl_edma.h"
#include "fsl_dmamux.h"

// Define the global DMA buffer for IBUS in board.c
uint8_t g_dmaRxBuffer[IBUS_DMA_BUFFER_SIZE] __attribute__((aligned(4)));

static edma_handle_t s_edmaHandle;
static ibus_rx_idle_callback_t s_ibus_callback = NULL;

static uart_rx_callback_t s_gps_callback = NULL;

void BOARD_InitGPS(uart_rx_callback_t callback) {
    s_gps_callback = callback;
    // Note: The MCUXpresso configuration handles the Peripheral setup 
    // and NVIC priority/enable. See LPUART1_GPS_init() in peripherals.c.
}

void LPUART1_IRQHandler(void) {
    uint32_t statusFlags = LPUART_GetStatusFlags(LPUART1);

    if ((statusFlags & kLPUART_RxDataRegFullFlag) != 0U) {
        // Read byte clears the flag automatically
        uint8_t byte = LPUART_ReadByte(LPUART1);
        if (s_gps_callback) {
            s_gps_callback(byte);
        }
    }

    if ((statusFlags & kLPUART_RxOverrunFlag) != 0U) {
        LPUART_ClearStatusFlags(LPUART1, kLPUART_RxOverrunFlag);
    }
    __DSB();
}

void BOARD_InitIBUS(ibus_rx_idle_callback_t callback)
{
    s_ibus_callback = callback;

    lpuart_config_t lpuartConfig;
    LPUART_GetDefaultConfig(&lpuartConfig);
    lpuartConfig.baudRate_Bps = 115200;
    lpuartConfig.enableTx = false;
    lpuartConfig.enableRx = true;
    lpuartConfig.rxFifoWatermark = 0; // DMA request on every byte

    LPUART_Init(IBUS_LPUART_INSTANCE, &lpuartConfig, BOARD_BOOTCLOCKRUN_UART_CLK_ROOT);

    EDMA_CreateHandle(&s_edmaHandle, IBUS_DMA_BASE, IBUS_DMA_CHANNEL);

    DMAMUX_SetSource(IBUS_DMAMUX_BASE, IBUS_DMA_CHANNEL, IBUS_DMA_SOURCE);
    DMAMUX_EnableChannel(IBUS_DMAMUX_BASE, IBUS_DMA_CHANNEL);

    edma_transfer_config_t transferConfig;
    EDMA_PrepareTransfer(&transferConfig,
                         (void *)(uint32_t)LPUART_GetDataRegisterAddress(IBUS_LPUART_INSTANCE),
                         1,
                         g_dmaRxBuffer,
                         1,
                         1,
                         IBUS_DMA_BUFFER_SIZE,
                         kEDMA_PeripheralToMemory);

    EDMA_SubmitTransfer(&s_edmaHandle, &transferConfig);

    // Hardware Loop / Ring Buffer Logic
    IBUS_DMA_BASE->TCD[IBUS_DMA_CHANNEL].DLAST_SGA = -((int32_t)IBUS_DMA_BUFFER_SIZE);
    IBUS_DMA_BASE->TCD[IBUS_DMA_CHANNEL].CSR &= ~(DMA_CSR_DREQ_MASK);

    EDMA_StartTransfer(&s_edmaHandle);

    LPUART_EnableInterrupts(IBUS_LPUART_INSTANCE, kLPUART_IdleLineInterruptEnable);
    // Note: Assuming priority 3 is safe above configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY (usually 2 for M7)
    NVIC_SetPriority(IBUS_LPUART_IRQn, 3);
    NVIC_EnableIRQ(IBUS_LPUART_IRQn);
    LPUART_EnableRxDMA(IBUS_LPUART_INSTANCE, true);
}

void LPUART4_IRQHandler(void) {
    uint32_t statusFlags = LPUART_GetStatusFlags(IBUS_LPUART_INSTANCE);

    if ((statusFlags & kLPUART_IdleLineFlag) != 0U) {
        LPUART_ClearStatusFlags(IBUS_LPUART_INSTANCE, kLPUART_IdleLineFlag);
        if (s_ibus_callback) {
            s_ibus_callback();
        }
    }

    if ((statusFlags & kLPUART_RxOverrunFlag) != 0U) {
        LPUART_ClearStatusFlags(IBUS_LPUART_INSTANCE, kLPUART_RxOverrunFlag);
    }
    __DSB();
}
