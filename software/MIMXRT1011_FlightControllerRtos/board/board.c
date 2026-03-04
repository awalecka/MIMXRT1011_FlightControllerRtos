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
#include "fsl_lpuart.h"
#include "fsl_edma.h"
#include "fsl_dmamux.h"

/* MPU configuration. */
void BOARD_ConfigMPU(void)
{
    /* Disable I cache and D cache */
    if (SCB_CCR_IC_Msk == (SCB_CCR_IC_Msk & SCB->CCR)) {
        SCB_DisableICache();
    }
    if (SCB_CCR_DC_Msk == (SCB_CCR_DC_Msk & SCB->CCR)) {
        SCB_DisableDCache();
    }

    /* Disable MPU */
    ARM_MPU_Disable();

    /* MPU configure settings ... */

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

// Define the global DMA buffers, aligned to 32 bytes for Cortex-M7 cache line sizes
uint8_t g_ibusDmaRxBuffer[IBUS_DMA_BUFFER_SIZE] __attribute__((aligned(32)));
uint8_t g_gpsDmaRxBuffer[GPS_DMA_BUFFER_SIZE] __attribute__((aligned(32)));

// Define arrays to hold state for all possible LPUART instances.
// The RT1011 has up to 4 LPUARTs. We allocate 5 to allow 1-based direct indexing.
#define MAX_LPUART_INSTANCES 5
static uart_idle_cb_t s_lpuart_idle_callbacks[MAX_LPUART_INSTANCES] = {NULL};
static edma_handle_t s_lpuart_edma_handles[MAX_LPUART_INSTANCES];

// Helper function to resolve the peripheral base address to an integer index
static uint32_t getLpuartInstance(LPUART_Type *base) {
    if (base == LPUART1) {
        return 1;
    } else if (base == LPUART2) {
        return 2;
    } else if (base == LPUART3) {
        return 3;
    } else if (base == LPUART4) {
        return 4;
    }
    return 0;
}

void boardInitUartDma(LPUART_Type *base, uint32_t baudRate, uint8_t *rxBuffer, size_t rxBufferSize,
		              uart_idle_cb_t idleCallback, DMA_Type *dmaBase, uint32_t dmaChannel,
					  DMAMUX_Type *dmamuxBase, uint32_t dmaSource, IRQn_Type irqn) {

    // Resolve the hardware instance to index our state arrays
    uint32_t instance = getLpuartInstance(base);

    // Store the callback
    s_lpuart_idle_callbacks[instance] = idleCallback;

    // Get the specific DMA handle for this instance
    edma_handle_t *handle = &s_lpuart_edma_handles[instance];

    lpuart_config_t lpuartConfig;
    LPUART_GetDefaultConfig(&lpuartConfig);
    lpuartConfig.baudRate_Bps = baudRate;
    lpuartConfig.enableTx = false;
    lpuartConfig.enableRx = true;
    lpuartConfig.rxFifoWatermark = 0;

    LPUART_Init(base, &lpuartConfig, BOARD_BOOTCLOCKRUN_UART_CLK_ROOT);

    EDMA_CreateHandle(handle, dmaBase, dmaChannel);

    DMAMUX_SetSource(dmamuxBase, dmaChannel, dmaSource);
    DMAMUX_EnableChannel(dmamuxBase, dmaChannel);

    edma_transfer_config_t transferConfig;
    EDMA_PrepareTransfer(&transferConfig,
                         (void *)(uint32_t)LPUART_GetDataRegisterAddress(base),
                         1,
                         rxBuffer,
                         1,
                         1,
                         rxBufferSize,
                         kEDMA_PeripheralToMemory);

    EDMA_SubmitTransfer(handle, &transferConfig);

    dmaBase->TCD[dmaChannel].DLAST_SGA = -((int32_t)rxBufferSize);
    dmaBase->TCD[dmaChannel].CSR &= ~(DMA_CSR_DREQ_MASK);

    EDMA_StartTransfer(handle);

    LPUART_EnableInterrupts(base, kLPUART_IdleLineInterruptEnable);
    NVIC_SetPriority(irqn, 3);
    NVIC_EnableIRQ(irqn);
    LPUART_EnableRxDMA(base, true);
}

// The IRQ handlers are strictly bound to hardware vectors, so they must explicitly index the array.
void LPUART1_IRQHandler(void) {
    uint32_t statusFlags = LPUART_GetStatusFlags(LPUART1);

    if ((statusFlags & kLPUART_IdleLineFlag) != 0U) {
        LPUART_ClearStatusFlags(LPUART1, kLPUART_IdleLineFlag);
        if (s_lpuart_idle_callbacks[1]) {
            s_lpuart_idle_callbacks[1]();
        }
    }

    if ((statusFlags & kLPUART_RxOverrunFlag) != 0U) {
        LPUART_ClearStatusFlags(LPUART1, kLPUART_RxOverrunFlag);
    }
    __DSB();
}

void LPUART4_IRQHandler(void) {
    uint32_t statusFlags = LPUART_GetStatusFlags(LPUART4);

    if ((statusFlags & kLPUART_IdleLineFlag) != 0U) {
        LPUART_ClearStatusFlags(LPUART4, kLPUART_IdleLineFlag);
        if (s_lpuart_idle_callbacks[4]) {
            s_lpuart_idle_callbacks[4]();
        }
    }

    if ((statusFlags & kLPUART_RxOverrunFlag) != 0U) {
        LPUART_ClearStatusFlags(LPUART4, kLPUART_RxOverrunFlag);
    }
    __DSB();
}
