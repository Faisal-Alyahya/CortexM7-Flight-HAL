/**
 * @file    imu_cortex_m7.c
 * @brief   Zero-Copy DMA SPI Driver with D-Cache Coherency and Deadlock Prevention
 */

#include "imu_cortex_m7.h"
#include "FreeRTOS.h"
#include "task.h"

/* --- Memory Alignment for Cortex-M7 D-Cache --- */
#if defined(__GNUC__)
    #define ALIGN_32BYTES __attribute__((aligned(CORTEX_M7_CACHE_LINE_SZ)))
#else
    #define ALIGN_32BYTES
#endif

ALIGN_32BYTES static uint8_t spi_tx_buffer[CORTEX_M7_CACHE_LINE_SZ] = {0};
ALIGN_32BYTES static uint8_t spi_rx_buffer[CORTEX_M7_CACHE_LINE_SZ] = {0};

/* External dependencies */
extern SPI_HandleTypeDef hspi1;
extern TaskHandle_t EKF3_TaskHandle;

void IMU_Trigger_HighSpeed_Read(void)
{
    spi_tx_buffer[0] = IMU_BURST_READ_CMD;

    /* Clean D-Cache to sync SRAM with CPU before DMA TX */
    SCB_CleanDCache_by_Addr((uint32_t*)spi_tx_buffer, CORTEX_M7_CACHE_LINE_SZ);

    /* Assert CS manually */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

    if (HAL_SPI_TransmitReceive_DMA(&hspi1, spi_tx_buffer, spi_rx_buffer, IMU_PAYLOAD_SIZE) != HAL_OK)
    {
        System_Trigger_Failsafe(FAILSAFE_CAUSE_IMU_DMA_ERROR);
    }
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == SPI1)
    {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

        /* Invalidate D-Cache to fetch fresh DMA RX data from SRAM */
        SCB_InvalidateDCache_by_Addr((uint32_t*)spi_rx_buffer, CORTEX_M7_CACHE_LINE_SZ);

        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        
        /* Notify EKF3 Task: SUCCESS */
        xTaskNotifyFromISR(EKF3_TaskHandle, IMU_EVENT_SUCCESS, eSetBits, &xHigherPriorityTaskWoken);
        
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == SPI1)
    {
        uint32_t error_code = hspi->ErrorCode;
        
        HAL_SPI_Abort(&hspi1);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
        
        HealthMonitor_ReportError(SENSOR_IMU_0, error_code);

        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        
        /* Notify EKF3 Task: ERROR (Prevents Task Deadlock) */
        xTaskNotifyFromISR(EKF3_TaskHandle, IMU_EVENT_HW_ERROR, eSetBits, &xHigherPriorityTaskWoken);
        
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

/* Stubs for external compilation */
__weak void System_Trigger_Failsafe(uint8_t cause) { (void)cause; }
__weak void HealthMonitor_ReportError(uint8_t sensor_id, uint32_t error_code) { (void)sensor_id; (void)error_code; }
