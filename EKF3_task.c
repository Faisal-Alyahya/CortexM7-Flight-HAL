/**
 * @file    ekf3_task.c
 * @brief   EKF3 Fusion Task demonstrating robust event handling and timeout management
 */

#include "imu_cortex_m7.h"
#include "FreeRTOS.h"
#include "task.h"

TaskHandle_t EKF3_TaskHandle;

void EKF3_Fusion_Task(void *pvParameters)
{
    uint32_t ulNotifiedValue;
    const TickType_t xMaxBlockTime = pdMS_TO_TICKS(15); /* 15ms Absolute Timeout */

    for(;;)
    {
        /* Trigger the hardware read */
        IMU_Trigger_HighSpeed_Read();

        /* Wait for DMA ISR notification (Success, Error, or Timeout) */
        BaseType_t xResult = xTaskNotifyWait(
            0x00,                 /* Do not clear bits on entry */
            0xFFFFFFFF,           /* Clear all bits on exit */
            &ulNotifiedValue,     /* Store the notified value */
            xMaxBlockTime);       /* Timeout to prevent deadlock if hardware silently fails */

        if (xResult == pdPASS)
        {
            if ((ulNotifiedValue & IMU_EVENT_SUCCESS) != 0)
            {
                /* Valid data received, proceed with EKF3 math */
                // EKF3_Update(spi_rx_buffer);
            }
            else if ((ulNotifiedValue & IMU_EVENT_HW_ERROR) != 0)
            {
                /* Bus fault detected. Avoid deadlock and switch to dead-reckoning */
                // EKF3_PredictOnly();
                // Sensor_SwitchToBackup();
            }
        }
        else
        {
            /* Hard Timeout: ISR never fired. Severe hardware failure. */
            System_Trigger_Failsafe(FAILSAFE_CAUSE_IMU_DMA_ERROR);
        }
    }
}
