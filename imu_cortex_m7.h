/**
 * @file    imu_cortex_m7.h
 * @brief   Hardware Abstraction Layer for High-Speed SPI IMU on Cortex-M7
 */

#ifndef IMU_CORTEX_M7_H
#define IMU_CORTEX_M7_H

#include <stdint.h>
#include "stm32h7xx_hal.h"

/* --- Hardware & Memory Definitions --- */
#define IMU_BURST_READ_CMD       0x80U
#define IMU_PAYLOAD_SIZE         14U   /* 14 bytes: Accel(6), Temp(2), Gyro(6) */
#define CORTEX_M7_CACHE_LINE_SZ  32U

/* --- FreeRTOS Event Bits --- */
#define IMU_EVENT_SUCCESS        (1UL << 0)
#define IMU_EVENT_HW_ERROR       (1UL << 1)

/* --- System Fault Codes --- */
#define FAILSAFE_CAUSE_IMU_DMA_ERROR 0x01U
#define SENSOR_IMU_0                 0x00U

/* --- Function Prototypes --- */
void IMU_Trigger_HighSpeed_Read(void);
void System_Trigger_Failsafe(uint8_t cause);
void HealthMonitor_ReportError(uint8_t sensor_id, uint32_t error_code);

#endif /* IMU_CORTEX_M7_H */
