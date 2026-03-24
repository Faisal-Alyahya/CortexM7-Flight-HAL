# Cortex-M7 Flight Controller HAL: DMA & Cache Coherency

This repository demonstrates a production-grade, zero-copy Hardware Abstraction Layer (HAL) for high-frequency SPI IMU sensor acquisition (specifically the InvenSense ICM-42688-P and Bosch BMI088). It is explicitly targeted at the STM32H753IIK6 ARM Cortex-M7 microcontroller, which powers the Pixhawk 6X flight controller.

## Architectural Challenges Solved

In high-performance flight controllers running ArduPilot's Extended Kalman Filter (EKF3) at a strictly deterministic 400Hz loop rate, CPU polling or standard interrupts introduce unacceptable latency. Utilizing Hardware DMA resolves latency but introduces critical edge cases on architectures with L1 D-Cache.

This implementation provides a deterministic solution to these challenges:

### 1. D-Cache Coherency & Memory Alignment
Uses CMSIS SCB_CleanDCache_by_Addr and SCB_InvalidateDCache_by_Addr to guarantee the CPU and DMA bus matrix remain perfectly synced. Static buffers are explicitly aligned to the Cortex-M7 32-byte cache line (`__attribute__((aligned(32)))`) to prevent adjacent memory corruption during cache invalidation.

### 2. Deadlock Prevention & Fault Tolerance
Replaces traditional Semaphores with FreeRTOS Task Notifications (`xTaskNotifyFromISR`) passing specific Event Bits (`IMU_EVENT_SUCCESS`, `IMU_EVENT_HW_ERROR`). 

If the SPI bus encounters a hardware fault (specifically, an SPI Overrun Error SPI_FLAG_OVR caused by EMI or bus contention), the Error ISR explicitly notifies the EKF3 task with an error bit. Combined with a strict 15ms task-level timeout, this guarantees the navigation task will never deadlock waiting for a stalled hardware peripheral, allowing the system to gracefully degrade to Dead Reckoning navigation or instantly switch to a redundant secondary IMU on a separate SPI bus.

### 3. Deterministic Execution
Strictly zero dynamic memory allocation (`malloc/free`) to comply with safety-critical embedded standards (MISRA-C:2012) and prevent heap fragmentation mid-flight.

## Use Case
Designed to be integrated directly into a UAV's Flight Management Unit (FMU) stack, connecting the low-level hardware SPI protocol to the high-level ROS2/MAVLink application layers with guaranteed sub-millisecond latency.
