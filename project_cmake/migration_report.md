# AeroPush Keil to CMake Migration Report

## 原 Keil 工程

- 主工程文件：`../AeroPush.uvprojx`
- 配置文件：`../AeroPush.uvoptx`
- 模板工程：`../Lib/STM32F4xx_StdPeriph_Templates/MDK-ARM/Project.uvprojx`
- 未发现主工程使用独立 `.sct` scatter 文件。

## 芯片型号

- Keil Device：`STM32F411CEUx`
- Keil CPU 描述：`IRAM(0x20000000,0x00020000) IROM(0x08000000,0x00080000) CPUTYPE("Cortex-M4") FPU2 DSP`
- 代码/工程宏：`STM32F411xE`
- 迁移链接脚本按 STM32F411CEUx 设置：
  - Flash：`0x08000000` 起始，`512K`
  - RAM：`0x20000000` 起始，`128K`

## 库类型

- 使用 SPL 标准外设库。
- 依据：
  - 宏定义包含 `USE_STDPERIPH_DRIVER`
  - 存在 `../Lib/Libraries/STM32F4xx_StdPeriph_Driver`
  - 存在 `../User/Inc/stm32f4xx_conf.h`
- 未迁移为 HAL，未引入 `USE_HAL_DRIVER`。
- 未发现本工程源文件直接使用 STM32 LL 驱动。
- 工程同时使用 FreeRTOS，且 Keil 工程中已经选择 GCC Cortex-M4F port：
  - `../FreeRTOS/Portable/GCC/ARM_CM4F/port.c`
  - `../FreeRTOS/Portable/MemMang/heap_4.c`

## 原工程宏定义

- `USE_STDPERIPH_DRIVER`
- `STM32F411xE`

## 原工程 Include 路径

- `../Lib/Libraries/STM32F4xx_StdPeriph_Driver/inc`
- `../Lib/Libraries/CMSIS/Include`
- `../Lib/Libraries/CMSIS/Device/ST/STM32F4xx/Include`
- `../User/Inc`
- `../FreeRTOS/Include`
- `../FreeRTOS/Config`
- `../FreeRTOS/Portable/GCC/ARM_CM4F`

## 原工程源文件列表

### Startup

- Keil/ARMCC 启动文件：`../Lib/Libraries/CMSIS/Device/ST/STM32F4xx/Source/Templates/arm/startup_stm32f411xe.s`

### CMSIS

- `../Lib/Libraries/CMSIS/Device/ST/STM32F4xx/Source/Templates/system_stm32f4xx.c`

### SPL

- `../Lib/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_gpio.c`
- `../Lib/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_rcc.c`
- `../Lib/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_usart.c`

### User

- `../User/Src/main.c`
- `../User/Src/app_main.c`
- `../User/Src/stm32f4xx_it.c`
- `../User/Src/app_tasks.c`
- `../User/Src/imu_service.c`
- `../User/Src/modem_service.c`
- `../User/Src/telemetry_service.c`
- `../User/Src/led_service.c`
- `../User/Src/debug_service.c`
- `../User/Src/freertos_objects.c`
- `../User/Src/freertos_hook.c`
- `../User/Src/app_status.c`
- `../User/Src/bsp_led.c`
- `../User/Src/bsp_debug_uart.c`
- `../User/Src/debug_log.c`
- `../User/Src/delay.c`
- `../User/Src/bsp_i2c_soft.c`
- `../User/Src/mpu9250_driver.c`

### FreeRTOS

- `../FreeRTOS/Source/event_groups.c`
- `../FreeRTOS/Source/list.c`
- `../FreeRTOS/Source/queue.c`
- `../FreeRTOS/Source/stream_buffer.c`
- `../FreeRTOS/Source/tasks.c`
- `../FreeRTOS/Source/timers.c`
- `../FreeRTOS/Portable/GCC/ARM_CM4F/port.c`
- `../FreeRTOS/Portable/MemMang/heap_4.c`

## 启动文件情况

- 原 Keil 工程使用 `Templates/arm/startup_stm32f411xe.s`。
- 该文件包含 `AREA`、`EXPORT`、`DCD`、`PROC` 等 ARMCC/Keil 汇编语法，不能直接用于 GCC。
- 新工程使用 ST SPL 包中现成的 GCC 启动文件：
  - `startup/startup_stm32f411xe.s`
- 已将复制后的启动文件 FPU 指令从 `softvfp` 调整为 `fpv4-sp-d16`，和 Cortex-M4F hard-float 编译参数一致。

## 链接脚本情况

- Keil scatter 文件不能作为 GCC 链接脚本使用。
- 新建 GCC 链接脚本：
  - `linker/STM32F411CEUx_FLASH.ld`
- 保留 GCC 启动文件需要的符号：
  - `Reset_Handler`
  - `SystemInit`
  - `main`
  - `_estack`
  - `_sidata`
  - `_sdata`
  - `_edata`
  - `_sbss`
  - `_ebss`

## FreeRTOS 中断映射

- `../FreeRTOS/Config/FreeRTOSConfig.h` 中已有：
  - `#define vPortSVCHandler     SVC_Handler`
  - `#define xPortPendSVHandler  PendSV_Handler`
  - `#define xPortSysTickHandler SysTick_Handler`
- `../User/Src/stm32f4xx_it.c` 没有定义 `SVC_Handler`、`PendSV_Handler`、`SysTick_Handler`。
- GCC 启动文件中这三个向量为弱符号，最终会由 FreeRTOS GCC port 提供的映射实现接管，不需要在 `stm32f4xx_it.c` 中重复定义。

## 可能的 GCC 兼容性问题

- Keil 启动文件必须替换为 GCC 启动文件。
- Keil scatter 文件不能复用，必须使用 `.ld`。
- 源码目前未发现用户代码使用 `__weak`、`__packed`、`__align` 等 ARMCC 专用写法。
- 代码使用 `snprintf`/`vsnprintf`，但未发现 Keil `fputc` 重定向；当前 CMake 通过 `--specs=nano.specs --specs=nosys.specs` 提供 newlib-nano 和基础 syscall stub。
- `../User/Src/main.c` 内部也调用 `SystemInit()`，而 GCC 启动文件会先调用一次 `SystemInit()`；这是原业务源码行为，本迁移未修改，后续如要收敛启动流程应单独评估。
