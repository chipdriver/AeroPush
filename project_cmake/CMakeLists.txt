cmake_minimum_required(VERSION 3.22)

project(AeroPush LANGUAGES C ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_EXECUTABLE_SUFFIX ".elf")

set(PROJECT_ROOT "${CMAKE_CURRENT_LIST_DIR}/..")
set(LINKER_SCRIPT "${CMAKE_CURRENT_LIST_DIR}/linker/STM32F411CEUx_FLASH.ld")

set(MCU_FLAGS
  -mcpu=cortex-m4
  -mthumb
  -mfpu=fpv4-sp-d16
  -mfloat-abi=hard
)

add_executable(${PROJECT_NAME}
  startup/startup_stm32f411xe.s
  Core/Src/syscalls.c
  ${PROJECT_ROOT}/Lib/Libraries/CMSIS/Device/ST/STM32F4xx/Source/Templates/system_stm32f4xx.c
  ${PROJECT_ROOT}/Lib/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_gpio.c
  ${PROJECT_ROOT}/Lib/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_rcc.c
  ${PROJECT_ROOT}/Lib/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_usart.c
  ${PROJECT_ROOT}/User/Src/main.c
  ${PROJECT_ROOT}/User/Src/app_main.c
  ${PROJECT_ROOT}/User/Src/stm32f4xx_it.c
  ${PROJECT_ROOT}/User/Src/app_tasks.c
  ${PROJECT_ROOT}/User/Src/imu_service.c
  ${PROJECT_ROOT}/User/Src/modem_service.c
  ${PROJECT_ROOT}/User/Src/telemetry_service.c
  ${PROJECT_ROOT}/User/Src/led_service.c
  ${PROJECT_ROOT}/User/Src/debug_service.c
  ${PROJECT_ROOT}/User/Src/freertos_objects.c
  ${PROJECT_ROOT}/User/Src/freertos_hook.c
  ${PROJECT_ROOT}/User/Src/app_status.c
  ${PROJECT_ROOT}/User/Src/bsp_led.c
  ${PROJECT_ROOT}/User/Src/bsp_debug_uart.c
  ${PROJECT_ROOT}/User/Src/debug_log.c
  ${PROJECT_ROOT}/User/Src/delay.c
  ${PROJECT_ROOT}/User/Src/bsp_i2c_soft.c
  ${PROJECT_ROOT}/User/Src/mpu9250_driver.c
  ${PROJECT_ROOT}/FreeRTOS/Source/event_groups.c
  ${PROJECT_ROOT}/FreeRTOS/Source/list.c
  ${PROJECT_ROOT}/FreeRTOS/Source/queue.c
  ${PROJECT_ROOT}/FreeRTOS/Source/stream_buffer.c
  ${PROJECT_ROOT}/FreeRTOS/Source/tasks.c
  ${PROJECT_ROOT}/FreeRTOS/Source/timers.c
  ${PROJECT_ROOT}/FreeRTOS/Portable/GCC/ARM_CM4F/port.c
  ${PROJECT_ROOT}/FreeRTOS/Portable/MemMang/heap_4.c
)

target_include_directories(${PROJECT_NAME} PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}/Core/Inc
  ${PROJECT_ROOT}/Lib/Libraries/STM32F4xx_StdPeriph_Driver/inc
  ${PROJECT_ROOT}/Lib/Libraries/CMSIS/Include
  ${PROJECT_ROOT}/Lib/Libraries/CMSIS/Device/ST/STM32F4xx/Include
  ${PROJECT_ROOT}/User/Inc
  ${PROJECT_ROOT}/FreeRTOS/Include
  ${PROJECT_ROOT}/FreeRTOS/Config
  ${PROJECT_ROOT}/FreeRTOS/Portable/GCC/ARM_CM4F
)

target_compile_definitions(${PROJECT_NAME} PRIVATE
  USE_STDPERIPH_DRIVER
  STM32F411xE
)

target_compile_options(${PROJECT_NAME} PRIVATE
  ${MCU_FLAGS}
  -Wall
  -Wextra
  -ffunction-sections
  -fdata-sections
  -fno-common
  -g3
  -Og
)

target_link_options(${PROJECT_NAME} PRIVATE
  ${MCU_FLAGS}
  -T${LINKER_SCRIPT}
  -Wl,--gc-sections
  -Wl,-Map=${CMAKE_BINARY_DIR}/${PROJECT_NAME}.map
  -Wl,--print-memory-usage
  --specs=nano.specs
  -Wl,-u,_printf_float
)

target_link_libraries(${PROJECT_NAME} PRIVATE m)

set_target_properties(${PROJECT_NAME} PROPERTIES
  LINK_DEPENDS "${LINKER_SCRIPT}"
)

add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
  COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${PROJECT_NAME}>
  COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${PROJECT_NAME}> ${CMAKE_BINARY_DIR}/${PROJECT_NAME}.hex
  COMMAND ${CMAKE_OBJCOPY} -O binary -S $<TARGET_FILE:${PROJECT_NAME}> ${CMAKE_BINARY_DIR}/${PROJECT_NAME}.bin
  BYPRODUCTS
    ${CMAKE_BINARY_DIR}/${PROJECT_NAME}.hex
    ${CMAKE_BINARY_DIR}/${PROJECT_NAME}.bin
  COMMENT "Generating hex and bin images"
)

set(_programmer_hints "")
if(DEFINED STM32_PROGRAMMER_CLI)
  list(APPEND _programmer_hints "${STM32_PROGRAMMER_CLI}")
endif()

file(GLOB_RECURSE _stm32_programmer_bins
  "C:/ST/STM32CubeCLT*/**/STM32_Programmer_CLI.exe"
  "C:/Program Files/STMicroelectronics/STM32CubeCLT*/**/STM32_Programmer_CLI.exe"
  "C:/Program Files (x86)/STMicroelectronics/STM32CubeCLT*/**/STM32_Programmer_CLI.exe"
  "D:/STM32CubeIDE/**/STM32_Programmer_CLI.exe"
)

foreach(_programmer_path IN LISTS _stm32_programmer_bins)
  get_filename_component(_programmer_dir "${_programmer_path}" DIRECTORY)
  list(APPEND _programmer_hints "${_programmer_dir}")
endforeach()

find_program(STM32_PROGRAMMER_CLI_PATH
  NAMES STM32_Programmer_CLI STM32_Programmer_CLI.exe
  HINTS ${_programmer_hints}
)

if(STM32_PROGRAMMER_CLI_PATH)
  add_custom_target(flash
    COMMAND "${STM32_PROGRAMMER_CLI_PATH}" -c port=SWD mode=UR -w "${CMAKE_BINARY_DIR}/${PROJECT_NAME}.hex" -v -rst
    DEPENDS ${PROJECT_NAME}
    USES_TERMINAL
    COMMENT "Flashing ${PROJECT_NAME}.hex through ST-LINK SWD"
  )
else()
  add_custom_target(flash
    COMMAND ${CMAKE_COMMAND} -E echo "STM32_Programmer_CLI not found. Install STM32CubeProgrammer or set it in PATH."
    COMMAND ${CMAKE_COMMAND} -E false
    DEPENDS ${PROJECT_NAME}
    USES_TERMINAL
  )
endif()
