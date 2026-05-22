# AeroPush CMake Migration

本目录是从 Keil uVision 5 工程迁移出的 VS Code + STM32Cube 工具链 CMake 工程。原 Keil 工程文件保留不变，业务源码仍沿用 `../User`、`../Lib`、`../FreeRTOS`，避免产生第二份源码。

## 当前工具链

- MCU：`STM32F411CEUx`
- 架构：Cortex-M4F
- 外设库：STM32F4 SPL 标准外设库
- RTOS：FreeRTOS，使用 `FreeRTOS/Portable/GCC/ARM_CM4F/port.c`
- 编译器：`arm-none-eabi-gcc`
- 构建系统：CMake + Ninja
- 烧录工具：`STM32_Programmer_CLI`

本机已探测到的 STM32CubeIDE 2.1.1 工具路径已经写入 `.vscode/settings.json`。如果换电脑，优先安装 STM32CubeCLT，并把 `stm32.gccBinPath`、`stm32.cmakePath`、`stm32.ninjaPath`、`stm32.programmerPath` 改成新机器路径。

## 构建命令

在 `project_cmake` 目录执行：

```powershell
$cmake = "D:\STM32CubeIDE\STM32CubeIDE_2.1.1\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.cmake.win32_1.1.101.202603101401\tools\bin\cmake.exe"
$gccBin = "D:\STM32CubeIDE\STM32CubeIDE_2.1.1\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.14.3.rel1.win32_1.0.100.202602081740\tools\bin"
$ninja = "D:\STM32CubeIDE\STM32CubeIDE_2.1.1\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.ninja.win32_1.1.100.202601091506\tools\bin\ninja.exe"
& $cmake -S "$PWD" -B "$PWD\build" -G Ninja "-DCMAKE_TOOLCHAIN_FILE=$PWD\cmake\gcc-arm-none-eabi.cmake" "-DARM_NONE_EABI_BIN_DIR=$gccBin" "-DCMAKE_MAKE_PROGRAM=$ninja"
& $cmake --build "$PWD\build" --target all
```

VS Code 中打开 `project_cmake` 文件夹后，可使用：

- `Terminal > Run Task > CMake Configure`
- `Terminal > Run Task > CMake Build`
- `Terminal > Run Task > Flash`

## 烧录命令

不会自动烧录。明确需要烧录时再执行：

```powershell
& "D:\STM32CubeIDE\STM32CubeIDE_2.1.1\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.cmake.win32_1.1.101.202603101401\tools\bin\cmake.exe" --build build --target flash
```

底层命令使用 ST-LINK SWD：

```powershell
STM32_Programmer_CLI -c port=SWD mode=UR -w build/AeroPush.hex -v -rst
```

## 调试

VS Code 中安装 Cortex-Debug 或 ST 官方 STM32 VS Code 插件后，使用 `.vscode/launch.json` 中的 `AeroPush ST-LINK Debug` 配置。需要连接 ST-LINK，接口为 SWD，入口停在 `main`。

## Keil 到 GCC 处理记录

- 启动文件：原 Keil `Templates/arm/startup_stm32f411xe.s` 为 ARMCC 语法，已替换为 ST 提供的 GCC 语法启动文件。
- 链接脚本：未复用 Keil scatter，新增 `linker/STM32F411CEUx_FLASH.ld`。
- 段权限：链接脚本使用 `PHDRS` 分离 Flash 只读可执行段和 RAM 可读写段，避免 GNU ld 生成 RWX LOAD segment。
- FPU 参数：CMake 使用 `-mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard`。
- FreeRTOS port：沿用原工程已经选择的 `FreeRTOS/Portable/GCC/ARM_CM4F/port.c`，未使用 RVDS/ARMCC port。
- FreeRTOS handler：`FreeRTOSConfig.h` 已把 `SVC_Handler`、`PendSV_Handler`、`SysTick_Handler` 映射到 GCC port 实现；`stm32f4xx_it.c` 未重复定义这三个 handler。
- syscall：新增 `Core/Src/syscalls.c`，提供 `_sbrk/_write/_read/_close/_fstat/_isatty/_lseek/_getpid/_kill/_exit`。其中 `_write` 走现有 `BSP_DebugUart_SendChar()`，属于 GCC/newlib 兼容层，不改变业务任务逻辑。
- 业务源码：未因迁移主动修改 `../User` 下源码。

## 生成文件

构建成功后应生成：

- `build/AeroPush.elf`
- `build/AeroPush.hex`
- `build/AeroPush.bin`
- `build/AeroPush.map`

## 本次验证结果

- 验证日期：2026-05-21
- CMake configure：通过。
- CMake clean build：通过。
- 编译日志：`build/cmake_build.log`
- 生成产物：
  - `build/AeroPush.elf`
  - `build/AeroPush.hex`
  - `build/AeroPush.bin`
  - `build/AeroPush.map`
- 内存占用：
  - Flash：`33560 B / 512 KB`
  - RAM：`29496 B / 128 KB`
- 残留警告：
  - `stm32f4xx.h` 内部 `DBGMCU_APB2_FZ_DBG_TIMx_STOP` 宏重复定义，属于当前 SPL/CMSIS 头文件组合的已有警告。
  - `system_stm32f4xx.c` 有 `-Wmisleading-indentation`，来自 ST 模板代码。
  - FreeRTOS GCC port 有一个 `pxVectorTable` 未使用警告。
  - `mpu9250_driver.c` 有一个静态函数未使用警告。
- 本次未执行 `flash` 目标，没有自动烧录。
