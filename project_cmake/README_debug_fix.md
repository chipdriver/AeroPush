# AeroPush VS Code ST-LINK Debug Fix

## 1. 当前错误

VS Code 选择 `AeroPush ST-LINK Debug` 后报：

```text
ST-LINK: GDB Server Quit Unexpectedly.
See gdb-server output in TERMINAL tab for more details.
```

本次只修复 VS Code / Cortex-Debug / ST-LINK GDB Server 相关配置，不修改业务源代码。

## 2. 原因判断

当前调试配置使用 Cortex-Debug：

```json
"type": "cortex-debug",
"servertype": "stlink"
```

`executable` 已经正确指向 ELF：

```text
${workspaceFolder}/build/AeroPush.elf
```

问题重点在 ST-LINK GDB Server 启动参数。Cortex-Debug 会调用 `ST-LINK_gdbserver.exe`，并需要通过 `-cp` 参数找到 STM32CubeProgrammer 的 `bin` 目录。当前 STM32CubeIDE 安装在 `D:\STM32CubeIDE\...`，如果不显式配置，Cortex-Debug 的自动搜索可能会落到默认的 `C:\ST` 或 `C:\Program Files\...` 路径，导致 GDB Server 启动后立即退出。

## 3. 修改过的文件

```text
E:\Stm32Project\AeroPush\project_cmake\.vscode\launch.json
E:\Stm32Project\AeroPush\project_cmake\.vscode\settings.json
E:\Stm32Project\AeroPush\project_cmake\README_debug_fix.md
```

未修改 `Application`、`BSP`、`Services`、`RTOS_App`、`User`、`FreeRTOS` 等业务或底层源代码。

## 4. launch.json 修改内容

`AeroPush ST-LINK Debug` 保持使用 Cortex-Debug + ST-LINK：

```json
"type": "cortex-debug",
"servertype": "stlink",
"interface": "swd",
"device": "STM32F411CEUx"
```

新增或确认以下字段：

```json
"executable": "${workspaceFolder}/build/AeroPush.elf",
"gdbPath": "${config:stm32.gdbPath}",
"serverpath": "${config:stm32.stlinkGdbServerPath}",
"stm32cubeprogrammer": "${config:stm32.programmerBinPath}",
"serverArgs": [
  "-k",
  "-v"
],
"showDevDebugOutput": "raw",
"showDevDebugTimestamps": true,
"runToEntryPoint": "main",
"preLaunchTask": "编译"
```

说明：

- `stm32cubeprogrammer` 会让 Cortex-Debug 给 `ST-LINK_gdbserver.exe` 传入正确的 `-cp <CubeProgrammer bin>`。
- `-k` 对应 ST-LINK GDB Server 的 under reset 初始化策略。
- Cortex-Debug 本身会自动加入 `--swd`、`--halt` 和动态 GDB 端口。
- 当前配置没有固定使用 `3333`、`61234`、`50000` 等端口，端口冲突风险较低。
- `showDevDebugOutput: raw` 用于输出完整 GDB 交互和 Server 启动细节。

## 5. 当前工具路径

GDB：

```text
D:\STM32CubeIDE\STM32CubeIDE_2.1.1\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.14.3.rel1.win32_1.0.100.202602081740\tools\bin\arm-none-eabi-gdb.exe
```

ST-LINK GDB Server：

```text
D:\STM32CubeIDE\STM32CubeIDE_2.1.1\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.stlink-gdb-server.win32_2.2.400.202601091506\tools\bin\ST-LINK_gdbserver.exe
```

STM32CubeProgrammer CLI：

```text
D:\STM32CubeIDE\STM32CubeIDE_2.1.1\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.cubeprogrammer.win32_2.2.400.202601091506\tools\bin\STM32_Programmer_CLI.exe
```

传给 GDB Server `-cp` 的目录：

```text
D:\STM32CubeIDE\STM32CubeIDE_2.1.1\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.cubeprogrammer.win32_2.2.400.202601091506\tools\bin
```

调试入口 ELF：

```text
E:\Stm32Project\AeroPush\project_cmake\build\AeroPush.elf
```

## 6. 重新调试前手动确认

不要在多个工具同时占用 ST-LINK 时启动调试。重新按 F5 前，请手动确认这些进程已经关闭：

```text
ST-LINK_gdbserver.exe
arm-none-eabi-gdb.exe
STM32_Programmer_CLI.exe
STLinkServer.exe
STM32CubeProgrammer.exe
```

可以在任务管理器里结束，也可以在你确认安全后手动执行对应的 `taskkill` 命令。本次没有自动结束任何进程。

## 7. 如果再次失败

请复制 VS Code 底部 `TERMINAL` 面板里 Cortex-Debug 打开的 `gdb-server` 终端输出，尤其是以下内容：

```text
ST-LINK_gdbserver.exe ...
-cp ...
--swd
--halt
Error ...
```

同时复制 `DEBUG CONSOLE` 中 raw GDB 输出的最后 50 到 100 行。
