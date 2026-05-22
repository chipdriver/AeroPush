set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

set(TOOLCHAIN_PREFIX arm-none-eabi)

set(_toolchain_bin_hints "")

if(DEFINED ARM_NONE_EABI_BIN_DIR)
  list(APPEND _toolchain_bin_hints "${ARM_NONE_EABI_BIN_DIR}")
endif()

if(DEFINED ENV{ARM_NONE_EABI_BIN_DIR})
  list(APPEND _toolchain_bin_hints "$ENV{ARM_NONE_EABI_BIN_DIR}")
endif()

if(DEFINED ENV{STM32_TOOLCHAIN_PATH})
  list(APPEND _toolchain_bin_hints "$ENV{STM32_TOOLCHAIN_PATH}/bin")
  list(APPEND _toolchain_bin_hints "$ENV{STM32_TOOLCHAIN_PATH}/tools/bin")
endif()

file(GLOB_RECURSE _stm32cubeide_gcc_bins
  "C:/ST/STM32CubeCLT*/**/arm-none-eabi-gcc.exe"
  "C:/Program Files/STMicroelectronics/STM32CubeCLT*/**/arm-none-eabi-gcc.exe"
  "C:/Program Files (x86)/STMicroelectronics/STM32CubeCLT*/**/arm-none-eabi-gcc.exe"
  "D:/STM32CubeIDE/**/arm-none-eabi-gcc.exe"
)

foreach(_gcc_path IN LISTS _stm32cubeide_gcc_bins)
  get_filename_component(_gcc_bin_dir "${_gcc_path}" DIRECTORY)
  list(APPEND _toolchain_bin_hints "${_gcc_bin_dir}")
endforeach()

find_program(ARM_NONE_EABI_GCC
  NAMES ${TOOLCHAIN_PREFIX}-gcc ${TOOLCHAIN_PREFIX}-gcc.exe
  HINTS ${_toolchain_bin_hints}
)

if(NOT ARM_NONE_EABI_GCC)
  message(FATAL_ERROR "arm-none-eabi-gcc not found. Install STM32CubeCLT or set ARM_NONE_EABI_BIN_DIR to the compiler bin directory.")
endif()

get_filename_component(ARM_NONE_EABI_BIN_DIR "${ARM_NONE_EABI_GCC}" DIRECTORY)

set(CMAKE_C_COMPILER "${ARM_NONE_EABI_BIN_DIR}/${TOOLCHAIN_PREFIX}-gcc.exe" CACHE FILEPATH "C compiler" FORCE)
set(CMAKE_ASM_COMPILER "${ARM_NONE_EABI_BIN_DIR}/${TOOLCHAIN_PREFIX}-gcc.exe" CACHE FILEPATH "ASM compiler" FORCE)
set(CMAKE_OBJCOPY "${ARM_NONE_EABI_BIN_DIR}/${TOOLCHAIN_PREFIX}-objcopy.exe" CACHE FILEPATH "objcopy" FORCE)
set(CMAKE_SIZE "${ARM_NONE_EABI_BIN_DIR}/${TOOLCHAIN_PREFIX}-size.exe" CACHE FILEPATH "size" FORCE)
set(CMAKE_GDB "${ARM_NONE_EABI_BIN_DIR}/${TOOLCHAIN_PREFIX}-gdb.exe" CACHE FILEPATH "gdb" FORCE)

set(CMAKE_C_COMPILER_WORKS TRUE)
set(CMAKE_ASM_COMPILER_WORKS TRUE)
