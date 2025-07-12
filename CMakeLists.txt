cmake_minimum_required(VERSION 3.15)
project(STM32F429_Project C ASM)

set(FREERTOS_KERNEL_PATH "FreeRTOS")
# Add the freertos_config for FreeRTOS-Kernel
add_library(freertos_config INTERFACE)
target_include_directories(freertos_config
    INTERFACE
    "Application"
)

# Select the heap port.  values between 1-4 will pick a heap.
set(FREERTOS_HEAP "4" CACHE STRING "" FORCE)

set(FREERTOS_PORT "GCC_ARM_CM4F" CACHE STRING "" FORCE)

# Adding the FreeRTOS-Kernel subdirectory
add_subdirectory(${FREERTOS_KERNEL_PATH} FreeRTOS-Kernel)

target_compile_options(freertos_kernel PRIVATE
    -mcpu=cortex-m4
    -mthumb
    -mfloat-abi=hard
    -mfpu=fpv4-sp-d16
    -Os
    -fdiagnostics-color=always
    -Wall
    -Wextra
    -Wno-conversion
    -Wno-unused-parameter
)

if(TARGET freertos_kernel_port)
    target_compile_options(freertos_kernel_port PRIVATE
        -mcpu=cortex-m4
        -mthumb
        -mfloat-abi=hard
        -mfpu=fpv4-sp-d16
    )
endif()

set_property(TARGET freertos_kernel PROPERTY C_STANDARD 90)

# 设置 CPU 架构
set(CPU_FLAGS "-mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard")

# 编译选项
set(CMAKE_C_FLAGS "${CPU_FLAGS} -Wall -O0 -g")
set(CMAKE_ASM_FLAGS "${CPU_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS
    "${CPU_FLAGS} -Wl,--gc-sections -T${CMAKE_SOURCE_DIR}/linker/STM32F429NI_FLASH.ld -Wl,-Map=${PROJECT_NAME}.map"
)
# 启动文件（汇编）
set(STARTUP startup/startup_stm32f429xx.s)

# 应用层源文件
file(GLOB_RECURSE APP_SRC
    "Application/*.c"
    "Application/led/*.c"
)

# 外设驱动（Drive/Source）
file(GLOB_RECURSE DRIVER_SRC
    "Drive/Source/*.c"
)

# 标准外设库
file(GLOB_RECURSE STD_PERIPH_SRC
    "STM32/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_gpio.c"
    "STM32/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_dma.c"
    "STM32/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_rcc.c"
    "STM32/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_usart.c"
    "STM32/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_exti.c"
    "STM32/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_i2c.c"
    "STM32/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_syscfg.c"
    "STM32/STM32F4xx_StdPeriph_Driver/src/misc.c"
)

# 添加 CMSIS-RTOS2 适配层源码
list(APPEND SOURCES STM32/CMSIS/RTOS2/Source/cmsis_os2.c)

# 所有源文件
set(SOURCES
    ${STARTUP}
    ${APP_SRC}
    ${DRIVER_SRC}
    ${STD_PERIPH_SRC}
)

# 头文件包含路径
include_directories(
    Application
    Drive/Include
    STM32/CMSIS/Include
    STM32/CMSIS/Device/ST/STM32F4xx/Include
    STM32/STM32F4xx_StdPeriph_Driver/inc
    STM32/CMSIS/RTOS2/Include
)
add_compile_definitions(STM32F429_439xx USE_STDPERIPH_DRIVER __FPU_PRESENT=1 __FPU_USED=1 ARM_MATH_CM4)
# 创建 elf 文件
add_executable(${PROJECT_NAME}.elf ${SOURCES})
target_link_libraries(${PROJECT_NAME}.elf freertos_kernel freertos_config)

add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex ${PROJECT_NAME}.elf ${PROJECT_NAME}.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary ${PROJECT_NAME}.elf ${PROJECT_NAME}.bin
)