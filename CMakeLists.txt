cmake_minimum_required(VERSION 3.15)
project(STM32F429_Project C ASM)

set(FREERTOS_KERNEL_PATH "FreeRTOS")
# Add the freertos_config for FreeRTOS-Kernel
add_library(freertos_config INTERFACE)
target_include_directories(freertos_config
    INTERFACE
    "Application"
    STM32/CMSIS/Device/ST/STM32F4xx/Include
    STM32/STM32F4xx_StdPeriph_Driver/inc
    STM32/CMSIS/Include
    STM32/CMSIS/RTOS2/Include
)

# Select the heap port.  values between 1-4 will pick a heap.
set(FREERTOS_HEAP "4" CACHE STRING "" FORCE)

set(FREERTOS_PORT "GCC_ARM_CM4F" CACHE STRING "" FORCE)

# Adding the FreeRTOS-Kernel subdirectory
add_subdirectory(${FREERTOS_KERNEL_PATH} FreeRTOS-Kernel)

target_compile_definitions(freertos_kernel PRIVATE STM32F429_439xx STM32F4 USE_STDPERIPH_DRIVER )

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
    -std=c11
)

target_include_directories(freertos_kernel PRIVATE
    ${CMAKE_SOURCE_DIR}/STM32/STM32F4xx_StdPeriph_Driver/inc
)

if(TARGET freertos_kernel_port)
    target_compile_options(freertos_kernel_port PRIVATE
        -mcpu=cortex-m4
        -mthumb
        -mfloat-abi=hard
        -mfpu=fpv4-sp-d16
        -std=c11
    )
    target_compile_definitions(freertos_kernel_port PRIVATE STM32F429_439xx USE_STDPERIPH_DRIVER __FPU_PRESENT=1 ARM_MATH_CM4)
endif()

# set_property(TARGET freertos_kernel PROPERTY C_STANDARD 11)
set(LV_BUILD_CONF_PATH "${CMAKE_SOURCE_DIR}/Application/lv_conf.h" CACHE PATH "lvgl conf" FORCE)
include_directories(${CMAKE_SOURCE_DIR}/Application)
add_subdirectory(lvgl)

# 同步编译选项到 lvgl
if(TARGET lvgl)
    target_compile_options(lvgl PRIVATE
        -mcpu=cortex-m4
        -mthumb
        -mfloat-abi=hard
        -mfpu=fpv4-sp-d16
        -Os
        -Wall
        -Wextra
        -Wno-conversion
        -Wno-unused-parameter
        -std=c11
    )
    target_compile_definitions(lvgl PRIVATE STM32F429_439xx USE_STDPERIPH_DRIVER __FPU_PRESENT=1 ARM_MATH_CM4)
    target_include_directories(lvgl PRIVATE
        ${CMAKE_SOURCE_DIR}/Application
        ${CMAKE_SOURCE_DIR}/Drive/Include
        ${CMAKE_SOURCE_DIR}/FreeRTOS/include
        ${CMAKE_SOURCE_DIR}/FreeRTOS/portable/GCC/ARM_CM4F
        ${CMAKE_SOURCE_DIR}/STM32/CMSIS/Include
        ${CMAKE_SOURCE_DIR}/STM32/CMSIS/Device/ST/STM32F4xx/Include
        ${CMAKE_SOURCE_DIR}/STM32/STM32F4xx_StdPeriph_Driver/inc
        ${CMAKE_SOURCE_DIR}/STM32/CMSIS/RTOS2/Include
    )
endif()


# 同步编译选项到 lvgl
if(TARGET lvgl_examples)
    target_compile_options(lvgl_examples PRIVATE
        -mcpu=cortex-m4
        -mthumb
        -mfloat-abi=hard
        -mfpu=fpv4-sp-d16
        -Os
        -Wall
        -Wextra
        -Wno-conversion
        -Wno-unused-parameter
        -std=c11
    )
    target_compile_definitions(lvgl_examples PRIVATE STM32F429_439xx USE_STDPERIPH_DRIVER __FPU_PRESENT=1 ARM_MATH_CM4)
    target_include_directories(lvgl_examples PRIVATE
        ${CMAKE_SOURCE_DIR}/Application
        ${CMAKE_SOURCE_DIR}/Drive/Include
        ${CMAKE_SOURCE_DIR}/FreeRTOS/include
        ${CMAKE_SOURCE_DIR}/FreeRTOS/portable/GCC/ARM_CM4F
        ${CMAKE_SOURCE_DIR}/STM32/CMSIS/Include
        ${CMAKE_SOURCE_DIR}/STM32/CMSIS/Device/ST/STM32F4xx/Include
        ${CMAKE_SOURCE_DIR}/STM32/STM32F4xx_StdPeriph_Driver/inc
        ${CMAKE_SOURCE_DIR}/STM32/CMSIS/RTOS2/Include
    )
endif()

# 同步编译选项到 lvgl
if(TARGET lvgl_demos)
    target_compile_options(lvgl_demos PRIVATE
        -mcpu=cortex-m4
        -mthumb
        -mfloat-abi=hard
        -mfpu=fpv4-sp-d16
        -Os
        -Wall
        -Wextra
        -Wno-conversion
        -Wno-unused-parameter
        -std=c11
    )
    target_compile_definitions(lvgl_demos PRIVATE STM32F429_439xx USE_STDPERIPH_DRIVER __FPU_PRESENT=1 ARM_MATH_CM4)
    target_include_directories(lvgl_demos PRIVATE
        ${CMAKE_SOURCE_DIR}/Application
        ${CMAKE_SOURCE_DIR}/Drive/Include
        ${CMAKE_SOURCE_DIR}/FreeRTOS/include
        ${CMAKE_SOURCE_DIR}/FreeRTOS/portable/GCC/ARM_CM4F
        ${CMAKE_SOURCE_DIR}/STM32/CMSIS/Include
        ${CMAKE_SOURCE_DIR}/STM32/CMSIS/Device/ST/STM32F4xx/Include
        ${CMAKE_SOURCE_DIR}/STM32/STM32F4xx_StdPeriph_Driver/inc
        ${CMAKE_SOURCE_DIR}/STM32/CMSIS/RTOS2/Include
    )
endif()

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
    "STM32/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_fmc.c"
    "STM32/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_ltdc.c"
    "STM32/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_syscfg.c"
    "STM32/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_dma2d.c"
    "STM32/STM32F4xx_StdPeriph_Driver/src/misc.c"
)


# 所有源文件
set(SOURCES
    ${STARTUP}
    ${APP_SRC}
    ${DRIVER_SRC}
    ${STD_PERIPH_SRC}
)

# 添加 CMSIS-RTOS2 适配层源码
list(APPEND SOURCES STM32/CMSIS/RTOS2/Source/cmsis_os2.c)
list(APPEND SOURCES STM32/CMSIS/RTOS2/Source/os_systick.c)

# 头文件包含路径
include_directories(
    Application
    Drive/Include
    lvgl
    FreeRTOS/include
    FreeRTOS/portable/GCC/ARM_CM4F
    STM32/CMSIS/Include
    STM32/CMSIS/Device/ST/STM32F4xx/Include
    STM32/STM32F4xx_StdPeriph_Driver/inc
    STM32/CMSIS/RTOS2/Include
)

add_compile_definitions(STM32F429_439xx USE_STDPERIPH_DRIVER __FPU_PRESENT=1 ARM_MATH_CM4)

# 创建 elf 文件
add_executable(${PROJECT_NAME}.elf
    ${SOURCES}
    $<TARGET_OBJECTS:freertos_kernel>
    $<TARGET_OBJECTS:lvgl_examples>
    $<TARGET_OBJECTS:lvgl_demos>
    $<TARGET_OBJECTS:lvgl>
)

target_link_libraries(${PROJECT_NAME}.elf freertos_config freertos_kernel)

add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex ${PROJECT_NAME}.elf ${PROJECT_NAME}.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary ${PROJECT_NAME}.elf ${PROJECT_NAME}.bin
)