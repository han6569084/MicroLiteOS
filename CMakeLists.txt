cmake_minimum_required(VERSION 3.15)

project(STM32F429_Project C ASM)

# 添加选项控制内存控制器类型
option(USE_FMC "Use FMC instead of FSMC for memory controller" ON)

# 公共变量
set(CPU_FLAGS -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard)
set(COMMON_C_FLAGS ${CPU_FLAGS} -Wall -O0 -g -fdiagnostics-color=always -Wextra -Wno-conversion -Wno-unused-parameter -std=c11)

# 基础宏定义
set(COMMON_DEFINES STM32F429_439xx USE_STDPERIPH_DRIVER __FPU_PRESENT=1 ARM_MATH_CM4)

# 根据选项添加内存控制器宏定义
if(USE_FMC)
    list(APPEND COMMON_DEFINES USE_FMC)
    message(STATUS "Memory Controller: FMC (Flexible Memory Controller)")
else()
    list(APPEND COMMON_DEFINES USE_FSMC)
    message(STATUS "Memory Controller: FSMC (Flexible Static Memory Controller)")
endif()

set(COMMON_INCLUDES
    ${CMAKE_SOURCE_DIR}/Application
    ${CMAKE_SOURCE_DIR}/ui
    ${CMAKE_SOURCE_DIR}/Drive/Include
    ${CMAKE_SOURCE_DIR}/FreeRTOS/include
    ${CMAKE_SOURCE_DIR}/FreeRTOS/portable/GCC/ARM_CM4F
    ${CMAKE_SOURCE_DIR}/CMSIS_5-develop/CMSIS/Core/Include
    ${CMAKE_SOURCE_DIR}/STM32/CMSIS/Device/ST/STM32F4xx/Include
    ${CMAKE_SOURCE_DIR}/STM32/STM32F4xx_StdPeriph_Driver/inc
    ${CMAKE_SOURCE_DIR}/CMSIS_5-develop/CMSIS/RTOS2/Include
    ${CMAKE_SOURCE_DIR}/CMSIS-FreeRTOS-main/CMSIS/RTOS2/FreeRTOS/Include
)

set(FREERTOS_KERNEL_PATH "FreeRTOS")
# Add the freertos_config for FreeRTOS-Kernel
add_library(freertos_config INTERFACE)
target_include_directories(freertos_config
    INTERFACE
    "Application"
    STM32/CMSIS/Device/ST/STM32F4xx/Include
    STM32/STM32F4xx_StdPeriph_Driver/inc
    CMSIS_5-develop/CMSIS/Core/Include
    CMSIS_5-develop/CMSIS/RTOS2/Include
    CMSIS-FreeRTOS-main/CMSIS/RTOS2/FreeRTOS/Include
)

# Select the heap port.  values between 1-4 will pick a heap.
set(FREERTOS_HEAP "4" CACHE STRING "" FORCE)

set(FREERTOS_PORT "GCC_ARM_CM4F" CACHE STRING "" FORCE)

# Adding the FreeRTOS-Kernel subdirectory
add_subdirectory(${FREERTOS_KERNEL_PATH} FreeRTOS-Kernel)


target_compile_definitions(freertos_kernel PRIVATE ${COMMON_DEFINES} STM32F4)
target_compile_options(freertos_kernel PRIVATE ${COMMON_C_FLAGS})
target_include_directories(freertos_kernel PRIVATE ${COMMON_INCLUDES})


if(TARGET freertos_kernel_port)
    target_compile_options(freertos_kernel_port PRIVATE ${CPU_FLAGS} -std=c11)
    target_compile_definitions(freertos_kernel_port PRIVATE ${COMMON_DEFINES})
endif()


set(LV_BUILD_CONF_PATH "${CMAKE_SOURCE_DIR}/Application/lv_conf.h" CACHE PATH "lvgl conf" FORCE)
include_directories(${CMAKE_SOURCE_DIR}/Application)
add_subdirectory(lvgl)

# 同步编译选项到 lvgl
if(TARGET lvgl)
    target_compile_options(lvgl PRIVATE ${COMMON_C_FLAGS})
    target_compile_definitions(lvgl PRIVATE ${COMMON_DEFINES})
    target_include_directories(lvgl PRIVATE ${COMMON_INCLUDES})
endif()



# 同步编译选项到 lvgl_examples
if(TARGET lvgl_examples)
    target_compile_options(lvgl_examples PRIVATE ${COMMON_C_FLAGS})
    target_compile_definitions(lvgl_examples PRIVATE ${COMMON_DEFINES})
    target_include_directories(lvgl_examples PRIVATE ${COMMON_INCLUDES})
endif()


# 同步编译选项到 lvgl_demos
if(TARGET lvgl_demos)
    target_compile_options(lvgl_demos PRIVATE ${COMMON_C_FLAGS})
    target_compile_definitions(lvgl_demos PRIVATE ${COMMON_DEFINES})
    target_include_directories(lvgl_demos PRIVATE ${COMMON_INCLUDES})
endif()

# 设置链接选项由 target_link_options 传递，避免参数被拼成单一字符串
# 启动文件（汇编）
set(STARTUP startup/startup_stm32f429xx.s)

# 应用层源文件
file(GLOB_RECURSE APP_SRC
    "Application/*.c"
)

file(GLOB_RECURSE COMPONENTS_SRC
    "cm_backtrace/*.c"
)

# 外设驱动（Drive/Source）
file(GLOB_RECURSE DRIVER_SRC
    "Drive/Source/*.c"
)

file(GLOB_RECURSE UI_SRC
    "ui/*.c"
)

# 标准外设库
file(GLOB_RECURSE STD_PERIPH_SRC
    "STM32/STM32F4xx_StdPeriph_Driver/src/*.c"
)

# 根据内存控制器选项选择性移除相应源文件
if(USE_FMC)
    # 使用FMC，移除FSMC源文件
    list(FILTER STD_PERIPH_SRC EXCLUDE REGEX ".*stm32f4xx_fsmc\\.c$")
    message(STATUS "Excluded stm32f4xx_fsmc.c (using FMC)")
else()
    # 使用FSMC，移除FMC源文件
    list(FILTER STD_PERIPH_SRC EXCLUDE REGEX ".*stm32f4xx_fmc\\.c$")
    message(STATUS "Excluded stm32f4xx_fmc.c (using FSMC)")
endif()


# 所有源文件
set(SOURCES
    ${STARTUP}
    ${APP_SRC}
    ${UI_SRC}
    ${COMPONENTS_SRC}
    ${DRIVER_SRC}
    ${STD_PERIPH_SRC}
)

# 添加 CMSIS-RTOS2 适配层源码
list(APPEND SOURCES CMSIS-FreeRTOS-main/CMSIS/RTOS2/FreeRTOS/Source/cmsis_os2.c)
list(APPEND SOURCES CMSIS_5-develop/CMSIS/RTOS2/Source/os_systick.c)



# 创建 elf 文件
add_executable(${PROJECT_NAME}.elf
    ${SOURCES}
    $<TARGET_OBJECTS:freertos_kernel>
    $<TARGET_OBJECTS:lvgl_examples>
    $<TARGET_OBJECTS:lvgl_demos>
    $<TARGET_OBJECTS:lvgl>
)

target_link_options(${PROJECT_NAME}.elf PRIVATE
    ${CPU_FLAGS}
    -Wl,--gc-sections
    -T${CMAKE_SOURCE_DIR}/linker/STM32F429NI_FLASH.ld
    -Wl,-Map=${PROJECT_NAME}.map
)

target_compile_options(${PROJECT_NAME}.elf
    PRIVATE
    ${COMMON_C_FLAGS}
)

target_compile_definitions(${PROJECT_NAME}.elf
    PRIVATE
    ${COMMON_DEFINES}
)

target_include_directories(${PROJECT_NAME}.elf
    PRIVATE
    ${COMMON_INCLUDES}
    ${CMAKE_SOURCE_DIR}/STM32/STM32F4xx_StdPeriph_Driver/inc
    ${CMAKE_SOURCE_DIR}/lvgl
    ${CMAKE_SOURCE_DIR}/cm_backtrace
)

target_link_libraries(${PROJECT_NAME}.elf freertos_config freertos_kernel)

add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex ${PROJECT_NAME}.elf ${PROJECT_NAME}.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary ${PROJECT_NAME}.elf ${PROJECT_NAME}.bin
)