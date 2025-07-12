# build/compiler/gcc-arm-none-eabi/toolchain.cmake
# 编译器路径前缀
set(TOOLCHAIN_BASE_PATH ${CMAKE_SOURCE_DIR}/build/compiler/arm-gnu-toolchain-14/arm-gnu-toolchain-14.3.rel1-x86_64-arm-none-eabi)

# 设置为交叉编译
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

# 工具链前缀
set(TOOLCHAIN_PREFIX arm-none-eabi)

# 查找工具链路径
find_program(CMAKE_C_COMPILER ${TOOLCHAIN_BASE_PATH}/bin/arm-none-eabi-gcc)
find_program(CMAKE_ASM_COMPILER ${TOOLCHAIN_BASE_PATH}/bin/arm-none-eabi-gcc)
find_program(CMAKE_OBJCOPY ${TOOLCHAIN_BASE_PATH}/bin/arm-none-eabi-objcopy)
find_program(CMAKE_SIZE ${TOOLCHAIN_BASE_PATH}/bin/arm-none-eabi-size)

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
