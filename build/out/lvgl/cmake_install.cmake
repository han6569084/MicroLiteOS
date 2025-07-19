# Install script for directory: /home/hanzhijian/workspace/stm32/lvgl

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "TRUE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/lvgl" TYPE DIRECTORY FILES "/home/hanzhijian/workspace/stm32/lvgl/src" FILES_MATCHING REGEX "/[^/]*\\.h$" REGEX "/[^/]*\\_private\\.h$" EXCLUDE)
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/lvgl" TYPE FILE FILES
    "/home/hanzhijian/workspace/stm32/Application/FreeRTOSConfig.h"
    "/home/hanzhijian/workspace/stm32/Application/bsp_mpu_exti.h"
    "/home/hanzhijian/workspace/stm32/Application/i2c.h"
    "/home/hanzhijian/workspace/stm32/Application/lv_conf.h"
    "/home/hanzhijian/workspace/stm32/Application/stm32f4xx_conf.h"
    "/home/hanzhijian/workspace/stm32/Application/stm32f4xx_it.h"
    "/home/hanzhijian/workspace/stm32/Application/usart.h"
    "/home/hanzhijian/workspace/stm32/lvgl/lv_version.h"
    "/home/hanzhijian/workspace/stm32/lvgl/lvgl.h"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY FILES "/home/hanzhijian/workspace/stm32/build/out/lvgl/lib/liblvgl.a")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/lvgl" TYPE FILE FILES
    "/home/hanzhijian/workspace/stm32/Application/FreeRTOSConfig.h"
    "/home/hanzhijian/workspace/stm32/Application/bsp_mpu_exti.h"
    "/home/hanzhijian/workspace/stm32/Application/i2c.h"
    "/home/hanzhijian/workspace/stm32/Application/lv_conf.h"
    "/home/hanzhijian/workspace/stm32/Application/stm32f4xx_conf.h"
    "/home/hanzhijian/workspace/stm32/Application/stm32f4xx_it.h"
    "/home/hanzhijian/workspace/stm32/Application/usart.h"
    "/home/hanzhijian/workspace/stm32/lvgl/lv_version.h"
    "/home/hanzhijian/workspace/stm32/lvgl/lvgl.h"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/pkgconfig" TYPE FILE FILES "/home/hanzhijian/workspace/stm32/build/out/lvgl/lvgl.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY FILES "/home/hanzhijian/workspace/stm32/build/out/lvgl/lib/liblvgl_thorvg.a")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/lvgl" TYPE FILE FILES
    "/home/hanzhijian/workspace/stm32/Application/FreeRTOSConfig.h"
    "/home/hanzhijian/workspace/stm32/Application/bsp_mpu_exti.h"
    "/home/hanzhijian/workspace/stm32/Application/i2c.h"
    "/home/hanzhijian/workspace/stm32/Application/lv_conf.h"
    "/home/hanzhijian/workspace/stm32/Application/stm32f4xx_conf.h"
    "/home/hanzhijian/workspace/stm32/Application/stm32f4xx_it.h"
    "/home/hanzhijian/workspace/stm32/Application/usart.h"
    "/home/hanzhijian/workspace/stm32/lvgl/lv_version.h"
    "/home/hanzhijian/workspace/stm32/lvgl/lvgl.h"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/lvgl" TYPE DIRECTORY FILES "/home/hanzhijian/workspace/stm32/lvgl/demos" FILES_MATCHING REGEX "/[^/]*\\.h$")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY FILES "/home/hanzhijian/workspace/stm32/build/out/lvgl/lib/liblvgl_demos.a")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/lvgl" TYPE FILE FILES
    "/home/hanzhijian/workspace/stm32/Application/FreeRTOSConfig.h"
    "/home/hanzhijian/workspace/stm32/Application/bsp_mpu_exti.h"
    "/home/hanzhijian/workspace/stm32/Application/i2c.h"
    "/home/hanzhijian/workspace/stm32/Application/lv_conf.h"
    "/home/hanzhijian/workspace/stm32/Application/stm32f4xx_conf.h"
    "/home/hanzhijian/workspace/stm32/Application/stm32f4xx_it.h"
    "/home/hanzhijian/workspace/stm32/Application/usart.h"
    "/home/hanzhijian/workspace/stm32/lvgl/lv_version.h"
    "/home/hanzhijian/workspace/stm32/lvgl/lvgl.h"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/lvgl" TYPE DIRECTORY FILES "/home/hanzhijian/workspace/stm32/lvgl/examples" FILES_MATCHING REGEX "/[^/]*\\.h$")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY FILES "/home/hanzhijian/workspace/stm32/build/out/lvgl/lib/liblvgl_examples.a")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/lvgl" TYPE FILE FILES
    "/home/hanzhijian/workspace/stm32/Application/FreeRTOSConfig.h"
    "/home/hanzhijian/workspace/stm32/Application/bsp_mpu_exti.h"
    "/home/hanzhijian/workspace/stm32/Application/i2c.h"
    "/home/hanzhijian/workspace/stm32/Application/lv_conf.h"
    "/home/hanzhijian/workspace/stm32/Application/stm32f4xx_conf.h"
    "/home/hanzhijian/workspace/stm32/Application/stm32f4xx_it.h"
    "/home/hanzhijian/workspace/stm32/Application/usart.h"
    "/home/hanzhijian/workspace/stm32/lvgl/lv_version.h"
    "/home/hanzhijian/workspace/stm32/lvgl/lvgl.h"
    )
endif()

