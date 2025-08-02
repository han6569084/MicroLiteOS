# MicroLiteOS 精细化项目结构设计

## 详细目录结构

```
MicroLiteOS/
├── README.md                    # 项目说明文档
├── CMakeLists.txt              # 顶层CMake配置
├── .gitignore                  # Git忽略文件配置
├── LICENSE                     # 许可证文件
├── VERSION                     # 版本信息
│
├── build/                      # 构建输出目录 (gitignore)
│   ├── Debug/
│   ├── Release/
│   ├── Test/
│   └── Coverage/
│
├── configs/                    # 配置管理层
│   ├── boards/                 # 板卡配置
│   │   ├── stm32f429_discovery/
│   │   │   ├── board_config.h
│   │   │   ├── pin_mapping.h
│   │   │   └── feature_config.h
│   │   └── stm32f407_custom/
│   ├── environments/           # 环境配置
│   │   ├── development/
│   │   │   ├── debug_config.h
│   │   │   └── log_config.h
│   │   ├── testing/
│   │   └── production/
│   │       ├── optimize_config.h
│   │       └── security_config.h
│   ├── features/               # 功能配置
│   │   ├── ui_features.h
│   │   ├── communication_features.h
│   │   └── sensor_features.h
│   └── cmake/                  # CMake配置模块
│       ├── toolchain/
│       │   ├── arm-none-eabi.cmake
│       │   └── gcc-flags.cmake
│       ├── dependencies/
│       │   ├── freertos.cmake
│       │   ├── lvgl.cmake
│       │   └── hal.cmake
│       └── utils/
│           ├── code_generation.cmake
│           └── documentation.cmake
│
├── application/                # 应用程序层
│   ├── bootloader/             # 引导加载器
│   │   ├── include/
│   │   │   ├── bootloader.h
│   │   │   └── update_manager.h
│   │   └── src/
│   │       ├── bootloader.c
│   │       └── update_manager.c
│   ├── main/                   # 主应用程序
│   │   ├── include/
│   │   │   ├── app_main.h
│   │   │   ├── app_config.h
│   │   │   ├── app_version.h
│   │   │   └── module_registry.h
│   │   └── src/
│   │       ├── main.c
│   │       ├── app_init.c
│   │       ├── app_task.c
│   │       └── module_registry.c
│   ├── tasks/                  # 应用任务管理
│   │   ├── include/
│   │   │   ├── task_manager.h
│   │   │   ├── task_monitor.h
│   │   │   └── task_scheduler.h
│   │   └── src/
│   │       ├── task_manager.c
│   │       ├── task_monitor.c
│   │       └── task_scheduler.c
│   └── services/               # 应用服务
│       ├── system_service/
│       │   ├── include/
│       │   │   └── system_service.h
│       │   └── src/
│       │       └── system_service.c
│       ├── data_service/
│       │   ├── include/
│       │   │   ├── data_collector.h
│       │   │   └── data_processor.h
│       │   └── src/
│       │       ├── data_collector.c
│       │       └── data_processor.c
│       └── ui_service/
│           ├── include/
│           │   ├── ui_service.h
│           │   └── screen_router.h
│           └── src/
│               ├── ui_service.c
│               └── screen_router.c
│
├── core/                       # 系统核心层
│   ├── kernel/                 # 内核管理
│   │   ├── freertos/
│   │   │   ├── config/
│   │   │   │   ├── FreeRTOSConfig.h
│   │   │   │   ├── task_priorities.h
│   │   │   │   └── stack_sizes.h
│   │   │   ├── extensions/
│   │   │   │   ├── include/
│   │   │   │   │   ├── freertos_ext.h
│   │   │   │   │   ├── task_stats.h
│   │   │   │   │   └── heap_monitor.h
│   │   │   │   └── src/
│   │   │   │       ├── freertos_hooks.c
│   │   │   │       ├── task_stats.c
│   │   │   │       └── heap_monitor.c
│   │   │   └── ports/
│   │   │       ├── cortex_m4/
│   │   │       └── cortex_m7/
│   │   ├── scheduler/
│   │   │   ├── include/
│   │   │   │   ├── scheduler.h
│   │   │   │   └── priority_manager.h
│   │   │   └── src/
│   │   │       ├── scheduler.c
│   │   │       └── priority_manager.c
│   │   └── ipc/                # 进程间通信
│   │       ├── include/
│   │       │   ├── message_queue.h
│   │       │   ├── semaphore_manager.h
│   │       │   └── event_flags.h
│   │       └── src/
│   │           ├── message_queue.c
│   │           ├── semaphore_manager.c
│   │           └── event_flags.c
│   ├── memory/                 # 内存管理子系统
│   │   ├── heap/
│   │   │   ├── include/
│   │   │   │   ├── heap_manager.h
│   │   │   │   ├── heap_allocator.h
│   │   │   │   └── heap_debug.h
│   │   │   └── src/
│   │   │       ├── heap_manager.c
│   │   │       ├── heap_allocator.c
│   │   │       └── heap_debug.c
│   │   ├── pool/
│   │   │   ├── include/
│   │   │   │   ├── memory_pool.h
│   │   │   │   └── pool_allocator.h
│   │   │   └── src/
│   │   │       ├── memory_pool.c
│   │   │       └── pool_allocator.c
│   │   ├── dma/
│   │   │   ├── include/
│   │   │   │   ├── dma_manager.h
│   │   │   │   └── dma_allocator.h
│   │   │   └── src/
│   │   │       ├── dma_manager.c
│   │   │       └── dma_allocator.c
│   │   └── cache/
│   │       ├── include/
│   │       │   ├── cache_manager.h
│   │       │   └── cache_coherency.h
│   │       └── src/
│   │           ├── cache_manager.c
│   │           └── cache_coherency.c
│   ├── time/                   # 时间管理子系统
│   │   ├── rtc/
│   │   │   ├── include/
│   │   │   │   ├── rtc_manager.h
│   │   │   │   └── rtc_calendar.h
│   │   │   └── src/
│   │   │       ├── rtc_manager.c
│   │   │       └── rtc_calendar.c
│   │   ├── timestamp/
│   │   │   ├── include/
│   │   │   │   ├── timestamp.h
│   │   │   │   └── time_sync.h
│   │   │   └── src/
│   │   │       ├── timestamp.c
│   │   │       └── time_sync.c
│   │   └── timer/
│   │       ├── include/
│   │       │   ├── soft_timer.h
│   │       │   ├── hw_timer.h
│   │       │   └── timer_pool.h
│   │       └── src/
│   │           ├── soft_timer.c
│   │           ├── hw_timer.c
│   │           └── timer_pool.c
│   ├── interrupt/              # 中断管理子系统
│   │   ├── include/
│   │   │   ├── irq_manager.h
│   │   │   ├── nvic_config.h
│   │   │   └── irq_priority.h
│   │   └── src/
│   │       ├── irq_manager.c
│   │       ├── nvic_config.c
│   │       └── irq_handlers.c
│   └── system/                 # 系统管理
│       ├── include/
│       │   ├── system_info.h
│       │   ├── error_handler.h
│       │   ├── reset_manager.h
│       │   ├── watchdog_manager.h
│       │   └── power_manager.h
│       └── src/
│           ├── system_info.c
│           ├── error_handler.c
│           ├── reset_manager.c
│           ├── watchdog_manager.c
│           └── power_manager.c
│
├── platform/                  # 平台抽象层
│   ├── abstraction/            # 硬件抽象层
│   │   ├── include/
│   │   │   ├── hal_interface.h  # HAL接口定义
│   │   │   ├── platform_types.h # 平台类型定义
│   │   │   └── platform_api.h   # 平台API接口
│   │   └── src/
│   │       ├── hal_adapter.c    # HAL适配器
│   │       └── platform_impl.c  # 平台实现
│   ├── stm32f4xx/             # STM32F4平台实现
│   │   ├── sdk/               # 原厂SDK
│   │   │   ├── CMSIS/         # ARM CMSIS
│   │   │   │   ├── Core/
│   │   │   │   │   ├── Include/
│   │   │   │   │   └── Source/
│   │   │   │   ├── Device/
│   │   │   │   │   └── ST/
│   │   │   │   │       └── STM32F4xx/
│   │   │   │   │           ├── Include/
│   │   │   │   │           └── Source/
│   │   │   │   └── RTOS2/
│   │   │   ├── HAL_Driver/    # STM32 HAL库
│   │   │   │   ├── Inc/
│   │   │   │   └── Src/
│   │   │   └── startup/       # 启动文件
│   │   │       ├── startup_stm32f429xx.s
│   │   │       └── system_stm32f4xx.c
│   │   ├── bsp/               # 板级支持包
│   │   │   ├── include/
│   │   │   │   ├── board.h     # 板卡定义
│   │   │   │   ├── pin_config.h # 引脚配置
│   │   │   │   ├── clock_config.h # 时钟配置
│   │   │   │   └── memory_map.h # 内存映射
│   │   │   └── src/
│   │   │       ├── board.c
│   │   │       ├── pin_config.c
│   │   │       ├── clock_config.c
│   │   │       └── system_init.c
│   │   ├── drivers/           # 平台特定驱动
│   │   │   ├── gpio/
│   │   │   │   ├── include/
│   │   │   │   │   └── stm32f4_gpio.h
│   │   │   │   └── src/
│   │   │   │       └── stm32f4_gpio.c
│   │   │   ├── rcc/
│   │   │   │   ├── include/
│   │   │   │   │   └── stm32f4_rcc.h
│   │   │   │   └── src/
│   │   │   │       └── stm32f4_rcc.c
│   │   │   └── nvic/
│   │   │       ├── include/
│   │   │       │   └── stm32f4_nvic.h
│   │   │       └── src/
│   │   │           └── stm32f4_nvic.c
│   │   └── linker/            # 链接脚本
│   │       ├── STM32F429NI_FLASH.ld
│   │       ├── memory_regions.ld
│   │       └── section_layout.ld
│   └── ports/                 # 其他平台移植
│       ├── cortex_m7/
│       └── risc_v/
│
├── drivers/                   # 设备驱动层
│   ├── bus/                   # 总线驱动
│   │   ├── gpio/
│   │   │   ├── core/
│   │   │   │   ├── include/
│   │   │   │   │   ├── gpio_driver.h
│   │   │   │   │   └── gpio_interface.h
│   │   │   │   └── src/
│   │   │   │       └── gpio_driver.c
│   │   │   └── extensions/
│   │   │       ├── include/
│   │   │       │   ├── gpio_interrupt.h
│   │   │       │   └── gpio_debounce.h
│   │   │       └── src/
│   │   │           ├── gpio_interrupt.c
│   │   │           └── gpio_debounce.c
│   │   ├── uart/
│   │   │   ├── core/
│   │   │   │   ├── include/
│   │   │   │   │   ├── uart_driver.h
│   │   │   │   │   └── uart_interface.h
│   │   │   │   └── src/
│   │   │   │       └── uart_driver.c
│   │   │   ├── protocols/
│   │   │   │   ├── include/
│   │   │   │   │   ├── uart_dma.h
│   │   │   │   │   └── uart_interrupt.h
│   │   │   │   └── src/
│   │   │   │       ├── uart_dma.c
│   │   │   │       └── uart_interrupt.c
│   │   │   └── utils/
│   │   │       ├── include/
│   │   │       │   ├── uart_buffer.h
│   │   │       │   └── uart_protocol.h
│   │   │       └── src/
│   │   │           ├── uart_buffer.c
│   │   │           └── uart_protocol.c
│   │   ├── i2c/
│   │   │   ├── core/
│   │   │   │   ├── include/
│   │   │   │   │   ├── i2c_driver.h
│   │   │   │   │   └── i2c_interface.h
│   │   │   │   └── src/
│   │   │   │       └── i2c_driver.c
│   │   │   ├── protocols/
│   │   │   │   ├── include/
│   │   │   │   │   ├── i2c_master.h
│   │   │   │   │   └── i2c_slave.h
│   │   │   │   └── src/
│   │   │   │       ├── i2c_master.c
│   │   │   │       └── i2c_slave.c
│   │   │   └── devices/
│   │   │       ├── include/
│   │   │       │   ├── i2c_eeprom.h
│   │   │       │   └── i2c_sensor.h
│   │   │       └── src/
│   │   │           ├── i2c_eeprom.c
│   │   │           └── i2c_sensor.c
│   │   └── spi/
│   │       ├── core/
│   │       │   ├── include/
│   │       │   │   ├── spi_driver.h
│   │       │   │   └── spi_interface.h
│   │       │   └── src/
│   │       │       └── spi_driver.c
│   │       ├── modes/
│   │       │   ├── include/
│   │       │   │   ├── spi_master.h
│   │       │   │   └── spi_slave.h
│   │       │   └── src/
│   │       │       ├── spi_master.c
│   │       │       └── spi_slave.c
│   │       └── protocols/
│   │           ├── include/
│   │           │   ├── spi_flash.h
│   │           │   └── spi_display.h
│   │           └── src/
│   │               ├── spi_flash.c
│   │               └── spi_display.c
│   ├── display/               # 显示设备驱动
│   │   ├── lcd/
│   │   │   ├── core/
│   │   │   │   ├── include/
│   │   │   │   │   ├── lcd_driver.h
│   │   │   │   │   └── lcd_interface.h
│   │   │   │   └── src/
│   │   │   │       └── lcd_driver.c
│   │   │   ├── controllers/
│   │   │   │   ├── ltdc/
│   │   │   │   │   ├── include/
│   │   │   │   │   │   └── ltdc_driver.h
│   │   │   │   │   └── src/
│   │   │   │   │       └── ltdc_driver.c
│   │   │   │   └── parallel/
│   │   │   │       ├── include/
│   │   │   │       │   └── parallel_lcd.h
│   │   │   │       └── src/
│   │   │   │           └── parallel_lcd.c
│   │   │   └── panels/
│   │   │       ├── include/
│   │   │       │   ├── panel_common.h
│   │   │       │   └── panel_config.h
│   │   │       └── src/
│   │   │           ├── panel_init.c
│   │   │           └── panel_control.c
│   │   └── framebuffer/
│   │       ├── include/
│   │       │   ├── framebuffer.h
│   │       │   └── fb_manager.h
│   │       └── src/
│   │           ├── framebuffer.c
│   │           └── fb_manager.c
│   ├── input/                 # 输入设备驱动
│   │   ├── touch/
│   │   │   ├── core/
│   │   │   │   ├── include/
│   │   │   │   │   ├── touch_driver.h
│   │   │   │   │   └── touch_interface.h
│   │   │   │   └── src/
│   │   │   │       └── touch_driver.c
│   │   │   ├── controllers/
│   │   │   │   ├── goodix/
│   │   │   │   │   ├── include/
│   │   │   │   │   │   ├── gt9xx_driver.h
│   │   │   │   │   │   └── gt9xx_config.h
│   │   │   │   │   └── src/
│   │   │   │   │       ├── gt9xx_driver.c
│   │   │   │   │       └── gt9xx_config.c
│   │   │   │   └── focaltech/
│   │   │   │       ├── include/
│   │   │   │       │   └── ft6x06_driver.h
│   │   │   │       └── src/
│   │   │   │           └── ft6x06_driver.c
│   │   │   └── calibration/
│   │   │       ├── include/
│   │   │       │   ├── touch_calibration.h
│   │   │       │   └── touch_transform.h
│   │   │       └── src/
│   │   │           ├── touch_calibration.c
│   │   │           └── touch_transform.c
│   │   └── keyboard/
│   │       ├── include/
│   │       │   ├── key_driver.h
│   │       │   └── key_matrix.h
│   │       └── src/
│   │           ├── key_driver.c
│   │           └── key_matrix.c
│   ├── storage/               # 存储设备驱动
│   │   ├── memory/
│   │   │   ├── sdram/
│   │   │   │   ├── include/
│   │   │   │   │   ├── sdram_driver.h
│   │   │   │   │   └── sdram_config.h
│   │   │   │   └── src/
│   │   │   │       ├── sdram_driver.c
│   │   │   │       └── sdram_test.c
│   │   │   ├── sram/
│   │   │   │   ├── include/
│   │   │   │   │   └── sram_driver.h
│   │   │   │   └── src/
│   │   │   │       └── sram_driver.c
│   │   │   └── flash/
│   │   │       ├── internal/
│   │   │       │   ├── include/
│   │   │       │   │   └── internal_flash.h
│   │   │       │   └── src/
│   │   │       │       └── internal_flash.c
│   │   │       └── external/
│   │   │           ├── include/
│   │   │           │   └── external_flash.h
│   │   │           └── src/
│   │   │               └── external_flash.c
│   │   └── filesystem/
│   │       ├── include/
│   │       │   ├── fs_interface.h
│   │       │   └── fat_fs.h
│   │       └── src/
│   │           ├── fs_manager.c
│   │           └── fat_fs.c
│   ├── sensors/               # 传感器驱动
│   │   ├── environmental/
│   │   │   ├── temperature/
│   │   │   │   ├── sht4x/
│   │   │   │   │   ├── include/
│   │   │   │   │   │   ├── sht4x_driver.h
│   │   │   │   │   │   └── sht4x_config.h
│   │   │   │   │   └── src/
│   │   │   │   │       ├── sht4x_driver.c
│   │   │   │   │       └── sht4x_calibration.c
│   │   │   │   └── common/
│   │   │   │       ├── include/
│   │   │   │       │   └── temp_sensor_common.h
│   │   │   │       └── src/
│   │   │   │           └── temp_sensor_common.c
│   │   │   └── humidity/
│   │   │       ├── include/
│   │   │       │   └── humidity_common.h
│   │   │       └── src/
│   │   │           └── humidity_common.c
│   │   ├── motion/
│   │   │   ├── accelerometer/
│   │   │   ├── gyroscope/
│   │   │   └── magnetometer/
│   │   └── common/
│   │       ├── include/
│   │       │   ├── sensor_interface.h
│   │       │   ├── sensor_manager.h
│   │       │   └── sensor_calibration.h
│   │       └── src/
│   │           ├── sensor_manager.c
│   │           └── sensor_calibration.c
│   └── indicators/            # 指示器驱动
│       ├── led/
│       │   ├── core/
│       │   │   ├── include/
│       │   │   │   ├── led_driver.h
│       │   │   │   └── led_interface.h
│       │   │   └── src/
│       │   │       └── led_driver.c
│       │   ├── effects/
│       │   │   ├── include/
│       │   │   │   ├── led_effects.h
│       │   │   │   └── led_patterns.h
│       │   │   └── src/
│       │   │       ├── led_effects.c
│       │   │       └── led_patterns.c
│       │   └── controllers/
│       │       ├── ws2812/
│       │       └── pwm_led/
│       └── buzzer/
│           ├── include/
│           │   └── buzzer_driver.h
│           └── src/
│               └── buzzer_driver.c
│
├── framework/                 # 系统框架层
│   ├── abstraction/           # 抽象层框架
│   │   ├── hal/
│   │   │   ├── include/
│   │   │   │   ├── hal_abstraction.h
│   │   │   │   └── hal_interface.h
│   │   │   └── src/
│   │   │       └── hal_abstraction.c
│   │   └── osal/              # 操作系统抽象层
│   │       ├── include/
│   │       │   ├── osal.h
│   │       │   ├── osal_thread.h
│   │       │   ├── osal_mutex.h
│   │       │   └── osal_timer.h
│   │       └── src/
│   │           ├── osal_freertos.c
│   │           ├── osal_thread.c
│   │           ├── osal_mutex.c
│   │           └── osal_timer.c
│   ├── communication/         # 通信框架
│   │   ├── protocol/
│   │   │   ├── include/
│   │   │   │   ├── protocol_stack.h
│   │   │   │   ├── protocol_parser.h
│   │   │   │   └── protocol_builder.h
│   │   │   └── src/
│   │   │       ├── protocol_stack.c
│   │   │       ├── protocol_parser.c
│   │   │       └── protocol_builder.c
│   │   ├── transport/
│   │   │   ├── include/
│   │   │   │   ├── transport_layer.h
│   │   │   │   ├── uart_transport.h
│   │   │   │   └── spi_transport.h
│   │   │   └── src/
│   │   │       ├── transport_layer.c
│   │   │       ├── uart_transport.c
│   │   │       └── spi_transport.c
│   │   ├── messaging/
│   │   │   ├── include/
│   │   │   │   ├── message_queue.h
│   │   │   │   ├── event_bus.h
│   │   │   │   └── publish_subscribe.h
│   │   │   └── src/
│   │   │       ├── message_queue.c
│   │   │       ├── event_bus.c
│   │   │       └── publish_subscribe.c
│   │   └── network/
│   │       ├── include/
│   │       │   ├── network_stack.h
│   │       │   └── socket_interface.h
│   │       └── src/
│   │           ├── network_stack.c
│   │           └── socket_interface.c
│   ├── storage/               # 存储框架
│   │   ├── filesystem/
│   │   │   ├── include/
│   │   │   │   ├── vfs.h        # 虚拟文件系统
│   │   │   │   ├── file_ops.h
│   │   │   │   └── dir_ops.h
│   │   │   └── src/
│   │   │       ├── vfs.c
│   │   │       ├── file_ops.c
│   │   │       └── dir_ops.c
│   │   ├── database/
│   │   │   ├── include/
│   │   │   │   ├── kvstore.h    # 键值存储
│   │   │   │   └── record_db.h
│   │   │   └── src/
│   │   │       ├── kvstore.c
│   │   │       └── record_db.c
│   │   ├── cache/
│   │   │   ├── include/
│   │   │   │   ├── cache_manager.h
│   │   │   │   └── lru_cache.h
│   │   │   └── src/
│   │   │       ├── cache_manager.c
│   │   │       └── lru_cache.c
│   │   └── config/
│   │       ├── include/
│   │       │   ├── config_manager.h
│   │       │   └── settings_store.h
│   │       └── src/
│   │           ├── config_manager.c
│   │           └── settings_store.c
│   ├── security/              # 安全框架
│   │   ├── crypto/
│   │   │   ├── include/
│   │   │   │   ├── crypto_api.h
│   │   │   │   ├── aes_crypto.h
│   │   │   │   └── hash_crypto.h
│   │   │   └── src/
│   │   │       ├── crypto_api.c
│   │   │       ├── aes_crypto.c
│   │   │       └── hash_crypto.c
│   │   ├── authentication/
│   │   │   ├── include/
│   │   │   │   ├── auth_manager.h
│   │   │   │   └── user_manager.h
│   │   │   └── src/
│   │   │       ├── auth_manager.c
│   │   │       └── user_manager.c
│   │   └── secure_storage/
│   │       ├── include/
│   │       │   ├── secure_store.h
│   │       │   └── key_manager.h
│   │       └── src/
│   │           ├── secure_store.c
│   │           └── key_manager.c
│   └── utility/               # 工具框架
│       ├── data_structures/
│       │   ├── include/
│       │   │   ├── linked_list.h
│       │   │   ├── ring_buffer.h
│       │   │   ├── hash_table.h
│       │   │   └── binary_tree.h
│       │   └── src/
│       │       ├── linked_list.c
│       │       ├── ring_buffer.c
│       │       ├── hash_table.c
│       │       └── binary_tree.c
│       ├── algorithms/
│       │   ├── include/
│       │   │   ├── sort_algorithms.h
│       │   │   ├── search_algorithms.h
│       │   │   └── filter_algorithms.h
│       │   └── src/
│       │       ├── sort_algorithms.c
│       │       ├── search_algorithms.c
│       │       └── filter_algorithms.c
│       └── patterns/
│           ├── include/
│           │   ├── state_machine.h
│           │   ├── observer_pattern.h
│           │   └── command_pattern.h
│           └── src/
│               ├── state_machine.c
│               ├── observer_pattern.c
│               └── command_pattern.c
│
├── uiframework/               # UI框架层
│   ├── lvgl/                  # LVGL集成
│   │   ├── port/
│   │   │   ├── include/
│   │   │   │   ├── lv_port_disp.h
│   │   │   │   ├── lv_port_indev.h
│   │   │   │   ├── lv_port_fs.h
│   │   │   │   └── lv_conf.h
│   │   │   └── src/
│   │   │       ├── lv_port_disp.c  # 显示移植
│   │   │       ├── lv_port_indev.c # 输入移植
│   │   │       └── lv_port_fs.c    # 文件系统移植
│   │   ├── drivers/
│   │   │   ├── include/
│   │   │   │   ├── lv_gpu_driver.h
│   │   │   │   └── lv_mem_driver.h
│   │   │   └── src/
│   │   │       ├── lv_gpu_driver.c
│   │   │       └── lv_mem_driver.c
│   │   └── config/
│   │       ├── include/
│   │       │   ├── lv_config_custom.h
│   │       │   └── lv_theme_custom.h
│   │       └── src/
│   │           ├── lv_config_custom.c
│   │           └── lv_theme_custom.c
│   ├── graphics/              # 图形中间件
│   │   ├── rendering/
│   │   │   ├── include/
│   │   │   │   ├── renderer.h
│   │   │   │   ├── 2d_renderer.h
│   │   │   │   └── gpu_renderer.h
│   │   │   └── src/
│   │   │       ├── renderer.c
│   │   │       ├── 2d_renderer.c
│   │   │       └── gpu_renderer.c
│   │   ├── framebuffer/
│   │   │   ├── include/
│   │   │   │   ├── fb_manager.h
│   │   │   │   ├── double_buffer.h
│   │   │   │   └── triple_buffer.h
│   │   │   └── src/
│   │   │       ├── fb_manager.c
│   │   │       ├── double_buffer.c
│   │   │       └── triple_buffer.c
│   │   ├── image/
│   │   │   ├── include/
│   │   │   │   ├── image_decoder.h
│   │   │   │   ├── jpeg_decoder.h
│   │   │   │   └── png_decoder.h
│   │   │   └── src/
│   │   │       ├── image_decoder.c
│   │   │       ├── jpeg_decoder.c
│   │   │       └── png_decoder.c
│   │   └── fonts/
│   │       ├── include/
│   │       │   ├── font_manager.h
│   │       │   └── font_loader.h
│   │       └── src/
│   │           ├── font_manager.c
│   │           └── font_loader.c
│   ├── input/                 # 输入管理
│   │   ├── touch/
│   │   │   ├── include/
│   │   │   │   ├── touch_manager.h
│   │   │   │   ├── gesture_recognition.h
│   │   │   │   └── multi_touch.h
│   │   │   └── src/
│   │   │       ├── touch_manager.c
│   │   │       ├── gesture_recognition.c
│   │   │       └── multi_touch.c
│   │   ├── keyboard/
│   │   │   ├── include/
│   │   │   │   ├── keyboard_manager.h
│   │   │   │   └── key_mapping.h
│   │   │   └── src/
│   │   │       ├── keyboard_manager.c
│   │   │       └── key_mapping.c
│   │   └── common/
│   │       ├── include/
│   │       │   ├── input_manager.h
│   │       │   └── input_event.h
│   │       └── src/
│   │           ├── input_manager.c
│   │           └── input_event.c
│   ├── animation/             # 动画系统
│   │   ├── include/
│   │   │   ├── animation_engine.h
│   │   │   ├── easing_functions.h
│   │   │   └── transition_effects.h
│   │   └── src/
│   │       ├── animation_engine.c
│   │       ├── easing_functions.c
│   │       └── transition_effects.c
│   └── themes/                # 主题系统
│       ├── include/
│       │   ├── theme_manager.h
│       │   ├── style_manager.h
│       │   └── color_palette.h
│       └── src/
│           ├── theme_manager.c
│           ├── style_manager.c
│           └── color_palette.c
│
├── framework/                 # 系统中间件框架
│   ├── communication/         # 通信框架
│   │   ├── include/
│   │   │   ├── protocol_stack.h
│   │   │   ├── message_queue.h
│   │   │   └── event_bus.h
│   │   └── src/
│   │       ├── protocol_stack.c
│   │       ├── message_queue.c
│   │       └── event_bus.c
│   ├── storage/               # 存储框架
│   │   ├── include/
│   │   │   ├── file_system.h
│   │   │   ├── flash_manager.h
│   │   │   └── config_manager.h
│   │   └── src/
│   │       ├── file_system.c
│   │       ├── flash_manager.c
│   │       └── config_manager.c
│   ├── timer/                 # 定时器框架
│   │   ├── include/
│   │   │   ├── soft_timer.h
│   │   │   └── timer_manager.h
│   │   └── src/
│   │       ├── soft_timer.c
│   │       └── timer_manager.c
│   └── security/              # 安全框架
│       ├── include/
│       │   ├── crypto.h
│       │   └── auth_manager.h
│       └── src/
│           ├── crypto.c
│           └── auth_manager.c
│
├── uiframework/               # UI框架
│   ├── lvgl/                  # LVGL相关封装
│   │   ├── include/
│   │   │   ├── lvgl_port.h     # LVGL移植
│   │   │   ├── lvgl_config.h   # LVGL配置
│   │   │   └── display_port.h  # 显示驱动移植
│   │   └── src/
│   │       ├── lvgl_port.c
│   │       ├── display_port.c
│   │       └── input_port.c
│   ├── graphics/              # 图形中间件
│   │   ├── include/
│   │   │   ├── framebuffer.h   # 帧缓冲管理
│   │   │   ├── font_manager.h  # 字体管理
│   │   │   └── image_decoder.h # 图像解码
│   │   └── src/
│   │       ├── framebuffer.c
│   │       ├── font_manager.c
│   │       └── image_decoder.c
│   ├── themes/                # UI主题
│   │   ├── include/
│   │   │   └── theme_manager.h
│   │   └── src/
│   │       └── theme_manager.c
│   └── widgets/               # 自定义控件
│       ├── include/
│       │   ├── custom_button.h
│       │   └── status_widget.h
│       └── src/
│           ├── custom_button.c
│           └── status_widget.c
│
├── components/                # 系统组件
│   ├── cmbacktrace/           # CM回溯组件
│   │   ├── include/
│   │   │   └── cm_backtrace.h
│   │   └── src/
│   │       └── cm_backtrace.c
│   ├── log/                   # 日志组件
│   │   ├── include/
│   │   │   ├── log.h
│   │   │   └── log_config.h
│   │   └── src/
│   │       ├── log.c
│   │       └── log_output.c
│   ├── shell/                 # 命令行组件
│   │   ├── include/
│   │   │   ├── shell.h
│   │   │   └── shell_command.h
│   │   └── src/
│   │       ├── shell.c
│   │       └── shell_command.c
│   ├── watchdog/              # 看门狗组件
│   │   ├── include/
│   │   │   └── watchdog.h
│   │   └── src/
│   │       └── watchdog.c
│   ├── power/                 # 电源管理组件
│   │   ├── include/
│   │   │   ├── power_manager.h
│   │   │   └── low_power.h
│   │   └── src/
│   │       ├── power_manager.c
│   │       └── low_power.c
│   └── diagnostic/            # 诊断组件
│       ├── include/
│       │   ├── system_monitor.h
│       │   └── health_check.h
│       └── src/
│           ├── system_monitor.c
│           └── health_check.c
│
├── 3rdparty/                  # 第三方库
│   ├── FreeRTOS/             # FreeRTOS源码
│   │   ├── Source/
│   │   ├── portable/
│   │   └── include/
│   ├── LVGL/                 # LVGL源码
│   │   ├── src/
│   │   ├── demos/
│   │   └── examples/
│   ├── CMSIS/                # CMSIS库
│   │   └── Include/
│   └── others/               # 其他第三方库
│
├── tools/                    # 开发工具
│   ├── scripts/              # 构建脚本
│   │   ├── build.sh
│   │   ├── flash.sh
│   │   ├── clean.sh
│   │   └── format_code.sh
│   ├── debug/                # 调试配置
│   │   ├── jlink/
│   │   ├── openocd/
│   │   └── gdb/
│   └── generators/           # 代码生成器
│       ├── config_gen.py
│       └── driver_gen.py
│
├── test/                     # 测试代码
│   ├── unit/                 # 单元测试
│   │   ├── core/
│   │   ├── drivers/
│   │   ├── framework/
│   │   └── components/
│   ├── integration/          # 集成测试
│   └── system/               # 系统测试
│
├── ui/                       # 用户界面应用
│   ├── screens/              # 界面屏幕
│   │   ├── include/
│   │   │   ├── main_screen.h
│   │   │   ├── settings_screen.h
│   │   │   └── status_screen.h
│   │   └── src/
│   │       ├── main_screen.c
│   │       ├── settings_screen.c
│   │       └── status_screen.c
│   ├── apps/                 # UI应用
│   │   ├── include/
│   │   │   ├── ui_app.h
│   │   │   └── screen_manager.h
│   │   └── src/
│   │       ├── ui_app.c
│   │       └── screen_manager.c
│   └── resources/            # UI资源
│       ├── fonts/
│       ├── images/
│       └── animations/
│
├── docs/                     # 文档
│   ├── api/                  # API文档
│   ├── design/               # 设计文档
│   ├── user_guide/           # 用户指南
│   ├── development/          # 开发文档
│   └── architecture/         # 架构文档
│
└── packages/                 # 软件包管理
    ├── requirements.txt      # 依赖需求
    ├── versions.lock         # 版本锁定
    └── update.sh            # 更新脚本
```
## 各层职责说明

### 1. Application 层 (应用程序入口)
- **职责**: 程序入口点，负责系统启动和各模块初始化
- **包含**: main函数、模块管理器、应用初始化流程
- **特点**: 统一管理所有模块的初始化顺序

### 2. Core 层 (系统核心)
- **FreeRTOS**: 实时操作系统内核，任务调度和管理
- **Memory**: 内存管理（堆、池、DMA内存）
- **Date**: 日期时间管理，RTC操作
- **System**: 系统核心功能（错误处理、复位管理、系统信息）

### 3. Platform 层 (平台SDK)
- **SDK**: 原厂提供的SDK（CMSIS、HAL库、启动文件）
- **BSP**: 板级支持包，硬件抽象层
- **Linker**: 链接脚本和内存配置

### 4. Drivers 层 (硬件驱动)
- **按外设分类**: 每个外设独立目录（GPIO、UART、I2C、SPI等）
- **具体实现**: 基于HAL库的外设驱动实现
- **传感器驱动**: 各种传感器的具体驱动代码

### 5. Framework 层 (系统中间件框架)
- **Communication**: 通信协议栈、消息队列、事件总线
- **Storage**: 文件系统、Flash管理、配置管理
- **Timer**: 软件定时器框架
- **Security**: 加密、认证等安全框架

### 6. UIFramework 层 (UI框架)
- **LVGL**: LVGL图形库的移植和配置
- **Graphics**: 图形中间件（帧缓冲、字体、图像解码）
- **Themes**: UI主题管理
- **Widgets**: 自定义UI控件

### 7. Components 层 (系统组件)
- **CM Backtrace**: 错误回溯组件
- **Log**: 日志系统组件
- **Shell**: 命令行组件
- **Watchdog**: 看门狗组件
- **Power**: 电源管理组件
- **Diagnostic**: 系统诊断组件

### 8. UI 层 (用户界面应用)
- **Screens**: 具体的界面屏幕实现
- **Apps**: UI应用程序
- **Resources**: UI资源文件

### 9. 3rdparty 层 (第三方库)
- 所有第三方库源码
- 版本控制和依赖管理

## 代码迁移映射

### 当前代码 → 新结构映射：

1. **Application/ 目录**:
   - `main.c` → `application/src/main.c`
   - `FreeRTOSConfig.h` → `core/freertos/include/FreeRTOSConfig.h`
   - `stm32f4xx_hal_conf.h` → `platform/stm32f4xx/bsp/include/`
   - `system_stm32f4xx.c` → `platform/stm32f4xx/bsp/src/`

2. **Drive/ 目录**:
   - `Source/bsp_led.c` → `drivers/led/src/led_driver.c`
   - `Source/bsp_lcd.c` → `drivers/lcd/src/lcd_driver.c`
   - `Source/bsp_sdram.c` → `drivers/sdram/src/sdram_driver.c`
   - `Source/bsp_touch_gtxx.c` → `drivers/touch/src/gt9xx_driver.c`
   - `Source/usart.c` → `drivers/uart/src/uart_driver.c`
   - `Source/i2c.c` → `drivers/i2c/src/i2c_driver.c`
   - `Source/sht4x_drv.c` → `drivers/sensors/sht4x/src/sht4x_driver.c`

3. **ui/ 目录**:
   - `ui_app.c` → `ui/apps/src/ui_app.c` + `uiframework/lvgl/src/lvgl_port.c`

4. **第三方库**:
   - `FreeRTOS/` → `3rdparty/FreeRTOS/`
   - `lvgl/` → `3rdparty/LVGL/`
   - `CMSIS_5-develop/` → `3rdparty/CMSIS/`
   - `STM32/STM32F4xx_HAL_Driver/` → `platform/stm32f4xx/sdk/HAL_Driver/`

5. **cm_backtrace/ 目录**:
   - 整个目录 → `components/cmbacktrace/`

6. **startup/ 和 linker/**:
   - 移动到 `platform/stm32f4xx/sdk/startup/` 和 `platform/stm32f4xx/linker/`

## 优势分析

1. **清晰的职责分离**:
   - Core专注系统核心功能
   - Framework提供通用中间件
   - Components提供可插拔功能组件

2. **便于模块化开发**:
   - 每个组件独立开发和测试
   - 标准化的接口设计
   - 便于功能裁剪和配置

3. **易于维护和扩展**:
   - 相关代码集中管理
   - 层次清晰，便于定位问题
   - 新增功能有明确的归属

4. **便于移植**:
   - Platform层隔离平台相关代码
   - 标准化的驱动接口
   - 框架层提供通用功能

5. **支持组件化**:
   - Components支持可插拔设计
   - 便于功能组合和配置
   - 支持模块化测试

## 初始化流程设计

```
main() [application/src/main.c]
├── HAL_Init() [platform初始化]
├── SystemClock_Config() [platform时钟配置]
├── Core_Init() [core层初始化]
│   ├── Memory_Manager_Init()
│   ├── FreeRTOS_Init()
│   └── Date_Manager_Init()
├── Framework_Init() [framework层初始化]
│   ├── Communication_Init()
│   ├── Storage_Init()
│   └── Timer_Init()
├── Components_Init() [components初始化]
│   ├── Log_Init()
│   ├── CMBacktrace_Init()
│   └── Shell_Init()
├── Drivers_Init() [drivers初始化]
│   ├── GPIO_Init()
│   ├── UART_Init()
│   ├── LCD_Init()
│   └── Touch_Init()
├── UIFramework_Init() [UI框架初始化]
│   ├── LVGL_Init()
│   └── Display_Port_Init()
├── UI_App_Init() [UI应用初始化]
└── osKernelStart() [启动调度器]
```

这个结构设计更符合您的需求，您觉得如何？

## 详细层级细化

### 1. 驱动层进一步细化

**总线驱动详细结构**:
- 每个总线类型(GPIO/UART/I2C/SPI)都有三层结构：
  - `core/`: 核心驱动实现，提供基础功能
  - `protocols/`: 协议层实现，支持不同通信协议
  - `extensions/`: 扩展功能，如DMA、缓冲区管理等

**显示驱动详细结构**:
- `lcd/`: LCD显示驱动
  - `core/`: 通用LCD驱动框架
  - `controllers/`: 不同控制器(LTDC/并行/SPI)的实现
  - `panels/`: 不同面板的配置和初始化
- `oled/`: OLED显示驱动
- `framebuffer/`: 帧缓冲管理

**输入驱动详细结构**:
- `touch/`: 触摸屏驱动
  - `core/`: 触摸核心驱动
  - `controllers/`: 不同触摸IC驱动(Goodix/FT5x06/电阻式)
  - `calibration/`: 触摸校准算法
  - `gesture/`: 手势识别功能
- `keyboard/`: 键盘驱动(矩阵式/薄膜式)
- `encoder/`: 旋转编码器驱动

**存储驱动详细结构**:
- `flash/`: Flash存储
  - `internal/`: 内部Flash
  - `external/`: 外部Flash(NOR/NAND)
  - `filesystem/`: 文件系统移植
- `sdram/`: SDRAM驱动
- `eeprom/`: EEPROM驱动(I2C/SPI接口)
- `sd_card/`: SD卡驱动

**传感器驱动详细结构**:
- `temperature/`: 温度传感器
  - `digital/`: 数字传感器(SHT4x/DS18B20/DHT22)
  - `analog/`: 模拟传感器(NTC/热电偶)
- `pressure/`: 压力传感器
- `humidity/`: 湿度传感器
- `imu/`: 惯性测量单元
- `common/`: 传感器通用管理

**指示器驱动详细结构**:
- `led/`: LED驱动
  - `simple/`: 简单LED
  - `rgb/`: RGB LED(WS2812等)
  - `matrix/`: LED矩阵
- `buzzer/`: 蜂鸣器驱动
- `relay/`: 继电器驱动

### 2. 框架层进一步细化

**通信框架详细结构**:
- `protocol/`: 协议栈实现
- `transport/`: 传输层抽象
- `messaging/`: 消息队列和事件总线
- `network/`: 网络协议栈

**存储框架详细结构**:
- `filesystem/`: 虚拟文件系统
- `database/`: 键值存储和记录数据库
- `cache/`: 缓存管理系统
- `config/`: 配置管理系统

**安全框架详细结构**:
- `crypto/`: 加密算法API
- `authentication/`: 认证管理
- `secure_storage/`: 安全存储

**工具框架详细结构**:
- `data_structures/`: 通用数据结构
- `algorithms/`: 算法库
- `patterns/`: 设计模式实现

### 3. UI框架层进一步细化

**LVGL集成详细结构**:
- `port/`: LVGL移植层
- `drivers/`: GPU和内存驱动
- `config/`: 自定义配置和主题

**图形中间件详细结构**:
- `rendering/`: 渲染引擎
- `framebuffer/`: 帧缓冲管理
- `image/`: 图像解码器
- `fonts/`: 字体管理

**输入管理详细结构**:
- `touch/`: 触摸管理和手势识别
- `keyboard/`: 键盘管理
- `common/`: 通用输入事件处理

### 4. 组件层进一步细化

**日志系统详细结构**:
- 支持多种输出(控制台/文件/网络)
- 自定义格式化器
- 日志级别管理
- 异步日志处理

**命令行接口详细结构**:
- 命令解析器
- 历史记录管理
- 内置命令集
- 可扩展命令系统

**电源管理详细结构**:
- 睡眠模式管理
- 时钟频率控制
- 电池监控
- 功耗优化

**诊断系统详细结构**:
- 系统状态监控
- 性能指标收集
- 健康检查
- 错误报告

### 5. 构建系统配置

**CMake配置结构**:
```cmake
# 主CMakeLists.txt
├── platform/stm32f4xx/CMakeLists.txt  # 平台配置
├── core/CMakeLists.txt                 # 核心层配置
├── framework/CMakeLists.txt            # 框架层配置
├── drivers/CMakeLists.txt              # 驱动层配置
├── components/CMakeLists.txt           # 组件层配置
├── uiframework/CMakeLists.txt          # UI框架配置
├── ui/CMakeLists.txt                   # UI应用配置
└── tests/CMakeLists.txt                # 测试配置
```

**配置选项系统**:
- Kconfig风格的配置选项
- 模块化编译控制
- 功能裁剪支持
- 平台适配配置

### 6. 测试系统

**测试层级结构**:
- `unit/`: 单元测试，针对每个模块
- `integration/`: 集成测试，测试模块间接口
- `performance/`: 性能测试和基准测试
- `mock/`: 模拟对象，用于隔离测试

这种细化的层级结构有以下优势：

1. **高度模块化**: 每个功能都有明确的层次和职责
2. **可扩展性**: 容易添加新的驱动、组件或功能
3. **可维护性**: 相关代码集中，便于定位和修改
4. **可测试性**: 每个模块都可以独立测试
5. **可配置性**: 支持功能裁剪和灵活配置
6. **可移植性**: 平台相关代码隔离，便于移植

您觉得这样的细化程度如何？需要对某些特定层级进行更详细的说明吗？
