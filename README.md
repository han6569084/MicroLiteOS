# MicroLiteOS

[English](README_EN.md) | 中文

一个基于VS Code开发、编译、调试的微型实时嵌入式系统

## 项目概述

当前网上MCU程序编译开发主要基于Keil 5，关于VS Code开发MCU程序的资料比较少。由于VS Code进行C/C++开发的便利性远超Keil，因此本项目构建了一个完整的VS Code MCU小型系统工程，方便学习研究。

本项目构建了一个基于VS Code开发、编译、调试的微型实时嵌入式系统，集成了多个最新版本的开源组件：

- **内核**: FreeRTOS V11.1.0 (最新Release版本)
- **GUI SDK**: LVGL 9.3.0 (最新Release版本)
- **CMSIS**: ARM官方CMSIS 5.9.0 (最新版本)
- **调试组件**: cm_backtrace、coredump、DWT runtime
- **内存管理**: tlfs内存管理
- **目标平台**: ARM Cortex-M4 (STM32F429)

## 特性

- ✅ 完整的VS Code开发环境配置
- ✅ 基于CMake的构建系统
- ✅ 集成调试支持(JLink)
- ✅ FreeRTOS实时操作系统
- ✅ LVGL图形用户界面
- ✅ CMSIS标准接口
- ✅ 故障追踪和调试工具
- ✅ 模块化设计，易于扩展

## 目录结构

```
MicroLiteOS/
├── Application/          # 应用程序代码
├── CMSIS_5-develop/      # CMSIS 5.9.0 标准库
├── FreeRTOS/            # FreeRTOS V11.1.0 内核
├── lvgl/                # LVGL 9.3.0 图形库
├── cm_backtrace/        # 故障追踪库
├── Drive/               # 驱动程序
├── STM32/               # STM32相关文件
├── startup/             # 启动文件
├── linker/              # 链接脚本
├── tools/               # 工具脚本
├── ui/                  # 用户界面
└── build/               # 构建输出目录
```

## 快速开始

### 环境要求

- VS Code
- ARM GCC工具链
- CMake
- JLink调试器(可选)

### 构建项目

1. 克隆仓库:
```bash
git clone https://github.com/yourusername/MicroLiteOS.git
cd MicroLiteOS
```

2. 构建项目:
```bash
./remake.sh
```

3. 烧录程序:
- 使用VS Code任务: `Flash with JLink`
- 或手动执行: `tools/flash.jlink`

### VS Code配置

项目已包含完整的VS Code配置:
- `.vscode/tasks.json` - 构建和烧录任务
- `.vscode/launch.json` - 调试配置
- `CMakeLists.txt` - CMake构建配置

## 支持的开发板

- STM32F429系列 (主要测试平台)
- 其他Cortex-M4系列 (需要适配)

## 组件版本

| 组件 | 版本 | 说明 |
|------|------|------|
| FreeRTOS | V11.1.0 | 最新Release版本 |
| LVGL | 9.3.0 | 最新Release版本 |
| CMSIS | 5.9.0 | ARM官方最新版本 |
| cm_backtrace | latest | 故障追踪组件 |

## 开发计划

- [ ] 集成更多外设驱动
- [ ] 添加网络协议栈
- [ ] 集成更多调试工具
- [ ] 支持更多开发板
- [ ] 完善文档和示例

## 贡献

欢迎提交Issue和Pull Request来改进项目。

## 许可证

本项目采用 MIT 许可证 - 查看 [LICENSE](LICENSE) 文件了解详情。

## 致谢

感谢以下开源项目:
- [FreeRTOS](https://www.freertos.org/)
- [LVGL](https://lvgl.io/)
- [ARM CMSIS](https://github.com/ARM-software/CMSIS_5)
- [cm_backtrace](https://github.com/armink/CmBacktrace)

## 联系方式

如有问题或建议，请通过Issue联系。
