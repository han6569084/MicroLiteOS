# MicroLiteOS

English | [中文](README.md)

A micro real-time embedded system based on VS Code development, compilation, and debugging

## Project Overview

Currently, MCU program compilation and development online mainly relies on Keil 5, with relatively few resources about VS Code MCU development. Since VS Code offers far greater convenience for C/C++ development than Keil, this project builds a complete VS Code MCU system project for learning and research purposes.

This project constructs a micro real-time embedded system based on VS Code development, compilation, and debugging, integrating multiple latest versions of open-source components:

- **Kernel**: FreeRTOS V11.1.0 (Latest Release)
- **GUI SDK**: LVGL 9.3.0 (Latest Release)
- **CMSIS**: ARM Official CMSIS 5.9.0 (Latest Version)
- **Debug Components**: cm_backtrace, coredump, DWT runtime
- **Mem System**: tlfs memory management
- **Target Platform**: ARM Cortex-M4 (STM32F429)

## Features

- ✅ Complete VS Code development environment configuration
- ✅ CMake-based build system
- ✅ Integrated debugging support (JLink)
- ✅ FreeRTOS real-time operating system
- ✅ LVGL graphical user interface
- ✅ CMSIS standard interface
- ✅ Fault tracing and debugging tools
- ✅ Modular design, easy to extend

## Directory Structure

```
MicroLiteOS/
├── Application/          # Application code
├── CMSIS_5-develop/      # CMSIS 5.9.0 standard library
├── FreeRTOS/            # FreeRTOS V11.1.0 kernel
├── lvgl/                # LVGL 9.3.0 graphics library
├── cm_backtrace/        # Fault tracing library
├── Drive/               # Driver programs
├── STM32/               # STM32 related files
├── startup/             # Startup files
├── linker/              # Linker scripts
├── tools/               # Tool scripts
├── ui/                  # User interface
└── build/               # Build output directory
```

## Quick Start

### Requirements

- VS Code
- ARM GCC Toolchain
- CMake
- JLink Debugger (Optional)

### Build Project

1. Clone repository:
```bash
git clone https://github.com/yourusername/MicroLiteOS.git
cd MicroLiteOS
```

2. Build project:
```bash
./remake.sh
```

3. Flash program:
- Use VS Code task: `Flash with JLink`
- Or manually execute: `tools/flash.jlink`

### VS Code Configuration

The project includes complete VS Code configuration:
- `.vscode/tasks.json` - Build and flash tasks
- `.vscode/launch.json` - Debug configuration
- `CMakeLists.txt` - CMake build configuration

## Supported Development Boards

- STM32F429 series (Main test platform)
- Other Cortex-M4 series (Requires adaptation)

## Component Versions

| Component | Version | Description |
|-----------|---------|-------------|
| FreeRTOS | V11.1.0 | Latest Release version |
| LVGL | 9.3.0 | Latest Release version |
| CMSIS | 5.9.0 | ARM official latest version |
| cm_backtrace | latest | Fault tracing component |

## Development Roadmap

- [ ] Integrate more peripheral drivers
- [ ] Add network protocol stack
- [ ] Integrate more debugging tools
- [ ] Support more development boards
- [ ] Improve documentation and examples

## Contributing

Issues and Pull Requests are welcome to improve the project.

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Acknowledgments

Thanks to the following open-source projects:
- [FreeRTOS](https://www.freertos.org/)
- [LVGL](https://lvgl.io/)
- [ARM CMSIS](https://github.com/ARM-software/CMSIS_5)
- [cm_backtrace](https://github.com/armink/CmBacktrace)

## Contact

For questions or suggestions, please contact via Issues.
