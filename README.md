# Motor Driver v6 Firmware

please led flash please please please

The repo contains the firmware for our 6th generation motor driver boards.

## Getting Started

You will need to install the following software:

- [STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html) - for building, flashing, and debugging the firmware
- [STM32CubeMX](https://www.st.com/en/development-tools/stm32cubemx.html) - for MCU/peripheral configuration and generating firmware code
- [MCSDK](https://www.st.com/en/embedded-software/x-cube-mcsdk.html) - motor control SDK with firmware + tools that our project uses
  - Unfortunately, this is SDK is only available for Windows. You can still build and flash our firmware without it, but you won't be able to use CubeMX and MC Workbench to edit the project config or generate code.

MCSDK includes a program called [Motor Control Workbench](https://wiki.st.com/stm32mcu/wiki/STM32MotorControl:STM32_MC_Workbench) for configuring MCSDK motor control parameters and generating a CubeMX-compatible project with all the motor control firmware code.

## Generated Code

Most of the firmware code is generated for us by STM32CubeMX and MC Workbench. This includes initialization code for setting up the MCU and peripherals, as well as the actual motor control code that uses MCSDK libraries.

The MCSDK firmware is documented in [UM1052](https://www.st.com/resource/zh/user_manual/cd00298474-stm32f-pmsm-single-dual-foc-sdk-v4-3-stmicroelectronics.pdf). You can access an HTML version of this manual by opening MC Workbench and going to About > Documentations > Documentation. 

> [!IMPORTANT]
> **Only write code between the user section comments in any generated files.** This ensures that our custom user code will not be deleted/modified when we regenerate project code. Generally, you should not edit generated files directly; edit the project configuration instead and regenerate the project code.

### Using MC Workbench

The MC Workbench file `MDv6.stwb6` used to generate our project is included in the repo. If you open this file in MC Workbench to edit parameters and regenerate the project, **you will end up generating code for a new project in a separate folder named `MDv6`.** Alternatively, you can open the `MDv6.ioc` file in MC Workbench, which will overwrite the existing project when regenerating rather than create a new project.

Note that after a project is generated using the `MDv6.stwb6` file, all hardware parameters are frozen (e.g. power stage parameters). This means that some parameters are not editable via opening the `MDv6.ioc` (even if it looks like you can via the Workbench UI... when you go to generate the project, your parameter changes won't appear in the code). Furthermore, editing `MDv6.ioc` but not `MDv6.stwb6` will leave `MDv6.stwb6` outdated, which isn't ideal.

Hence, the following process is recommended for editing MCSDK parameters: 

1. Open the `MDv6.stwb6` file with MC Workbench and edit the parameters.
2. Generate the project. The generated project will appear in a separate folder named `MDv6` within the repo.
3. Replace the contents of the repo with the generated project's contents.
4. Manually determine what changes need to be kept/discarded by analyzing the Git diff (highly recommend using sort of Git GUI tool). Be careful with the `MDv6.ioc` file and make sure that our custom pinout / SPI and DMA initialization code doesn't get deleted.

## SPI Interface

The part of the firmware that we have implemented ourselves is the SPI interface module (`Inc/interface.h`, `Src/interface.c`) for communicating with the MD boards over SPI. This module implements a simple SPI protocol for controlling the motor. The master (i.e. robot's Raspberry Pi) can send commands to set the target speed or torque, configure parameters, and select the type of telemetry response. The slave (this firmware) processes the commands and responds with telemetry data.

All communications are done using 6 byte long frames. We communicate in full-duplex mode, so master sends a frame while simultaneously receiving a frame. The code in `interface.c` is documented with information about the types of commands/responses that can be sent/received and the frame formats.

