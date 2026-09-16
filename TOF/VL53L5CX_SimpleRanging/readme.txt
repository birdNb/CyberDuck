/**
  @page VL53L5CX_SimpleRanging example based on VL53L5CX-SATEL breakout board and STM32 Nucleo boards
  
  @verbatim
  ******************** (C) COPYRIGHT 2021 STMicroelectronics *******************
  * @file    readme.txt  
  * @brief   This application contains an example which shows how to perform
  *          ranging measurement with the VL53L5CX-SATEL breakout board.
  *          The communication is done using a UART connection with PC.
  ******************************************************************************
  *
  * Copyright (c) 2021 STMicroelectronics. All rights reserved.
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
@endverbatim

@par Example Description 

This sample application shows how to use the VL53L5CX-SATEL breakout board on a STM32 Nucleo board 
to perform ranging measurements and print the results on a hyper-terminal such as TeraTerm through a serial link.
Please refer to the user manual for more details.

The sample application uses the following default serial settings:
  - Baud Rate: 460800
  - Data Bits: 8
  - Parity: None
  - Stop Bits: 1

------------------------------------
WARNING: When starting the project from Example Selector in STM32CubeMX and regenerating 
it from ioc file, you may face a build issue. To solve it, remove from the IDE project 
the file stm32f4xx_nucleo.c in Application/User virtual folder and delete from Src and 
Inc folders the files: stm32f4xx_nucleo.c, stm32f4xx_nucleo.h and stm32f4xx_nucleo_errno.h.
------------------------------------

@par Keywords

TOF, I2C, VCOM

@par Hardware and Software environment

  - This example runs on VL53L5CX-SATEL breakout board attached to NUCLEO-F401RE or NUCLEO-L476RG boards.
  - Connect the VL53L5CX-SATEL pins to the following STM32 Nucleo board inputs:
    - SCL <---> D15
    - SDA <---> D14
    - INT <---> A2
    - I2C_RST <---> A1
    - LPn <---> A3
    - PWR_EN <---> A5
    - IOVDD <---> 3V3
    - AVDD <---> 5V
  - If you power the Nucleo board via USB 3.0 port, please check that you have flashed the last version of
    the firmware of ST-Link v2 inside the Nucleo board. In order to flash the last available firmware of the 
	ST-Link v2, you can use the STM32 ST Link Utility.
  - This example has been tested on STMicroelectronics NUCLEO-F401RE RevC and NUCLEO-L476RG RevC and can be easily tailored 
    to any other supported device and development board.

ADDITIONAL_COMP : VL53L5CX https://www.st.com/content/st_com/en/products/imaging-and-photonics-solutions/time-of-flight-sensors/vl53l5cx.html
IONAL_COMP : VL53L5CX-SATEL https://www.st.com/en/evaluation-tools/vl53l5cx-satel.html

@par How to use it? 

This package contains projects for 3 IDEs: IAR, Keil uVision and STM32CubeIDE. In order to make the 
program work, you must do the following:
  - WARNING: before opening the project with any toolchain be sure your folder
    installation path is not too in-depth since the toolchain may report errors
    after building.
  - Open STM32CubeIDE or alternatively the Keil uVision toolchain or the IAR toolchain.
  - Rebuild all files and load your image into target memory.
  - Run the example.
  - Alternatively, you can download the pre-built binaries in "Binary" 
    folder included in the distributed package.

 * <h3><center>&copy; COPYRIGHT STMicroelectronics</center></h3>
 */
