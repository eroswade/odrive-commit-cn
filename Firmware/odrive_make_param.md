arm-none-eabi-g++ -std=c++17 
-c Board/v3/board.cpp 

-Wno-register 
-Wno-psabi 
-Wall 
-Wdouble-promotion 
-Wfloat-conversion 
-Wno-nonnull -gdwarf-2 -Og 

-fdata-sections 
-ffunction-sections 

-mthumb 
-mfloat-abi=hard 
-mcpu=cortex-m4 
-mfpu=fpv4-sp-d16 

-IBoard/v3/Inc 
-IBoard/v3/../../ThirdParty/FreeRTOS/Source/portable/GCC/ARM_CM4F -DSTM32F405xx 
-IThirdParty/STM32F4xx_HAL_Driver/Inc 
-I./. 
-I./MotorControl 
-I./fibre-cpp/include 
-IThirdParty/FreeRTOS/Source/include 
-IThirdParty/FreeRTOS/Source/CMSIS_RTOS 
-IThirdParty/CMSIS/Include -IThirdParty/CMSIS/Device/ST/STM32F7xx/Include 
-IThirdParty/CMSIS/Device/ST/STM32F4xx/Include 
-IThirdParty/STM32_USB_Device_Library/Core/Inc 
-IThirdParty/STM32_USB_Device_Library/Class/CDC/Inc 
-Ifibre-cpp/include 

-DFPU_FPV4 
-DHW_VERSION_VOLTAGE=24 
-DHW_VERSION_MAJOR=3 
-DFIBRE_ENABLE_SERVER 
-DHW_VERSION_MINOR=5 
-DARM_MATH_CM4 
-DUSE_HAL_DRIVER 
-DFIBRE_ENABLE_SERVER=1 
-DFIBRE_ENABLE_CLIENT=0 
-DFIBRE_ENABLE_EVENT_LOOP=0 
-DFIBRE_ALLOW_HEAP=0 
-DFIBRE_MAX_LOG_VERBOSITY=0 
-DFIBRE_DEFAULT_LOG_VERBOSITY=2 
-DFIBRE_ENABLE_LIBUSB_BACKEND=0 
-DFIBRE_ENABLE_TCP_SERVER_BACKEND=0 
-DFIBRE_ENABLE_TCP_CLIENT_BACKEND=0 
