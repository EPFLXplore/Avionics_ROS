################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../System/Core/Src/Interface.cpp \
../System/Core/Src/System.cpp 

OBJS += \
./System/Core/Src/Interface.o \
./System/Core/Src/System.o 

CPP_DEPS += \
./System/Core/Src/Interface.d \
./System/Core/Src/System.d 


# Each subdirectory must supply rules for building sources it contributes
System/Core/Src/%.o System/Core/Src/%.su System/Core/Src/%.cyclo: ../System/Core/Src/%.cpp System/Core/Src/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m7 -std=gnu++14 -g3 -DDEBUG -DUSE_PWR_LDO_SUPPLY -DUSE_HAL_DRIVER -DSTM32H743xx -c -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I"/home/pedro/Pedro/Xplore/VegaTestingCode/vega-microROS/System/Core/Inc" -I"/home/pedro/Pedro/Xplore/VegaTestingCode/vega-microROS/System/Threads/Inc" -I"/home/pedro/Pedro/Xplore/VegaTestingCode/vega-microROS/System/Utils/Inc" -I"/home/pedro/Pedro/Xplore/VegaTestingCode/vega-microROS/micro_ros_stm32cubemx_utils/microros_static_library_ide/libmicroros/include" -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-System-2f-Core-2f-Src

clean-System-2f-Core-2f-Src:
	-$(RM) ./System/Core/Src/Interface.cyclo ./System/Core/Src/Interface.d ./System/Core/Src/Interface.o ./System/Core/Src/Interface.su ./System/Core/Src/System.cyclo ./System/Core/Src/System.d ./System/Core/Src/System.o ./System/Core/Src/System.su

.PHONY: clean-System-2f-Core-2f-Src

