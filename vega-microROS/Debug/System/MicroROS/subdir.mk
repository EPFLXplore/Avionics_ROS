################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../System/MicroROS/custom_memory_manager.c \
../System/MicroROS/microros_allocators.c \
../System/MicroROS/microros_time.c \
../System/MicroROS/usb_cdc_transport.c 

C_DEPS += \
./System/MicroROS/custom_memory_manager.d \
./System/MicroROS/microros_allocators.d \
./System/MicroROS/microros_time.d \
./System/MicroROS/usb_cdc_transport.d 

OBJS += \
./System/MicroROS/custom_memory_manager.o \
./System/MicroROS/microros_allocators.o \
./System/MicroROS/microros_time.o \
./System/MicroROS/usb_cdc_transport.o 


# Each subdirectory must supply rules for building sources it contributes
System/MicroROS/%.o System/MicroROS/%.su System/MicroROS/%.cyclo: ../System/MicroROS/%.c System/MicroROS/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_PWR_LDO_SUPPLY -DUSE_HAL_DRIVER -DSTM32H743xx -c -I../Core/Inc -I"/home/pedro/Pedro/Xplore/VegaTestingCode/vega-microROS/micro_ros_stm32cubemx_utils/microros_static_library_ide/libmicroros/include" -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I"/home/pedro/Pedro/Xplore/VegaTestingCode/vega-microROS/System/Core/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-System-2f-MicroROS

clean-System-2f-MicroROS:
	-$(RM) ./System/MicroROS/custom_memory_manager.cyclo ./System/MicroROS/custom_memory_manager.d ./System/MicroROS/custom_memory_manager.o ./System/MicroROS/custom_memory_manager.su ./System/MicroROS/microros_allocators.cyclo ./System/MicroROS/microros_allocators.d ./System/MicroROS/microros_allocators.o ./System/MicroROS/microros_allocators.su ./System/MicroROS/microros_time.cyclo ./System/MicroROS/microros_time.d ./System/MicroROS/microros_time.o ./System/MicroROS/microros_time.su ./System/MicroROS/usb_cdc_transport.cyclo ./System/MicroROS/usb_cdc_transport.d ./System/MicroROS/usb_cdc_transport.o ./System/MicroROS/usb_cdc_transport.su

.PHONY: clean-System-2f-MicroROS

