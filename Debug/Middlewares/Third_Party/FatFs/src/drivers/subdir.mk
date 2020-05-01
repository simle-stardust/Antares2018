################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Middlewares/Third_Party/FatFs/src/drivers/sd_diskio.c 

OBJS += \
./Middlewares/Third_Party/FatFs/src/drivers/sd_diskio.o 

C_DEPS += \
./Middlewares/Third_Party/FatFs/src/drivers/sd_diskio.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/Third_Party/FatFs/src/drivers/%.o: ../Middlewares/Third_Party/FatFs/src/drivers/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -mfloat-abi=soft '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32L151xD '-D__weak=__attribute__((weak))' '-D__packed="__attribute__((__packed__))"' -I"/home/szymon/simle/Antares2018/Inc" -I"/home/szymon/simle/Antares2018/Drivers/STM32L1xx_HAL_Driver/Inc" -I"/home/szymon/simle/Antares2018/Drivers/STM32L1xx_HAL_Driver/Inc/Legacy" -I"/home/szymon/simle/Antares2018/Middlewares/Third_Party/FatFs/src/drivers" -I"/home/szymon/simle/Antares2018/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3" -I"/home/szymon/simle/Antares2018/Drivers/CMSIS/Device/ST/STM32L1xx/Include" -I"/home/szymon/simle/Antares2018/Middlewares/Third_Party/FatFs/src" -I"/home/szymon/simle/Antares2018/Middlewares/Third_Party/FreeRTOS/Source/include" -I"/home/szymon/simle/Antares2018/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS" -I"/home/szymon/simle/Antares2018/Drivers/CMSIS/Include" -I"/home/szymon/simle/Antares2018/Inc" -I"/home/szymon/simle/Antares2018/Drivers/CMSIS/Include" -I"/home/szymon/simle/Antares2018/Drivers/CMSIS/Include" -I"/home/szymon/simle/Antares2018/Drivers/CMSIS/Include" -I"/home/szymon/simle/Antares2018/Drivers/CMSIS/Include" -I"/home/szymon/simle/Antares2018/Drivers/CMSIS/Include" -I"/home/szymon/simle/Antares2018/Drivers/CMSIS/Include" -I"/home/szymon/simle/Antares2018/Drivers/CMSIS/Include" -I"/home/szymon/simle/Antares2018/Drivers/CMSIS/Include" -I"/home/szymon/simle/Antares2018/Drivers/CMSIS/Include" -I"/home/szymon/simle/Antares2018/Drivers/CMSIS/Include"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


