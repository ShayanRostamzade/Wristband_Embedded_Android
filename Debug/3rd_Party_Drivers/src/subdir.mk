################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../3rd_Party_Drivers/src/fonts.c \
../3rd_Party_Drivers/src/ssd1306.c 

OBJS += \
./3rd_Party_Drivers/src/fonts.o \
./3rd_Party_Drivers/src/ssd1306.o 

C_DEPS += \
./3rd_Party_Drivers/src/fonts.d \
./3rd_Party_Drivers/src/ssd1306.d 


# Each subdirectory must supply rules for building sources it contributes
3rd_Party_Drivers/src/fonts.o: ../3rd_Party_Drivers/src/fonts.c 3rd_Party_Drivers/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I"/Users/shayanrostamzade/STM32CubeIDE/workspace_1.6.1/Medical_Wrist_Band/3rd_Party_Drivers/inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"3rd_Party_Drivers/src/fonts.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
3rd_Party_Drivers/src/ssd1306.o: ../3rd_Party_Drivers/src/ssd1306.c 3rd_Party_Drivers/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I"/Users/shayanrostamzade/STM32CubeIDE/workspace_1.6.1/Medical_Wrist_Band/3rd_Party_Drivers/inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"3rd_Party_Drivers/src/ssd1306.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

