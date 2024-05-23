################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../OLED_Trinh/ssd1306.c \
../OLED_Trinh/ssd1306_fonts.c 

OBJS += \
./OLED_Trinh/ssd1306.o \
./OLED_Trinh/ssd1306_fonts.o 

C_DEPS += \
./OLED_Trinh/ssd1306.d \
./OLED_Trinh/ssd1306_fonts.d 


# Each subdirectory must supply rules for building sources it contributes
OLED_Trinh/%.o OLED_Trinh/%.su OLED_Trinh/%.cyclo: ../OLED_Trinh/%.c OLED_Trinh/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -DUSE_FULL_LL_DRIVER -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3 -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I"E:/Embedded/STM32CubeIDEws/workspace_1.14.0/f1_freertos_test/Utils" -I"E:/Embedded/STM32CubeIDEws/workspace_1.14.0/f1_freertos_test/OLED_Trinh" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-OLED_Trinh

clean-OLED_Trinh:
	-$(RM) ./OLED_Trinh/ssd1306.cyclo ./OLED_Trinh/ssd1306.d ./OLED_Trinh/ssd1306.o ./OLED_Trinh/ssd1306.su ./OLED_Trinh/ssd1306_fonts.cyclo ./OLED_Trinh/ssd1306_fonts.d ./OLED_Trinh/ssd1306_fonts.o ./OLED_Trinh/ssd1306_fonts.su

.PHONY: clean-OLED_Trinh

