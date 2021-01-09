################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/Src/stm32f103xx_dma_driver.c \
../Drivers/Src/stm32f103xx_gpio_driver.c \
../Drivers/Src/stm32f103xx_rcc_driver.c \
../Drivers/Src/stm32f103xx_tim_driver.c \
../Drivers/Src/stm32f103xx_usart_driver.c 

OBJS += \
./Drivers/Src/stm32f103xx_dma_driver.o \
./Drivers/Src/stm32f103xx_gpio_driver.o \
./Drivers/Src/stm32f103xx_rcc_driver.o \
./Drivers/Src/stm32f103xx_tim_driver.o \
./Drivers/Src/stm32f103xx_usart_driver.o 

C_DEPS += \
./Drivers/Src/stm32f103xx_dma_driver.d \
./Drivers/Src/stm32f103xx_gpio_driver.d \
./Drivers/Src/stm32f103xx_rcc_driver.d \
./Drivers/Src/stm32f103xx_tim_driver.d \
./Drivers/Src/stm32f103xx_usart_driver.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/Src/stm32f103xx_dma_driver.o: ../Drivers/Src/stm32f103xx_dma_driver.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DSTM32F103RCTx -DSTM32 -DSTM32F1 -DDEBUG -c -I../Inc -I"C:/Users/rkdgu/Documents/Projects/24V BLDC Motor Controller/Firmware/BLDC_Motor_Controller_24V/BLDC_Motor_Controller_24V/Src" -I"C:/Users/rkdgu/Documents/Projects/24V BLDC Motor Controller/Firmware/BLDC_Motor_Controller_24V/BLDC_Motor_Controller_24V/Drivers/Inc" -I"C:/Users/rkdgu/Documents/Projects/24V BLDC Motor Controller/Firmware/BLDC_Motor_Controller_24V/BLDC_Motor_Controller_24V/Drivers/Src" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/Src/stm32f103xx_dma_driver.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Drivers/Src/stm32f103xx_gpio_driver.o: ../Drivers/Src/stm32f103xx_gpio_driver.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DSTM32F103RCTx -DSTM32 -DSTM32F1 -DDEBUG -c -I../Inc -I"C:/Users/rkdgu/Documents/Projects/24V BLDC Motor Controller/Firmware/BLDC_Motor_Controller_24V/BLDC_Motor_Controller_24V/Src" -I"C:/Users/rkdgu/Documents/Projects/24V BLDC Motor Controller/Firmware/BLDC_Motor_Controller_24V/BLDC_Motor_Controller_24V/Drivers/Inc" -I"C:/Users/rkdgu/Documents/Projects/24V BLDC Motor Controller/Firmware/BLDC_Motor_Controller_24V/BLDC_Motor_Controller_24V/Drivers/Src" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/Src/stm32f103xx_gpio_driver.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Drivers/Src/stm32f103xx_rcc_driver.o: ../Drivers/Src/stm32f103xx_rcc_driver.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DSTM32F103RCTx -DSTM32 -DSTM32F1 -DDEBUG -c -I../Inc -I"C:/Users/rkdgu/Documents/Projects/24V BLDC Motor Controller/Firmware/BLDC_Motor_Controller_24V/BLDC_Motor_Controller_24V/Src" -I"C:/Users/rkdgu/Documents/Projects/24V BLDC Motor Controller/Firmware/BLDC_Motor_Controller_24V/BLDC_Motor_Controller_24V/Drivers/Inc" -I"C:/Users/rkdgu/Documents/Projects/24V BLDC Motor Controller/Firmware/BLDC_Motor_Controller_24V/BLDC_Motor_Controller_24V/Drivers/Src" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/Src/stm32f103xx_rcc_driver.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Drivers/Src/stm32f103xx_tim_driver.o: ../Drivers/Src/stm32f103xx_tim_driver.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DSTM32F103RCTx -DSTM32 -DSTM32F1 -DDEBUG -c -I../Inc -I"C:/Users/rkdgu/Documents/Projects/24V BLDC Motor Controller/Firmware/BLDC_Motor_Controller_24V/BLDC_Motor_Controller_24V/Src" -I"C:/Users/rkdgu/Documents/Projects/24V BLDC Motor Controller/Firmware/BLDC_Motor_Controller_24V/BLDC_Motor_Controller_24V/Drivers/Inc" -I"C:/Users/rkdgu/Documents/Projects/24V BLDC Motor Controller/Firmware/BLDC_Motor_Controller_24V/BLDC_Motor_Controller_24V/Drivers/Src" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/Src/stm32f103xx_tim_driver.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Drivers/Src/stm32f103xx_usart_driver.o: ../Drivers/Src/stm32f103xx_usart_driver.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DSTM32F103RCTx -DSTM32 -DSTM32F1 -DDEBUG -c -I../Inc -I"C:/Users/rkdgu/Documents/Projects/24V BLDC Motor Controller/Firmware/BLDC_Motor_Controller_24V/BLDC_Motor_Controller_24V/Src" -I"C:/Users/rkdgu/Documents/Projects/24V BLDC Motor Controller/Firmware/BLDC_Motor_Controller_24V/BLDC_Motor_Controller_24V/Drivers/Inc" -I"C:/Users/rkdgu/Documents/Projects/24V BLDC Motor Controller/Firmware/BLDC_Motor_Controller_24V/BLDC_Motor_Controller_24V/Drivers/Src" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/Src/stm32f103xx_usart_driver.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

