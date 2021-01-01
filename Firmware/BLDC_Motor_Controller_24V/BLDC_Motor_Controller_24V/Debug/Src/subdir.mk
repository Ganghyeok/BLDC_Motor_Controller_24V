################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/bldc.c \
../Src/it.c \
../Src/main.c \
../Src/msp.c \
../Src/syscalls.c \
../Src/sysmem.c \
../Src/user_func.c 

OBJS += \
./Src/bldc.o \
./Src/it.o \
./Src/main.o \
./Src/msp.o \
./Src/syscalls.o \
./Src/sysmem.o \
./Src/user_func.o 

C_DEPS += \
./Src/bldc.d \
./Src/it.d \
./Src/main.d \
./Src/msp.d \
./Src/syscalls.d \
./Src/sysmem.d \
./Src/user_func.d 


# Each subdirectory must supply rules for building sources it contributes
Src/bldc.o: ../Src/bldc.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DSTM32F103RCTx -DSTM32 -DSTM32F1 -DDEBUG -c -I../Inc -I"C:/Users/rkdgu/Documents/Projects/24V BLDC Motor Controller/Firmware/BLDC_Motor_Controller_24V/BLDC_Motor_Controller_24V/Src" -I"C:/Users/rkdgu/Documents/Projects/24V BLDC Motor Controller/Firmware/BLDC_Motor_Controller_24V/BLDC_Motor_Controller_24V/Drivers/Inc" -I"C:/Users/rkdgu/Documents/Projects/24V BLDC Motor Controller/Firmware/BLDC_Motor_Controller_24V/BLDC_Motor_Controller_24V/Drivers/Src" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Src/bldc.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Src/it.o: ../Src/it.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DSTM32F103RCTx -DSTM32 -DSTM32F1 -DDEBUG -c -I../Inc -I"C:/Users/rkdgu/Documents/Projects/24V BLDC Motor Controller/Firmware/BLDC_Motor_Controller_24V/BLDC_Motor_Controller_24V/Src" -I"C:/Users/rkdgu/Documents/Projects/24V BLDC Motor Controller/Firmware/BLDC_Motor_Controller_24V/BLDC_Motor_Controller_24V/Drivers/Inc" -I"C:/Users/rkdgu/Documents/Projects/24V BLDC Motor Controller/Firmware/BLDC_Motor_Controller_24V/BLDC_Motor_Controller_24V/Drivers/Src" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Src/it.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Src/main.o: ../Src/main.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DSTM32F103RCTx -DSTM32 -DSTM32F1 -DDEBUG -c -I../Inc -I"C:/Users/rkdgu/Documents/Projects/24V BLDC Motor Controller/Firmware/BLDC_Motor_Controller_24V/BLDC_Motor_Controller_24V/Src" -I"C:/Users/rkdgu/Documents/Projects/24V BLDC Motor Controller/Firmware/BLDC_Motor_Controller_24V/BLDC_Motor_Controller_24V/Drivers/Inc" -I"C:/Users/rkdgu/Documents/Projects/24V BLDC Motor Controller/Firmware/BLDC_Motor_Controller_24V/BLDC_Motor_Controller_24V/Drivers/Src" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Src/main.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Src/msp.o: ../Src/msp.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DSTM32F103RCTx -DSTM32 -DSTM32F1 -DDEBUG -c -I../Inc -I"C:/Users/rkdgu/Documents/Projects/24V BLDC Motor Controller/Firmware/BLDC_Motor_Controller_24V/BLDC_Motor_Controller_24V/Src" -I"C:/Users/rkdgu/Documents/Projects/24V BLDC Motor Controller/Firmware/BLDC_Motor_Controller_24V/BLDC_Motor_Controller_24V/Drivers/Inc" -I"C:/Users/rkdgu/Documents/Projects/24V BLDC Motor Controller/Firmware/BLDC_Motor_Controller_24V/BLDC_Motor_Controller_24V/Drivers/Src" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Src/msp.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Src/syscalls.o: ../Src/syscalls.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DSTM32F103RCTx -DSTM32 -DSTM32F1 -DDEBUG -c -I../Inc -I"C:/Users/rkdgu/Documents/Projects/24V BLDC Motor Controller/Firmware/BLDC_Motor_Controller_24V/BLDC_Motor_Controller_24V/Src" -I"C:/Users/rkdgu/Documents/Projects/24V BLDC Motor Controller/Firmware/BLDC_Motor_Controller_24V/BLDC_Motor_Controller_24V/Drivers/Inc" -I"C:/Users/rkdgu/Documents/Projects/24V BLDC Motor Controller/Firmware/BLDC_Motor_Controller_24V/BLDC_Motor_Controller_24V/Drivers/Src" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Src/syscalls.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Src/sysmem.o: ../Src/sysmem.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DSTM32F103RCTx -DSTM32 -DSTM32F1 -DDEBUG -c -I../Inc -I"C:/Users/rkdgu/Documents/Projects/24V BLDC Motor Controller/Firmware/BLDC_Motor_Controller_24V/BLDC_Motor_Controller_24V/Src" -I"C:/Users/rkdgu/Documents/Projects/24V BLDC Motor Controller/Firmware/BLDC_Motor_Controller_24V/BLDC_Motor_Controller_24V/Drivers/Inc" -I"C:/Users/rkdgu/Documents/Projects/24V BLDC Motor Controller/Firmware/BLDC_Motor_Controller_24V/BLDC_Motor_Controller_24V/Drivers/Src" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Src/sysmem.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Src/user_func.o: ../Src/user_func.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DSTM32F103RCTx -DSTM32 -DSTM32F1 -DDEBUG -c -I../Inc -I"C:/Users/rkdgu/Documents/Projects/24V BLDC Motor Controller/Firmware/BLDC_Motor_Controller_24V/BLDC_Motor_Controller_24V/Src" -I"C:/Users/rkdgu/Documents/Projects/24V BLDC Motor Controller/Firmware/BLDC_Motor_Controller_24V/BLDC_Motor_Controller_24V/Drivers/Inc" -I"C:/Users/rkdgu/Documents/Projects/24V BLDC Motor Controller/Firmware/BLDC_Motor_Controller_24V/BLDC_Motor_Controller_24V/Drivers/Src" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Src/user_func.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

