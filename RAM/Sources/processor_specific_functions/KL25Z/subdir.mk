################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../../../makefile.local

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS_QUOTED += \
"../Sources/processor_specific_functions/KL25Z/ADC.c" \
"../Sources/processor_specific_functions/KL25Z/PWM.c" \
"../Sources/processor_specific_functions/KL25Z/SPI.c" \
"../Sources/processor_specific_functions/KL25Z/dma.c" \
"../Sources/processor_specific_functions/KL25Z/i2c.c" \
"../Sources/processor_specific_functions/KL25Z/misc.c" \
"../Sources/processor_specific_functions/KL25Z/uart.c" \

C_SRCS += \
../Sources/processor_specific_functions/KL25Z/ADC.c \
../Sources/processor_specific_functions/KL25Z/PWM.c \
../Sources/processor_specific_functions/KL25Z/SPI.c \
../Sources/processor_specific_functions/KL25Z/dma.c \
../Sources/processor_specific_functions/KL25Z/i2c.c \
../Sources/processor_specific_functions/KL25Z/misc.c \
../Sources/processor_specific_functions/KL25Z/uart.c \

OBJS += \
./Sources/processor_specific_functions/KL25Z/ADC.o \
./Sources/processor_specific_functions/KL25Z/PWM.o \
./Sources/processor_specific_functions/KL25Z/SPI.o \
./Sources/processor_specific_functions/KL25Z/dma.o \
./Sources/processor_specific_functions/KL25Z/i2c.o \
./Sources/processor_specific_functions/KL25Z/misc.o \
./Sources/processor_specific_functions/KL25Z/uart.o \

C_DEPS += \
./Sources/processor_specific_functions/KL25Z/ADC.d \
./Sources/processor_specific_functions/KL25Z/PWM.d \
./Sources/processor_specific_functions/KL25Z/SPI.d \
./Sources/processor_specific_functions/KL25Z/dma.d \
./Sources/processor_specific_functions/KL25Z/i2c.d \
./Sources/processor_specific_functions/KL25Z/misc.d \
./Sources/processor_specific_functions/KL25Z/uart.d \

OBJS_QUOTED += \
"./Sources/processor_specific_functions/KL25Z/ADC.o" \
"./Sources/processor_specific_functions/KL25Z/PWM.o" \
"./Sources/processor_specific_functions/KL25Z/SPI.o" \
"./Sources/processor_specific_functions/KL25Z/dma.o" \
"./Sources/processor_specific_functions/KL25Z/i2c.o" \
"./Sources/processor_specific_functions/KL25Z/misc.o" \
"./Sources/processor_specific_functions/KL25Z/uart.o" \

C_DEPS_QUOTED += \
"./Sources/processor_specific_functions/KL25Z/ADC.d" \
"./Sources/processor_specific_functions/KL25Z/PWM.d" \
"./Sources/processor_specific_functions/KL25Z/SPI.d" \
"./Sources/processor_specific_functions/KL25Z/dma.d" \
"./Sources/processor_specific_functions/KL25Z/i2c.d" \
"./Sources/processor_specific_functions/KL25Z/misc.d" \
"./Sources/processor_specific_functions/KL25Z/uart.d" \

OBJS_OS_FORMAT += \
./Sources/processor_specific_functions/KL25Z/ADC.o \
./Sources/processor_specific_functions/KL25Z/PWM.o \
./Sources/processor_specific_functions/KL25Z/SPI.o \
./Sources/processor_specific_functions/KL25Z/dma.o \
./Sources/processor_specific_functions/KL25Z/i2c.o \
./Sources/processor_specific_functions/KL25Z/misc.o \
./Sources/processor_specific_functions/KL25Z/uart.o \


# Each subdirectory must supply rules for building sources it contributes
Sources/processor_specific_functions/KL25Z/ADC.o: ../Sources/processor_specific_functions/KL25Z/ADC.c
	@echo 'Building file: $<'
	@echo 'Executing target #9 $<'
	@echo 'Invoking: ARM Ltd Windows GCC C Compiler'
	"$(ARMSourceryDirEnv)/arm-none-eabi-gcc" "$<" @"Sources/processor_specific_functions/KL25Z/ADC.args" -MMD -MP -MF"$(@:%.o=%.d)" -o"Sources/processor_specific_functions/KL25Z/ADC.o"
	@echo 'Finished building: $<'
	@echo ' '

Sources/processor_specific_functions/KL25Z/PWM.o: ../Sources/processor_specific_functions/KL25Z/PWM.c
	@echo 'Building file: $<'
	@echo 'Executing target #10 $<'
	@echo 'Invoking: ARM Ltd Windows GCC C Compiler'
	"$(ARMSourceryDirEnv)/arm-none-eabi-gcc" "$<" @"Sources/processor_specific_functions/KL25Z/PWM.args" -MMD -MP -MF"$(@:%.o=%.d)" -o"Sources/processor_specific_functions/KL25Z/PWM.o"
	@echo 'Finished building: $<'
	@echo ' '

Sources/processor_specific_functions/KL25Z/SPI.o: ../Sources/processor_specific_functions/KL25Z/SPI.c
	@echo 'Building file: $<'
	@echo 'Executing target #11 $<'
	@echo 'Invoking: ARM Ltd Windows GCC C Compiler'
	"$(ARMSourceryDirEnv)/arm-none-eabi-gcc" "$<" @"Sources/processor_specific_functions/KL25Z/SPI.args" -MMD -MP -MF"$(@:%.o=%.d)" -o"Sources/processor_specific_functions/KL25Z/SPI.o"
	@echo 'Finished building: $<'
	@echo ' '

Sources/processor_specific_functions/KL25Z/dma.o: ../Sources/processor_specific_functions/KL25Z/dma.c
	@echo 'Building file: $<'
	@echo 'Executing target #12 $<'
	@echo 'Invoking: ARM Ltd Windows GCC C Compiler'
	"$(ARMSourceryDirEnv)/arm-none-eabi-gcc" "$<" @"Sources/processor_specific_functions/KL25Z/dma.args" -MMD -MP -MF"$(@:%.o=%.d)" -o"Sources/processor_specific_functions/KL25Z/dma.o"
	@echo 'Finished building: $<'
	@echo ' '

Sources/processor_specific_functions/KL25Z/i2c.o: ../Sources/processor_specific_functions/KL25Z/i2c.c
	@echo 'Building file: $<'
	@echo 'Executing target #13 $<'
	@echo 'Invoking: ARM Ltd Windows GCC C Compiler'
	"$(ARMSourceryDirEnv)/arm-none-eabi-gcc" "$<" @"Sources/processor_specific_functions/KL25Z/i2c.args" -MMD -MP -MF"$(@:%.o=%.d)" -o"Sources/processor_specific_functions/KL25Z/i2c.o"
	@echo 'Finished building: $<'
	@echo ' '

Sources/processor_specific_functions/KL25Z/misc.o: ../Sources/processor_specific_functions/KL25Z/misc.c
	@echo 'Building file: $<'
	@echo 'Executing target #14 $<'
	@echo 'Invoking: ARM Ltd Windows GCC C Compiler'
	"$(ARMSourceryDirEnv)/arm-none-eabi-gcc" "$<" @"Sources/processor_specific_functions/KL25Z/misc.args" -MMD -MP -MF"$(@:%.o=%.d)" -o"Sources/processor_specific_functions/KL25Z/misc.o"
	@echo 'Finished building: $<'
	@echo ' '

Sources/processor_specific_functions/KL25Z/uart.o: ../Sources/processor_specific_functions/KL25Z/uart.c
	@echo 'Building file: $<'
	@echo 'Executing target #15 $<'
	@echo 'Invoking: ARM Ltd Windows GCC C Compiler'
	"$(ARMSourceryDirEnv)/arm-none-eabi-gcc" "$<" @"Sources/processor_specific_functions/KL25Z/uart.args" -MMD -MP -MF"$(@:%.o=%.d)" -o"Sources/processor_specific_functions/KL25Z/uart.o"
	@echo 'Finished building: $<'
	@echo ' '


