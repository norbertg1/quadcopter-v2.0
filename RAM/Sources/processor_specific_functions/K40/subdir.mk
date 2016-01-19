################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../../../makefile.local

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS_QUOTED += \
"../Sources/processor_specific_functions/K40/ADC.c" \
"../Sources/processor_specific_functions/K40/PWM.c" \
"../Sources/processor_specific_functions/K40/SPI.c" \
"../Sources/processor_specific_functions/K40/i2c.c" \
"../Sources/processor_specific_functions/K40/misc.c" \
"../Sources/processor_specific_functions/K40/uart.c" \

C_SRCS += \
../Sources/processor_specific_functions/K40/ADC.c \
../Sources/processor_specific_functions/K40/PWM.c \
../Sources/processor_specific_functions/K40/SPI.c \
../Sources/processor_specific_functions/K40/i2c.c \
../Sources/processor_specific_functions/K40/misc.c \
../Sources/processor_specific_functions/K40/uart.c \

OBJS += \
./Sources/processor_specific_functions/K40/ADC.o \
./Sources/processor_specific_functions/K40/PWM.o \
./Sources/processor_specific_functions/K40/SPI.o \
./Sources/processor_specific_functions/K40/i2c.o \
./Sources/processor_specific_functions/K40/misc.o \
./Sources/processor_specific_functions/K40/uart.o \

C_DEPS += \
./Sources/processor_specific_functions/K40/ADC.d \
./Sources/processor_specific_functions/K40/PWM.d \
./Sources/processor_specific_functions/K40/SPI.d \
./Sources/processor_specific_functions/K40/i2c.d \
./Sources/processor_specific_functions/K40/misc.d \
./Sources/processor_specific_functions/K40/uart.d \

OBJS_QUOTED += \
"./Sources/processor_specific_functions/K40/ADC.o" \
"./Sources/processor_specific_functions/K40/PWM.o" \
"./Sources/processor_specific_functions/K40/SPI.o" \
"./Sources/processor_specific_functions/K40/i2c.o" \
"./Sources/processor_specific_functions/K40/misc.o" \
"./Sources/processor_specific_functions/K40/uart.o" \

C_DEPS_QUOTED += \
"./Sources/processor_specific_functions/K40/ADC.d" \
"./Sources/processor_specific_functions/K40/PWM.d" \
"./Sources/processor_specific_functions/K40/SPI.d" \
"./Sources/processor_specific_functions/K40/i2c.d" \
"./Sources/processor_specific_functions/K40/misc.d" \
"./Sources/processor_specific_functions/K40/uart.d" \

OBJS_OS_FORMAT += \
./Sources/processor_specific_functions/K40/ADC.o \
./Sources/processor_specific_functions/K40/PWM.o \
./Sources/processor_specific_functions/K40/SPI.o \
./Sources/processor_specific_functions/K40/i2c.o \
./Sources/processor_specific_functions/K40/misc.o \
./Sources/processor_specific_functions/K40/uart.o \


# Each subdirectory must supply rules for building sources it contributes
Sources/processor_specific_functions/K40/ADC.o: ../Sources/processor_specific_functions/K40/ADC.c
	@echo 'Building file: $<'
	@echo 'Executing target #9 $<'
	@echo 'Invoking: ARM Ltd Windows GCC C Compiler'
	"$(ARMSourceryDirEnv)/arm-none-eabi-gcc" "$<" @"Sources/processor_specific_functions/K40/ADC.args" -Wa,-adhlns="$@.lst" -MMD -MP -MF"$(@:%.o=%.d)" -o"Sources/processor_specific_functions/K40/ADC.o"
	@echo 'Finished building: $<'
	@echo ' '

Sources/processor_specific_functions/K40/PWM.o: ../Sources/processor_specific_functions/K40/PWM.c
	@echo 'Building file: $<'
	@echo 'Executing target #10 $<'
	@echo 'Invoking: ARM Ltd Windows GCC C Compiler'
	"$(ARMSourceryDirEnv)/arm-none-eabi-gcc" "$<" @"Sources/processor_specific_functions/K40/PWM.args" -Wa,-adhlns="$@.lst" -MMD -MP -MF"$(@:%.o=%.d)" -o"Sources/processor_specific_functions/K40/PWM.o"
	@echo 'Finished building: $<'
	@echo ' '

Sources/processor_specific_functions/K40/SPI.o: ../Sources/processor_specific_functions/K40/SPI.c
	@echo 'Building file: $<'
	@echo 'Executing target #11 $<'
	@echo 'Invoking: ARM Ltd Windows GCC C Compiler'
	"$(ARMSourceryDirEnv)/arm-none-eabi-gcc" "$<" @"Sources/processor_specific_functions/K40/SPI.args" -Wa,-adhlns="$@.lst" -MMD -MP -MF"$(@:%.o=%.d)" -o"Sources/processor_specific_functions/K40/SPI.o"
	@echo 'Finished building: $<'
	@echo ' '

Sources/processor_specific_functions/K40/i2c.o: ../Sources/processor_specific_functions/K40/i2c.c
	@echo 'Building file: $<'
	@echo 'Executing target #12 $<'
	@echo 'Invoking: ARM Ltd Windows GCC C Compiler'
	"$(ARMSourceryDirEnv)/arm-none-eabi-gcc" "$<" @"Sources/processor_specific_functions/K40/i2c.args" -Wa,-adhlns="$@.lst" -MMD -MP -MF"$(@:%.o=%.d)" -o"Sources/processor_specific_functions/K40/i2c.o"
	@echo 'Finished building: $<'
	@echo ' '

Sources/processor_specific_functions/K40/misc.o: ../Sources/processor_specific_functions/K40/misc.c
	@echo 'Building file: $<'
	@echo 'Executing target #13 $<'
	@echo 'Invoking: ARM Ltd Windows GCC C Compiler'
	"$(ARMSourceryDirEnv)/arm-none-eabi-gcc" "$<" @"Sources/processor_specific_functions/K40/misc.args" -Wa,-adhlns="$@.lst" -MMD -MP -MF"$(@:%.o=%.d)" -o"Sources/processor_specific_functions/K40/misc.o"
	@echo 'Finished building: $<'
	@echo ' '

Sources/processor_specific_functions/K40/uart.o: ../Sources/processor_specific_functions/K40/uart.c
	@echo 'Building file: $<'
	@echo 'Executing target #14 $<'
	@echo 'Invoking: ARM Ltd Windows GCC C Compiler'
	"$(ARMSourceryDirEnv)/arm-none-eabi-gcc" "$<" @"Sources/processor_specific_functions/K40/uart.args" -Wa,-adhlns="$@.lst" -MMD -MP -MF"$(@:%.o=%.d)" -o"Sources/processor_specific_functions/K40/uart.o"
	@echo 'Finished building: $<'
	@echo ' '


