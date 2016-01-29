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
"../Sources/processor_specific_functions/K40/rtc.c" \
"../Sources/processor_specific_functions/K40/uart.c" \

C_SRCS += \
../Sources/processor_specific_functions/K40/ADC.c \
../Sources/processor_specific_functions/K40/PWM.c \
../Sources/processor_specific_functions/K40/SPI.c \
../Sources/processor_specific_functions/K40/i2c.c \
../Sources/processor_specific_functions/K40/misc.c \
../Sources/processor_specific_functions/K40/rtc.c \
../Sources/processor_specific_functions/K40/uart.c \

OBJS += \
./Sources/processor_specific_functions/K40/ADC_c.obj \
./Sources/processor_specific_functions/K40/PWM_c.obj \
./Sources/processor_specific_functions/K40/SPI_c.obj \
./Sources/processor_specific_functions/K40/i2c_c.obj \
./Sources/processor_specific_functions/K40/misc_c.obj \
./Sources/processor_specific_functions/K40/rtc_c.obj \
./Sources/processor_specific_functions/K40/uart_c.obj \

OBJS_QUOTED += \
"./Sources/processor_specific_functions/K40/ADC_c.obj" \
"./Sources/processor_specific_functions/K40/PWM_c.obj" \
"./Sources/processor_specific_functions/K40/SPI_c.obj" \
"./Sources/processor_specific_functions/K40/i2c_c.obj" \
"./Sources/processor_specific_functions/K40/misc_c.obj" \
"./Sources/processor_specific_functions/K40/rtc_c.obj" \
"./Sources/processor_specific_functions/K40/uart_c.obj" \

C_DEPS += \
./Sources/processor_specific_functions/K40/ADC_c.d \
./Sources/processor_specific_functions/K40/PWM_c.d \
./Sources/processor_specific_functions/K40/SPI_c.d \
./Sources/processor_specific_functions/K40/i2c_c.d \
./Sources/processor_specific_functions/K40/misc_c.d \
./Sources/processor_specific_functions/K40/rtc_c.d \
./Sources/processor_specific_functions/K40/uart_c.d \

C_DEPS_QUOTED += \
"./Sources/processor_specific_functions/K40/ADC_c.d" \
"./Sources/processor_specific_functions/K40/PWM_c.d" \
"./Sources/processor_specific_functions/K40/SPI_c.d" \
"./Sources/processor_specific_functions/K40/i2c_c.d" \
"./Sources/processor_specific_functions/K40/misc_c.d" \
"./Sources/processor_specific_functions/K40/rtc_c.d" \
"./Sources/processor_specific_functions/K40/uart_c.d" \

OBJS_OS_FORMAT += \
./Sources/processor_specific_functions/K40/ADC_c.obj \
./Sources/processor_specific_functions/K40/PWM_c.obj \
./Sources/processor_specific_functions/K40/SPI_c.obj \
./Sources/processor_specific_functions/K40/i2c_c.obj \
./Sources/processor_specific_functions/K40/misc_c.obj \
./Sources/processor_specific_functions/K40/rtc_c.obj \
./Sources/processor_specific_functions/K40/uart_c.obj \


# Each subdirectory must supply rules for building sources it contributes
Sources/processor_specific_functions/K40/ADC_c.obj: ../Sources/processor_specific_functions/K40/ADC.c
	@echo 'Building file: $<'
	@echo 'Executing target #10 $<'
	@echo 'Invoking: ARM Compiler'
	"$(ARM_ToolsDirEnv)/mwccarm" -gccinc @@"Sources/processor_specific_functions/K40/ADC.args" -o "Sources/processor_specific_functions/K40/ADC_c.obj" -c "$<" -MD -gccdep
	@echo 'Finished building: $<'
	@echo ' '

Sources/processor_specific_functions/K40/%.d: ../Sources/processor_specific_functions/K40/%.c
	@echo 'Regenerating dependency file: $@'
	
	@echo ' '

Sources/processor_specific_functions/K40/PWM_c.obj: ../Sources/processor_specific_functions/K40/PWM.c
	@echo 'Building file: $<'
	@echo 'Executing target #11 $<'
	@echo 'Invoking: ARM Compiler'
	"$(ARM_ToolsDirEnv)/mwccarm" -gccinc @@"Sources/processor_specific_functions/K40/PWM.args" -o "Sources/processor_specific_functions/K40/PWM_c.obj" -c "$<" -MD -gccdep
	@echo 'Finished building: $<'
	@echo ' '

Sources/processor_specific_functions/K40/SPI_c.obj: ../Sources/processor_specific_functions/K40/SPI.c
	@echo 'Building file: $<'
	@echo 'Executing target #12 $<'
	@echo 'Invoking: ARM Compiler'
	"$(ARM_ToolsDirEnv)/mwccarm" -gccinc @@"Sources/processor_specific_functions/K40/SPI.args" -o "Sources/processor_specific_functions/K40/SPI_c.obj" -c "$<" -MD -gccdep
	@echo 'Finished building: $<'
	@echo ' '

Sources/processor_specific_functions/K40/i2c_c.obj: ../Sources/processor_specific_functions/K40/i2c.c
	@echo 'Building file: $<'
	@echo 'Executing target #13 $<'
	@echo 'Invoking: ARM Compiler'
	"$(ARM_ToolsDirEnv)/mwccarm" -gccinc @@"Sources/processor_specific_functions/K40/i2c.args" -o "Sources/processor_specific_functions/K40/i2c_c.obj" -c "$<" -MD -gccdep
	@echo 'Finished building: $<'
	@echo ' '

Sources/processor_specific_functions/K40/misc_c.obj: ../Sources/processor_specific_functions/K40/misc.c
	@echo 'Building file: $<'
	@echo 'Executing target #14 $<'
	@echo 'Invoking: ARM Compiler'
	"$(ARM_ToolsDirEnv)/mwccarm" -gccinc @@"Sources/processor_specific_functions/K40/misc.args" -o "Sources/processor_specific_functions/K40/misc_c.obj" -c "$<" -MD -gccdep
	@echo 'Finished building: $<'
	@echo ' '

Sources/processor_specific_functions/K40/rtc_c.obj: ../Sources/processor_specific_functions/K40/rtc.c
	@echo 'Building file: $<'
	@echo 'Executing target #15 $<'
	@echo 'Invoking: ARM Compiler'
	"$(ARM_ToolsDirEnv)/mwccarm" -gccinc @@"Sources/processor_specific_functions/K40/rtc.args" -o "Sources/processor_specific_functions/K40/rtc_c.obj" -c "$<" -MD -gccdep
	@echo 'Finished building: $<'
	@echo ' '

Sources/processor_specific_functions/K40/uart_c.obj: ../Sources/processor_specific_functions/K40/uart.c
	@echo 'Building file: $<'
	@echo 'Executing target #16 $<'
	@echo 'Invoking: ARM Compiler'
	"$(ARM_ToolsDirEnv)/mwccarm" -gccinc @@"Sources/processor_specific_functions/K40/uart.args" -o "Sources/processor_specific_functions/K40/uart_c.obj" -c "$<" -MD -gccdep
	@echo 'Finished building: $<'
	@echo ' '


