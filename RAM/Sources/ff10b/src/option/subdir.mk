################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../../../../makefile.local

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS_QUOTED += \
"../Sources/ff10b/src/option/unicode.c" \

C_SRCS += \
../Sources/ff10b/src/option/unicode.c \

OBJS += \
./Sources/ff10b/src/option/unicode.o \

C_DEPS += \
./Sources/ff10b/src/option/unicode.d \

OBJS_QUOTED += \
"./Sources/ff10b/src/option/unicode.o" \

C_DEPS_QUOTED += \
"./Sources/ff10b/src/option/unicode.d" \

OBJS_OS_FORMAT += \
./Sources/ff10b/src/option/unicode.o \


# Each subdirectory must supply rules for building sources it contributes
Sources/ff10b/src/option/unicode.o: ../Sources/ff10b/src/option/unicode.c
	@echo 'Building file: $<'
	@echo 'Executing target #19 $<'
	@echo 'Invoking: ARM Ltd Windows GCC C Compiler'
	"$(ARMSourceryDirEnv)/arm-none-eabi-gcc" "$<" @"Sources/ff10b/src/option/unicode.args" -MMD -MP -MF"$(@:%.o=%.d)" -o"Sources/ff10b/src/option/unicode.o"
	@echo 'Finished building: $<'
	@echo ' '


