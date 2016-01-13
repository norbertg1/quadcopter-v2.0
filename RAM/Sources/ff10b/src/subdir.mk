################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../../../makefile.local

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS_QUOTED += \
"../Sources/ff10b/src/diskio.c" \
"../Sources/ff10b/src/ff.c" \

C_SRCS += \
../Sources/ff10b/src/diskio.c \
../Sources/ff10b/src/ff.c \

OBJS += \
./Sources/ff10b/src/diskio.o \
./Sources/ff10b/src/ff.o \

C_DEPS += \
./Sources/ff10b/src/diskio.d \
./Sources/ff10b/src/ff.d \

OBJS_QUOTED += \
"./Sources/ff10b/src/diskio.o" \
"./Sources/ff10b/src/ff.o" \

C_DEPS_QUOTED += \
"./Sources/ff10b/src/diskio.d" \
"./Sources/ff10b/src/ff.d" \

OBJS_OS_FORMAT += \
./Sources/ff10b/src/diskio.o \
./Sources/ff10b/src/ff.o \


# Each subdirectory must supply rules for building sources it contributes
Sources/ff10b/src/diskio.o: ../Sources/ff10b/src/diskio.c
	@echo 'Building file: $<'
	@echo 'Executing target #20 $<'
	@echo 'Invoking: ARM Ltd Windows GCC C Compiler'
	"$(ARMSourceryDirEnv)/arm-none-eabi-gcc" "$<" @"Sources/ff10b/src/diskio.args" -MMD -MP -MF"$(@:%.o=%.d)" -o"Sources/ff10b/src/diskio.o"
	@echo 'Finished building: $<'
	@echo ' '

Sources/ff10b/src/ff.o: ../Sources/ff10b/src/ff.c
	@echo 'Building file: $<'
	@echo 'Executing target #21 $<'
	@echo 'Invoking: ARM Ltd Windows GCC C Compiler'
	"$(ARMSourceryDirEnv)/arm-none-eabi-gcc" "$<" @"Sources/ff10b/src/ff.args" -MMD -MP -MF"$(@:%.o=%.d)" -o"Sources/ff10b/src/ff.o"
	@echo 'Finished building: $<'
	@echo ' '


