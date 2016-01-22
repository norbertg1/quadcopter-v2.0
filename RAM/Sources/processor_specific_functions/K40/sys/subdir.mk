################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../../../../makefile.local

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS_QUOTED += \
"../Sources/processor_specific_functions/K40/sys/Clock.c" \
"../Sources/processor_specific_functions/K40/sys/SysTick.c" \
"../Sources/processor_specific_functions/K40/sys/arm_cm4.c" \
"../Sources/processor_specific_functions/K40/sys/sa_mtb.c" \

C_SRCS += \
../Sources/processor_specific_functions/K40/sys/Clock.c \
../Sources/processor_specific_functions/K40/sys/SysTick.c \
../Sources/processor_specific_functions/K40/sys/arm_cm4.c \
../Sources/processor_specific_functions/K40/sys/sa_mtb.c \

OBJS += \
./Sources/processor_specific_functions/K40/sys/Clock.o \
./Sources/processor_specific_functions/K40/sys/SysTick.o \
./Sources/processor_specific_functions/K40/sys/arm_cm4.o \
./Sources/processor_specific_functions/K40/sys/sa_mtb.o \

C_DEPS += \
./Sources/processor_specific_functions/K40/sys/Clock.d \
./Sources/processor_specific_functions/K40/sys/SysTick.d \
./Sources/processor_specific_functions/K40/sys/arm_cm4.d \
./Sources/processor_specific_functions/K40/sys/sa_mtb.d \

OBJS_QUOTED += \
"./Sources/processor_specific_functions/K40/sys/Clock.o" \
"./Sources/processor_specific_functions/K40/sys/SysTick.o" \
"./Sources/processor_specific_functions/K40/sys/arm_cm4.o" \
"./Sources/processor_specific_functions/K40/sys/sa_mtb.o" \

C_DEPS_QUOTED += \
"./Sources/processor_specific_functions/K40/sys/Clock.d" \
"./Sources/processor_specific_functions/K40/sys/SysTick.d" \
"./Sources/processor_specific_functions/K40/sys/arm_cm4.d" \
"./Sources/processor_specific_functions/K40/sys/sa_mtb.d" \

OBJS_OS_FORMAT += \
./Sources/processor_specific_functions/K40/sys/Clock.o \
./Sources/processor_specific_functions/K40/sys/SysTick.o \
./Sources/processor_specific_functions/K40/sys/arm_cm4.o \
./Sources/processor_specific_functions/K40/sys/sa_mtb.o \


# Each subdirectory must supply rules for building sources it contributes
Sources/processor_specific_functions/K40/sys/Clock.o: ../Sources/processor_specific_functions/K40/sys/Clock.c
	@echo 'Building file: $<'
	@echo 'Executing target #15 $<'
	@echo 'Invoking: ARM Ltd Windows GCC C Compiler'
	"$(ARMSourceryDirEnv)/arm-none-eabi-gcc" "$<" @"Sources/processor_specific_functions/K40/sys/Clock.args" -Wa,-adhlns="$@.lst" -MMD -MP -MF"$(@:%.o=%.d)" -o"Sources/processor_specific_functions/K40/sys/Clock.o"
	@echo 'Finished building: $<'
	@echo ' '

Sources/processor_specific_functions/K40/sys/SysTick.o: ../Sources/processor_specific_functions/K40/sys/SysTick.c
	@echo 'Building file: $<'
	@echo 'Executing target #16 $<'
	@echo 'Invoking: ARM Ltd Windows GCC C Compiler'
	"$(ARMSourceryDirEnv)/arm-none-eabi-gcc" "$<" @"Sources/processor_specific_functions/K40/sys/SysTick.args" -Wa,-adhlns="$@.lst" -MMD -MP -MF"$(@:%.o=%.d)" -o"Sources/processor_specific_functions/K40/sys/SysTick.o"
	@echo 'Finished building: $<'
	@echo ' '

Sources/processor_specific_functions/K40/sys/arm_cm4.o: ../Sources/processor_specific_functions/K40/sys/arm_cm4.c
	@echo 'Building file: $<'
	@echo 'Executing target #17 $<'
	@echo 'Invoking: ARM Ltd Windows GCC C Compiler'
	"$(ARMSourceryDirEnv)/arm-none-eabi-gcc" "$<" @"Sources/processor_specific_functions/K40/sys/arm_cm4.args" -Wa,-adhlns="$@.lst" -MMD -MP -MF"$(@:%.o=%.d)" -o"Sources/processor_specific_functions/K40/sys/arm_cm4.o"
	@echo 'Finished building: $<'
	@echo ' '

Sources/processor_specific_functions/K40/sys/sa_mtb.o: ../Sources/processor_specific_functions/K40/sys/sa_mtb.c
	@echo 'Building file: $<'
	@echo 'Executing target #18 $<'
	@echo 'Invoking: ARM Ltd Windows GCC C Compiler'
	"$(ARMSourceryDirEnv)/arm-none-eabi-gcc" "$<" @"Sources/processor_specific_functions/K40/sys/sa_mtb.args" -Wa,-adhlns="$@.lst" -MMD -MP -MF"$(@:%.o=%.d)" -o"Sources/processor_specific_functions/K40/sys/sa_mtb.o"
	@echo 'Finished building: $<'
	@echo ' '


