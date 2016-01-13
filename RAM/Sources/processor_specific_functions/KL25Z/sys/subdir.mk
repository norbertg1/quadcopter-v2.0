################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../../../../makefile.local

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS_QUOTED += \
"../Sources/processor_specific_functions/KL25Z/sys/ARM_SysTick.c" \
"../Sources/processor_specific_functions/KL25Z/sys/CrystalClock.c" \
"../Sources/processor_specific_functions/KL25Z/sys/arm_cm0.c" \

C_SRCS += \
../Sources/processor_specific_functions/KL25Z/sys/ARM_SysTick.c \
../Sources/processor_specific_functions/KL25Z/sys/CrystalClock.c \
../Sources/processor_specific_functions/KL25Z/sys/arm_cm0.c \

OBJS += \
./Sources/processor_specific_functions/KL25Z/sys/ARM_SysTick.o \
./Sources/processor_specific_functions/KL25Z/sys/CrystalClock.o \
./Sources/processor_specific_functions/KL25Z/sys/arm_cm0.o \

C_DEPS += \
./Sources/processor_specific_functions/KL25Z/sys/ARM_SysTick.d \
./Sources/processor_specific_functions/KL25Z/sys/CrystalClock.d \
./Sources/processor_specific_functions/KL25Z/sys/arm_cm0.d \

OBJS_QUOTED += \
"./Sources/processor_specific_functions/KL25Z/sys/ARM_SysTick.o" \
"./Sources/processor_specific_functions/KL25Z/sys/CrystalClock.o" \
"./Sources/processor_specific_functions/KL25Z/sys/arm_cm0.o" \

C_DEPS_QUOTED += \
"./Sources/processor_specific_functions/KL25Z/sys/ARM_SysTick.d" \
"./Sources/processor_specific_functions/KL25Z/sys/CrystalClock.d" \
"./Sources/processor_specific_functions/KL25Z/sys/arm_cm0.d" \

OBJS_OS_FORMAT += \
./Sources/processor_specific_functions/KL25Z/sys/ARM_SysTick.o \
./Sources/processor_specific_functions/KL25Z/sys/CrystalClock.o \
./Sources/processor_specific_functions/KL25Z/sys/arm_cm0.o \


# Each subdirectory must supply rules for building sources it contributes
Sources/processor_specific_functions/KL25Z/sys/ARM_SysTick.o: ../Sources/processor_specific_functions/KL25Z/sys/ARM_SysTick.c
	@echo 'Building file: $<'
	@echo 'Executing target #16 $<'
	@echo 'Invoking: ARM Ltd Windows GCC C Compiler'
	"$(ARMSourceryDirEnv)/arm-none-eabi-gcc" "$<" @"Sources/processor_specific_functions/KL25Z/sys/ARM_SysTick.args" -MMD -MP -MF"$(@:%.o=%.d)" -o"Sources/processor_specific_functions/KL25Z/sys/ARM_SysTick.o"
	@echo 'Finished building: $<'
	@echo ' '

Sources/processor_specific_functions/KL25Z/sys/CrystalClock.o: ../Sources/processor_specific_functions/KL25Z/sys/CrystalClock.c
	@echo 'Building file: $<'
	@echo 'Executing target #17 $<'
	@echo 'Invoking: ARM Ltd Windows GCC C Compiler'
	"$(ARMSourceryDirEnv)/arm-none-eabi-gcc" "$<" @"Sources/processor_specific_functions/KL25Z/sys/CrystalClock.args" -MMD -MP -MF"$(@:%.o=%.d)" -o"Sources/processor_specific_functions/KL25Z/sys/CrystalClock.o"
	@echo 'Finished building: $<'
	@echo ' '

Sources/processor_specific_functions/KL25Z/sys/arm_cm0.o: ../Sources/processor_specific_functions/KL25Z/sys/arm_cm0.c
	@echo 'Building file: $<'
	@echo 'Executing target #18 $<'
	@echo 'Invoking: ARM Ltd Windows GCC C Compiler'
	"$(ARMSourceryDirEnv)/arm-none-eabi-gcc" "$<" @"Sources/processor_specific_functions/KL25Z/sys/arm_cm0.args" -MMD -MP -MF"$(@:%.o=%.d)" -o"Sources/processor_specific_functions/KL25Z/sys/arm_cm0.o"
	@echo 'Finished building: $<'
	@echo ' '


