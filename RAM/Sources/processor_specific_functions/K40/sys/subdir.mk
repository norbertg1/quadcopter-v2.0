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
./Sources/processor_specific_functions/K40/sys/Clock_c.obj \
./Sources/processor_specific_functions/K40/sys/SysTick_c.obj \
./Sources/processor_specific_functions/K40/sys/arm_cm4_c.obj \
./Sources/processor_specific_functions/K40/sys/sa_mtb_c.obj \

OBJS_QUOTED += \
"./Sources/processor_specific_functions/K40/sys/Clock_c.obj" \
"./Sources/processor_specific_functions/K40/sys/SysTick_c.obj" \
"./Sources/processor_specific_functions/K40/sys/arm_cm4_c.obj" \
"./Sources/processor_specific_functions/K40/sys/sa_mtb_c.obj" \

C_DEPS += \
./Sources/processor_specific_functions/K40/sys/Clock_c.d \
./Sources/processor_specific_functions/K40/sys/SysTick_c.d \
./Sources/processor_specific_functions/K40/sys/arm_cm4_c.d \
./Sources/processor_specific_functions/K40/sys/sa_mtb_c.d \

C_DEPS_QUOTED += \
"./Sources/processor_specific_functions/K40/sys/Clock_c.d" \
"./Sources/processor_specific_functions/K40/sys/SysTick_c.d" \
"./Sources/processor_specific_functions/K40/sys/arm_cm4_c.d" \
"./Sources/processor_specific_functions/K40/sys/sa_mtb_c.d" \

OBJS_OS_FORMAT += \
./Sources/processor_specific_functions/K40/sys/Clock_c.obj \
./Sources/processor_specific_functions/K40/sys/SysTick_c.obj \
./Sources/processor_specific_functions/K40/sys/arm_cm4_c.obj \
./Sources/processor_specific_functions/K40/sys/sa_mtb_c.obj \


# Each subdirectory must supply rules for building sources it contributes
Sources/processor_specific_functions/K40/sys/Clock_c.obj: ../Sources/processor_specific_functions/K40/sys/Clock.c
	@echo 'Building file: $<'
	@echo 'Executing target #17 $<'
	@echo 'Invoking: ARM Compiler'
	"$(ARM_ToolsDirEnv)/mwccarm" -gccinc @@"Sources/processor_specific_functions/K40/sys/Clock.args" -o "Sources/processor_specific_functions/K40/sys/Clock_c.obj" -c "$<" -MD -gccdep
	@echo 'Finished building: $<'
	@echo ' '

Sources/processor_specific_functions/K40/sys/%.d: ../Sources/processor_specific_functions/K40/sys/%.c
	@echo 'Regenerating dependency file: $@'
	
	@echo ' '

Sources/processor_specific_functions/K40/sys/SysTick_c.obj: ../Sources/processor_specific_functions/K40/sys/SysTick.c
	@echo 'Building file: $<'
	@echo 'Executing target #18 $<'
	@echo 'Invoking: ARM Compiler'
	"$(ARM_ToolsDirEnv)/mwccarm" -gccinc @@"Sources/processor_specific_functions/K40/sys/SysTick.args" -o "Sources/processor_specific_functions/K40/sys/SysTick_c.obj" -c "$<" -MD -gccdep
	@echo 'Finished building: $<'
	@echo ' '

Sources/processor_specific_functions/K40/sys/arm_cm4_c.obj: ../Sources/processor_specific_functions/K40/sys/arm_cm4.c
	@echo 'Building file: $<'
	@echo 'Executing target #19 $<'
	@echo 'Invoking: ARM Compiler'
	"$(ARM_ToolsDirEnv)/mwccarm" -gccinc @@"Sources/processor_specific_functions/K40/sys/arm_cm4.args" -o "Sources/processor_specific_functions/K40/sys/arm_cm4_c.obj" -c "$<" -MD -gccdep
	@echo 'Finished building: $<'
	@echo ' '

Sources/processor_specific_functions/K40/sys/sa_mtb_c.obj: ../Sources/processor_specific_functions/K40/sys/sa_mtb.c
	@echo 'Building file: $<'
	@echo 'Executing target #20 $<'
	@echo 'Invoking: ARM Compiler'
	"$(ARM_ToolsDirEnv)/mwccarm" -gccinc @@"Sources/processor_specific_functions/K40/sys/sa_mtb.args" -o "Sources/processor_specific_functions/K40/sys/sa_mtb_c.obj" -c "$<" -MD -gccdep
	@echo 'Finished building: $<'
	@echo ' '


