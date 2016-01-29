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
./Sources/ff10b/src/option/unicode_c.obj \

OBJS_QUOTED += \
"./Sources/ff10b/src/option/unicode_c.obj" \

C_DEPS += \
./Sources/ff10b/src/option/unicode_c.d \

C_DEPS_QUOTED += \
"./Sources/ff10b/src/option/unicode_c.d" \

OBJS_OS_FORMAT += \
./Sources/ff10b/src/option/unicode_c.obj \


# Each subdirectory must supply rules for building sources it contributes
Sources/ff10b/src/option/unicode_c.obj: ../Sources/ff10b/src/option/unicode.c
	@echo 'Building file: $<'
	@echo 'Executing target #23 $<'
	@echo 'Invoking: ARM Compiler'
	"$(ARM_ToolsDirEnv)/mwccarm" -gccinc @@"Sources/ff10b/src/option/unicode.args" -o "Sources/ff10b/src/option/unicode_c.obj" -c "$<" -MD -gccdep
	@echo 'Finished building: $<'
	@echo ' '

Sources/ff10b/src/option/%.d: ../Sources/ff10b/src/option/%.c
	@echo 'Regenerating dependency file: $@'
	
	@echo ' '


