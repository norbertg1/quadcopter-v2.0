################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.local

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS_QUOTED += \
"../Sources/BMP180.c" \
"../Sources/Interrupts.c" \
"../Sources/MPU6050.c" \
"../Sources/Motor.c" \
"../Sources/SDcard.c" \
"../Sources/init.c" \
"../Sources/main.c" \
"../Sources/sa_mtb.c" \

C_SRCS += \
../Sources/BMP180.c \
../Sources/Interrupts.c \
../Sources/MPU6050.c \
../Sources/Motor.c \
../Sources/SDcard.c \
../Sources/init.c \
../Sources/main.c \
../Sources/sa_mtb.c \

OBJS += \
./Sources/BMP180_c.obj \
./Sources/Interrupts_c.obj \
./Sources/MPU6050_c.obj \
./Sources/Motor_c.obj \
./Sources/SDcard_c.obj \
./Sources/init_c.obj \
./Sources/main_c.obj \
./Sources/sa_mtb_c.obj \

OBJS_QUOTED += \
"./Sources/BMP180_c.obj" \
"./Sources/Interrupts_c.obj" \
"./Sources/MPU6050_c.obj" \
"./Sources/Motor_c.obj" \
"./Sources/SDcard_c.obj" \
"./Sources/init_c.obj" \
"./Sources/main_c.obj" \
"./Sources/sa_mtb_c.obj" \

C_DEPS += \
./Sources/BMP180_c.d \
./Sources/Interrupts_c.d \
./Sources/MPU6050_c.d \
./Sources/Motor_c.d \
./Sources/SDcard_c.d \
./Sources/init_c.d \
./Sources/main_c.d \
./Sources/sa_mtb_c.d \

C_DEPS_QUOTED += \
"./Sources/BMP180_c.d" \
"./Sources/Interrupts_c.d" \
"./Sources/MPU6050_c.d" \
"./Sources/Motor_c.d" \
"./Sources/SDcard_c.d" \
"./Sources/init_c.d" \
"./Sources/main_c.d" \
"./Sources/sa_mtb_c.d" \

OBJS_OS_FORMAT += \
./Sources/BMP180_c.obj \
./Sources/Interrupts_c.obj \
./Sources/MPU6050_c.obj \
./Sources/Motor_c.obj \
./Sources/SDcard_c.obj \
./Sources/init_c.obj \
./Sources/main_c.obj \
./Sources/sa_mtb_c.obj \


# Each subdirectory must supply rules for building sources it contributes
Sources/BMP180_c.obj: ../Sources/BMP180.c
	@echo 'Building file: $<'
	@echo 'Executing target #1 $<'
	@echo 'Invoking: ARM Compiler'
	"$(ARM_ToolsDirEnv)/mwccarm" -gccinc @@"Sources/BMP180.args" -o "Sources/BMP180_c.obj" -c "$<" -MD -gccdep
	@echo 'Finished building: $<'
	@echo ' '

Sources/%.d: ../Sources/%.c
	@echo 'Regenerating dependency file: $@'
	
	@echo ' '

Sources/Interrupts_c.obj: ../Sources/Interrupts.c
	@echo 'Building file: $<'
	@echo 'Executing target #2 $<'
	@echo 'Invoking: ARM Compiler'
	"$(ARM_ToolsDirEnv)/mwccarm" -gccinc @@"Sources/Interrupts.args" -o "Sources/Interrupts_c.obj" -c "$<" -MD -gccdep
	@echo 'Finished building: $<'
	@echo ' '

Sources/MPU6050_c.obj: ../Sources/MPU6050.c
	@echo 'Building file: $<'
	@echo 'Executing target #3 $<'
	@echo 'Invoking: ARM Compiler'
	"$(ARM_ToolsDirEnv)/mwccarm" -gccinc @@"Sources/MPU6050.args" -o "Sources/MPU6050_c.obj" -c "$<" -MD -gccdep
	@echo 'Finished building: $<'
	@echo ' '

Sources/Motor_c.obj: ../Sources/Motor.c
	@echo 'Building file: $<'
	@echo 'Executing target #4 $<'
	@echo 'Invoking: ARM Compiler'
	"$(ARM_ToolsDirEnv)/mwccarm" -gccinc @@"Sources/Motor.args" -o "Sources/Motor_c.obj" -c "$<" -MD -gccdep
	@echo 'Finished building: $<'
	@echo ' '

Sources/SDcard_c.obj: ../Sources/SDcard.c
	@echo 'Building file: $<'
	@echo 'Executing target #5 $<'
	@echo 'Invoking: ARM Compiler'
	"$(ARM_ToolsDirEnv)/mwccarm" -gccinc @@"Sources/SDcard.args" -o "Sources/SDcard_c.obj" -c "$<" -MD -gccdep
	@echo 'Finished building: $<'
	@echo ' '

Sources/init_c.obj: ../Sources/init.c
	@echo 'Building file: $<'
	@echo 'Executing target #6 $<'
	@echo 'Invoking: ARM Compiler'
	"$(ARM_ToolsDirEnv)/mwccarm" -gccinc @@"Sources/init.args" -o "Sources/init_c.obj" -c "$<" -MD -gccdep
	@echo 'Finished building: $<'
	@echo ' '

Sources/main_c.obj: ../Sources/main.c
	@echo 'Building file: $<'
	@echo 'Executing target #7 $<'
	@echo 'Invoking: ARM Compiler'
	"$(ARM_ToolsDirEnv)/mwccarm" -gccinc @@"Sources/main.args" -o "Sources/main_c.obj" -c "$<" -MD -gccdep
	@echo 'Finished building: $<'
	@echo ' '

Sources/sa_mtb_c.obj: ../Sources/sa_mtb.c
	@echo 'Building file: $<'
	@echo 'Executing target #8 $<'
	@echo 'Invoking: ARM Compiler'
	"$(ARM_ToolsDirEnv)/mwccarm" -gccinc @@"Sources/sa_mtb.args" -o "Sources/sa_mtb_c.obj" -c "$<" -MD -gccdep
	@echo 'Finished building: $<'
	@echo ' '


