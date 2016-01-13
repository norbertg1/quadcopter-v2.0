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
./Sources/BMP180.o \
./Sources/Interrupts.o \
./Sources/MPU6050.o \
./Sources/Motor.o \
./Sources/SDcard.o \
./Sources/init.o \
./Sources/main.o \
./Sources/sa_mtb.o \

C_DEPS += \
./Sources/BMP180.d \
./Sources/Interrupts.d \
./Sources/MPU6050.d \
./Sources/Motor.d \
./Sources/SDcard.d \
./Sources/init.d \
./Sources/main.d \
./Sources/sa_mtb.d \

OBJS_QUOTED += \
"./Sources/BMP180.o" \
"./Sources/Interrupts.o" \
"./Sources/MPU6050.o" \
"./Sources/Motor.o" \
"./Sources/SDcard.o" \
"./Sources/init.o" \
"./Sources/main.o" \
"./Sources/sa_mtb.o" \

C_DEPS_QUOTED += \
"./Sources/BMP180.d" \
"./Sources/Interrupts.d" \
"./Sources/MPU6050.d" \
"./Sources/Motor.d" \
"./Sources/SDcard.d" \
"./Sources/init.d" \
"./Sources/main.d" \
"./Sources/sa_mtb.d" \

OBJS_OS_FORMAT += \
./Sources/BMP180.o \
./Sources/Interrupts.o \
./Sources/MPU6050.o \
./Sources/Motor.o \
./Sources/SDcard.o \
./Sources/init.o \
./Sources/main.o \
./Sources/sa_mtb.o \


# Each subdirectory must supply rules for building sources it contributes
Sources/BMP180.o: ../Sources/BMP180.c
	@echo 'Building file: $<'
	@echo 'Executing target #1 $<'
	@echo 'Invoking: ARM Ltd Windows GCC C Compiler'
	"$(ARMSourceryDirEnv)/arm-none-eabi-gcc" "$<" @"Sources/BMP180.args" -MMD -MP -MF"$(@:%.o=%.d)" -o"Sources/BMP180.o"
	@echo 'Finished building: $<'
	@echo ' '

Sources/Interrupts.o: ../Sources/Interrupts.c
	@echo 'Building file: $<'
	@echo 'Executing target #2 $<'
	@echo 'Invoking: ARM Ltd Windows GCC C Compiler'
	"$(ARMSourceryDirEnv)/arm-none-eabi-gcc" "$<" @"Sources/Interrupts.args" -MMD -MP -MF"$(@:%.o=%.d)" -o"Sources/Interrupts.o"
	@echo 'Finished building: $<'
	@echo ' '

Sources/MPU6050.o: ../Sources/MPU6050.c
	@echo 'Building file: $<'
	@echo 'Executing target #3 $<'
	@echo 'Invoking: ARM Ltd Windows GCC C Compiler'
	"$(ARMSourceryDirEnv)/arm-none-eabi-gcc" "$<" @"Sources/MPU6050.args" -MMD -MP -MF"$(@:%.o=%.d)" -o"Sources/MPU6050.o"
	@echo 'Finished building: $<'
	@echo ' '

Sources/Motor.o: ../Sources/Motor.c
	@echo 'Building file: $<'
	@echo 'Executing target #4 $<'
	@echo 'Invoking: ARM Ltd Windows GCC C Compiler'
	"$(ARMSourceryDirEnv)/arm-none-eabi-gcc" "$<" @"Sources/Motor.args" -MMD -MP -MF"$(@:%.o=%.d)" -o"Sources/Motor.o"
	@echo 'Finished building: $<'
	@echo ' '

Sources/SDcard.o: ../Sources/SDcard.c
	@echo 'Building file: $<'
	@echo 'Executing target #5 $<'
	@echo 'Invoking: ARM Ltd Windows GCC C Compiler'
	"$(ARMSourceryDirEnv)/arm-none-eabi-gcc" "$<" @"Sources/SDcard.args" -MMD -MP -MF"$(@:%.o=%.d)" -o"Sources/SDcard.o"
	@echo 'Finished building: $<'
	@echo ' '

Sources/init.o: ../Sources/init.c
	@echo 'Building file: $<'
	@echo 'Executing target #6 $<'
	@echo 'Invoking: ARM Ltd Windows GCC C Compiler'
	"$(ARMSourceryDirEnv)/arm-none-eabi-gcc" "$<" @"Sources/init.args" -MMD -MP -MF"$(@:%.o=%.d)" -o"Sources/init.o"
	@echo 'Finished building: $<'
	@echo ' '

Sources/main.o: ../Sources/main.c
	@echo 'Building file: $<'
	@echo 'Executing target #7 $<'
	@echo 'Invoking: ARM Ltd Windows GCC C Compiler'
	"$(ARMSourceryDirEnv)/arm-none-eabi-gcc" "$<" @"Sources/main.args" -MMD -MP -MF"$(@:%.o=%.d)" -o"Sources/main.o"
	@echo 'Finished building: $<'
	@echo ' '

Sources/sa_mtb.o: ../Sources/sa_mtb.c
	@echo 'Building file: $<'
	@echo 'Executing target #8 $<'
	@echo 'Invoking: ARM Ltd Windows GCC C Compiler'
	"$(ARMSourceryDirEnv)/arm-none-eabi-gcc" "$<" @"Sources/sa_mtb.args" -MMD -MP -MF"$(@:%.o=%.d)" -o"Sources/sa_mtb.o"
	@echo 'Finished building: $<'
	@echo ' '


