################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../source/command_handler.cpp \
../source/flight_controller.cpp \
../source/heartbeat_task.cpp \
../source/ibus_handler.cpp \
../source/logging_task.cpp \
../source/main.cpp \
../source/state_manager.cpp \
../source/state_tasks.cpp \
../source/utils.cpp 

C_SRCS += \
../source/i2c_sync.c \
../source/lis3mdl.c \
../source/lsm6dsox.c \
../source/semihost_hardfault.c 

CPP_DEPS += \
./source/command_handler.d \
./source/flight_controller.d \
./source/heartbeat_task.d \
./source/ibus_handler.d \
./source/logging_task.d \
./source/main.d \
./source/state_manager.d \
./source/state_tasks.d \
./source/utils.d 

C_DEPS += \
./source/i2c_sync.d \
./source/lis3mdl.d \
./source/lsm6dsox.d \
./source/semihost_hardfault.d 

OBJS += \
./source/command_handler.o \
./source/flight_controller.o \
./source/heartbeat_task.o \
./source/i2c_sync.o \
./source/ibus_handler.o \
./source/lis3mdl.o \
./source/logging_task.o \
./source/lsm6dsox.o \
./source/main.o \
./source/semihost_hardfault.o \
./source/state_manager.o \
./source/state_tasks.o \
./source/utils.o 


# Each subdirectory must supply rules for building sources it contributes
source/%.o: ../source/%.cpp source/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: MCU C++ Compiler'
	arm-none-eabi-c++ -std=gnu++23 -DCPU_MIMXRT1011DAE5A -DEIGEN_NO_MALLOC -DSDK_DEBUGCONSOLE=0 -DXIP_BOOT_HEADER_ENABLE=1 -DXIP_EXTERNAL_FLASH=1 -DCPU_MIMXRT1011DAE5A_cm7 -D__MCUXPRESSO -D__USE_CMSIS -DDEBUG -D__NEWLIB__ -DSERIAL_PORT_TYPE_SWO=1 -DDISABLEFLOAT16 -DSDK_OS_FREE_RTOS -I"C:\nxp\workspace\MIMXRT1011_FlightControllerRtos\drivers" -I"C:\nxp\workspace\MIMXRT1011_FlightControllerRtos\utilities" -I"C:\nxp\workspace\MIMXRT1011_FlightControllerRtos\component\serial_manager" -I"C:\nxp\workspace\MIMXRT1011_FlightControllerRtos\device" -I"C:\nxp\workspace\MIMXRT1011_FlightControllerRtos\CMSIS" -I"C:\nxp\workspace\MIMXRT1011_FlightControllerRtos\component\lists" -I"C:\nxp\workspace\MIMXRT1011_FlightControllerRtos\CMSIS_driver\Include" -I"C:\nxp\workspace\MIMXRT1011_FlightControllerRtos\component\uart" -I"C:\nxp\workspace\MIMXRT1011_FlightControllerRtos\xip" -I"C:\nxp\workspace\MIMXRT1011_FlightControllerRtos\CMSIS\DSP\Include" -I"C:\nxp\workspace\MIMXRT1011_FlightControllerRtos\CMSIS\DSP\PrivateInclude" -I"C:\nxp\workspace\MIMXRT1011_FlightControllerRtos\CMSIS\DSP\Source\DistanceFunctions" -I"C:\nxp\workspace\MIMXRT1011_FlightControllerRtos\freertos\freertos-kernel\include" -I"C:\nxp\workspace\MIMXRT1011_FlightControllerRtos\freertos\freertos-kernel\portable\GCC\ARM_CM4F" -I"C:\nxp\workspace\MIMXRT1011_FlightControllerRtos\board" -I"C:\nxp\workspace\MIMXRT1011_FlightControllerRtos\source" -I"C:\nxp\workspace\MIMXRT1011_FlightControllerRtos\fusion" -I"C:\nxp\workspace\MIMXRT1011_FlightControllerRtos\Eigen" -O3 -fno-common -g3 -gdwarf-4 -Wall -c -ffunction-sections -fdata-sections -fno-builtin -fno-rtti -fno-exceptions -fmacro-prefix-map="$(<D)/"= -mcpu=cortex-m7 -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -D__NEWLIB__ -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

source/%.o: ../source/%.c source/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: MCU C Compiler'
	arm-none-eabi-gcc -std=gnu17 -D__NEWLIB__ -DEIGEN_NO_MALLOC -DSDK_DEBUGCONSOLE=0 -DXIP_EXTERNAL_FLASH=1 -DXIP_BOOT_HEADER_ENABLE=1 -DCPU_MIMXRT1011DAE5A -DCPU_MIMXRT1011DAE5A_cm7 -D__MCUXPRESSO -D__USE_CMSIS -DDEBUG -DSERIAL_PORT_TYPE_SWO=1 -DDISABLEFLOAT16 -DSDK_OS_FREE_RTOS -I"C:\nxp\workspace\MIMXRT1011_FlightControllerRtos\drivers" -I"C:\nxp\workspace\MIMXRT1011_FlightControllerRtos\utilities" -I"C:\nxp\workspace\MIMXRT1011_FlightControllerRtos\component\serial_manager" -I"C:\nxp\workspace\MIMXRT1011_FlightControllerRtos\device" -I"C:\nxp\workspace\MIMXRT1011_FlightControllerRtos\CMSIS" -I"C:\nxp\workspace\MIMXRT1011_FlightControllerRtos\component\lists" -I"C:\nxp\workspace\MIMXRT1011_FlightControllerRtos\CMSIS_driver\Include" -I"C:\nxp\workspace\MIMXRT1011_FlightControllerRtos\component\uart" -I"C:\nxp\workspace\MIMXRT1011_FlightControllerRtos\xip" -I"C:\nxp\workspace\MIMXRT1011_FlightControllerRtos\CMSIS\DSP\Include" -I"C:\nxp\workspace\MIMXRT1011_FlightControllerRtos\CMSIS\DSP\PrivateInclude" -I"C:\nxp\workspace\MIMXRT1011_FlightControllerRtos\CMSIS\DSP\Source\DistanceFunctions" -I"C:\nxp\workspace\MIMXRT1011_FlightControllerRtos\freertos\freertos-kernel\include" -I"C:\nxp\workspace\MIMXRT1011_FlightControllerRtos\freertos\freertos-kernel\portable\GCC\ARM_CM4F" -I"C:\nxp\workspace\MIMXRT1011_FlightControllerRtos\board" -I"C:\nxp\workspace\MIMXRT1011_FlightControllerRtos\source" -I"C:\nxp\workspace\MIMXRT1011_FlightControllerRtos\fusion" -I"C:\nxp\workspace\MIMXRT1011_FlightControllerRtos\Eigen" -O3 -fno-common -g3 -gdwarf-4 -Wall -c -ffunction-sections -fdata-sections -fno-builtin -fmacro-prefix-map="$(<D)/"= -mcpu=cortex-m7 -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -D__NEWLIB__ -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


clean: clean-source

clean-source:
	-$(RM) ./source/command_handler.d ./source/command_handler.o ./source/flight_controller.d ./source/flight_controller.o ./source/heartbeat_task.d ./source/heartbeat_task.o ./source/i2c_sync.d ./source/i2c_sync.o ./source/ibus_handler.d ./source/ibus_handler.o ./source/lis3mdl.d ./source/lis3mdl.o ./source/logging_task.d ./source/logging_task.o ./source/lsm6dsox.d ./source/lsm6dsox.o ./source/main.d ./source/main.o ./source/semihost_hardfault.d ./source/semihost_hardfault.o ./source/state_manager.d ./source/state_manager.o ./source/state_tasks.d ./source/state_tasks.o ./source/utils.d ./source/utils.o

.PHONY: clean-source

