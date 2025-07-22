################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../startup/startup_mimxrt1011.cpp 

CPP_DEPS += \
./startup/startup_mimxrt1011.d 

OBJS += \
./startup/startup_mimxrt1011.o 


# Each subdirectory must supply rules for building sources it contributes
startup/%.o: ../startup/%.cpp startup/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: MCU C++ Compiler'
	arm-none-eabi-c++ -DCPU_MIMXRT1011DAE5A -DSDK_DEBUGCONSOLE=0 -DXIP_BOOT_HEADER_ENABLE=1 -DXIP_EXTERNAL_FLASH=1 -DCPU_MIMXRT1011DAE5A_cm7 -D__MCUXPRESSO -D__USE_CMSIS -DDEBUG -D__NEWLIB__ -DSERIAL_PORT_TYPE_SWO=1 -DSDK_OS_FREE_RTOS -I"C:\nxp\workspace\MIMXRT1011_FlightControllerRtos\drivers" -I"C:\nxp\workspace\MIMXRT1011_FlightControllerRtos\utilities" -I"C:\nxp\workspace\MIMXRT1011_FlightControllerRtos\component\serial_manager" -I"C:\nxp\workspace\MIMXRT1011_FlightControllerRtos\device" -I"C:\nxp\workspace\MIMXRT1011_FlightControllerRtos\CMSIS" -I"C:\nxp\workspace\MIMXRT1011_FlightControllerRtos\component\lists" -I"C:\nxp\workspace\MIMXRT1011_FlightControllerRtos\CMSIS_driver\Include" -I"C:\nxp\workspace\MIMXRT1011_FlightControllerRtos\component\uart" -I"C:\nxp\workspace\MIMXRT1011_FlightControllerRtos\xip" -I"C:\nxp\workspace\MIMXRT1011_FlightControllerRtos\freertos\freertos-kernel\include" -I"C:\nxp\workspace\MIMXRT1011_FlightControllerRtos\freertos\freertos-kernel\portable\GCC\ARM_CM4F" -I"C:\nxp\workspace\MIMXRT1011_FlightControllerRtos\board" -I"C:\nxp\workspace\MIMXRT1011_FlightControllerRtos\source" -I"C:\nxp\workspace\MIMXRT1011_FlightControllerRtos\fusion" -I"C:\nxp\workspace\MIMXRT1011_FlightControllerRtos\Eigen" -O3 -fno-common -g3 -gdwarf-4 -Wall -c -ffunction-sections -fdata-sections -fno-builtin -fno-rtti -fno-exceptions -fmacro-prefix-map="$(<D)/"= -mcpu=cortex-m7 -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -D__NEWLIB__ -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


clean: clean-startup

clean-startup:
	-$(RM) ./startup/startup_mimxrt1011.d ./startup/startup_mimxrt1011.o

.PHONY: clean-startup

