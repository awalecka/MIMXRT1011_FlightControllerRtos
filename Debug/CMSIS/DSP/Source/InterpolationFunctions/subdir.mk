################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../CMSIS/DSP/Source/InterpolationFunctions/InterpolationFunctions.c \
../CMSIS/DSP/Source/InterpolationFunctions/InterpolationFunctionsF16.c 

C_DEPS += \
./CMSIS/DSP/Source/InterpolationFunctions/InterpolationFunctions.d \
./CMSIS/DSP/Source/InterpolationFunctions/InterpolationFunctionsF16.d 

OBJS += \
./CMSIS/DSP/Source/InterpolationFunctions/InterpolationFunctions.o \
./CMSIS/DSP/Source/InterpolationFunctions/InterpolationFunctionsF16.o 


# Each subdirectory must supply rules for building sources it contributes
CMSIS/DSP/Source/InterpolationFunctions/%.o: ../CMSIS/DSP/Source/InterpolationFunctions/%.c CMSIS/DSP/Source/InterpolationFunctions/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: MCU C Compiler'
	arm-none-eabi-gcc -std=gnu17 -D__NEWLIB__ -DEIGEN_NO_MALLOC -DSDK_DEBUGCONSOLE=0 -DXIP_EXTERNAL_FLASH=1 -DXIP_BOOT_HEADER_ENABLE=1 -DCPU_MIMXRT1011DAE5A -DCPU_MIMXRT1011DAE5A_cm7 -D__MCUXPRESSO -D__USE_CMSIS -DDEBUG -DSERIAL_PORT_TYPE_SWO=1 -DDISABLEFLOAT16 -DSDK_OS_FREE_RTOS -I"C:\nxp\workspace\MIMXRT1011_FlightControllerRtos\drivers" -I"C:\nxp\workspace\MIMXRT1011_FlightControllerRtos\source\config" -I"C:\nxp\workspace\MIMXRT1011_FlightControllerRtos\source\control" -I"C:\nxp\workspace\MIMXRT1011_FlightControllerRtos\source\drivers" -I"C:\nxp\workspace\MIMXRT1011_FlightControllerRtos\source\radio" -I"C:\nxp\workspace\MIMXRT1011_FlightControllerRtos\source\sensors" -I"C:\nxp\workspace\MIMXRT1011_FlightControllerRtos\source\system" -I"C:\nxp\workspace\MIMXRT1011_FlightControllerRtos\source\utils" -I"C:\nxp\workspace\MIMXRT1011_FlightControllerRtos\utilities" -I"C:\nxp\workspace\MIMXRT1011_FlightControllerRtos\component\serial_manager" -I"C:\nxp\workspace\MIMXRT1011_FlightControllerRtos\device" -I"C:\nxp\workspace\MIMXRT1011_FlightControllerRtos\CMSIS" -I"C:\nxp\workspace\MIMXRT1011_FlightControllerRtos\component\lists" -I"C:\nxp\workspace\MIMXRT1011_FlightControllerRtos\CMSIS_driver\Include" -I"C:\nxp\workspace\MIMXRT1011_FlightControllerRtos\component\uart" -I"C:\nxp\workspace\MIMXRT1011_FlightControllerRtos\xip" -I"C:\nxp\workspace\MIMXRT1011_FlightControllerRtos\CMSIS\DSP\Include" -I"C:\nxp\workspace\MIMXRT1011_FlightControllerRtos\CMSIS\DSP\PrivateInclude" -I"C:\nxp\workspace\MIMXRT1011_FlightControllerRtos\CMSIS\DSP\Source\DistanceFunctions" -I"C:\nxp\workspace\MIMXRT1011_FlightControllerRtos\freertos\freertos-kernel\include" -I"C:\nxp\workspace\MIMXRT1011_FlightControllerRtos\freertos\freertos-kernel\portable\GCC\ARM_CM4F" -I"C:\nxp\workspace\MIMXRT1011_FlightControllerRtos\board" -I"C:\nxp\workspace\MIMXRT1011_FlightControllerRtos\source" -I"C:\nxp\workspace\MIMXRT1011_FlightControllerRtos\fusion" -I"C:\nxp\workspace\MIMXRT1011_FlightControllerRtos\Eigen" -O0 -fno-common -g3 -gdwarf-4 -Wall -c -ffunction-sections -fdata-sections -fno-builtin -fmerge-constants -fmacro-prefix-map="$(<D)/"= -mcpu=cortex-m7 -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -D__NEWLIB__ -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


clean: clean-CMSIS-2f-DSP-2f-Source-2f-InterpolationFunctions

clean-CMSIS-2f-DSP-2f-Source-2f-InterpolationFunctions:
	-$(RM) ./CMSIS/DSP/Source/InterpolationFunctions/InterpolationFunctions.d ./CMSIS/DSP/Source/InterpolationFunctions/InterpolationFunctions.o ./CMSIS/DSP/Source/InterpolationFunctions/InterpolationFunctionsF16.d ./CMSIS/DSP/Source/InterpolationFunctions/InterpolationFunctionsF16.o

.PHONY: clean-CMSIS-2f-DSP-2f-Source-2f-InterpolationFunctions

