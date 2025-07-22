################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../drivers/fsl_clock.c \
../drivers/fsl_common.c \
../drivers/fsl_common_arm.c \
../drivers/fsl_dmamux.c \
../drivers/fsl_edma.c \
../drivers/fsl_flexio.c \
../drivers/fsl_gpio.c \
../drivers/fsl_lpi2c.c \
../drivers/fsl_lpi2c_cmsis.c \
../drivers/fsl_lpi2c_edma.c \
../drivers/fsl_lpuart.c \
../drivers/fsl_lpuart_cmsis.c \
../drivers/fsl_lpuart_edma.c 

C_DEPS += \
./drivers/fsl_clock.d \
./drivers/fsl_common.d \
./drivers/fsl_common_arm.d \
./drivers/fsl_dmamux.d \
./drivers/fsl_edma.d \
./drivers/fsl_flexio.d \
./drivers/fsl_gpio.d \
./drivers/fsl_lpi2c.d \
./drivers/fsl_lpi2c_cmsis.d \
./drivers/fsl_lpi2c_edma.d \
./drivers/fsl_lpuart.d \
./drivers/fsl_lpuart_cmsis.d \
./drivers/fsl_lpuart_edma.d 

OBJS += \
./drivers/fsl_clock.o \
./drivers/fsl_common.o \
./drivers/fsl_common_arm.o \
./drivers/fsl_dmamux.o \
./drivers/fsl_edma.o \
./drivers/fsl_flexio.o \
./drivers/fsl_gpio.o \
./drivers/fsl_lpi2c.o \
./drivers/fsl_lpi2c_cmsis.o \
./drivers/fsl_lpi2c_edma.o \
./drivers/fsl_lpuart.o \
./drivers/fsl_lpuart_cmsis.o \
./drivers/fsl_lpuart_edma.o 


# Each subdirectory must supply rules for building sources it contributes
drivers/%.o: ../drivers/%.c drivers/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: MCU C Compiler'
	arm-none-eabi-gcc -D__NEWLIB__ -D SDK_DEBUGCONSOLE=0 -DXIP_EXTERNAL_FLASH=1 -DXIP_BOOT_HEADER_ENABLE=1 -DCPU_MIMXRT1011DAE5A -DCPU_MIMXRT1011DAE5A_cm7 -D__MCUXPRESSO -D__USE_CMSIS -DDEBUG -DSERIAL_PORT_TYPE_SWO=1 -DSDK_OS_FREE_RTOS -I"C:\nxp\workspace\MIMXRT1011_FlightControllerRtos\drivers" -I"C:\nxp\workspace\MIMXRT1011_FlightControllerRtos\utilities" -I"C:\nxp\workspace\MIMXRT1011_FlightControllerRtos\component\serial_manager" -I"C:\nxp\workspace\MIMXRT1011_FlightControllerRtos\device" -I"C:\nxp\workspace\MIMXRT1011_FlightControllerRtos\CMSIS" -I"C:\nxp\workspace\MIMXRT1011_FlightControllerRtos\component\lists" -I"C:\nxp\workspace\MIMXRT1011_FlightControllerRtos\CMSIS_driver\Include" -I"C:\nxp\workspace\MIMXRT1011_FlightControllerRtos\component\uart" -I"C:\nxp\workspace\MIMXRT1011_FlightControllerRtos\xip" -I"C:\nxp\workspace\MIMXRT1011_FlightControllerRtos\freertos\freertos-kernel\include" -I"C:\nxp\workspace\MIMXRT1011_FlightControllerRtos\freertos\freertos-kernel\portable\GCC\ARM_CM4F" -I"C:\nxp\workspace\MIMXRT1011_FlightControllerRtos\board" -I"C:\nxp\workspace\MIMXRT1011_FlightControllerRtos\source" -I"C:\nxp\workspace\MIMXRT1011_FlightControllerRtos\fusion" -I"C:\nxp\workspace\MIMXRT1011_FlightControllerRtos\Eigen" -O3 -fno-common -g3 -gdwarf-4 -Wall -c -ffunction-sections -fdata-sections -fno-builtin -fmacro-prefix-map="$(<D)/"= -mcpu=cortex-m7 -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -D__NEWLIB__ -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


clean: clean-drivers

clean-drivers:
	-$(RM) ./drivers/fsl_clock.d ./drivers/fsl_clock.o ./drivers/fsl_common.d ./drivers/fsl_common.o ./drivers/fsl_common_arm.d ./drivers/fsl_common_arm.o ./drivers/fsl_dmamux.d ./drivers/fsl_dmamux.o ./drivers/fsl_edma.d ./drivers/fsl_edma.o ./drivers/fsl_flexio.d ./drivers/fsl_flexio.o ./drivers/fsl_gpio.d ./drivers/fsl_gpio.o ./drivers/fsl_lpi2c.d ./drivers/fsl_lpi2c.o ./drivers/fsl_lpi2c_cmsis.d ./drivers/fsl_lpi2c_cmsis.o ./drivers/fsl_lpi2c_edma.d ./drivers/fsl_lpi2c_edma.o ./drivers/fsl_lpuart.d ./drivers/fsl_lpuart.o ./drivers/fsl_lpuart_cmsis.d ./drivers/fsl_lpuart_cmsis.o ./drivers/fsl_lpuart_edma.d ./drivers/fsl_lpuart_edma.o

.PHONY: clean-drivers

