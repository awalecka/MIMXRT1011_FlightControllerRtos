################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../freertos/freertos-kernel/croutine.c \
../freertos/freertos-kernel/event_groups.c \
../freertos/freertos-kernel/list.c \
../freertos/freertos-kernel/queue.c \
../freertos/freertos-kernel/stream_buffer.c \
../freertos/freertos-kernel/tasks.c \
../freertos/freertos-kernel/timers.c 

C_DEPS += \
./freertos/freertos-kernel/croutine.d \
./freertos/freertos-kernel/event_groups.d \
./freertos/freertos-kernel/list.d \
./freertos/freertos-kernel/queue.d \
./freertos/freertos-kernel/stream_buffer.d \
./freertos/freertos-kernel/tasks.d \
./freertos/freertos-kernel/timers.d 

OBJS += \
./freertos/freertos-kernel/croutine.o \
./freertos/freertos-kernel/event_groups.o \
./freertos/freertos-kernel/list.o \
./freertos/freertos-kernel/queue.o \
./freertos/freertos-kernel/stream_buffer.o \
./freertos/freertos-kernel/tasks.o \
./freertos/freertos-kernel/timers.o 


# Each subdirectory must supply rules for building sources it contributes
freertos/freertos-kernel/%.o: ../freertos/freertos-kernel/%.c freertos/freertos-kernel/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: MCU C Compiler'
	arm-none-eabi-gcc -std=gnu17 -D__NEWLIB__ -DEIGEN_NO_MALLOC -DSDK_DEBUGCONSOLE=0 -DXIP_EXTERNAL_FLASH=1 -DXIP_BOOT_HEADER_ENABLE=1 -DCPU_MIMXRT1011DAE5A -DCPU_MIMXRT1011DAE5A_cm7 -D__MCUXPRESSO -D__USE_CMSIS -DDEBUG -DSERIAL_PORT_TYPE_SWO=1 -DDISABLEFLOAT16 -DSDK_OS_FREE_RTOS -I"C:\nxp\workspace\MIMXRT1011_FlightControllerRtos\drivers" -I"C:\nxp\workspace\MIMXRT1011_FlightControllerRtos\source\config" -I"C:\nxp\workspace\MIMXRT1011_FlightControllerRtos\source\control" -I"C:\nxp\workspace\MIMXRT1011_FlightControllerRtos\source\drivers" -I"C:\nxp\workspace\MIMXRT1011_FlightControllerRtos\source\radio" -I"C:\nxp\workspace\MIMXRT1011_FlightControllerRtos\source\sensors" -I"C:\nxp\workspace\MIMXRT1011_FlightControllerRtos\source\system" -I"C:\nxp\workspace\MIMXRT1011_FlightControllerRtos\source\utils" -I"C:\nxp\workspace\MIMXRT1011_FlightControllerRtos\utilities" -I"C:\nxp\workspace\MIMXRT1011_FlightControllerRtos\component\serial_manager" -I"C:\nxp\workspace\MIMXRT1011_FlightControllerRtos\device" -I"C:\nxp\workspace\MIMXRT1011_FlightControllerRtos\CMSIS" -I"C:\nxp\workspace\MIMXRT1011_FlightControllerRtos\component\lists" -I"C:\nxp\workspace\MIMXRT1011_FlightControllerRtos\CMSIS_driver\Include" -I"C:\nxp\workspace\MIMXRT1011_FlightControllerRtos\component\uart" -I"C:\nxp\workspace\MIMXRT1011_FlightControllerRtos\xip" -I"C:\nxp\workspace\MIMXRT1011_FlightControllerRtos\CMSIS\DSP\Include" -I"C:\nxp\workspace\MIMXRT1011_FlightControllerRtos\CMSIS\DSP\PrivateInclude" -I"C:\nxp\workspace\MIMXRT1011_FlightControllerRtos\CMSIS\DSP\Source\DistanceFunctions" -I"C:\nxp\workspace\MIMXRT1011_FlightControllerRtos\freertos\freertos-kernel\include" -I"C:\nxp\workspace\MIMXRT1011_FlightControllerRtos\freertos\freertos-kernel\portable\GCC\ARM_CM4F" -I"C:\nxp\workspace\MIMXRT1011_FlightControllerRtos\board" -I"C:\nxp\workspace\MIMXRT1011_FlightControllerRtos\source" -I"C:\nxp\workspace\MIMXRT1011_FlightControllerRtos\fusion" -I"C:\nxp\workspace\MIMXRT1011_FlightControllerRtos\Eigen" -O0 -fno-common -g3 -gdwarf-4 -Wall -c -ffunction-sections -fdata-sections -fno-builtin -fmerge-constants -fmacro-prefix-map="$(<D)/"= -mcpu=cortex-m7 -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -D__NEWLIB__ -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


clean: clean-freertos-2f-freertos-2d-kernel

clean-freertos-2f-freertos-2d-kernel:
	-$(RM) ./freertos/freertos-kernel/croutine.d ./freertos/freertos-kernel/croutine.o ./freertos/freertos-kernel/event_groups.d ./freertos/freertos-kernel/event_groups.o ./freertos/freertos-kernel/list.d ./freertos/freertos-kernel/list.o ./freertos/freertos-kernel/queue.d ./freertos/freertos-kernel/queue.o ./freertos/freertos-kernel/stream_buffer.d ./freertos/freertos-kernel/stream_buffer.o ./freertos/freertos-kernel/tasks.d ./freertos/freertos-kernel/tasks.o ./freertos/freertos-kernel/timers.d ./freertos/freertos-kernel/timers.o

.PHONY: clean-freertos-2f-freertos-2d-kernel

