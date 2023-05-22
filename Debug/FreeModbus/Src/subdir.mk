################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../FreeModbus/Src/mb.c \
../FreeModbus/Src/mb_m.c \
../FreeModbus/Src/mbascii.c \
../FreeModbus/Src/mbcrc.c \
../FreeModbus/Src/mbfunccoils.c \
../FreeModbus/Src/mbfunccoils_m.c \
../FreeModbus/Src/mbfuncdiag.c \
../FreeModbus/Src/mbfuncdisc.c \
../FreeModbus/Src/mbfuncdisc_m.c \
../FreeModbus/Src/mbfuncholding.c \
../FreeModbus/Src/mbfuncholding_m.c \
../FreeModbus/Src/mbfuncinput.c \
../FreeModbus/Src/mbfuncinput_m.c \
../FreeModbus/Src/mbfuncother.c \
../FreeModbus/Src/mbrtu.c \
../FreeModbus/Src/mbrtu_m.c \
../FreeModbus/Src/mbtcp.c \
../FreeModbus/Src/mbutils.c \
../FreeModbus/Src/port.c \
../FreeModbus/Src/portevent.c \
../FreeModbus/Src/portevent_m.c \
../FreeModbus/Src/portserial.c \
../FreeModbus/Src/portserial_m.c \
../FreeModbus/Src/porttimer.c \
../FreeModbus/Src/porttimer_m.c \
../FreeModbus/Src/user_mb_app.c \
../FreeModbus/Src/user_mb_app_m.c 

OBJS += \
./FreeModbus/Src/mb.o \
./FreeModbus/Src/mb_m.o \
./FreeModbus/Src/mbascii.o \
./FreeModbus/Src/mbcrc.o \
./FreeModbus/Src/mbfunccoils.o \
./FreeModbus/Src/mbfunccoils_m.o \
./FreeModbus/Src/mbfuncdiag.o \
./FreeModbus/Src/mbfuncdisc.o \
./FreeModbus/Src/mbfuncdisc_m.o \
./FreeModbus/Src/mbfuncholding.o \
./FreeModbus/Src/mbfuncholding_m.o \
./FreeModbus/Src/mbfuncinput.o \
./FreeModbus/Src/mbfuncinput_m.o \
./FreeModbus/Src/mbfuncother.o \
./FreeModbus/Src/mbrtu.o \
./FreeModbus/Src/mbrtu_m.o \
./FreeModbus/Src/mbtcp.o \
./FreeModbus/Src/mbutils.o \
./FreeModbus/Src/port.o \
./FreeModbus/Src/portevent.o \
./FreeModbus/Src/portevent_m.o \
./FreeModbus/Src/portserial.o \
./FreeModbus/Src/portserial_m.o \
./FreeModbus/Src/porttimer.o \
./FreeModbus/Src/porttimer_m.o \
./FreeModbus/Src/user_mb_app.o \
./FreeModbus/Src/user_mb_app_m.o 

C_DEPS += \
./FreeModbus/Src/mb.d \
./FreeModbus/Src/mb_m.d \
./FreeModbus/Src/mbascii.d \
./FreeModbus/Src/mbcrc.d \
./FreeModbus/Src/mbfunccoils.d \
./FreeModbus/Src/mbfunccoils_m.d \
./FreeModbus/Src/mbfuncdiag.d \
./FreeModbus/Src/mbfuncdisc.d \
./FreeModbus/Src/mbfuncdisc_m.d \
./FreeModbus/Src/mbfuncholding.d \
./FreeModbus/Src/mbfuncholding_m.d \
./FreeModbus/Src/mbfuncinput.d \
./FreeModbus/Src/mbfuncinput_m.d \
./FreeModbus/Src/mbfuncother.d \
./FreeModbus/Src/mbrtu.d \
./FreeModbus/Src/mbrtu_m.d \
./FreeModbus/Src/mbtcp.d \
./FreeModbus/Src/mbutils.d \
./FreeModbus/Src/port.d \
./FreeModbus/Src/portevent.d \
./FreeModbus/Src/portevent_m.d \
./FreeModbus/Src/portserial.d \
./FreeModbus/Src/portserial_m.d \
./FreeModbus/Src/porttimer.d \
./FreeModbus/Src/porttimer_m.d \
./FreeModbus/Src/user_mb_app.d \
./FreeModbus/Src/user_mb_app_m.d 


# Each subdirectory must supply rules for building sources it contributes
FreeModbus/Src/%.o FreeModbus/Src/%.su FreeModbus/Src/%.cyclo: ../FreeModbus/Src/%.c FreeModbus/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../FreeModbus/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3 -O1 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-FreeModbus-2f-Src

clean-FreeModbus-2f-Src:
	-$(RM) ./FreeModbus/Src/mb.cyclo ./FreeModbus/Src/mb.d ./FreeModbus/Src/mb.o ./FreeModbus/Src/mb.su ./FreeModbus/Src/mb_m.cyclo ./FreeModbus/Src/mb_m.d ./FreeModbus/Src/mb_m.o ./FreeModbus/Src/mb_m.su ./FreeModbus/Src/mbascii.cyclo ./FreeModbus/Src/mbascii.d ./FreeModbus/Src/mbascii.o ./FreeModbus/Src/mbascii.su ./FreeModbus/Src/mbcrc.cyclo ./FreeModbus/Src/mbcrc.d ./FreeModbus/Src/mbcrc.o ./FreeModbus/Src/mbcrc.su ./FreeModbus/Src/mbfunccoils.cyclo ./FreeModbus/Src/mbfunccoils.d ./FreeModbus/Src/mbfunccoils.o ./FreeModbus/Src/mbfunccoils.su ./FreeModbus/Src/mbfunccoils_m.cyclo ./FreeModbus/Src/mbfunccoils_m.d ./FreeModbus/Src/mbfunccoils_m.o ./FreeModbus/Src/mbfunccoils_m.su ./FreeModbus/Src/mbfuncdiag.cyclo ./FreeModbus/Src/mbfuncdiag.d ./FreeModbus/Src/mbfuncdiag.o ./FreeModbus/Src/mbfuncdiag.su ./FreeModbus/Src/mbfuncdisc.cyclo ./FreeModbus/Src/mbfuncdisc.d ./FreeModbus/Src/mbfuncdisc.o ./FreeModbus/Src/mbfuncdisc.su ./FreeModbus/Src/mbfuncdisc_m.cyclo ./FreeModbus/Src/mbfuncdisc_m.d ./FreeModbus/Src/mbfuncdisc_m.o ./FreeModbus/Src/mbfuncdisc_m.su ./FreeModbus/Src/mbfuncholding.cyclo ./FreeModbus/Src/mbfuncholding.d ./FreeModbus/Src/mbfuncholding.o ./FreeModbus/Src/mbfuncholding.su ./FreeModbus/Src/mbfuncholding_m.cyclo ./FreeModbus/Src/mbfuncholding_m.d ./FreeModbus/Src/mbfuncholding_m.o ./FreeModbus/Src/mbfuncholding_m.su ./FreeModbus/Src/mbfuncinput.cyclo ./FreeModbus/Src/mbfuncinput.d ./FreeModbus/Src/mbfuncinput.o ./FreeModbus/Src/mbfuncinput.su ./FreeModbus/Src/mbfuncinput_m.cyclo ./FreeModbus/Src/mbfuncinput_m.d ./FreeModbus/Src/mbfuncinput_m.o ./FreeModbus/Src/mbfuncinput_m.su ./FreeModbus/Src/mbfuncother.cyclo ./FreeModbus/Src/mbfuncother.d ./FreeModbus/Src/mbfuncother.o ./FreeModbus/Src/mbfuncother.su ./FreeModbus/Src/mbrtu.cyclo ./FreeModbus/Src/mbrtu.d ./FreeModbus/Src/mbrtu.o ./FreeModbus/Src/mbrtu.su ./FreeModbus/Src/mbrtu_m.cyclo ./FreeModbus/Src/mbrtu_m.d ./FreeModbus/Src/mbrtu_m.o ./FreeModbus/Src/mbrtu_m.su ./FreeModbus/Src/mbtcp.cyclo ./FreeModbus/Src/mbtcp.d ./FreeModbus/Src/mbtcp.o ./FreeModbus/Src/mbtcp.su ./FreeModbus/Src/mbutils.cyclo ./FreeModbus/Src/mbutils.d ./FreeModbus/Src/mbutils.o ./FreeModbus/Src/mbutils.su ./FreeModbus/Src/port.cyclo ./FreeModbus/Src/port.d ./FreeModbus/Src/port.o ./FreeModbus/Src/port.su ./FreeModbus/Src/portevent.cyclo ./FreeModbus/Src/portevent.d ./FreeModbus/Src/portevent.o ./FreeModbus/Src/portevent.su ./FreeModbus/Src/portevent_m.cyclo ./FreeModbus/Src/portevent_m.d ./FreeModbus/Src/portevent_m.o ./FreeModbus/Src/portevent_m.su ./FreeModbus/Src/portserial.cyclo ./FreeModbus/Src/portserial.d ./FreeModbus/Src/portserial.o ./FreeModbus/Src/portserial.su ./FreeModbus/Src/portserial_m.cyclo ./FreeModbus/Src/portserial_m.d ./FreeModbus/Src/portserial_m.o ./FreeModbus/Src/portserial_m.su ./FreeModbus/Src/porttimer.cyclo ./FreeModbus/Src/porttimer.d ./FreeModbus/Src/porttimer.o ./FreeModbus/Src/porttimer.su ./FreeModbus/Src/porttimer_m.cyclo ./FreeModbus/Src/porttimer_m.d ./FreeModbus/Src/porttimer_m.o ./FreeModbus/Src/porttimer_m.su ./FreeModbus/Src/user_mb_app.cyclo ./FreeModbus/Src/user_mb_app.d ./FreeModbus/Src/user_mb_app.o ./FreeModbus/Src/user_mb_app.su ./FreeModbus/Src/user_mb_app_m.cyclo ./FreeModbus/Src/user_mb_app_m.d ./FreeModbus/Src/user_mb_app_m.o ./FreeModbus/Src/user_mb_app_m.su

.PHONY: clean-FreeModbus-2f-Src

