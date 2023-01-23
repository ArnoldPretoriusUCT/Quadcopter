################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
C:\Program\ Files\ (x86)\Arduino\hardware\teensy\avr\libraries\Wire\Wire.cpp \
C:\Program\ Files\ (x86)\Arduino\hardware\teensy\avr\libraries\Wire\WireIMXRT.cpp \
C:\Program\ Files\ (x86)\Arduino\hardware\teensy\avr\libraries\Wire\WireKinetis.cpp 

LINK_OBJ += \
.\libraries\Wire\Wire.cpp.o \
.\libraries\Wire\WireIMXRT.cpp.o \
.\libraries\Wire\WireKinetis.cpp.o 

CPP_DEPS += \
.\libraries\Wire\Wire.cpp.d \
.\libraries\Wire\WireIMXRT.cpp.d \
.\libraries\Wire\WireKinetis.cpp.d 


# Each subdirectory must supply rules for building sources it contributes
libraries\Wire\Wire.cpp.o: C:\Program\ Files\ (x86)\Arduino\hardware\teensy\avr\libraries\Wire\Wire.cpp
	@echo 'Building file: $<'
	@echo 'Starting C++ compile'
	"C:/Program Files (x86)/Arduino/hardware/teensy/../tools/arm/bin/arm-none-eabi-g++" -c -O3 -g -Wall -ffunction-sections -fdata-sections -nostdlib -MMD -fno-exceptions -fpermissive -felide-constructors -std=gnu++14 -Wno-error=narrowing -fno-rtti -mthumb -mcpu=cortex-m4 -fsingle-precision-constant -D__MK20DX256__ -DTEENSYDUINO=147 -DARDUINO=10802 -DF_CPU=96000000 -DUSB_SERIAL -DLAYOUT_UNITED_KINGDOM "-IC:/Academia/Sloeber/floki/Release/pch"   -I"C:\Program Files (x86)\Arduino\hardware\teensy\avr\cores\teensy3" -I"C:\Academia\Arduino\libraries\ATTITUDECONTROL" -I"C:\Academia\Arduino\libraries\Common" -I"C:\Academia\Arduino\libraries\ENCODER2" -I"C:\Academia\Arduino\libraries\HEAVECONTROL" -I"C:\Academia\Arduino\libraries\I2Cdev" -I"C:\Academia\Arduino\libraries\IMU" -I"C:\Academia\Arduino\libraries\LED" -I"C:\Program Files\Sloeber\arduinoPlugin\libraries\MatrixMath\1.0.0" -I"C:\Program Files (x86)\Arduino\hardware\teensy\avr\libraries\Metro" -I"C:\Academia\Arduino\libraries\MOTORSPEEDCONTROL" -I"C:\Academia\Arduino\libraries\MPU6050" -I"C:\Academia\Arduino\libraries\PWM" -I"C:\Academia\Arduino\libraries\RATECONTROL" -I"C:\Academia\Arduino\libraries\WATCHDOG" -I"C:\Academia\Arduino\libraries\WIXEL" -I"C:\Academia\Arduino\libraries\ECF" -I"C:\Academia\Arduino\libraries\QuaternionMath" -I"C:\Academia\Arduino\libraries\RADIO" -I"C:\Program Files (x86)\Arduino\hardware\teensy\avr\libraries\Wire\utility" -I"C:\Program Files (x86)\Arduino\hardware\teensy\avr\libraries\Wire" -I"C:\Academia\Arduino\libraries\THRUSTCONTROL" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -D__IN_ECLIPSE__=1 -x c++ "$<"  -o  "$@"
	@echo 'Finished building: $<'
	@echo ' '

libraries\Wire\WireIMXRT.cpp.o: C:\Program\ Files\ (x86)\Arduino\hardware\teensy\avr\libraries\Wire\WireIMXRT.cpp
	@echo 'Building file: $<'
	@echo 'Starting C++ compile'
	"C:/Program Files (x86)/Arduino/hardware/teensy/../tools/arm/bin/arm-none-eabi-g++" -c -O3 -g -Wall -ffunction-sections -fdata-sections -nostdlib -MMD -fno-exceptions -fpermissive -felide-constructors -std=gnu++14 -Wno-error=narrowing -fno-rtti -mthumb -mcpu=cortex-m4 -fsingle-precision-constant -D__MK20DX256__ -DTEENSYDUINO=147 -DARDUINO=10802 -DF_CPU=96000000 -DUSB_SERIAL -DLAYOUT_UNITED_KINGDOM "-IC:/Academia/Sloeber/floki/Release/pch"   -I"C:\Program Files (x86)\Arduino\hardware\teensy\avr\cores\teensy3" -I"C:\Academia\Arduino\libraries\ATTITUDECONTROL" -I"C:\Academia\Arduino\libraries\Common" -I"C:\Academia\Arduino\libraries\ENCODER2" -I"C:\Academia\Arduino\libraries\HEAVECONTROL" -I"C:\Academia\Arduino\libraries\I2Cdev" -I"C:\Academia\Arduino\libraries\IMU" -I"C:\Academia\Arduino\libraries\LED" -I"C:\Program Files\Sloeber\arduinoPlugin\libraries\MatrixMath\1.0.0" -I"C:\Program Files (x86)\Arduino\hardware\teensy\avr\libraries\Metro" -I"C:\Academia\Arduino\libraries\MOTORSPEEDCONTROL" -I"C:\Academia\Arduino\libraries\MPU6050" -I"C:\Academia\Arduino\libraries\PWM" -I"C:\Academia\Arduino\libraries\RATECONTROL" -I"C:\Academia\Arduino\libraries\WATCHDOG" -I"C:\Academia\Arduino\libraries\WIXEL" -I"C:\Academia\Arduino\libraries\ECF" -I"C:\Academia\Arduino\libraries\QuaternionMath" -I"C:\Academia\Arduino\libraries\RADIO" -I"C:\Program Files (x86)\Arduino\hardware\teensy\avr\libraries\Wire\utility" -I"C:\Program Files (x86)\Arduino\hardware\teensy\avr\libraries\Wire" -I"C:\Academia\Arduino\libraries\THRUSTCONTROL" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -D__IN_ECLIPSE__=1 -x c++ "$<"  -o  "$@"
	@echo 'Finished building: $<'
	@echo ' '

libraries\Wire\WireKinetis.cpp.o: C:\Program\ Files\ (x86)\Arduino\hardware\teensy\avr\libraries\Wire\WireKinetis.cpp
	@echo 'Building file: $<'
	@echo 'Starting C++ compile'
	"C:/Program Files (x86)/Arduino/hardware/teensy/../tools/arm/bin/arm-none-eabi-g++" -c -O3 -g -Wall -ffunction-sections -fdata-sections -nostdlib -MMD -fno-exceptions -fpermissive -felide-constructors -std=gnu++14 -Wno-error=narrowing -fno-rtti -mthumb -mcpu=cortex-m4 -fsingle-precision-constant -D__MK20DX256__ -DTEENSYDUINO=147 -DARDUINO=10802 -DF_CPU=96000000 -DUSB_SERIAL -DLAYOUT_UNITED_KINGDOM "-IC:/Academia/Sloeber/floki/Release/pch"   -I"C:\Program Files (x86)\Arduino\hardware\teensy\avr\cores\teensy3" -I"C:\Academia\Arduino\libraries\ATTITUDECONTROL" -I"C:\Academia\Arduino\libraries\Common" -I"C:\Academia\Arduino\libraries\ENCODER2" -I"C:\Academia\Arduino\libraries\HEAVECONTROL" -I"C:\Academia\Arduino\libraries\I2Cdev" -I"C:\Academia\Arduino\libraries\IMU" -I"C:\Academia\Arduino\libraries\LED" -I"C:\Program Files\Sloeber\arduinoPlugin\libraries\MatrixMath\1.0.0" -I"C:\Program Files (x86)\Arduino\hardware\teensy\avr\libraries\Metro" -I"C:\Academia\Arduino\libraries\MOTORSPEEDCONTROL" -I"C:\Academia\Arduino\libraries\MPU6050" -I"C:\Academia\Arduino\libraries\PWM" -I"C:\Academia\Arduino\libraries\RATECONTROL" -I"C:\Academia\Arduino\libraries\WATCHDOG" -I"C:\Academia\Arduino\libraries\WIXEL" -I"C:\Academia\Arduino\libraries\ECF" -I"C:\Academia\Arduino\libraries\QuaternionMath" -I"C:\Academia\Arduino\libraries\RADIO" -I"C:\Program Files (x86)\Arduino\hardware\teensy\avr\libraries\Wire\utility" -I"C:\Program Files (x86)\Arduino\hardware\teensy\avr\libraries\Wire" -I"C:\Academia\Arduino\libraries\THRUSTCONTROL" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -D__IN_ECLIPSE__=1 -x c++ "$<"  -o  "$@"
	@echo 'Finished building: $<'
	@echo ' '

