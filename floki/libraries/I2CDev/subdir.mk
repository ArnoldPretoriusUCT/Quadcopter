################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
C:\Program\ Files\ (x86)\Arduino\libraries\I2CDev\I2Cdev.cpp 

LINK_OBJ += \
.\libraries\I2CDev\I2Cdev.cpp.o 

CPP_DEPS += \
.\libraries\I2CDev\I2Cdev.cpp.d 


# Each subdirectory must supply rules for building sources it contributes
libraries\I2CDev\I2Cdev.cpp.o: C:\Program\ Files\ (x86)\Arduino\libraries\I2CDev\I2Cdev.cpp
	@echo 'Building file: $<'
	@echo 'Starting C++ compile'
	"C:/Program Files (x86)/Arduino/hardware/teensy/../tools/arm/bin/arm-none-eabi-g++" -c -O3 -g -Wall -ffunction-sections -fdata-sections -nostdlib -MMD -fno-exceptions -fpermissive -felide-constructors -std=gnu++14 -Wno-error=narrowing -fno-rtti -mthumb -mcpu=cortex-m4 -fsingle-precision-constant -D__MK20DX256__ -DTEENSYDUINO=147 -DARDUINO=10802 -DF_CPU=96000000 -DUSB_SERIAL -DLAYOUT_UNITED_KINGDOM "-IC:/Academia/Sloeber/floki/Release/pch"   -I"C:\Program Files (x86)\Arduino\hardware\teensy\avr\cores\teensy3" -I"C:\Program Files (x86)\Arduino\libraries\Personal" -I"C:\Program Files\Sloeber\arduinoPlugin\libraries\MatrixMath\1.0.0" -I"C:\Program Files (x86)\Arduino\hardware\teensy\avr\libraries\Metro" -I"C:\Program Files (x86)\Arduino\hardware\teensy\avr\libraries\SPI" -I"C:\Program Files (x86)\Arduino\hardware\teensy\avr\libraries\Wire\utility" -I"C:\Program Files (x86)\Arduino\hardware\teensy\avr\libraries\Wire" -I"C:\Program Files (x86)\Arduino\libraries\I2CDev" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -D__IN_ECLIPSE__=1 -x c++ "$<"  -o  "$@"
	@echo 'Finished building: $<'
	@echo ' '


