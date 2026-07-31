# SIL 预测试：host 编译真实固件代码（SimpleFOC + App）+ 电机模拟器
CXX      = g++
OPT      = -O0 -g
STD      = -std=gnu++17
WARN     = -Wall
DEFS     = 
INCS     = -I. -Istubs \
           -I../../firmware/App/platform \
           -I../../firmware/App \
           -I../../firmware/App/drivers \
           -I../../firmware/App/sensors \
           -I../../firmware/Core/Inc \
           -I../../firmware/vendors/simplefoc/src

CXXFLAGS = $(STD) $(WARN) $(OPT) $(DEFS) $(INCS)

SRCS = sil_main.cpp motor_model.cpp hal_stubs.cpp platform_stub.cpp \
       ../../firmware/App/platform/print.cpp \
       ../../firmware/App/platform/hardware_uart_stream.cpp \
       ../../firmware/App/drivers/hardware_api_stm32.cpp \
       ../../firmware/App/drivers/drv8316ct.cpp \
       ../../firmware/App/sensors/tim_encoder.cpp \
       ../../firmware/App/sensors/iprop_current_sense.cpp \
       ../../firmware/App/app.cpp \
       ../../firmware/vendors/simplefoc/src/BLDCMotor.cpp \
       ../../firmware/vendors/simplefoc/src/common/foc_utils.cpp \
       ../../firmware/vendors/simplefoc/src/common/lowpass_filter.cpp \
       ../../firmware/vendors/simplefoc/src/common/pid.cpp \
       ../../firmware/vendors/simplefoc/src/common/time_utils.cpp \
       ../../firmware/vendors/simplefoc/src/common/base_classes/CurrentSense.cpp \
       ../../firmware/vendors/simplefoc/src/common/base_classes/FOCMotor.cpp \
       ../../firmware/vendors/simplefoc/src/common/base_classes/Sensor.cpp \
       ../../firmware/vendors/simplefoc/src/communication/Commander.cpp \
       ../../firmware/vendors/simplefoc/src/communication/SimpleFOCDebug.cpp \
       ../../firmware/vendors/simplefoc/src/drivers/BLDCDriver3PWM.cpp

OBJS = $(SRCS:.cpp=.o)

all: sil_test

sil_test: $(OBJS)
	$(CXX) $(OBJS) -o $@

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

run: sil_test
	./sil_test

clean:
	rm -f $(OBJS) sil_test

.PHONY: all run clean
