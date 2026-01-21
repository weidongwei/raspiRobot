# 定义编译器
CXX = g++
# 定义编译选项
CXXFLAGS = -O3 `pkg-config --cflags opencv4`
# 定义链接库
LDFLAGS = -lwiringPi `pkg-config --libs opencv4`

# 定义目标文件名
TARGET = motor
# 定义所有的源文件
SRCS = main.cpp thread.cpp communicate.cpp motor.cpp ImgProc.cpp

# 编译规则
$(TARGET): $(SRCS)
	$(CXX) -o $(TARGET) $(SRCS) $(CXXFLAGS) $(LDFLAGS)

# 清理编译结果
clean:
	rm -f $(TARGET)

