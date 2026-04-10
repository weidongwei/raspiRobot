# 定义编译器
CXX = g++
# 定义编译选项
CXXFLAGS = -O2 `pkg-config --cflags opencv4`
# 定义链接库
LDFLAGS = -lwiringPi `pkg-config --libs opencv4`

# 定义目标文件名
TARGET = motor
# 定义所有的源文件
SRCS = main.cpp thread.cpp communicate.cpp motor.cpp ImgProc.cpp detectLaser.cpp SeamTracker.cpp calibrateCamera.cpp

# 将 .cpp 后缀替换为 .o
OBJS = $(SRCS:.cpp=.o)

# --- 关键修改点：添加默认目标和链接规则 ---
# 放在最前面的 target 会被默认执行
all: $(TARGET)

$(TARGET): $(OBJS)
	$(CXX) -o $(TARGET) $(OBJS) $(LDFLAGS)

# 模式规则：将每个 .cpp 编译成对应的 .o
%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

# 清理
clean:
	rm -f $(TARGET) $(OBJS)

