#motor: main.cpp motor.cpp thread.cpp ImgProc.cpp
#	g++ -o motor main.cpp motor.cpp thread.cpp ImgProc.cpp `pkg-config --cflags --libs opencv4`
#	g++ -o motor main.cpp motor.cpp -I/usr/include/opencv4 -L/usr/local/lib -lopencv_core -lopencv_imgcodecs -lopencv_highgui -lopencv_videoio -lopencv_imgproc


#g++ -o motorTest motor.cpp motorTest.cpp

CXX = g++
CXXFLAGS = -O2 -Wall `pkg-config --cflags opencv4`
LDFLAGS = `pkg-config --libs opencv4` -lgpiod

SRCS = main.cpp motor.cpp thread.cpp ImgProc.cpp
OBJS = $(SRCS:.cpp=.o)
TARGET = motor

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CXX) -o $@ $^ $(LDFLAGS)

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

clean:
	rm -f $(OBJS) $(TARGET)

