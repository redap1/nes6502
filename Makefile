CXX = g++
CXXFLAGS = -std=c++17 -Wall

SRCS = main.cpp CPU/nes6502.cpp
OBJS = $(SRCS:.cpp=.o)

all: nes

nes: $(OBJS)
	$(CXX) $(CXXFLAGS) -o nes $(OBJS)

clean:
	rm -f nes *.o CPU/*.o