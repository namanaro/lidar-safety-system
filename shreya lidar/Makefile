CXX = g++
CXXFLAGS = -Os -Wall -Wextra -std=c++17

all: lidar-guard lidar-ui

lidar-guard: guard.cpp config.cpp config.h
	$(CXX) $(CXXFLAGS) -o $@ guard.cpp config.cpp

lidar-ui: ui.cpp config.cpp config.h
	$(CXX) $(CXXFLAGS) -o $@ ui.cpp config.cpp

clean:
	rm -f lidar-guard lidar-ui
