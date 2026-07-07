CXX      ?= g++
CXXFLAGS ?= -O2 -std=c++17 -Wall -Wextra

all: slowmo_cam

slowmo_cam: slowmo_cam.cpp
	$(CXX) $(CXXFLAGS) -o $@ $< -lpthread

clean:
	rm -f slowmo_cam

.PHONY: all clean
