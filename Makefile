CXX       ?= g++
CXXFLAGS  ?= -O2 -std=c++17 -Wall -Wextra
TRACR_DIR ?= $(HOME)/src/tracr

all: slowmo_cam

slowmo_cam: slowmo_cam.cpp
	$(CXX) $(CXXFLAGS) -o $@ $< -lpthread

# profiling build: TraCR instrumentation compiled in (writes ./tracr/ traces)
tracr: slowmo_cam.cpp
	$(CXX) $(CXXFLAGS) -DENABLE_TRACR -DUSE_HW_COUNTER \
	  -I$(TRACR_DIR)/include -I$(TRACR_DIR)/extern \
	  -o slowmo_cam_tracr $< -lpthread

clean:
	rm -f slowmo_cam slowmo_cam_tracr

.PHONY: all clean tracr
