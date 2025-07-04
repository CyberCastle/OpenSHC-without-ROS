CXX=g++
CXXFLAGS=-std=c++17 -I../include -I../test -I/usr/include/eigen3

TARGETS=test_example_usage test_dh_parameters

all: $(TARGETS)

# Build example_usage with RK4 source
test_example_usage: test_example_usage.cpp ../src/simple_rk4.cpp
	$(CXX) $(CXXFLAGS) $^ -o $@

# Build dh parameters example
test_dh_parameters: test_dh_parameters.cpp
	$(CXX) $(CXXFLAGS) $^ -o $@

clean:
	rm -f $(TARGETS)

.PHONY: all clean
