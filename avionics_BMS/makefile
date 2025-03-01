# Compiler and flags
CXX = g++
CXXFLAGS = -std=c++11 -O0 -g #-pthread

# Include directories
#INCLUDES = -I/pathToDirectories

# Library directories and libraries
#LIBDIRS = -L/..
LIBS = -lCppLinuxSerial

# Define variables for directories and files
BIN_DIR = bin

# Executable name
TARGET = example_usage

# Source files
SRCS = BMS_control.cpp  example_usage.cpp
OBJS = $(SRCS:.cpp=.o)

# Default rule to build the executable
all: $(TARGET)

# Rule to build the executable
$(TARGET): $(OBJS)
	#@mkdir -p $(BIN_DIR)  # Create the bin directory if it doesn't exist
	$(CXX) $(CXXFLAGS) -o $(TARGET) $(OBJS) $(LIBS)

# Rule to compile source files into object files
%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Clean up
clean:
	rm -rf $(OBJS) $(BIN_DIR) $(TARGET)