PROJECT_DIR = $(CURDIR)
LIB_DIR = $(CURDIR)

# Directories
BUILD_DIR := lib
INCLUDE_DIR := .
SRC_DIR := src

# Compiler
CXX := g++

# Flags
WFLAGS := -Wall -Wextra -Werror -Wshadow 
MFLAGS := -ffloat-store -fno-fast-math
CFLAGS := $(WFLAGS) $(MFLAGS) 
CFLAGS += -I$(INCLUDE_DIR)
CFLAGS += -DLIB_DIR=\"$(LIB_DIR)\"

# Files
LIBRARY := data_handler
TARGET := $(BUILD_DIR)/lib$(LIBRARY).a
SOURCES := $(wildcard $(SRC_DIR)/*.cpp)
OBJECTS := $(patsubst $(SRC_DIR)/%.cpp,$(BUILD_DIR)/%.o,$(SOURCES))

# Unit Test Files
TEST_DIR := test
TEST_BUILD := $(TEST_DIR)/build

TEST_TARGET := $(TEST_BUILD)/test
TEST_SOURCES := $(wildcard $(TEST_DIR)/*.cpp)
TEST_OBJECTS :=  $(patsubst $(TEST_DIR)/%.cpp, $(TEST_BUILD)/%.o,$(TEST_SOURCES)) 

# Static C++ Code Analyser
CPPCHECK := cppcheck


.PHONEY: all clean test run cppcheck

# Linking
$(TARGET): $(OBJECTS)
	@mkdir -p $(dir $@)
	ar rcs $@ $^

# Compiling 
$(BUILD_DIR)/%.o: $(SRC_DIR)/%.cpp
	@mkdir -p $(dir $@)
	$(CXX) -g  $(CFLAGS) -c $^ -o $@ 

all: $(TARGET)

clean:
	rm -rf $(BUILD_DIR)
	rm -rf $(TEST_BUILD)

# Test Linking
$(TEST_TARGET): $(TARGET) $(TEST_OBJECTS) 
	@mkdir -p $(dir $@)
	$(CXX) $(TEST_OBJECTS) -L$(BUILD_DIR) -l$(LIBRARY) -o $@ 
	
# Test Compling
$(TEST_BUILD)/%.o: $(TEST_DIR)/%.cpp 
	@mkdir -p $(dir $@)
	$(CXX) -g $(CFLAGS) -c $^ -o $@ 

test: $(TEST_TARGET)

run: $(TEST_TARGET)
	PROJECT_DIR=$(PROJECT_DIR) $(TEST_TARGET)

# Static C++ Code Analyser
cppcheck: 
	$(CPPCHECK) --quiet --enable=all --suppress=missingIncludeSystem --error-exitcode=1 -I $(INCLUDE_DIR) $(SRC_DIR)
