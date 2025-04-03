# Directories
BUILD_DIR := build
BIN_DIR := $(BUILD_DIR)/bin
INCLUDE_DIR := include
SRC_DIR := src
TEST_DIR := test

# Compiler
CXX := g++
CFLAGS := -I $(INCLUDE_DIR)

# Files
TARGET := $(BIN_DIR)/data_extractor
SOURCES := $(wildcard $(SRC_DIR)/*.cpp)
OBJECTS := $(patsubst $(SRC_DIR)/%.cpp,$(BUILD_DIR)/%.o,$(SOURCES))

# Unit Test Files
TEST_TARGET := $(BIN_DIR)/test
TEST := $(wildcard $(TEST_DIR)/*.cpp)
TEST_OBJECTS :=  $(patsubst $(TEST_DIR)/%.cpp, $(BUILD_DIR)/%.o,$(TEST)) 
## Include all source files except main.cpp
TEST_SOURCES := $(filter-out $(SRC_DIR)/main.cpp, $(SOURCES)) 
TEST_SRC_OBJECTS := $(patsubst $(SRC_DIR)/%.cpp,$(BUILD_DIR)/%.o,$(TEST_SOURCES))

.PHONEY: all run clean test

# Linking
$(TARGET): $(OBJECTS)
	@mkdir -p $(dir $@)
	$(CXX) -o $@ $^

# Compiling 
$(BUILD_DIR)/%.o: $(SRC_DIR)/%.cpp
	@mkdir -p $(dir $@)
	$(CXX) $(CFLAGS) -c $^ -o $@ 

all: $(TARGET)

run: $(TARGET)
	$(TARGET)

clean:
	rm -rf $(BUILD_DIR)

# Test Linking
$(TEST_TARGET): $(TEST_OBJECTS) $(TEST_SRC_OBJECTS)
	@mkdir -p $(dir $@)
	$(CXX) -o $@ $^
	
# Test Compling
$(BUILD_DIR)/%.o: $(TEST_DIR)/%.cpp 
	@mkdir -p $(dir $@)
	$(CXX) $(CFLAGS) -c $^ -o $@ 

test: $(TEST_TARGET)
	$(TEST_TARGET)
