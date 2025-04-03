# Directories
BUILD_DIR := lib
INCLUDE_DIR := include
SRC_DIR := src

# Compiler
CXX := g++

# Flags
WFLAGS := -Wall -Wextra -Werror -Wshadow
CFLAGS := $(WFLAGS)

# Files
LIBRARY := data_extractor
TARGET := $(BUILD_DIR)/lib$(LIBRARY).a
SOURCES := $(wildcard $(SRC_DIR)/*.cpp)
OBJECTS := $(patsubst $(SRC_DIR)/%.cpp,$(BUILD_DIR)/%.o,$(SOURCES))

# Unit Test Files
TEST_DIR := test
TEST_BUILD := $(TEST_DIR)/build

TEST_TARGET := $(TEST_BUILD)/test
TEST_SOURCES := $(wildcard $(TEST_DIR)/*.cpp)
TEST_OBJECTS :=  $(patsubst $(TEST_DIR)/%.cpp, $(TEST_BUILD)/%.o,$(TEST_SOURCES)) 

.PHONEY: all clean test

# Linking
$(TARGET): $(OBJECTS)
	@mkdir -p $(dir $@)
	ar rcs $@ $^

# Compiling 
$(BUILD_DIR)/%.o: $(SRC_DIR)/%.cpp
	@mkdir -p $(dir $@)
	$(CXX) -I $(INCLUDE_DIR) $(CFLAGS) -c $^ -o $@ 

all: $(TARGET)

clean:
	rm -rf $(BUILD_DIR)

# Test Linking
$(TEST_TARGET): $(TARGET) $(TEST_OBJECTS) 
	@mkdir -p $(dir $@)
	$(CXX) $(TEST_OBJECTS) -L$(BUILD_DIR) -l$(LIBRARY) -o $@ 
	
# Test Compling
$(TEST_BUILD)/%.o: $(TEST_DIR)/%.cpp 
	@mkdir -p $(dir $@)
	$(CXX) -I $(INCLUDE_DIR) $(CFLAGS) -c $^ -o $@ 

test: $(TEST_TARGET)
	$(TEST_TARGET)
