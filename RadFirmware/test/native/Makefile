CXX ?= c++
CXXFLAGS ?= -std=c++17 -Wall -Wextra -g

SRCS = test_main.cpp test_sensor.cpp test_sequencer.cpp \
       ../../src/CommandParser.cpp ../../src/Sequencer.cpp
HDRS = ../../src/Command.h ../../src/LineAssembler.h ../../src/SyrenCodec.h \
       ../../src/CircularMath.h ../../src/SensorRing.h ../../src/Sequencer.h rad_test.h
BIN = rad_tests

.PHONY: test clean

test: $(BIN)
	./$(BIN)

$(BIN): $(SRCS) $(HDRS)
	$(CXX) $(CXXFLAGS) -o $@ $(SRCS)

clean:
	rm -f $(BIN)
