CXX ?= c++
CXXFLAGS ?= -std=c++17 -Wall -Wextra -g

SRCS = test_main.cpp test_sensor.cpp test_sequencer.cpp test_motion.cpp \
       ../../src/CommandParser.cpp ../../src/Sequencer.cpp
HDRS = ../../src/Command.h ../../src/LineAssembler.h ../../src/SyrenCodec.h \
       ../../src/CircularMath.h ../../src/SensorRing.h ../../src/Sequencer.h \
       ../../src/MotionController.h ../../src/DisplaySleep.h rad_test.h
BIN = rad_tests

.PHONY: test clean

test: $(BIN)
	./$(BIN)

$(BIN): $(SRCS) $(HDRS)
	$(CXX) $(CXXFLAGS) -o $@ $(SRCS)

parse_cli: parse_cli.cpp ../../src/CommandParser.cpp ../../src/Sequencer.cpp $(HDRS)
	$(CXX) $(CXXFLAGS) -o $@ parse_cli.cpp ../../src/CommandParser.cpp ../../src/Sequencer.cpp

clean:
	rm -f $(BIN) parse_cli
