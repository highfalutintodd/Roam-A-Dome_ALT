CXX ?= c++
CXXFLAGS ?= -std=c++17 -Wall -Wextra -g

SRCS = test_main.cpp ../../src/CommandParser.cpp
BIN = rad_tests

.PHONY: test clean

test: $(BIN)
	./$(BIN)

$(BIN): $(SRCS) ../../src/Command.h ../../src/LineAssembler.h ../../src/SyrenCodec.h rad_test.h
	$(CXX) $(CXXFLAGS) -o $@ $(SRCS)

clean:
	rm -f $(BIN)
