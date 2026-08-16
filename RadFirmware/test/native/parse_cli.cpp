// Tiny CLI: parse each stdin line with the v2 CommandParser (and, for motion
// lines, the Sequencer step grammar); print the verdict. Driven by
// tools/conformance.py to replay the DroidNet command library.
#include "../../src/Command.h"
#include "../../src/Sequencer.h"

#include <cstdio>
#include <cstring>

int main() {
    char line[512];
    while (std::fgets(line, sizeof(line), stdin) != nullptr) {
        size_t n = std::strlen(line);
        while (n > 0 && (line[n - 1] == '\n' || line[n - 1] == '\r'))
            line[--n] = '\0';
        rad::Command cmd;
        const char* verdict = "?";
        switch (rad::parseLine(line, cmd)) {
        case rad::ParseStatus::kOk:
            if (cmd.id == rad::CmdId::kMotion)
                verdict = rad::Sequencer::validateScript(cmd.text) ? "OK" : "INVALID";
            else
                verdict = "OK";
            break;
        case rad::ParseStatus::kEmpty:
            verdict = "EMPTY";
            break;
        case rad::ParseStatus::kUnknown:
            verdict = "UNKNOWN";
            break;
        case rad::ParseStatus::kInvalid:
            verdict = "INVALID";
            break;
        }
        std::printf("%s\t%s\n", verdict, line);
    }
    return 0;
}
