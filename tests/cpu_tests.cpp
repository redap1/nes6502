#include <catch2/catch_test_macros.hpp>
#include "../src/CPU/nes6502.h"
#include "../src/Bus/Bus.h"

TEST_CASE("CPU Reset", "[cpu]") {
    nes6502 cpu;
    Bus bus;

    cpu.connectBus(&bus);

    bus.memory[0xFFFC] = 0x00;
    bus.memory[0xFFFD] = 0x80;

    cpu.reset();

    REQUIRE(cpu.pc == 0x8000);
    REQUIRE(cpu.getCycles() == 8);
    REQUIRE(cpu.accum == 0x00);
    REQUIRE(cpu.status.reg == 0x24);
}


