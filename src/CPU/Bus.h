#include <cstdint>
#include "nes6502.h"

class Bus {
public:
    Bus();

    //devices connected to bus
    nes6502 cpu;

    Byte RAM[64 * 1024];

private:
    void cpu_write(Word addr, Byte data);
    Byte cpu_read(Word addr);
};