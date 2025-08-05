#include <cstdint>
#include <array>
#include "CPU/nes6502.h"

using Byte = uint8_t;
using Word = uint16_t;

class Bus {
public:
    Bus();

    //devices connected to bus
    
    std::array<Byte, 64 * 1024> memory {};

    void write(Word addr, Byte data);
    Byte read(Word addr);
};