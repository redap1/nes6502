#include <cstdint>
#include <array>
#include "CPU/nes6502.h"

using Byte = std::uint8_t;
using Word = std::uint16_t;

class Bus {
public:
    Bus();

    //devices connected to bus
    
    std::array<Byte, 64 * 1024> memory {};

    void write(Word addr, Byte data);
    Byte read(Word addr);
};
