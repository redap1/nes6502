#include <cstdint>

using Byte = uint8_t;
using Word = uint16_t;

class nes6502;

class Bus {
public:
    Bus();

    //devices connected to bus
    void connectCPU(nes6502* cpu) { cpu_ = cpu; }

    Byte RAM[64 * 1024];

    void write(Word addr, Byte data);
    Byte read(Word addr);

private:
    nes6502* cpu_ = nullptr;
};