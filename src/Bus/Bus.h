#include <cstdint>
#include <array>
using Byte = uint8_t;
using Word = uint16_t;

class nes6502;

class Bus {
public:
    Bus();

    //devices connected to bus
    void connectCPU(nes6502* cpu) { cpu_ = cpu; }

    std::array<Byte, 64 * 1024> memory {};

    void write(Word addr, Byte data);
    Byte read(Word addr);

private:
    nes6502* cpu_ = nullptr;
};