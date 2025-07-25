#include "nes6502.h"
#include "Bus.h"

void nes6502::cpuWrite(Word addr, Byte data) {
    bus_->write(addr, data);
}

Byte nes6502::cpuRead(Word addr) {
    return bus_->read(addr);
}





