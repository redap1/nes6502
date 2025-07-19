#include "Bus.h"

Bus::Bus() {
    cpu.connectBus(this);
}

void Bus::cpu_write(Word addr, Byte data) {
    RAM[addr] = data;
}

Byte Bus::cpu_read(Word addr) {
    return RAM[addr];
}


