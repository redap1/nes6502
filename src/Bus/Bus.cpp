#include "Bus.h"

Bus::Bus() {
    
}

void Bus::write(Word addr, Byte data) {
    memory[addr] = data;
}

Byte Bus::read(Word addr) {
    return memory[addr];
}


