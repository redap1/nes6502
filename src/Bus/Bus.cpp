#include "Bus.h"

Bus::Bus() {

}

void Bus::write(Word addr, Byte data) {
    RAM[addr] = data;
}

Byte Bus::read(Word addr) {
    return RAM[addr];
}


