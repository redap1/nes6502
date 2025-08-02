#include "src/CPU/nes6502.h"
#include "src/Bus/Bus.h"

#include <iostream>

int main () {
    Bus nes;
    nes6502 cpu;

    cpu.connectBus(&nes);
    nes.connectCPU(&cpu);
    
    return 0;
}