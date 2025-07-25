#pragma once

#include <vector>
#include <cstdint>
#include <string>

using Byte = std::uint8_t;
using Word = std::uint16_t;

class Bus;

class nes6502 {
public:
    nes6502();

    void connectBus(Bus* bus) { bus_ = bus; }

    // input signals
    void clock();
    void NMI();         // non-maskable interrupt
    void IRQ();         // interrupt request
    void RST();         // reset

    // registers
    Byte accum;         // accumulator
    Byte index_x;       // index registers x and y
    Byte index_y;
    Byte stkp;          // stack pointer
    Word pc;            // program counter

    union Status
    {
        struct {
            Byte C : 1;      // carry bit
            Byte Z : 1;      // zero
            Byte I : 1;      // disable interrupts
            Byte D : 1;      // decimal mode (unused)
            Byte B : 1;      // break
            Byte U : 1;      // unused
            Byte V : 1;      // overflow
            Byte N : 1;      // negative
        } flags;

        Byte full_status;
    };

    union Status status;    // status register

private:
    Bus* bus_ = nullptr;

    // read/write via bus
    void cpuWrite(Word addr, Byte data);
    Byte cpuRead(Word addr);

    Byte opcode;
    Byte cycles;
    Word addr_abs;
    Word addr_rel;

    // Addressing Modes
    enum AddressingMode {
        Implied,
        Immediate,
        ZeroPage,
        ZeroPageX,
        ZeroPageY,
        Relative,
        Absolute,
        AbsoluteX,
        AbsoluteY,
        Indirect,
        IndirectX,
        IndirectY
    };

    // will be a switch statement that does all addressing mode logic
    Byte resolveAddress(AddressingMode mode);

    // Instructions, only official opcodes will be implemented
    Byte ADC();    Byte AND();    Byte ASL();    Byte BCC();
    Byte BCS();    Byte BEQ();    Byte BIT();    Byte BMI();
    Byte BNE();    Byte BPL();    Byte BRK();    Byte BVC();
    Byte BVS();    Byte CLC();    Byte CLD();    Byte CLI();
    Byte CLV();    Byte CMP();    Byte CPX();    Byte CPY();
    Byte DEC();    Byte DEX();    Byte DEY();    Byte EOR();
    Byte INC();    Byte INX();    Byte INY();    Byte JMP();
    Byte JSR();    Byte LDA();    Byte LDX();    Byte LDY();
    Byte LSR();    Byte NOP();    Byte CRA();    Byte PHA();
    Byte PHP();    Byte PLA();    Byte PLP();    Byte ROL();
    Byte ROR();    Byte RTI();    Byte RTS();    Byte SBC();
    Byte SEC();    Byte SED();    Byte SEI();    Byte STA();
    Byte STX();    Byte STY();    Byte TAX();    Byte TAY();
    Byte TSX();    Byte TXA();    Byte TXS();    Byte TYA(); 

    // Instruction lookup table
    struct INSTR {
        std::string name;
        AddressingMode mode;
        Byte cycles;
        Byte nes6502::*function(void);
    };

    std::vector<INSTR> instr_table;
};