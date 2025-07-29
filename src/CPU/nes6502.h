#pragma once

#include <vector>
#include <cstdint>
#include <string>
#include <functional>

using Byte = std::uint8_t;
using Word = std::uint16_t;

inline constexpr Word PC_RESET_ADDR = 0xFFFC;
inline constexpr Byte STK_START = 0xFD;

class Bus;

class nes6502 {
public:
    nes6502();

    void connectBus(Bus* bus) { bus_ = bus; }

    // input signals
    void clock();
    void reset();             // reset
    void NMI();               // non-maskable interrupt
    void IRQ();               // interrupt request

    // registers
    Byte accum   {};          // accumulator
    Byte index_x {};          // index registers x and y
    Byte index_y {};
    Byte stkp    {};          // stack pointer
    Word pc      {};          // program counter

    union Status
    {
        struct {
            Byte C : 1;       // carry bit
            Byte Z : 1;       // zero
            Byte I : 1;       // disable interrupts
            Byte D : 1;       // decimal mode (unused)
            Byte B : 1;       // break
            Byte U : 1;       // unused
            Byte V : 1;       // overflow
            Byte N : 1;       // negative
        } flags;

        Byte full_status;
    };

    union Status status {};      // status register

private:
    Bus* bus_ = nullptr;

    // read/write via bus
    void cpuWrite(Word addr, Byte data);
    Byte cpuRead(Word addr);

    Byte opcode   {};
    Byte cycles   {};
    Word addr_abs {};
    Word addr_rel {};

    // Addressing Modes
    enum AddressingMode {
        Imp,
        Imm,
        Accum,
        ZP,
        ZPX,
        ZPY,
        Rel,
        Abs,
        AbsX,
        AbsY,
        Ind,
        IndX,
        IndY
    };

    // will be a switch statement that does all addressing mode logic
    bool resolveAddress(AddressingMode mode);

    // Instructions, only official opcodes will be implemented for now
    bool ADC();    bool AND();    bool ASL();    bool BCC();
    bool BCS();    bool BEQ();    bool BIT();    bool BMI();
    bool BNE();    bool BPL();    bool BRK();    bool BVC();
    bool BVS();    bool CLC();    bool CLD();    bool CLI();
    bool CLV();    bool CMP();    bool CPX();    bool CPY();
    bool DEC();    bool DEX();    bool DEY();    bool EOR();
    bool INC();    bool INX();    bool INY();    bool JMP();
    bool JSR();    bool LDA();    bool LDX();    bool LDY();
    bool LSR();    bool NOP();    bool ORA();    bool PHA();
    bool PHP();    bool PLA();    bool PLP();    bool ROL();
    bool ROR();    bool RTI();    bool RTS();    bool SBC();
    bool SEC();    bool SED();    bool SEI();    bool STA();
    bool STX();    bool STY();    bool TAX();    bool TAY();
    bool TSX();    bool TXA();    bool TXS();    bool TYA();

    // will capture all unofficial/unused opcodes
    bool XXX();    

    // Instruction lookup table
    struct INSTR {
        std::string name;
        AddressingMode mode;
        Byte cycles;
        std::function<bool()> operate;
    };

    std::vector<INSTR> instr_table;
};