#include "nes6502.h"
#include "Bus.h"

nes6502::nes6502() {

    // initialize instruction lookup table
    using c = nes6502;                               // used to make table a little shorter
    using std::bind;

    instr_table = 
    {
        {"BRK", Imp, 7, bind(&c::BRK, this)}, {"ORA", IndX, 6, bind(&c::ORA, this)}, {"XXX", Imp, 2, bind(&c::XXX, this)}, {"XXX", Imp, 2, bind(&c::XXX, this)}, {"XXX", Imp, 2, bind(&c::XXX, this)}, {"ORA", ZP, 3, bind(&c::ORA, this)}, {"ASL", ZP, 5, bind(&c::ASL, this)}, {"XXX", Imp, 2, bind(&c::XXX, this)}, {"PHP", Imp, 3, bind(&c::PHP, this)}, {"ORA", Imm, 2, bind(&c::ORA, this)}, {"ASL", Accum, 2, bind(&c::ASL, this)}, {"XXX", Imp, 2, bind(&c::XXX, this)}, {"XXX", Imp, 2, bind(&c::XXX, this)}, {"ORA", Abs, 4, bind(&c::ORA, this)}, {"ASL", Abs, 6, bind(&c::ASL, this)}, {"XXX", Imp, 2, bind(&c::XXX, this)},
        {"BPL", Rel, 2, bind(&c::BPL, this)}, {"ORA", IndY, 5, bind(&c::ORA, this)}, {"XXX", Imp, 2, bind(&c::XXX, this)}, {"XXX", Imp, 2, bind(&c::XXX, this)}, {"XXX", Imp, 2, bind(&c::XXX, this)}, {"ORA", ZPX, 4, bind(&c::ORA, this)}, {"ASL", ZPX, 6, bind(&c::ASL, this)}, {"XXX", Imp, 2, bind(&c::XXX, this)}, {"CLC", Imp, 2, bind(&c::CLC, this)}, {"ORA", AbsY, 4, bind(&c::ORA, this)}, {"XXX", Imp, 2, bind(&c::XXX, this)}, {"XXX", Imp, 2, bind(&c::XXX, this)}, {"XXX", Imp, 2, bind(&c::XXX, this)}, {"ORA", AbsX, 4, bind(&c::ORA, this)}, {"ASL", AbsX, 7, bind(&c::ASL, this)}, {"XXX", Imp, 2, bind(&c::XXX, this)},
        {"JSR", Abs, 6, bind(&c::JSR, this)}, {"AND", IndX, 6, bind(&c::AND, this)}, {"XXX", Imp, 2, bind(&c::XXX, this)}, {"XXX", Imp, 2, bind(&c::XXX, this)}, {"BIT", ZP, 3, bind(&c::BIT, this)}, {"AND", ZP, 3, bind(&c::AND, this)}, {"ROL", ZP, 5, bind(&c::ROL, this)}, {"XXX", Imp, 2, bind(&c::XXX, this)}, {"PLP", Imp, 4, bind(&c::PLP, this)}, {"AND", Imm, 2, bind(&c::AND, this)}, {"ROL", Accum, 2, bind(&c::ROL, this)}, {"XXX", Imp, 2, bind(&c::XXX, this)}, {"BIT", Abs, 4, bind(&c::BIT, this)}, {"AND", Abs, 4, bind(&c::AND, this)}, {"ROL", Abs, 6, bind(&c::ROL, this)}, {"XXX", Imp, 2, bind(&c::XXX, this)},
        {"BMI", Rel, 2, bind(&c::BMI, this)}, {"AND", IndY, 5, bind(&c::AND, this)}, {"XXX", Imp, 2, bind(&c::XXX, this)}, {"XXX", Imp, 2, bind(&c::XXX, this)}, {"XXX", Imp, 2, bind(&c::XXX, this)}, {"AND", ZPX, 4, bind(&c::AND, this)}, {"ROL", ZPX, 6, bind(&c::ROL, this)}, {"XXX", Imp, 2, bind(&c::XXX, this)}, {"SEC", Imp, 2, bind(&c::SEC, this)}, {"AND", AbsY, 4, bind(&c::AND, this)}, {"XXX", Imp, 2, bind(&c::XXX, this)}, {"XXX", Imp, 2, bind(&c::XXX, this)}, {"XXX", Imp, 2, bind(&c::XXX, this)}, {"AND", AbsX, 4, bind(&c::AND, this)}, {"ROL", AbsX, 7, bind(&c::ROL, this)}, {"XXX", Imp, 2, bind(&c::XXX, this)},
        {"RTI", Imp, 6, bind(&c::RTI, this)}, {"EOR", IndX, 6, bind(&c::EOR, this)}, {"XXX", Imp, 2, bind(&c::XXX, this)}, {"XXX", Imp, 2, bind(&c::XXX, this)}, {"XXX", Imp, 2, bind(&c::XXX, this)}, {"EOR", ZP, 3, bind(&c::EOR, this)}, {"LSR", ZP, 5, bind(&c::LSR, this)}, {"XXX", Imp, 2, bind(&c::XXX, this)}, {"PHA", Imp, 3, bind(&c::PHA, this)}, {"EOR", Imm, 2, bind(&c::EOR, this)}, {"LSR", Accum, 2, bind(&c::LSR, this)}, {"XXX", Imp, 2, bind(&c::XXX, this)}, {"JMP", Abs, 3, bind(&c::JMP, this)}, {"EOR", Abs, 4, bind(&c::EOR, this)}, {"LSR", Abs, 6, bind(&c::LSR, this)}, {"XXX", Imp, 2, bind(&c::XXX, this)},
        {"BVC", Rel, 2, bind(&c::BVC, this)}, {"EOR", IndY, 5, bind(&c::EOR, this)}, {"XXX", Imp, 2, bind(&c::XXX, this)}, {"XXX", Imp, 2, bind(&c::XXX, this)}, {"XXX", Imp, 2, bind(&c::XXX, this)}, {"EOR", ZPX, 4, bind(&c::EOR, this)}, {"LSR", ZPX, 6, bind(&c::LSR, this)}, {"XXX", Imp, 2, bind(&c::XXX, this)}, {"CLI", Imp, 2, bind(&c::CLI, this)}, {"EOR", AbsY, 4, bind(&c::EOR, this)}, {"XXX", Imp, 2, bind(&c::XXX, this)}, {"XXX", Imp, 2, bind(&c::XXX, this)}, {"XXX", Imp, 2, bind(&c::XXX, this)}, {"EOR", AbsX, 4, bind(&c::EOR, this)}, {"LSR", AbsX, 7, bind(&c::LSR, this)}, {"XXX", Imp, 2, bind(&c::XXX, this)},
        {"RTS", Imp, 6, bind(&c::RTS, this)}, {"ADC", IndX, 6, bind(&c::ADC, this)}, {"XXX", Imp, 2, bind(&c::XXX, this)}, {"XXX", Imp, 2, bind(&c::XXX, this)}, {"XXX", Imp, 2, bind(&c::XXX, this)}, {"ADC", ZP, 3, bind(&c::ADC, this)}, {"ROR", ZP, 5, bind(&c::ROR, this)}, {"XXX", Imp, 2, bind(&c::XXX, this)}, {"PLA", Imp, 4, bind(&c::PLA, this)}, {"ADC", Imm, 2, bind(&c::ADC, this)}, {"ROR", Accum, 2, bind(&c::ROR, this)}, {"XXX", Imp, 2, bind(&c::XXX, this)}, {"JMP", Ind, 5, bind(&c::JMP, this)}, {"ADC", Abs, 4, bind(&c::ADC, this)}, {"ROR", Abs, 5, bind(&c::ROR, this)}, {"XXX", Imp, 2, bind(&c::XXX, this)},
        {"BVS", Rel, 2, bind(&c::BVS, this)}, {"ADC", IndY, 5, bind(&c::ADC, this)}, {"XXX", Imp, 2, bind(&c::XXX, this)}, {"XXX", Imp, 2, bind(&c::XXX, this)}, {"XXX", Imp, 2, bind(&c::XXX, this)}, {"ADC", ZPX, 4, bind(&c::ADC, this)}, {"ROR", ZPX, 6, bind(&c::ROR, this)}, {"XXX", Imp, 2, bind(&c::XXX, this)}, {"SEI", Imp, 2, bind(&c::SEI, this)}, {"ADC", AbsY, 4, bind(&c::ADC, this)}, {"XXX", Imp, 2, bind(&c::XXX, this)}, {"XXX", Imp, 2, bind(&c::XXX, this)}, {"XXX", Imp, 2, bind(&c::XXX, this)}, {"ADC", AbsX, 4, bind(&c::ADC, this)}, {"ROR", AbsX, 7, bind(&c::ROR, this)}, {"XXX", Imp, 2, bind(&c::XXX, this)},
        {"XXX", Imp, 2, bind(&c::XXX, this)}, {"STA", IndX, 6, bind(&c::STA, this)}, {"XXX", Imp, 2, bind(&c::XXX, this)}, {"XXX", Imp, 2, bind(&c::XXX, this)}, {"STY", ZP, 3, bind(&c::STY, this)}, {"STA", ZP, 3, bind(&c::STA, this)}, {"STX", ZP, 3, bind(&c::STX, this)}, {"XXX", Imp, 2, bind(&c::XXX, this)}, {"DEY", Imp, 2, bind(&c::DEY, this)}, {"XXX", Imp, 2, bind(&c::XXX, this)}, {"TXA", Imp, 2, bind(&c::TXA, this)}, {"XXX", Imp, 2, bind(&c::XXX, this)}, {"STY", Abs, 4, bind(&c::STY, this)}, {"STA", Abs, 4, bind(&c::STA, this)}, {"STX", Abs, 4, bind(&c::STX, this)}, {"XXX", Imp, 2, bind(&c::XXX, this)},
	    {"BCC", Rel, 2, bind(&c::BCC, this)}, {"STA", IndY, 6, bind(&c::STA, this)}, {"XXX", Imp, 2, bind(&c::XXX, this)}, {"XXX", Imp, 2, bind(&c::XXX, this)}, {"STY", ZPX, 4, bind(&c::STY, this)}, {"STA", ZPX, 4, bind(&c::STA, this)}, {"STX", ZPY, 4, bind(&c::STX, this)}, {"XXX", Imp, 2, bind(&c::XXX, this)}, {"TYA", Imp, 2, bind(&c::TYA, this)}, {"STA", AbsY, 5, bind(&c::STA, this)}, {"TXS", Imp, 2, bind(&c::TXS, this)}, {"XXX", Imp, 2, bind(&c::XXX, this)}, {"XXX", Imp, 2, bind(&c::XXX, this)}, {"STA", AbsX, 5, bind(&c::STA, this)}, {"XXX", Imp, 2, bind(&c::XXX, this)}, {"XXX", Imp, 2, bind(&c::XXX, this)},
        {"LDY", Imm, 2, bind(&c::LDY, this)}, {"LDA", IndX, 6, bind(&c::LDA, this)}, {"LDX", Imm, 2, bind(&c::LDX, this)}, {"XXX", Imp, 2, bind(&c::XXX, this)}, {"LDY", ZP, 3, bind(&c::LDY, this)}, {"LDA", ZP, 3, bind(&c::LDA, this)}, {"LDX", ZP, 3, bind(&c::LDX, this)}, {"XXX", Imp, 2, bind(&c::XXX, this)}, {"TAY", Imp, 2, bind(&c::TAY, this)}, {"LDA", Imm, 2, bind(&c::LDA, this)}, {"TAX", Imp, 2, bind(&c::TAX, this)}, {"XXX", Imp, 2, bind(&c::XXX, this)}, {"LDY", Abs, 4, bind(&c::LDY, this)}, {"LDA", Abs, 4, bind(&c::LDA, this)}, {"LDX", Abs, 4, bind(&c::LDX, this)}, {"XXX", Imp, 2, bind(&c::XXX, this)},
        {"BCS", Rel, 2, bind(&c::BCS, this)}, {"LDA", IndY, 5, bind(&c::LDA, this)}, {"XXX", Imp, 2, bind(&c::XXX, this)}, {"XXX", Imp, 2, bind(&c::XXX, this)}, {"LDY", ZPX, 4, bind(&c::LDY, this)}, {"LDA", ZPX, 4, bind(&c::LDX, this)}, {"LDX", ZPY, 4, bind(&c::LDX, this)}, {"XXX", Imp, 2, bind(&c::XXX, this)}, {"CLV", Imp, 2, bind(&c::CLV, this)}, {"LDA", AbsY, 4, bind(&c::LDA, this)}, {"TSX", Imp, 2, bind(&c::TSX, this)}, {"XXX", Imp, 2, bind(&c::XXX, this)}, {"LDY", AbsX, 4, bind(&c::LDY, this)}, {"LDA", AbsX, 4, bind(&c::LDA, this)}, {"LDX", AbsY, 4, bind(&c::LDX, this)}, {"XXX", Imp, 2, bind(&c::XXX, this)},
        {"CPY", Imm, 2, bind(&c::CPY, this)}, {"CMP", IndX, 6, bind(&c::CMP, this)}, {"XXX", Imp, 2, bind(&c::XXX, this)}, {"XXX", Imp, 2, bind(&c::XXX, this)}, {"CPY", ZP, 3, bind(&c::CPY, this)}, {"CMP", ZP, 3, bind(&c::CMP, this)}, {"DEC", ZP, 5, bind(&c::DEC, this)}, {"XXX", Imp, 2, bind(&c::XXX, this)}, {"INY", Imp, 2, bind(&c::INY, this)}, {"CMP", Imm, 2, bind(&c::CMP, this)}, {"DEX", Imp, 2, bind(&c::DEX, this)}, {"XXX", Imp, 2, bind(&c::XXX, this)}, {"CPY", Abs, 4, bind(&c::CPY, this)}, {"CMP", Abs, 4, bind(&c::CMP, this)}, {"DEC", Abs, 6, bind(&c::DEC, this)}, {"XXX", Imp, 2, bind(&c::XXX, this)},
        {"BNE", Rel, 2, bind(&c::BNE, this)}, {"CMP", IndY, 5, bind(&c::CMP, this)}, {"XXX", Imp, 2, bind(&c::XXX, this)}, {"XXX", Imp, 2, bind(&c::XXX, this)}, {"XXX", Imp, 2, bind(&c::XXX, this)}, {"CMP", ZPX, 4, bind(&c::CMP, this)}, {"DEC", ZPX, 6, bind(&c::DEC, this)}, {"XXX", Imp, 2, bind(&c::XXX, this)}, {"CLD", Imp, 2, bind(&c::CLD, this)}, {"CMP", AbsY, 4, bind(&c::CMP, this)}, {"XXX", Imp, 2, bind(&c::XXX, this)}, {"XXX", Imp, 2, bind(&c::XXX, this)}, {"XXX", Imp, 2, bind(&c::XXX, this)}, {"CMP", AbsX, 4, bind(&c::CMP, this)}, {"DEC", AbsX, 7, bind(&c::DEC, this)}, {"XXX", Imp, 2, bind(&c::XXX, this)},
        {"CPX", Imm, 2, bind(&c::CPX, this)}, {"SBC", IndX, 6, bind(&c::SBC, this)}, {"XXX", Imp, 2, bind(&c::XXX, this)}, {"XXX", Imp, 2, bind(&c::XXX, this)}, {"CPX", ZP, 3, bind(&c::CPX, this)}, {"SBC", ZP, 3, bind(&c::SBC, this)}, {"INC", ZP, 5, bind(&c::INC, this)}, {"XXX", Imp, 2, bind(&c::XXX, this)}, {"INX", Imp, 2, bind(&c::INX, this)}, {"SBC", Imm, 2, bind(&c::SBC, this)}, {"NOP", Imp, 2, bind(&c::NOP, this)}, {"XXX", Imp, 2, bind(&c::XXX, this)}, {"CPX", Abs, 4, bind(&c::CPX, this)}, {"SBC", Abs, 4, bind(&c::SBC, this)}, {"INC", Abs, 6, bind(&c::INC, this)}, {"XXX", Imp, 2, bind(&c::XXX, this)},
        {"BEQ", Rel, 2, bind(&c::BEQ, this)}, {"SBC", IndY, 5, bind(&c::SBC, this)}, {"XXX", Imp, 2, bind(&c::XXX, this)}, {"XXX", Imp, 2, bind(&c::XXX, this)}, {"XXX", Imp, 2, bind(&c::XXX, this)}, {"SBC", ZPX, 4, bind(&c::SBC, this)}, {"INC", ZPX, 6, bind(&c::INC, this)}, {"XXX", Imp, 2, bind(&c::XXX, this)}, {"SED", Imp, 2, bind(&c::SED, this)}, {"SBC", Imp, 2, bind(&c::SBC, this)}, {"XXX", Imp, 2, bind(&c::XXX, this)}, {"XXX", Imp, 2, bind(&c::XXX, this)}, {"XXX", Imp, 2, bind(&c::XXX, this)}, {"SBC", AbsX, 4, bind(&c::SBC, this)}, {"INC", AbsX, 7, bind(&c::INC, this)}, {"XXX", Imp, 2, bind(&c::XXX, this)}
    };
}

void nes6502::cpuWrite(Word addr, Byte data) {
    bus_->write(addr, data);
}

Byte nes6502::cpuRead(Word addr) {
    return bus_->read(addr);
}

Byte nes6502::resolveAddress(AddressingMode mode) {
    switch (mode) {
        case Imp:
        case Imm:
        case Accum:
        case Rel:
        case ZP:
        case ZPX:
        case ZPY:
        case Abs:
        case AbsX:
        case AbsY:
        case Ind:
        case IndX:
        case IndY:
        default: break;
    }
    return 0x00;
}





