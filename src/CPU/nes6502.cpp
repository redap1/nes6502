#include "nes6502.h"
#include "Bus.h"

using std::bind;

nes6502::nes6502() {
    // initialize instruction lookup table
    using c = nes6502;                               // used to make table a little shorter
    instr_table = 
    {
        {"BRK", Imp, 7, bind(&c::BRK, this)}, {"ORA", IndX, 6, bind(&c::ORA, this)}, {"XXX", Imp, 2, bind(&c::XXX, this)}, {"XXX", Imp, 2, bind(&c::XXX, this)}, {"XXX", Imp, 2, bind(&c::XXX, this)}, {"ORA", ZP, 3, bind(&c::ORA, this)}, {"ASL", ZP, 5, bind(&c::ASL, this)}, {"XXX", Imp, 2, bind(&c::XXX, this)}, {"PHP", Imp, 3, bind(&c::PHP, this)}, {"ORA", Imm, 2, bind(&c::ORA, this)}, {"ASL", Accum, 2, bind(&c::ASL, this)}, {"XXX", Imp, 2, bind(&c::XXX, this)}, {"XXX", Imp, 2, bind(&c::XXX, this)}, {"ORA", Abs, 4, bind(&c::ORA, this)}, {"ASL", Abs, 6, bind(&c::ASL, this)}, {"XXX", Imp, 2, bind(&c::XXX, this)},
        {"BPL", Rel, 2, bind(&c::BPL, this)}, {"ORA", IndY, 5, bind(&c::ORA, this)}, {"XXX", Imp, 2, bind(&c::XXX, this)}, {"XXX", Imp, 2, bind(&c::XXX, this)}, {"XXX", Imp, 2, bind(&c::XXX, this)}, {"ORA", ZPX, 4, bind(&c::ORA, this)}, {"ASL", ZPX, 6, bind(&c::ASL, this)}, {"XXX", Imp, 2, bind(&c::XXX, this)}, {"CLC", Imp, 2, bind(&c::CLC, this)}, {"ORA", AbsY, 4, bind(&c::ORA, this)}, {"XXX", Imp, 2, bind(&c::XXX, this)}, {"XXX", Imp, 2, bind(&c::XXX, this)}, {"XXX", Imp, 2, bind(&c::XXX, this)}, {"ORA", AbsX, 4, bind(&c::ORA, this)}, {"ASL", AbsX, 7, bind(&c::ASL, this)}, {"XXX", Imp, 2, bind(&c::XXX, this)},
        {"JSR", Abs, 6, bind(&c::JSR, this)}, {"AND", IndX, 6, bind(&c::AND, this)}, {"XXX", Imp, 2, bind(&c::XXX, this)}, {"XXX", Imp, 2, bind(&c::XXX, this)}, {"BIT", ZP, 3, bind(&c::BIT, this)}, {"AND", ZP, 3, bind(&c::AND, this)}, {"ROL", ZP, 5, bind(&c::ROL, this)}, {"XXX", Imp, 2, bind(&c::XXX, this)}, {"PLP", Imp, 4, bind(&c::PLP, this)}, {"AND", Imm, 2, bind(&c::AND, this)}, {"ROL", Accum, 2, bind(&c::ROL, this)}, {"XXX", Imp, 2, bind(&c::XXX, this)}, {"BIT", Abs, 4, bind(&c::BIT, this)}, {"AND", Abs, 4, bind(&c::AND, this)}, {"ROL", Abs, 6, bind(&c::ROL, this)}, {"XXX", Imp, 2, bind(&c::XXX, this)},
        {"BMI", Rel, 2, bind(&c::BMI, this)}, {"AND", IndY, 5, bind(&c::AND, this)}, {"XXX", Imp, 2, bind(&c::XXX, this)}, {"XXX", Imp, 2, bind(&c::XXX, this)}, {"XXX", Imp, 2, bind(&c::XXX, this)}, {"AND", ZPX, 4, bind(&c::AND, this)}, {"ROL", ZPX, 6, bind(&c::ROL, this)}, {"XXX", Imp, 2, bind(&c::XXX, this)}, {"SEC", Imp, 2, bind(&c::SEC, this)}, {"AND", AbsY, 4, bind(&c::AND, this)}, {"XXX", Imp, 2, bind(&c::XXX, this)}, {"XXX", Imp, 2, bind(&c::XXX, this)}, {"XXX", Imp, 2, bind(&c::XXX, this)}, {"AND", AbsX, 4, bind(&c::AND, this)}, {"ROL", AbsX, 7, bind(&c::ROL, this)}, {"XXX", Imp, 2, bind(&c::XXX, this)},
        {"RTI", Imp, 6, bind(&c::RTI, this)}, {"EOR", IndX, 6, bind(&c::EOR, this)}, {"XXX", Imp, 2, bind(&c::XXX, this)}, {"XXX", Imp, 2, bind(&c::XXX, this)}, {"XXX", Imp, 2, bind(&c::XXX, this)}, {"EOR", ZP, 3, bind(&c::EOR, this)}, {"LSR", ZP, 5, bind(&c::LSR, this)}, {"XXX", Imp, 2, bind(&c::XXX, this)}, {"PHA", Imp, 3, bind(&c::PHA, this)}, {"EOR", Imm, 2, bind(&c::EOR, this)}, {"LSR", Accum, 2, bind(&c::LSR, this)}, {"XXX", Imp, 2, bind(&c::XXX, this)}, {"JMP", Abs, 3, bind(&c::JMP, this)}, {"EOR", Abs, 4, bind(&c::EOR, this)}, {"LSR", Abs, 6, bind(&c::LSR, this)}, {"XXX", Imp, 2, bind(&c::XXX, this)},
        // first 5 rows correct

    };
}

// bus wrapper functions
void nes6502::cpuWrite(Word addr, Byte data) {
    bus_->write(addr, data);
}

Byte nes6502::cpuRead(Word addr) {
    return bus_->read(addr);
}





