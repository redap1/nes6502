#include "nes6502.h"
#include "Bus.h"

nes6502::nes6502() {

    // initialize instruction lookup table
    using c = nes6502;                                          // used to make table a little shorter
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

    reset();
}

void nes6502::clock() {
    if (cycles == 0) {
        opcode = read(pc++);
        INSTR instr = instr_table[opcode];

        // make sure unused flag is always high
        status.flags.U = 1;
        
        cycles = instr.cycles;
        
        bool addr_mode_add_cycle = resolveAddress(instr.mode);
        bool op_add_cycle = instr.operate();

        // check if additional cycles are needed
        cycles += (addr_mode_add_cycle & op_add_cycle) ? 1 : 0;

        // just in case :P
        status.flags.U = 1;
    }

    --cycles;
}

void nes6502::reset() {
    Word low = read(PC_RESET_ADDR);
    Word high = read(PC_RESET_ADDR + 1);

    pc = (high << 8) | low;
    stkp = STK_START;

    accum = 0x00;
    index_x = 0x00;
    index_y = 0x00;
    status.reg = 0x00;

    // disable interrupts, make sure unused flag is always high
    status.flags.I = 1;
    status.flags.U = 1;

    // takes 8 cycles
    cycles = 8;
}

void nes6502::NMI() {
    write(STK_ADDR_START + stkp--, (pc >> 8) & 0x00FF);
    write(STK_ADDR_START + stkp--, pc & 0x00FF);

    status.flags.B = 1;
    status.flags.U = 1;
    status.flags.I = 1;

    write(STK_ADDR_START + stkp--, status.reg);
    
    Word low = read(IRQ_VECTOR);
    Word high = read(IRQ_VECTOR + 1);

    pc = (high << 8) | low;
        
    cycles = 7;
}

void nes6502::IRQ() {
    if (status.flags.I == 0) {
        write(STK_ADDR_START + stkp--, (pc >> 8) & 0x00FF);
        write(STK_ADDR_START + stkp--, pc & 0x00FF);

        status.flags.B = 1;
        status.flags.U = 1;
        status.flags.I = 1;

        write(STK_ADDR_START + stkp--, status.reg);
        
        Word low = read(IRQ_VECTOR);
        Word high = read(IRQ_VECTOR + 1);

        pc = (high << 8) | low;

        cycles = 7;
    }
}

void nes6502::write(Word addr, Byte data) {
    bus_->write(addr, data);
}

Byte nes6502::read(Word addr) {
    return bus_->read(addr);
}

bool nes6502::resolveAddress(AddressingMode mode) {
    /* with all of these addressing modes, we keep in mind that pc has already been incremented at this point in the cycle.
       this means that if the mode asks for another byte, we just read at the current pc. */
    switch (mode) {
        case Imp:
            // no extra memory accesses needed for instruction
            return false;
        case Imm:
            // the next byte is used as a value
            addr_abs = pc++;
            return false;
        case Accum:
            // skip for now
            return false;
        case Rel:
            // used with only branch instructions; second byte is an offset that is added to pc when counter is set at next instruction
            addr_rel = read(pc++);
            if (addr_rel & 0x80) {
                addr_rel |= 0xFF00;                                               // turn negative if MSB is 1 (negative)
            }
            return false;
        case ZP:
            // fetch second byte of instruction and assume zero page high byte
            addr_abs = read(pc++);
            addr_abs &= 0x00FF;
            return false;
        case ZPX:
            // zero page with x register offset
            addr_abs = read(pc) + index_x;
            addr_abs &= 0x00FF;
            return false;
        case ZPY:
            // zero page with y register offset
            addr_abs = read(pc) + index_y;
            addr_abs &= 0x00FF;
            return false;
        case Abs:
            // load a full 16-bit address using the second and third bytes
            Byte low = read(pc++);
            Byte high = read(pc++);

            addr_abs = (high << 8) | low;
            return false;
        case AbsX:
            // absolute with x register offset, need to account for page changes
            Byte low = read(pc++);
            Byte high = read(pc++);

            addr_abs = ((high << 8) | low) + index_x;

            // needs an additional clock cycle if the page changes (high byte changes from the high byte in memory)
            return ((addr_abs & 0xFF00) != (high << 8));
        case AbsY:
            // absolute with y register offset
            Byte low = read(pc++);
            Byte high = read(pc++);

            addr_abs = ((high << 8) | low) + index_x;

            // needs an additional clock cycle if the page changes (high byte changes from the high byte in memory)
            return ((addr_abs & 0xFF00) != (high << 8)); 
        case Ind:
            // the second byte is the lower byte of a memory address, the third byte is the higher
            Byte low = read(pc++);
            Byte high = read(pc++);
            Byte addr = (high << 8) | low;

            // emulate 6502 bug: wrap around page if we cross page boundary
            if (addr & 0x00FF == 0x00FF) {
                addr_abs |= read(addr & 0xFF00);
            }
            else {
                addr_abs |= read(addr + 1);
            }

            addr_abs |= read(addr);
            return false;
        case IndX:
            // the second byte of the instruction is added to register x; this points to a memory location on page 0 which contains the effective address
            Byte ind_low = read(pc++);
            Word addr = ind_low + index_x;
            
            Byte low = read(addr & 0x00FF);
            Byte high = read((addr + 1) & 0x00FF);

            addr_abs = (high << 8) | low;
            return false;

        case IndY:
            // the second byte stores a pointer on page 0; the contents of memory at the address in pointer are offset by y
            Word ptr = read(pc++) & 0x00FF;
            
            Byte low = read(ptr & 0x00FF);
            Byte high = read((ptr + 1) & 0x00FF);

            addr_abs = ((high << 8) | low) + index_y;

            // check for page crossing
            return ((addr_abs & 0xFF00) != (high << 8));
        default: return false; 
    }
}

Byte nes6502::fetch() {
    if (instr_table[opcode].mode != Imp && instr_table[opcode].mode != Accum) {
        return read(addr_abs);
    }
    
    if (instr_table[opcode].mode == Accum) {
        return accum;
    }

    // implied or no fetch needed
    return 0x00;
}






