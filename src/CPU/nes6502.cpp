#include "nes6502.h"
#include "Bus/Bus.h"

nes6502::nes6502() {

  // initialize instruction lookup table
  using c = nes6502; // used to make table a little shorter
  using std::bind;

  instr_table = {{"BRK", Imp, 7, bind(&c::BRK, this)},
                 {"ORA", IndX, 6, bind(&c::ORA, this)},
                 {"XXX", Imp, 2, bind(&c::XXX, this)},
                 {"XXX", Imp, 2, bind(&c::XXX, this)},
                 {"XXX", Imp, 2, bind(&c::XXX, this)},
                 {"ORA", ZP, 3, bind(&c::ORA, this)},
                 {"ASL", ZP, 5, bind(&c::ASL, this)},
                 {"XXX", Imp, 2, bind(&c::XXX, this)},
                 {"PHP", Imp, 3, bind(&c::PHP, this)},
                 {"ORA", Imm, 2, bind(&c::ORA, this)},
                 {"ASL", Accum, 2, bind(&c::ASL, this)},
                 {"XXX", Imp, 2, bind(&c::XXX, this)},
                 {"XXX", Imp, 2, bind(&c::XXX, this)},
                 {"ORA", Abs, 4, bind(&c::ORA, this)},
                 {"ASL", Abs, 6, bind(&c::ASL, this)},
                 {"XXX", Imp, 2, bind(&c::XXX, this)},
                 {"BPL", Rel, 2, bind(&c::BPL, this)},
                 {"ORA", IndY, 5, bind(&c::ORA, this)},
                 {"XXX", Imp, 2, bind(&c::XXX, this)},
                 {"XXX", Imp, 2, bind(&c::XXX, this)},
                 {"XXX", Imp, 2, bind(&c::XXX, this)},
                 {"ORA", ZPX, 4, bind(&c::ORA, this)},
                 {"ASL", ZPX, 6, bind(&c::ASL, this)},
                 {"XXX", Imp, 2, bind(&c::XXX, this)},
                 {"CLC", Imp, 2, bind(&c::CLC, this)},
                 {"ORA", AbsY, 4, bind(&c::ORA, this)},
                 {"XXX", Imp, 2, bind(&c::XXX, this)},
                 {"XXX", Imp, 2, bind(&c::XXX, this)},
                 {"XXX", Imp, 2, bind(&c::XXX, this)},
                 {"ORA", AbsX, 4, bind(&c::ORA, this)},
                 {"ASL", AbsX, 7, bind(&c::ASL, this)},
                 {"XXX", Imp, 2, bind(&c::XXX, this)},
                 {"JSR", Abs, 6, bind(&c::JSR, this)},
                 {"AND", IndX, 6, bind(&c::AND, this)},
                 {"XXX", Imp, 2, bind(&c::XXX, this)},
                 {"XXX", Imp, 2, bind(&c::XXX, this)},
                 {"BIT", ZP, 3, bind(&c::BIT, this)},
                 {"AND", ZP, 3, bind(&c::AND, this)},
                 {"ROL", ZP, 5, bind(&c::ROL, this)},
                 {"XXX", Imp, 2, bind(&c::XXX, this)},
                 {"PLP", Imp, 4, bind(&c::PLP, this)},
                 {"AND", Imm, 2, bind(&c::AND, this)},
                 {"ROL", Accum, 2, bind(&c::ROL, this)},
                 {"XXX", Imp, 2, bind(&c::XXX, this)},
                 {"BIT", Abs, 4, bind(&c::BIT, this)},
                 {"AND", Abs, 4, bind(&c::AND, this)},
                 {"ROL", Abs, 6, bind(&c::ROL, this)},
                 {"XXX", Imp, 2, bind(&c::XXX, this)},
                 {"BMI", Rel, 2, bind(&c::BMI, this)},
                 {"AND", IndY, 5, bind(&c::AND, this)},
                 {"XXX", Imp, 2, bind(&c::XXX, this)},
                 {"XXX", Imp, 2, bind(&c::XXX, this)},
                 {"XXX", Imp, 2, bind(&c::XXX, this)},
                 {"AND", ZPX, 4, bind(&c::AND, this)},
                 {"ROL", ZPX, 6, bind(&c::ROL, this)},
                 {"XXX", Imp, 2, bind(&c::XXX, this)},
                 {"SEC", Imp, 2, bind(&c::SEC, this)},
                 {"AND", AbsY, 4, bind(&c::AND, this)},
                 {"XXX", Imp, 2, bind(&c::XXX, this)},
                 {"XXX", Imp, 2, bind(&c::XXX, this)},
                 {"XXX", Imp, 2, bind(&c::XXX, this)},
                 {"AND", AbsX, 4, bind(&c::AND, this)},
                 {"ROL", AbsX, 7, bind(&c::ROL, this)},
                 {"XXX", Imp, 2, bind(&c::XXX, this)},
                 {"RTI", Imp, 6, bind(&c::RTI, this)},
                 {"EOR", IndX, 6, bind(&c::EOR, this)},
                 {"XXX", Imp, 2, bind(&c::XXX, this)},
                 {"XXX", Imp, 2, bind(&c::XXX, this)},
                 {"XXX", Imp, 2, bind(&c::XXX, this)},
                 {"EOR", ZP, 3, bind(&c::EOR, this)},
                 {"LSR", ZP, 5, bind(&c::LSR, this)},
                 {"XXX", Imp, 2, bind(&c::XXX, this)},
                 {"PHA", Imp, 3, bind(&c::PHA, this)},
                 {"EOR", Imm, 2, bind(&c::EOR, this)},
                 {"LSR", Accum, 2, bind(&c::LSR, this)},
                 {"XXX", Imp, 2, bind(&c::XXX, this)},
                 {"JMP", Abs, 3, bind(&c::JMP, this)},
                 {"EOR", Abs, 4, bind(&c::EOR, this)},
                 {"LSR", Abs, 6, bind(&c::LSR, this)},
                 {"XXX", Imp, 2, bind(&c::XXX, this)},
                 {"BVC", Rel, 2, bind(&c::BVC, this)},
                 {"EOR", IndY, 5, bind(&c::EOR, this)},
                 {"XXX", Imp, 2, bind(&c::XXX, this)},
                 {"XXX", Imp, 2, bind(&c::XXX, this)},
                 {"XXX", Imp, 2, bind(&c::XXX, this)},
                 {"EOR", ZPX, 4, bind(&c::EOR, this)},
                 {"LSR", ZPX, 6, bind(&c::LSR, this)},
                 {"XXX", Imp, 2, bind(&c::XXX, this)},
                 {"CLI", Imp, 2, bind(&c::CLI, this)},
                 {"EOR", AbsY, 4, bind(&c::EOR, this)},
                 {"XXX", Imp, 2, bind(&c::XXX, this)},
                 {"XXX", Imp, 2, bind(&c::XXX, this)},
                 {"XXX", Imp, 2, bind(&c::XXX, this)},
                 {"EOR", AbsX, 4, bind(&c::EOR, this)},
                 {"LSR", AbsX, 7, bind(&c::LSR, this)},
                 {"XXX", Imp, 2, bind(&c::XXX, this)},
                 {"RTS", Imp, 6, bind(&c::RTS, this)},
                 {"ADC", IndX, 6, bind(&c::ADC, this)},
                 {"XXX", Imp, 2, bind(&c::XXX, this)},
                 {"XXX", Imp, 2, bind(&c::XXX, this)},
                 {"XXX", Imp, 2, bind(&c::XXX, this)},
                 {"ADC", ZP, 3, bind(&c::ADC, this)},
                 {"ROR", ZP, 5, bind(&c::ROR, this)},
                 {"XXX", Imp, 2, bind(&c::XXX, this)},
                 {"PLA", Imp, 4, bind(&c::PLA, this)},
                 {"ADC", Imm, 2, bind(&c::ADC, this)},
                 {"ROR", Accum, 2, bind(&c::ROR, this)},
                 {"XXX", Imp, 2, bind(&c::XXX, this)},
                 {"JMP", Ind, 5, bind(&c::JMP, this)},
                 {"ADC", Abs, 4, bind(&c::ADC, this)},
                 {"ROR", Abs, 5, bind(&c::ROR, this)},
                 {"XXX", Imp, 2, bind(&c::XXX, this)},
                 {"BVS", Rel, 2, bind(&c::BVS, this)},
                 {"ADC", IndY, 5, bind(&c::ADC, this)},
                 {"XXX", Imp, 2, bind(&c::XXX, this)},
                 {"XXX", Imp, 2, bind(&c::XXX, this)},
                 {"XXX", Imp, 2, bind(&c::XXX, this)},
                 {"ADC", ZPX, 4, bind(&c::ADC, this)},
                 {"ROR", ZPX, 6, bind(&c::ROR, this)},
                 {"XXX", Imp, 2, bind(&c::XXX, this)},
                 {"SEI", Imp, 2, bind(&c::SEI, this)},
                 {"ADC", AbsY, 4, bind(&c::ADC, this)},
                 {"XXX", Imp, 2, bind(&c::XXX, this)},
                 {"XXX", Imp, 2, bind(&c::XXX, this)},
                 {"XXX", Imp, 2, bind(&c::XXX, this)},
                 {"ADC", AbsX, 4, bind(&c::ADC, this)},
                 {"ROR", AbsX, 7, bind(&c::ROR, this)},
                 {"XXX", Imp, 2, bind(&c::XXX, this)},
                 {"XXX", Imp, 2, bind(&c::XXX, this)},
                 {"STA", IndX, 6, bind(&c::STA, this)},
                 {"XXX", Imp, 2, bind(&c::XXX, this)},
                 {"XXX", Imp, 2, bind(&c::XXX, this)},
                 {"STY", ZP, 3, bind(&c::STY, this)},
                 {"STA", ZP, 3, bind(&c::STA, this)},
                 {"STX", ZP, 3, bind(&c::STX, this)},
                 {"XXX", Imp, 2, bind(&c::XXX, this)},
                 {"DEY", Imp, 2, bind(&c::DEY, this)},
                 {"XXX", Imp, 2, bind(&c::XXX, this)},
                 {"TXA", Imp, 2, bind(&c::TXA, this)},
                 {"XXX", Imp, 2, bind(&c::XXX, this)},
                 {"STY", Abs, 4, bind(&c::STY, this)},
                 {"STA", Abs, 4, bind(&c::STA, this)},
                 {"STX", Abs, 4, bind(&c::STX, this)},
                 {"XXX", Imp, 2, bind(&c::XXX, this)},
                 {"BCC", Rel, 2, bind(&c::BCC, this)},
                 {"STA", IndY, 6, bind(&c::STA, this)},
                 {"XXX", Imp, 2, bind(&c::XXX, this)},
                 {"XXX", Imp, 2, bind(&c::XXX, this)},
                 {"STY", ZPX, 4, bind(&c::STY, this)},
                 {"STA", ZPX, 4, bind(&c::STA, this)},
                 {"STX", ZPY, 4, bind(&c::STX, this)},
                 {"XXX", Imp, 2, bind(&c::XXX, this)},
                 {"TYA", Imp, 2, bind(&c::TYA, this)},
                 {"STA", AbsY, 5, bind(&c::STA, this)},
                 {"TXS", Imp, 2, bind(&c::TXS, this)},
                 {"XXX", Imp, 2, bind(&c::XXX, this)},
                 {"XXX", Imp, 2, bind(&c::XXX, this)},
                 {"STA", AbsX, 5, bind(&c::STA, this)},
                 {"XXX", Imp, 2, bind(&c::XXX, this)},
                 {"XXX", Imp, 2, bind(&c::XXX, this)},
                 {"LDY", Imm, 2, bind(&c::LDY, this)},
                 {"LDA", IndX, 6, bind(&c::LDA, this)},
                 {"LDX", Imm, 2, bind(&c::LDX, this)},
                 {"XXX", Imp, 2, bind(&c::XXX, this)},
                 {"LDY", ZP, 3, bind(&c::LDY, this)},
                 {"LDA", ZP, 3, bind(&c::LDA, this)},
                 {"LDX", ZP, 3, bind(&c::LDX, this)},
                 {"XXX", Imp, 2, bind(&c::XXX, this)},
                 {"TAY", Imp, 2, bind(&c::TAY, this)},
                 {"LDA", Imm, 2, bind(&c::LDA, this)},
                 {"TAX", Imp, 2, bind(&c::TAX, this)},
                 {"XXX", Imp, 2, bind(&c::XXX, this)},
                 {"LDY", Abs, 4, bind(&c::LDY, this)},
                 {"LDA", Abs, 4, bind(&c::LDA, this)},
                 {"LDX", Abs, 4, bind(&c::LDX, this)},
                 {"XXX", Imp, 2, bind(&c::XXX, this)},
                 {"BCS", Rel, 2, bind(&c::BCS, this)},
                 {"LDA", IndY, 5, bind(&c::LDA, this)},
                 {"XXX", Imp, 2, bind(&c::XXX, this)},
                 {"XXX", Imp, 2, bind(&c::XXX, this)},
                 {"LDY", ZPX, 4, bind(&c::LDY, this)},
                 {"LDA", ZPX, 4, bind(&c::LDX, this)},
                 {"LDX", ZPY, 4, bind(&c::LDX, this)},
                 {"XXX", Imp, 2, bind(&c::XXX, this)},
                 {"CLV", Imp, 2, bind(&c::CLV, this)},
                 {"LDA", AbsY, 4, bind(&c::LDA, this)},
                 {"TSX", Imp, 2, bind(&c::TSX, this)},
                 {"XXX", Imp, 2, bind(&c::XXX, this)},
                 {"LDY", AbsX, 4, bind(&c::LDY, this)},
                 {"LDA", AbsX, 4, bind(&c::LDA, this)},
                 {"LDX", AbsY, 4, bind(&c::LDX, this)},
                 {"XXX", Imp, 2, bind(&c::XXX, this)},
                 {"CPY", Imm, 2, bind(&c::CPY, this)},
                 {"CMP", IndX, 6, bind(&c::CMP, this)},
                 {"XXX", Imp, 2, bind(&c::XXX, this)},
                 {"XXX", Imp, 2, bind(&c::XXX, this)},
                 {"CPY", ZP, 3, bind(&c::CPY, this)},
                 {"CMP", ZP, 3, bind(&c::CMP, this)},
                 {"DEC", ZP, 5, bind(&c::DEC, this)},
                 {"XXX", Imp, 2, bind(&c::XXX, this)},
                 {"INY", Imp, 2, bind(&c::INY, this)},
                 {"CMP", Imm, 2, bind(&c::CMP, this)},
                 {"DEX", Imp, 2, bind(&c::DEX, this)},
                 {"XXX", Imp, 2, bind(&c::XXX, this)},
                 {"CPY", Abs, 4, bind(&c::CPY, this)},
                 {"CMP", Abs, 4, bind(&c::CMP, this)},
                 {"DEC", Abs, 6, bind(&c::DEC, this)},
                 {"XXX", Imp, 2, bind(&c::XXX, this)},
                 {"BNE", Rel, 2, bind(&c::BNE, this)},
                 {"CMP", IndY, 5, bind(&c::CMP, this)},
                 {"XXX", Imp, 2, bind(&c::XXX, this)},
                 {"XXX", Imp, 2, bind(&c::XXX, this)},
                 {"XXX", Imp, 2, bind(&c::XXX, this)},
                 {"CMP", ZPX, 4, bind(&c::CMP, this)},
                 {"DEC", ZPX, 6, bind(&c::DEC, this)},
                 {"XXX", Imp, 2, bind(&c::XXX, this)},
                 {"CLD", Imp, 2, bind(&c::CLD, this)},
                 {"CMP", AbsY, 4, bind(&c::CMP, this)},
                 {"XXX", Imp, 2, bind(&c::XXX, this)},
                 {"XXX", Imp, 2, bind(&c::XXX, this)},
                 {"XXX", Imp, 2, bind(&c::XXX, this)},
                 {"CMP", AbsX, 4, bind(&c::CMP, this)},
                 {"DEC", AbsX, 7, bind(&c::DEC, this)},
                 {"XXX", Imp, 2, bind(&c::XXX, this)},
                 {"CPX", Imm, 2, bind(&c::CPX, this)},
                 {"SBC", IndX, 6, bind(&c::SBC, this)},
                 {"XXX", Imp, 2, bind(&c::XXX, this)},
                 {"XXX", Imp, 2, bind(&c::XXX, this)},
                 {"CPX", ZP, 3, bind(&c::CPX, this)},
                 {"SBC", ZP, 3, bind(&c::SBC, this)},
                 {"INC", ZP, 5, bind(&c::INC, this)},
                 {"XXX", Imp, 2, bind(&c::XXX, this)},
                 {"INX", Imp, 2, bind(&c::INX, this)},
                 {"SBC", Imm, 2, bind(&c::SBC, this)},
                 {"NOP", Imp, 2, bind(&c::NOP, this)},
                 {"XXX", Imp, 2, bind(&c::XXX, this)},
                 {"CPX", Abs, 4, bind(&c::CPX, this)},
                 {"SBC", Abs, 4, bind(&c::SBC, this)},
                 {"INC", Abs, 6, bind(&c::INC, this)},
                 {"XXX", Imp, 2, bind(&c::XXX, this)},
                 {"BEQ", Rel, 2, bind(&c::BEQ, this)},
                 {"SBC", IndY, 5, bind(&c::SBC, this)},
                 {"XXX", Imp, 2, bind(&c::XXX, this)},
                 {"XXX", Imp, 2, bind(&c::XXX, this)},
                 {"XXX", Imp, 2, bind(&c::XXX, this)},
                 {"SBC", ZPX, 4, bind(&c::SBC, this)},
                 {"INC", ZPX, 6, bind(&c::INC, this)},
                 {"XXX", Imp, 2, bind(&c::XXX, this)},
                 {"SED", Imp, 2, bind(&c::SED, this)},
                 {"SBC", Imp, 2, bind(&c::SBC, this)},
                 {"XXX", Imp, 2, bind(&c::XXX, this)},
                 {"XXX", Imp, 2, bind(&c::XXX, this)},
                 {"XXX", Imp, 2, bind(&c::XXX, this)},
                 {"SBC", AbsX, 4, bind(&c::SBC, this)},
                 {"INC", AbsX, 7, bind(&c::INC, this)},
                 {"XXX", Imp, 2, bind(&c::XXX, this)}};
}

void nes6502::clock() {
  if (cycles == 0) {
    opcode = read(pc++);
    INSTR instr = instr_table[opcode];

    // make sure unused flag is always high
    status.flags.U = 1;

    cycles = instr.cycles;

    bool extra_cycle_addrmode = resolveAddress(instr.mode);
    bool extra_cycle_opcode = instr.operate();

    // check if additional cycles are needed
    cycles += (extra_cycle_addrmode && extra_cycle_opcode) ? 1 : 0;

    // just in case :P
    status.flags.U = 1;
  }

  --cycles;
}

void nes6502::reset() {
  Word low = (Word)read(PC_RESET_ADDR);
  Word high = (Word)read(PC_RESET_ADDR + 1);

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

  Word low = (Word)read(IRQ_VECTOR);
  Word high = (Word)read(IRQ_VECTOR + 1);

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

    Word low = (Word)read(IRQ_VECTOR);
    Word high = (Word)read(IRQ_VECTOR + 1);

    pc = (high << 8) | low;

    cycles = 7;
  }
}

void nes6502::write(Word addr, Byte data) { bus_->write(addr, data); }

Byte nes6502::read(Word addr) { return bus_->read(addr); }

bool nes6502::resolveAddress(AddressingMode mode) {

  Word low{};
  Word high{};
  Word addr{};
  Word ptr{};

  Byte ind_low{};

  /* with all of these addressing modes, we keep in mind that pc has already
     been incremented at this point in the cycle. this means that if the mode
     asks for another byte, we just read at the current pc. */
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
    // used with only branch instructions; second byte is an offset that is
    // added to pc when counter is set at next instruction
    addr_rel = (Word)read(pc++);
    if (addr_rel & 0x80) {
      addr_rel |= 0xFF00; // turn negative if MSB is 1 (negative)
    }
    return false;
  case ZP:
    // fetch second byte of instruction and assume zero page high byte
    addr_abs = (Word)read(pc++);
    addr_abs &= 0x00FF;
    return false;
  case ZPX:
    // zero page with x register offset
    addr_abs = (Word)read(pc) + index_x;
    addr_abs &= 0x00FF;
    return false;
  case ZPY:
    // zero page with y register offset
    addr_abs = (Word)read(pc) + index_y;
    addr_abs &= 0x00FF;
    return false;
  case Abs:
    // load a full 16-bit address using the second and third bytes
    low = (Word)read(pc++);
    high = (Word)read(pc++);

    addr_abs = (high << 8) | low;
    return false;
  case AbsX:
    // absolute with x register offset, need to account for page changes
    low = (Word)read(pc++);
    high = (Word)read(pc++);

    addr_abs = ((high << 8) | low) + index_x;

    // needs an additional clock cycle if the page changes (high byte changes
    // from the high byte in memory)
    return ((addr_abs & 0xFF00) != (high << 8));
  case AbsY:
    // absolute with y register offset
    low = (Word)read(pc++);
    high = (Word)read(pc++);

    addr_abs = ((high << 8) | low) + index_x;

    // needs an additional clock cycle if the page changes (high byte changes
    // from the high byte in memory)
    return ((addr_abs & 0xFF00) != (high << 8));
  case Ind:
    // the second byte is the lower byte of a memory address, the third byte is
    // the higher
    low = (Word)read(pc++);
    high = (Word)read(pc++);
    addr = (high << 8) | low;

    // emulate 6502 bug: wrap around page if we cross page boundary
    if ((addr & 0x00FF) == 0x00FF) {
      addr_abs |= read(addr & 0xFF00) << 8;
    }
    // not crossed, normal high bit
    else {
      addr_abs |= read(addr + 1) << 8;
    }

    addr_abs |= read(addr);
    return false;
  case IndX:
    // the second byte of the instruction is added to register x; this points to
    // a memory location on page 0 which contains the effective address
    ind_low = read(pc++);
    addr = ind_low + index_x;

    low = read(addr & 0x00FF);
    high = read((addr + 1) & 0x00FF);

    addr_abs = (((Word)high) << 8) | (Word)low;
    return false;

  case IndY:
    // the second byte stores a pointer on page 0; the contents of memory at the
    // address in pointer are offset by y
    ptr = (Word)read(pc++) & 0x00FF;

    low = (Word)read(ptr & 0x00FF);
    high = (Word)read((ptr + 1) & 0x00FF);

    addr_abs = ((high << 8) | low) + index_y;

    // check for page crossing
    return ((addr_abs & 0xFF00) != (high << 8));
  default:
    return false;
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

// opcodes start
bool nes6502::ADC() { return false; }

bool nes6502::AND() {
  Byte data = fetch();
  accum &= data;

  status.flags.Z = (accum == 0x00) ? 1 : 0;
  status.flags.N = (accum & 0x80) ? 1 : 0;

  return true;
}

bool nes6502::ASL() {
  Word data = (Word)fetch();
  data <<= 1;

  // check for implied mode
  if (instr_table[opcode].mode != Imp) {
    write(addr_abs, (Byte)data & 0x00FF);
  } else {
    accum = data & 0x00FF;
  }

  status.flags.C = ((data & 0xFF00) > 0) ? 1 : 0;
  status.flags.Z = ((data & 0x00FF) == 0) ? 1 : 0;
  status.flags.N = (data & 0x80) ? 1 : 0;

  return false;
}

bool nes6502::BCC() {
  if (!status.flags.C) {
    // add extra cycle if we do branch
    ++cycles;

    Word jump = pc + addr_rel;

    // if page crossed, add another cycle
    if ((jump & 0xFF00) != (pc & 0xFF00)) {
      ++cycles;
    }

    pc = jump;
  }
  return false;
}

bool nes6502::BCS() {
  if (status.flags.C) {
    ++cycles;

    Word jump = pc + addr_rel;

    if ((jump & 0xFF00) != (pc & 0xFF00)) {
      ++cycles;
    }

    pc = jump;
  }
  return false;
}

bool nes6502::BEQ() {
  if (status.flags.Z) {
    ++cycles;

    Word jump = pc + addr_rel;

    if ((jump & 0xFF00) != (pc & 0xFF00)) {
      ++cycles;
    }

    pc = jump;
  }
  return false;
}

bool nes6502::BIT() {
  Byte data = fetch();

  Byte test = accum & data;

  status.flags.Z = (test == 0x00) ? 1 : 0;
  status.flags.V = (test & (1 << 6)) ? 1 : 0;
  status.flags.N = (test & (1 << 7)) ? 1 : 0;

  return false;
}

bool nes6502::BMI() {
  if (status.flags.N) {
    ++cycles;

    Word jump = pc + addr_rel;

    if ((jump & 0xFF00) != (pc & 0xFF00)) {
      ++cycles;
    }

    pc = jump;
  }
  return false;
}

bool nes6502::BNE() {
  if (!status.flags.Z) {
    ++cycles;

    Word jump = pc + addr_rel;

    if ((jump & 0xFF00) != (pc & 0xFF00)) {
      ++cycles;
    }

    pc = jump;
  }
  return false;
}

bool nes6502::BPL() {
  if (!status.flags.N) {
    ++cycles;

    Word jump = pc + addr_rel;

    if ((jump & 0xFF00) != (pc & 0xFF00)) {
      ++cycles;
    }

    pc = jump;
  }
  return false;
}

bool nes6502::BRK() {
  // increment pc to account for unused byte in the BRK instruction
  ++pc;

  write(STK_ADDR_START + stkp--, (pc >> 8) & 0x00FF);
  write(STK_ADDR_START + stkp--, pc & 0x00FF);

  // incur interrupt
  status.flags.I = 1;
  status.flags.B = 1;

  write(STK_ADDR_START + stkp--, status.reg);

  // set pc to contents at memory address 0xFFFE + 0xFFFF
  Word low = read(0xFFFE);
  Word high = read(0xFFFF);

  pc = (high << 8) | low;

  return false;
}

bool nes6502::BVC() {
  if (!status.flags.V) {
    ++cycles;

    Word jump = pc + addr_rel;

    if ((jump & 0xFF00) != (pc & 0xFF00)) {
      ++cycles;
    }

    pc = jump;
  }
  return false;
}

bool nes6502::BVS() {
  if (status.flags.V) {
    ++cycles;

    Word jump = pc + addr_rel;

    if ((jump & 0xFF00) != (pc & 0xFF00)) {
      ++cycles;
    }

    pc = jump;
  }
  return false;
}

bool nes6502::CLC() {
  status.flags.C = 0;
  return false;
}

bool nes6502::CLD() {
  status.flags.D = 0;
  return false;
}

bool nes6502::CLI() {
  status.flags.I = 0;
  return false;
}

bool nes6502::CLV() {
  status.flags.V = 0;
  return false;
}

bool nes6502::CMP() {
  Byte data = fetch();
  Word cmp = (Word)accum - (Word)data;

  status.flags.C = (accum >= data) ? 1 : 0;
  status.flags.Z = (cmp == 0) ? 1 : 0;
  status.flags.N = (cmp & 0x0080) ? 1 : 0;

  return true;
}

bool nes6502::CPX() {
  Byte data = fetch();
  Word cmp = (Word)index_x - (Word)data;

  status.flags.C = (index_x >= data) ? 1 : 0;
  status.flags.Z = (cmp == 0) ? 1 : 0;
  status.flags.N = (cmp & 0x0080) ? 1 : 0;

  return false;
}

bool nes6502::CPY() {
  Byte data = fetch();
  Word cmp = (Word)index_y - (Word)data;

  status.flags.C = (index_y >= data) ? 1 : 0;
  status.flags.Z = (cmp == 0x0000) ? 1 : 0;
  status.flags.N = (cmp & 0x0080) ? 1 : 0;

  return false;
}

bool nes6502::DEC() {
  Byte data = fetch();
  --data;
  write(addr_abs, data);

  status.flags.Z = (data == 0x00) ? 1 : 0;
  status.flags.N = (data & 0x80) ? 1 : 0;

  return false;
}

bool nes6502::DEX() {
  --index_x;

  status.flags.Z = (index_x == 0x00) ? 1 : 0;
  status.flags.N = (index_x & 0x80) ? 1 : 0;

  return false;
}

bool nes6502::DEY() {
  --index_y;

  status.flags.Z = (index_x == 0x00) ? 1 : 0;
  status.flags.N = (index_x & 0x80) ? 1 : 0;

  return false;
}

bool nes6502::EOR() {
  Byte data = fetch();
  accum ^= data;

  status.flags.Z = (accum == 0x00) ? 1 : 0;
  status.flags.N = (accum & 0x80) ? 1 : 0;

  return true;
}

bool nes6502::INC() {
  Byte data = fetch();
  ++data;
  write(addr_abs, data);

  status.flags.Z = (data == 0x00) ? 1 : 0;
  status.flags.N = (data & 0x80) ? 1 : 0;

  return false;
}

bool nes6502::INX() {
  ++index_x;

  status.flags.Z = (index_x == 0x00) ? 1 : 0;
  status.flags.N = (index_x & 0x80) ? 1 : 0;

  return false;
}

bool nes6502::INY() {
  ++index_y;

  status.flags.Z = (index_x == 0x00) ? 1 : 0;
  status.flags.N = (index_x & 0x80) ? 1 : 0;

  return false;
}

bool nes6502::JMP() {
  pc = addr_abs;

  return false;
}

bool nes6502::JSR() {
  // decrement pc because return addr on stack points to one addr before next
  // instr
  --pc;

  write(STK_ADDR_START + stkp--, (pc >> 8) & 0x00FF);
  write(STK_ADDR_START + stkp--, pc & 0x00FF);

  pc = addr_abs;

  return false;
}

bool nes6502::LDA() {
  Byte data = fetch();
  accum = data;

  status.flags.Z = (accum == 0x00) ? 1 : 0;
  status.flags.N = (accum & 0x80) ? 1 : 0;

  return false;
}

bool nes6502::LDX() {
  Byte data = fetch();
  index_x = data;

  status.flags.Z = (index_x == 0x00) ? 1 : 0;
  status.flags.N = (index_x & 0x80) ? 1 : 0;

  return false;
}

bool nes6502::LDY() {
  Byte data = fetch();
  index_y = data;

  status.flags.Z = (index_y == 0x00) ? 1 : 0;
  status.flags.N = (index_y & 0x80) ? 1 : 0;

  return false;
}

bool nes6502::LSR() {
  Word data = (Word)fetch();

  status.flags.C = (data & 0x0001) ? 1 : 0;

  data >>= 1;
  status.flags.Z = (data == 0x0000) ? 1 : 0;
  status.flags.N = (data & 0x0080) ? 1 : 0;

  if (instr_table[opcode].mode == Accum) {
    accum = data & 0x00FF;
  } else {
    write(addr_abs, data & 0x00FF);
  }

  return false;
}

bool nes6502::NOP() { return false; }

bool nes6502::ORA() {
  Byte data = fetch();
  accum |= data;

  status.flags.Z = (accum == 0x00) ? 1 : 0;
  status.flags.N = (accum & 0x80) ? 1 : 0;

  return true;
}

bool nes6502::PHA() {
  write(STK_ADDR_START + stkp--, accum);

  return false;
}

bool nes6502::PHP() {
  status.flags.B = 1;
  write(STK_ADDR_START + stkp--, status.reg);
  status.flags.B = 0;

  return false;
}

bool nes6502::PLA() {
  accum = read(STK_ADDR_START + ++stkp);

  status.flags.Z = (accum == 0x00) ? 1 : 0;
  status.flags.N = (accum & 0x80) ? 1 : 0;

  return false;
}

bool nes6502::PLP() { return false; }

bool nes6502::ROL() {
  Byte data = fetch();

  bool carry = data & 0x80;
  data <<= 1;
  data |= (status.flags.C) ? 1 : 0;
  status.flags.C = (carry) ? 1 : 0;

  status.flags.Z = (data == 0x00) ? 1 : 0;
  status.flags.N = (data & 0x80) ? 1 : 0;

  return false;
}

bool nes6502::ROR() { return false; }

bool nes6502::RTI() { return false; }

bool nes6502::RTS() { return false; }

bool nes6502::SBC() { return false; }

bool nes6502::SEC() { return false; }

bool nes6502::SED() { return false; }

bool nes6502::SEI() { return false; }

bool nes6502::STA() { return false; }

bool nes6502::STX() { return false; }

bool nes6502::STY() { return false; }

bool nes6502::TAX() { return false; }

bool nes6502::TAY() { return false; }

bool nes6502::TSX() { return false; }

bool nes6502::TXA() { return false; }

bool nes6502::TXS() { return false; }

bool nes6502::TYA() { return false; }

bool nes6502::XXX() { return false; }
