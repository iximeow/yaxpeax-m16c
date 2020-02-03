#![no_std]

#![allow(non_snake_case)]

#[cfg(feature="use-serde")]
#[macro_use]
extern crate serde_derive;

use core::fmt;

use yaxpeax_arch::{Arch, Decoder, LengthedInstruction};

#[allow(non_camel_case_types)]
#[derive(Debug, Copy, Clone)]
pub enum Opcode {
    ABS,
    ADC(Size),
    ADCF(Size),
    ADD(Size),
    ADJNZ(Size),
    AND(Size),
    BAND,
    BCLR,
    BMEQ,
    BMGEU,
    BMGT,
    BMGTU,
    BMLE,
    BMLEU,
    BMLTU,
    BMLT,
    BMNO,
    BMGE,
    BMN,
    BMNE,
    BMO,
    BMPZ,
    BNAND,
    BNOR,
    BNOT,
    BNTST,
    BNXOR,
    BOR,
    BRK,
    BSET,
    BTST,
    BTSTC,
    BTSTS,
    BXOR,
    CMP(Size),
    DADC,
    DADD,
    DEC(Size),
    DIV(Size),
    DIVU(Size),
    DIVX(Size),
    DSBB,
    DSUB,
    ENTER,
    EXITD,
    EXTS,
    GE,
    INC(Size),
    INT,
    INTO,
    JEQ,
    JGE,
    JGEU,
    JGT,
    JGTU,
    JLE,
    JLEU,
    JLT,
    JLTU,
    JMPI,
    JMPS,
    JMP(Size),
    JN,
    JNE,
    JNO,
    JO,
    JPZ,
    JSRI,
    JSRS,
    JSR(Size),
    LDC,
    LDCTX,
    LDE,
    LDIPL,
    LT,
    MOV(Size),
    MOVA,
    MOVHH,
    MOVHL,
    MOVLH,
    MOVLL,
    MUL,
    MULU(Size),
    NEG,
    NO,
    NOP,
    NOT(Size),
    OR(Size),
    POP,
    POPC,
    POPM,
    PUSH(Size),
    PUSHA,
    PUSHC,
    PUSHM,
    REIT,
    RMPA(Size),
    ROLC,
    RORC,
    ROT(Size),
    RTS,
    SBB(Size),
    SHA(Size),
    SHL(Size),
    SMOVB(Size),
    SMOVF(Size),
    SSTR(Size),
    STC,
    STCTX,
    STE,
    STNZ,
    STX,
    STZ,
    STZX,
    SUB(Size),
    TST(Size),
    UND,
    WAIT,
    XCHG(Size),
    XOR(Size),
}

impl fmt::Display for Opcode {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match self {
            Opcode::ABS => write!(f, "abs"),
            Opcode::ADC(size) => write!(f, "adc.{}", size),
            Opcode::ADCF(size) => write!(f, "adcf.{}", size),
            Opcode::ADD(size) => write!(f, "add.{}", size),
            Opcode::ADJNZ(size) => write!(f, "adjnz.{}", size),
            Opcode::AND(size) => write!(f, "and.{}", size),
            Opcode::BAND => write!(f, "band"),
            Opcode::BCLR => write!(f, "bclr"),
            Opcode::BMEQ => write!(f, "bmeq"),
            Opcode::BMGEU => write!(f, "bmgeu"),
            Opcode::BMGT => write!(f, "bmgt"),
            Opcode::BMGTU => write!(f, "bmgtu"),
            Opcode::BMLE => write!(f, "bmle"),
            Opcode::BMLEU => write!(f, "bmleu"),
            Opcode::BMLTU => write!(f, "bmltu"),
            Opcode::BMLT => write!(f, "bmlt"),
            Opcode::BMNO => write!(f, "bmno"),
            Opcode::BMGE => write!(f, "bmge"),
            Opcode::BMN => write!(f, "bmn"),
            Opcode::BMNE => write!(f, "bmne"),
            Opcode::BMO => write!(f, "bmo"),
            Opcode::BMPZ => write!(f, "bmpz"),
            Opcode::BNAND => write!(f, "bnand"),
            Opcode::BNOR => write!(f, "bnor"),
            Opcode::BNOT => write!(f, "bnot"),
            Opcode::BNTST => write!(f, "bntst"),
            Opcode::BNXOR => write!(f, "bnxor"),
            Opcode::BOR => write!(f, "bor"),
            Opcode::BRK => write!(f, "brk"),
            Opcode::BSET => write!(f, "bset"),
            Opcode::BTST => write!(f, "btst"),
            Opcode::BTSTC => write!(f, "btstc"),
            Opcode::BTSTS => write!(f, "btsts"),
            Opcode::BXOR => write!(f, "bxor"),
            Opcode::CMP(size) => write!(f, "cmp.{}", size),
            Opcode::DADC => write!(f, "dadc"),
            Opcode::DADD => write!(f, "dadd"),
            Opcode::DEC(size) => write!(f, "dec.{}", size),
            Opcode::DIV(size) => write!(f, "div.{}", size),
            Opcode::DIVU(size) => write!(f, "divu.{}", size),
            Opcode::DIVX(size) => write!(f, "divx.{}", size),
            Opcode::DSBB => write!(f, "dsbb"),
            Opcode::DSUB => write!(f, "dsub"),
            Opcode::ENTER => write!(f, "enter"),
            Opcode::EXITD => write!(f, "exitd"),
            Opcode::EXTS => write!(f, "exts"),
            Opcode::GE => write!(f, "ge"),
            Opcode::INC(size) => write!(f, "inc.{}", size),
            Opcode::INT => write!(f, "int"),
            Opcode::INTO => write!(f, "into"),
            Opcode::JEQ => write!(f, "jeq"),
            Opcode::JGE => write!(f, "jge"),
            Opcode::JGEU => write!(f, "jgeu"),
            Opcode::JGT => write!(f, "jgt"),
            Opcode::JGTU => write!(f, "jgtu"),
            Opcode::JLE => write!(f, "jle"),
            Opcode::JLEU => write!(f, "jleu"),
            Opcode::JLT => write!(f, "jlt"),
            Opcode::JLTU => write!(f, "jltu"),
            Opcode::JMPI => write!(f, "jmpi"),
            Opcode::JMPS => write!(f, "jmps"),
            Opcode::JMP(size) => write!(f, "jmp.{}", size),
            Opcode::JN => write!(f, "jn"),
            Opcode::JNE => write!(f, "jne"),
            Opcode::JNO => write!(f, "jno"),
            Opcode::JO => write!(f, "jo"),
            Opcode::JPZ => write!(f, "jpz"),
            Opcode::JSRI => write!(f, "jsri"),
            Opcode::JSRS => write!(f, "jsrs"),
            Opcode::JSR(size) => write!(f, "jsr.{}", size),
            Opcode::LDC => write!(f, "ldc"),
            Opcode::LDCTX => write!(f, "ldctx"),
            Opcode::LDE => write!(f, "lde"),
            Opcode::LDIPL => write!(f, "ldipl"),
            Opcode::LT => write!(f, "lt"),
            Opcode::MOV(size) => write!(f, "mov.{}", size),
            Opcode::MOVA => write!(f, "mova"),
            Opcode::MOVHH => write!(f, "movhh"),
            Opcode::MOVHL => write!(f, "movhl"),
            Opcode::MOVLH => write!(f, "movlh"),
            Opcode::MOVLL => write!(f, "movll"),
            Opcode::MUL => write!(f, "mul"),
            Opcode::MULU(size) => write!(f, "mulu.{}", size),
            Opcode::NEG => write!(f, "neg"),
            Opcode::NO => write!(f, "no"),
            Opcode::NOP => write!(f, "nop"),
            Opcode::NOT(size) => write!(f, "not.{}", size),
            Opcode::OR(size) => write!(f, "or.{}", size),
            Opcode::POP => write!(f, "pop"),
            Opcode::POPC => write!(f, "popc"),
            Opcode::POPM => write!(f, "popm"),
            Opcode::PUSH(size) => write!(f, "push.{}", size),
            Opcode::PUSHA => write!(f, "pusha"),
            Opcode::PUSHC => write!(f, "pushc"),
            Opcode::PUSHM => write!(f, "pushm"),
            Opcode::REIT => write!(f, "reit"),
            Opcode::RMPA(size) => write!(f, "rmpa.{}", size),
            Opcode::ROLC => write!(f, "rolc"),
            Opcode::RORC => write!(f, "rorc"),
            Opcode::ROT(size) => write!(f, "rot.{}", size),
            Opcode::RTS => write!(f, "rts"),
            Opcode::SBB(size) => write!(f, "sbb.{}", size),
            Opcode::SHA(size) => write!(f, "sha.{}", size),
            Opcode::SHL(size) => write!(f, "shl.{}", size),
            Opcode::SMOVB(size) => write!(f, "smovb.{}", size),
            Opcode::SMOVF(size) => write!(f, "smovf.{}", size),
            Opcode::SSTR(size) => write!(f, "sstr.{}", size),
            Opcode::STC => write!(f, "stc"),
            Opcode::STCTX => write!(f, "stctx"),
            Opcode::STE => write!(f, "ste"),
            Opcode::STNZ => write!(f, "stnz"),
            Opcode::STX => write!(f, "stx"),
            Opcode::STZ => write!(f, "stz"),
            Opcode::STZX => write!(f, "stzx"),
            Opcode::SUB(size) => write!(f, "sub.{}", size),
            Opcode::TST(size) => write!(f, "tst.{}", size),
            Opcode::UND => write!(f, "und"),
            Opcode::WAIT => write!(f, "wait"),
            Opcode::XCHG(size) => write!(f, "xchg.{}", size),
            Opcode::XOR(size) => write!(f, "xor.{}", size),
        }
    }
}

#[derive(Debug, Copy, Clone)]
pub enum Register {
    R0L,
    R0H,
    R1L,
    R1H,
    R0,
    R1,
    R2,
    R3,
    R2R0,
    R3R1,
    A0,
    A1,
    A1A0,
    FB,
    SB,
    SP,
    INTBL,
    INTBH,
    FLG,
    ISP,
}

impl fmt::Display for Register {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        use Register::*;
        match self {
            R0L => write!(f, "r0l"),
            R0H => write!(f, "r0h"),
            R1L => write!(f, "r1l"),
            R1H => write!(f, "r1h"),
            R0 => write!(f, "r0"),
            R1 => write!(f, "r1"),
            R2 => write!(f, "r2"),
            R3 => write!(f, "r3"),
            R2R0 => write!(f, "r2r0"),
            R3R1 => write!(f, "r3r1"),
            A0 => write!(f, "a0"),
            A1 => write!(f, "a1"),
            A1A0 => write!(f, "a1a0"),
            FB => write!(f, "fb"),
            SB => write!(f, "sb"),
            SP => write!(f, "sp"),
            INTBL => write!(f, "intbl"),
            INTBH => write!(f, "intbh"),
            FLG => write!(f, "flg"),
            ISP => write!(f, "isp"),
        }
    }
}

#[derive(Debug, Copy, Clone)]
pub enum Operand {
    Register(Register),
    RegisterBit(Register, u8),
    ImmediateI16(i16),
    ImmediateU16(u16),
    RegDerefBit(Register),
    A0A1DispBit(Register, u16),
    RegDispBit(Register, u16, u8),
    RegDeref(Register, i16),
    AbsoluteBit(u16),
    Absolute(u32),
    JmpAbsolute(u32),
    Displacement(i16),
    JmpDisplacement(i16),
    RegList(u8),
}

impl fmt::Display for Operand {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match self {
            Operand::RegList(list) => {
                let regs = [
                    Register::FB,
                    Register::SB,
                    Register::A1,
                    Register::A0,
                    Register::R3,
                    Register::R2,
                    Register::R1,
                    Register::R0,
                ];

                let mut printed = false;

                for i in 0..8 {
                    if list & (1 << i) != 0 {
                        if printed {
                            write!(f, ", ")?;
                        } else {
                            printed = true;
                        }
                        write!(f, "{}", regs[i as usize])?;
                    }
                }

                Ok(())
            }
            Operand::Register(reg) => write!(f, "{}", reg),
            Operand::ImmediateI16(imm) => {
                if *imm == core::i16::MIN {
                    write!(f, "#-8000h")
                } else if *imm < 0 {
                    write!(f, "#-{:02x}h", -*imm)
                } else {
                    write!(f, "#{:02x}h", *imm)
                }
            },
            Operand::ImmediateU16(imm) => write!(f, "#{:02x}h", imm),
            Operand::RegDerefBit(reg) => {
                write!(f, "bit,[{}]", reg)
            }
            Operand::A0A1DispBit(reg, offset) => {
                write!(f, "{}[{}]", offset, reg)
            }
            Operand::RegDispBit(reg, offset, bit) => {
                write!(f, "{},{}[{}]", bit, offset, reg)
            }
            Operand::RegDeref(reg, offset) => {
                if *offset == core::i16::MIN {
                    write!(f, "#-8000h")
                } else if *offset == 0 {
                    write!(f, "[{}]", reg)
                } else if *offset < 0 {
                    write!(f, "-{}[{}]", -offset, reg)
                } else {
                    write!(f, "{}[{}]", offset, reg)
                }
            }
            Operand::JmpDisplacement(offset) => {
                if *offset == core::i16::MIN {
                    write!(f, "$-32768")
                } else if *offset < 0 {
                    write!(f, "$-{}", -*offset)
                } else {
                    write!(f, "$+{}", *offset)
                }
            }
            Operand::Displacement(offset) => {
                if *offset == core::i16::MIN {
                    write!(f, "[$-32768]")
                } else if *offset < 0 {
                    write!(f, "[$-{}]", -*offset)
                } else {
                    write!(f, "[$+{}]", *offset)
                }
            }
            Operand::Absolute(addr) => {
                write!(f, "[#{:x}h]", addr)
            }
            Operand::JmpAbsolute(addr) => {
                write!(f, "#{:x}h", addr)
            }
            _ => Ok(())
        }
    }
}

#[allow(non_camel_case_types)]
#[derive(Debug, Copy, Clone, PartialEq)]
enum OperandSpec {
    Nothing,
    Zero,
    Imm4,
    Imm8,
    Imm82, // Imm8, part t2. used in STZX.
    Imm16,
    Label8,
    Bit0,
    Bit1,
    Bit2,
    Bit3,
    Bit4,
    Bit5,
    Bit6,
    Bit7,
    R0L,
    R0H,
    R1L,
    R1H,
    R0,
    R1,
    R2,
    R3,
    R2R0,
    R3R1,
    A0,
    A1,
    A1A0,
    Bit_R0,
    Bit_R1,
    Bit_R2,
    Bit_R3,
    Bit_A0,
    Bit_A1,
    Bit_Deref_A0,
    Bit_Deref_A1,
    Bit_Disp8_A0,
    Bit_Disp8_A1,
    Bit_Disp8_SB,
    Bit_Disp8_FB,
    Bit_Disp16_A0,
    Bit_Disp16_A1,
    Bit_Disp16_SB,
    Bit_Abs16,
    Disp8_A0,
    Disp8_A1,
    Disp8_SB,
    Disp8_FB,
    Disp8_SP,
    Disp16_A0,
    Disp16_A1,
    Disp16_SB,
    Disp20_A0,
    Disp20_A1,
    Deref_A0,
    Deref_A1,
    Deref_A1A0,
    Abs16,
    Disp2_8_A0,
    Disp2_8_A1,
    Disp2_8_SB,
    Disp2_8_FB,
    Disp2_16_A0,
    Disp2_16_A1,
    Disp2_16_SB,
    Abs2_16,
    Abs20,
    JmpAbs20,
    Disp8,
    JmpDisp8,
    JmpDisp16,
    RegList,
    INTBL,
    INTBH,
    FLG,
    ISP,
    SP,
    SB,
    FB,
}

#[derive(Debug, Copy, Clone, PartialEq)]
pub enum Size {
    B, // 0 in almost(?) all places a size bit is present
    W, // 1 in almost(?) all places a size bit is present
    L, // for sha.l/shl.l
    S, // for jmp.s, three bits.
    A, // for jmp.a, twenty bit absolute.
}

impl fmt::Display for Size {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match self {
            Size::B => write!(f, "b"),
            Size::W => write!(f, "w"),
            Size::L => write!(f, "l"),
            Size::S => write!(f, "s"),
            Size::A => write!(f, "a"),
        }
    }
}

impl Size {
    fn as_bytes(&self) -> u8 {
        match self {
            Size::B => 1,
            Size::W => 2,
            Size::L => { panic!("should not have to get number of bytes for L"); },
            Size::S => { panic!("should not have to get number of bytes for S"); },
            Size::A => { panic!("should not have to get number of bytes for A"); },
        }
    }
}

#[derive(Debug, Copy, Clone)]
pub struct Instruction {
    pub opcode: Opcode,
    operands: [OperandSpec; 3],
    // we need both `dispabs` and `imm_wide` to handle the widest instructions we can see:
    // LDE or STE with 20-bit displacement and 16-bit displacement.
    // either an Imm16, an Imm8, or Imm82:Imm8. mutually exclusive with 20-bit disp/abs, so put those here too.
    //
    // Imm4 will sometimes cohabitate with two displacements (ADJNZ with displacement dest and
    // label), so cram the label byte into here as well. accessed as `OperandSpec::Label8`.
    imm_wide: u32,
    dispabs: u16, // 8 or 16-bit displacement or absolute address
    length: u8,
}

impl Instruction {
    pub fn operand(&self, idx: u8) -> Option<Operand> {
        use OperandSpec::*;
        let operand = match self.operands[idx as usize] {
            Nothing => { return None; },
            Zero => Operand::ImmediateU16(0),
            Imm4 => Operand::ImmediateI16(self.imm_wide as u8 as i8 as i16),
            Imm8 => Operand::ImmediateI16(self.imm_wide as u8 as i8 as i16),
            Imm16 => Operand::ImmediateI16(self.imm_wide as i16),
            // Imm8 part 2 is used in STZX, and is the second byte of imm_wide
            Imm82 |
            // and Label8 is used in ADJNZ, also stored in the second byte of inn_wide
            Label8 => Operand::ImmediateI16((self.imm_wide as i16) >> 8),
            Bit0 => Operand::ImmediateU16(0),
            Bit1 => Operand::ImmediateU16(1),
            Bit2 => Operand::ImmediateU16(2),
            Bit3 => Operand::ImmediateU16(3),
            Bit4 => Operand::ImmediateU16(4),
            Bit5 => Operand::ImmediateU16(5),
            Bit6 => Operand::ImmediateU16(6),
            Bit7 => Operand::ImmediateU16(7),
            R0L => Operand::Register(Register::R0L),
            R0H => Operand::Register(Register::R0H),
            R1L => Operand::Register(Register::R1L),
            R1H => Operand::Register(Register::R1H),
            R0 => Operand::Register(Register::R0),
            R1 => Operand::Register(Register::R1),
            R2 => Operand::Register(Register::R2),
            R3 => Operand::Register(Register::R3),
            R2R0 => Operand::Register(Register::R2R0),
            R3R1 => Operand::Register(Register::R3R1),
            A0 => Operand::Register(Register::A0),
            A1 => Operand::Register(Register::A1),
            A1A0 => Operand::Register(Register::A1A0),
            Bit_R0 => Operand::RegisterBit(Register::R0, self.dispabs as u8),
            Bit_R1 => Operand::RegisterBit(Register::R1, self.dispabs as u8),
            Bit_R2 => Operand::RegisterBit(Register::R2, self.dispabs as u8),
            Bit_R3 => Operand::RegisterBit(Register::R3, self.dispabs as u8),
            Bit_A0 => Operand::RegisterBit(Register::A0, self.dispabs as u8),
            Bit_A1 => Operand::RegisterBit(Register::A1, self.dispabs as u8),
            Bit_Deref_A0 => Operand::RegDerefBit(Register::A0),
            Bit_Deref_A1 => Operand::RegDerefBit(Register::A1),
            Bit_Disp8_A0 => Operand::A0A1DispBit(Register::A0, self.dispabs as u8 as u16),
            Bit_Disp8_A1 => Operand::A0A1DispBit(Register::A1, self.dispabs as u8 as u16),
            Bit_Disp8_SB => Operand::RegDispBit(Register::SB, (self.dispabs as u8 >> 3) as u16, self.dispabs as u8 & 0b111),
            Bit_Disp8_FB => Operand::RegDispBit(Register::FB, (self.dispabs as u8 >> 3) as u16, self.dispabs as u8 & 0b111),
            Bit_Disp16_A0 => Operand::A0A1DispBit(Register::A0, self.dispabs as u16),
            Bit_Disp16_A1 => Operand::A0A1DispBit(Register::A1, self.dispabs as u16),
            Bit_Disp16_SB => Operand::RegDispBit(Register::SB, self.dispabs as u16 >> 4, self.dispabs as u8 & 0b1111),
            Bit_Abs16 => Operand::AbsoluteBit(self.dispabs),
            Disp8_A0 => Operand::RegDeref(Register::A0, self.dispabs as i8 as i16),
            Disp8_A1 => Operand::RegDeref(Register::A1, self.dispabs as i8 as i16),
            Disp8_SB => Operand::RegDeref(Register::SB, self.dispabs as i8 as i16),
            Disp8_FB => Operand::RegDeref(Register::FB, self.dispabs as i8 as i16),
            Disp8_SP => Operand::RegDeref(Register::SP, self.dispabs as i8 as i16),
            Disp16_A0 => Operand::RegDeref(Register::A0, self.dispabs as i16),
            Disp16_A1 => Operand::RegDeref(Register::A1, self.dispabs as i16),
            Disp16_SB => Operand::RegDeref(Register::SB, self.dispabs as i16),
            Disp20_A0 => Operand::RegDeref(Register::A0, self.imm_wide as i16),
            Disp20_A1 => Operand::RegDeref(Register::A1, self.imm_wide as i16),
            Deref_A0 => Operand::RegDeref(Register::A0, self.dispabs as i16),
            Deref_A1 => Operand::RegDeref(Register::A1, self.dispabs as i16),
            Deref_A1A0 => Operand::RegDeref(Register::A1A0, self.dispabs as i16),
            Abs16 => Operand::Absolute(self.dispabs as u32),
            Disp2_8_A0 => Operand::RegDeref(Register::A0, self.imm_wide as i8 as i16),
            Disp2_8_A1 => Operand::RegDeref(Register::A1, self.imm_wide as i8 as i16),
            Disp2_8_SB => Operand::RegDeref(Register::SB, self.imm_wide as i8 as i16),
            Disp2_8_FB => Operand::RegDeref(Register::FB, self.imm_wide as i8 as i16),
            Disp2_16_A0 => Operand::RegDeref(Register::A0, self.imm_wide as i16),
            Disp2_16_A1 => Operand::RegDeref(Register::A1, self.imm_wide as i16),
            Disp2_16_SB => Operand::RegDeref(Register::SB, self.imm_wide as i16),
            Abs2_16 => Operand::Absolute(self.imm_wide),
            Abs20 => Operand::Absolute(self.imm_wide),
            JmpAbs20 => Operand::JmpAbsolute(self.imm_wide),
            Disp8 => Operand::Displacement(self.dispabs as u8 as i8 as i16),
            JmpDisp8 => Operand::JmpDisplacement(self.dispabs as u8 as i8 as i16),
            JmpDisp16 => Operand::JmpDisplacement(self.dispabs as i16),
            RegList => Operand::RegList(self.imm_wide as u8),
            INTBL => Operand::Register(Register::INTBL),
            INTBH => Operand::Register(Register::INTBH),
            FLG => Operand::Register(Register::FLG),
            ISP => Operand::Register(Register::ISP),
            SP => Operand::Register(Register::SP),
            SB => Operand::Register(Register::SB),
            FB => Operand::Register(Register::FB),
        };
        Some(operand)
    }
}

#[cfg(feature="use-serde")]
#[derive(Debug, Serialize, Deserialize)]
pub struct M16C;

#[cfg(not(feature="use-serde"))]
#[derive(Debug)]
pub struct M16C;

impl Arch for M16C {
    type Address = u32;
    type Instruction = Instruction;
    type DecodeError = DecodeError;
    type Decoder = InstDecoder;
    type Operand = Operand;
}

impl fmt::Display for Instruction {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "{}", self.opcode)?;
        match self.operand(0) {
            None => return Ok(()),
            Some(op) => { write!(f, " {}", op)?; }
        }
        match self.operand(1) {
            None => return Ok(()),
            Some(op) => { write!(f, ", {}", op)?; }
        }
        Ok(())
    }
}

impl LengthedInstruction for Instruction {
    type Unit = <M16C as Arch>::Address;

    fn min_size() -> Self::Unit {
        1
    }
    fn len(&self) -> Self::Unit {
        self.length as u32
    }
}

impl Default for Instruction {
    fn default() -> Self {
        Instruction {
            opcode: Opcode::NOP,
            operands: [OperandSpec::Nothing, OperandSpec::Nothing, OperandSpec::Nothing],
            imm_wide: 0,
            dispabs: 0,
            length: 0
        }
    }
}

#[derive(Debug, PartialEq)]
pub enum DecodeError {
    ExhaustedInput,
    InvalidOpcode,
    InvalidOperand,
}

impl fmt::Display for DecodeError {
    fn fmt(&self, f:  &mut fmt::Formatter) -> fmt::Result {
        match self {
            DecodeError::ExhaustedInput => write!(f, "exhausted input"),
            DecodeError::InvalidOpcode => write!(f, "invalid opcode"),
            DecodeError::InvalidOperand => write!(f, "invalid operand"),
        }
    }
}

impl yaxpeax_arch::DecodeError for DecodeError {
    fn data_exhausted(&self) -> bool { self == &DecodeError::ExhaustedInput }
    fn bad_opcode(&self) -> bool { self == &DecodeError::InvalidOpcode }
    fn bad_operand(&self) -> bool { self == &DecodeError::InvalidOperand }
}

impl yaxpeax_arch::Instruction for Instruction {
    // currently only accept instructions that are well-defined.
    fn well_defined(&self) -> bool { true }
}

#[derive(Default, Debug)]
pub struct InstDecoder {}

impl fmt::Display for InstDecoder {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "m16c")
    }
}

impl Decoder<Instruction> for InstDecoder {
    type Error = DecodeError;

    fn decode_into<T: IntoIterator<Item=u8>>(&self, inst: &mut Instruction, bytes: T) -> Result<(), DecodeError> {
        let mut bytes_iter = bytes.into_iter();
        decode(self, inst, &mut bytes_iter)
    }
}

enum OperandCategory {
    ADJNZ,
    Op74,
    Op76,
    Op78,
    Op7A,
    Op7B,
    Op7C,
    Op7D,
    Op7E,
    OpEB,
    SrcDestRegOrDeref,
    Imm4Dest,
    JmpDispOpcodeLow3,
}

enum OperandInterpretation {
    Just([OperandSpec; 3]),
    Reinterpret(OperandCategory)
}

fn decode<T: Iterator<Item=u8>>(_decoder: &InstDecoder, inst: &mut Instruction, bytes: &mut T) -> Result<(), DecodeError> {
    let byte = bytes.next().ok_or(DecodeError::ExhaustedInput)?;
    inst.length += 1;
    // IF there is a size bit, it is the low bit of the first byte.
    let size = if byte & 1 == 0 {
        Size::B
    } else {
        Size::W
    };
    use OperandInterpretation::*;
    let (opcode, interpretation) = match byte {
        0b0000_0000 => (Opcode::BRK, Just([OperandSpec::Nothing, OperandSpec::Nothing, OperandSpec::Nothing])),
        0b0000_0001 => (Opcode::MOV(Size::B), Just([OperandSpec::R0L, OperandSpec::Disp8_SB, OperandSpec::Nothing])),
        0b0000_0010 => (Opcode::MOV(Size::B), Just([OperandSpec::R0L, OperandSpec::Disp8_FB, OperandSpec::Nothing])),
        0b0000_0011 => (Opcode::MOV(Size::B), Just([OperandSpec::R0L, OperandSpec::Abs16, OperandSpec::Nothing])),
        0b0000_0100 => (Opcode::NOP, Just([OperandSpec::Nothing, OperandSpec::Nothing, OperandSpec::Nothing])),
        0b0000_0101 => (Opcode::MOV(Size::B), Just([OperandSpec::R0H, OperandSpec::Disp8_SB, OperandSpec::Nothing])),
        0b0000_0110 => (Opcode::MOV(Size::B), Just([OperandSpec::R0H, OperandSpec::Disp8_FB, OperandSpec::Nothing])),
        0b0000_0111 => (Opcode::MOV(Size::B), Just([OperandSpec::R0H, OperandSpec::Abs16, OperandSpec::Nothing])),
        0b0000_1000 => (Opcode::MOV(Size::B), Just([OperandSpec::R0H, OperandSpec::R0L, OperandSpec::Nothing])),
        0b0000_1001 => (Opcode::MOV(Size::B), Just([OperandSpec::Disp8_SB, OperandSpec::R0L, OperandSpec::Nothing])),
        0b0000_1010 => (Opcode::MOV(Size::B), Just([OperandSpec::Disp8_FB, OperandSpec::R0L, OperandSpec::Nothing])),
        0b0000_1011 => (Opcode::MOV(Size::B), Just([OperandSpec::Abs16, OperandSpec::R0L, OperandSpec::Nothing])),
        0b0000_1100 => (Opcode::MOV(Size::B), Just([OperandSpec::R0L, OperandSpec::R0H, OperandSpec::Nothing])),
        0b0000_1101 => (Opcode::MOV(Size::B), Just([OperandSpec::Disp8_SB, OperandSpec::R0H, OperandSpec::Nothing])),
        0b0000_1110 => (Opcode::MOV(Size::B), Just([OperandSpec::Disp8_FB, OperandSpec::R0H, OperandSpec::Nothing])),
        0b0000_1111 => (Opcode::MOV(Size::B), Just([OperandSpec::Abs16, OperandSpec::R0H, OperandSpec::Nothing])),
        0b0001_0000 => (Opcode::AND(Size::B), Just([OperandSpec::R0H, OperandSpec::R0L, OperandSpec::Nothing])), // src is written R0L/R0H, what picks the register?
        0b0001_0001 => (Opcode::AND(Size::B), Just([OperandSpec::Disp8_SB, OperandSpec::R0L, OperandSpec::Nothing])),
        0b0001_0010 => (Opcode::AND(Size::B), Just([OperandSpec::Disp8_FB, OperandSpec::R0L, OperandSpec::Nothing])),
        0b0001_0011 => (Opcode::AND(Size::B), Just([OperandSpec::Abs16, OperandSpec::R0L, OperandSpec::Nothing])),
        0b0001_0100 => (Opcode::AND(Size::B), Just([OperandSpec::R0L, OperandSpec::R0H, OperandSpec::Nothing])), // src is written R0L/R0H, what picks the register?
        0b0001_0101 => (Opcode::AND(Size::B), Just([OperandSpec::Disp8_SB, OperandSpec::R0H, OperandSpec::Nothing])),
        0b0001_0110 => (Opcode::AND(Size::B), Just([OperandSpec::Disp8_FB, OperandSpec::R0H, OperandSpec::Nothing])),
        0b0001_0111 => (Opcode::AND(Size::B), Just([OperandSpec::Abs16, OperandSpec::R0H, OperandSpec::Nothing])),
        0b0001_1000 => (Opcode::OR(Size::B), Just([OperandSpec::R0H, OperandSpec::R0L, OperandSpec::Nothing])), // src is written R0L/R0H, what picks the register?
        0b0001_1001 => (Opcode::OR(Size::B), Just([OperandSpec::Disp8_SB, OperandSpec::R0L, OperandSpec::Nothing])),
        0b0001_1010 => (Opcode::OR(Size::B), Just([OperandSpec::Disp8_FB, OperandSpec::R0L, OperandSpec::Nothing])),
        0b0001_1011 => (Opcode::OR(Size::B), Just([OperandSpec::Abs16, OperandSpec::R0L, OperandSpec::Nothing])),
        0b0001_1100 => (Opcode::OR(Size::B), Just([OperandSpec::R0L, OperandSpec::R0H, OperandSpec::Nothing])), // src is written R0L/R0H, what picks the register?
        0b0001_1101 => (Opcode::OR(Size::B), Just([OperandSpec::Disp8_SB, OperandSpec::R0H, OperandSpec::Nothing])),
        0b0001_1110 => (Opcode::OR(Size::B), Just([OperandSpec::Disp8_FB, OperandSpec::R0H, OperandSpec::Nothing])),
        0b0001_1111 => (Opcode::OR(Size::B), Just([OperandSpec::Abs16, OperandSpec::R0H, OperandSpec::Nothing])),
        0b0010_0000 => (Opcode::ADD(Size::B), Just([OperandSpec::R0H, OperandSpec::R0L, OperandSpec::Nothing])), // src is written R0L/R0H, what picks the register?
        0b0010_0001 => (Opcode::ADD(Size::B), Just([OperandSpec::Disp8_SB, OperandSpec::R0L, OperandSpec::Nothing])),
        0b0010_0010 => (Opcode::ADD(Size::B), Just([OperandSpec::Disp8_FB, OperandSpec::R0L, OperandSpec::Nothing])),
        0b0010_0011 => (Opcode::ADD(Size::B), Just([OperandSpec::Abs16, OperandSpec::R0L, OperandSpec::Nothing])),
        0b0010_0100 => (Opcode::ADD(Size::B), Just([OperandSpec::R0L, OperandSpec::R0H, OperandSpec::Nothing])), // src is written R0L/R0H, what picks the register?
        0b0010_0101 => (Opcode::ADD(Size::B), Just([OperandSpec::Disp8_SB, OperandSpec::R0H, OperandSpec::Nothing])),
        0b0010_0110 => (Opcode::ADD(Size::B), Just([OperandSpec::Disp8_FB, OperandSpec::R0H, OperandSpec::Nothing])),
        0b0010_0111 => (Opcode::ADD(Size::B), Just([OperandSpec::Abs16, OperandSpec::R0H, OperandSpec::Nothing])),
        0b0010_1000 => (Opcode::SUB(Size::B), Just([OperandSpec::R0H, OperandSpec::R0L, OperandSpec::Nothing])), // src is written R0L/R0H, what picks the register?
        0b0010_1001 => (Opcode::SUB(Size::B), Just([OperandSpec::Disp8_SB, OperandSpec::R0L, OperandSpec::Nothing])),
        0b0010_1010 => (Opcode::SUB(Size::B), Just([OperandSpec::Disp8_FB, OperandSpec::R0L, OperandSpec::Nothing])),
        0b0010_1011 => (Opcode::SUB(Size::B), Just([OperandSpec::Abs16, OperandSpec::R0L, OperandSpec::Nothing])),
        0b0010_1100 => (Opcode::SUB(Size::B), Just([OperandSpec::R0L, OperandSpec::R0H, OperandSpec::Nothing])), // src is written R0L/R0H, what picks the register?
        0b0010_1101 => (Opcode::SUB(Size::B), Just([OperandSpec::Disp8_SB, OperandSpec::R0H, OperandSpec::Nothing])),
        0b0010_1110 => (Opcode::SUB(Size::B), Just([OperandSpec::Disp8_FB, OperandSpec::R0H, OperandSpec::Nothing])),
        0b0010_1111 => (Opcode::SUB(Size::B), Just([OperandSpec::Abs16, OperandSpec::R0H, OperandSpec::Nothing])),
        0b0011_0000 => (Opcode::MOV(Size::B), Just([OperandSpec::R0H, OperandSpec::A0, OperandSpec::Nothing])), // src is written R0L/R0H, what picks the register?
        0b0011_0001 => (Opcode::MOV(Size::B), Just([OperandSpec::Disp8_SB, OperandSpec::A0, OperandSpec::Nothing])),
        0b0011_0010 => (Opcode::MOV(Size::B), Just([OperandSpec::Disp8_FB, OperandSpec::A0, OperandSpec::Nothing])),
        0b0011_0011 => (Opcode::MOV(Size::B), Just([OperandSpec::Abs16, OperandSpec::A0, OperandSpec::Nothing])),
        0b0011_0100 => (Opcode::MOV(Size::B), Just([OperandSpec::R0L, OperandSpec::A1, OperandSpec::Nothing])), // src is written R0L/R0H, what picks the register?
        0b0011_0101 => (Opcode::MOV(Size::B), Just([OperandSpec::Disp8_SB, OperandSpec::A1, OperandSpec::Nothing])),
        0b0011_0110 => (Opcode::MOV(Size::B), Just([OperandSpec::Disp8_FB, OperandSpec::A1, OperandSpec::Nothing])),
        0b0011_0111 => (Opcode::MOV(Size::B), Just([OperandSpec::Abs16, OperandSpec::A1, OperandSpec::Nothing])),
        0b0011_1000 => (Opcode::CMP(Size::B), Just([OperandSpec::R0H, OperandSpec::R0L, OperandSpec::Nothing])), // src is written R0L/R0H, what picks the register?
        0b0011_1001 => (Opcode::CMP(Size::B), Just([OperandSpec::Disp8_SB, OperandSpec::R0L, OperandSpec::Nothing])),
        0b0011_1010 => (Opcode::CMP(Size::B), Just([OperandSpec::Disp8_FB, OperandSpec::R0L, OperandSpec::Nothing])),
        0b0011_1011 => (Opcode::CMP(Size::B), Just([OperandSpec::Abs16, OperandSpec::R0L, OperandSpec::Nothing])),
        0b0011_1100 => (Opcode::CMP(Size::B), Just([OperandSpec::R0L, OperandSpec::R0H, OperandSpec::Nothing])), // src is written R0L/R0H, what picks the register?
        0b0011_1101 => (Opcode::CMP(Size::B), Just([OperandSpec::Disp8_SB, OperandSpec::R0H, OperandSpec::Nothing])),
        0b0011_1110 => (Opcode::CMP(Size::B), Just([OperandSpec::Disp8_FB, OperandSpec::R0H, OperandSpec::Nothing])),
        0b0011_1111 => (Opcode::CMP(Size::B), Just([OperandSpec::Abs16, OperandSpec::R0H, OperandSpec::Nothing])),
        0b0100_0000 => (Opcode::BCLR, Just([OperandSpec::Bit0, OperandSpec::Disp8_SB, OperandSpec::Nothing])),
        0b0100_0001 => (Opcode::BCLR, Just([OperandSpec::Bit1, OperandSpec::Disp8_SB, OperandSpec::Nothing])),
        0b0100_0010 => (Opcode::BCLR, Just([OperandSpec::Bit2, OperandSpec::Disp8_SB, OperandSpec::Nothing])),
        0b0100_0011 => (Opcode::BCLR, Just([OperandSpec::Bit3, OperandSpec::Disp8_SB, OperandSpec::Nothing])),
        0b0100_0100 => (Opcode::BCLR, Just([OperandSpec::Bit4, OperandSpec::Disp8_SB, OperandSpec::Nothing])),
        0b0100_0101 => (Opcode::BCLR, Just([OperandSpec::Bit5, OperandSpec::Disp8_SB, OperandSpec::Nothing])),
        0b0100_0110 => (Opcode::BCLR, Just([OperandSpec::Bit6, OperandSpec::Disp8_SB, OperandSpec::Nothing])),
        0b0100_0111 => (Opcode::BCLR, Just([OperandSpec::Bit7, OperandSpec::Disp8_SB, OperandSpec::Nothing])),
        0b0100_1000 => (Opcode::BSET, Just([OperandSpec::Bit0, OperandSpec::Disp8_SB, OperandSpec::Nothing])),
        0b0100_1001 => (Opcode::BSET, Just([OperandSpec::Bit1, OperandSpec::Disp8_SB, OperandSpec::Nothing])),
        0b0100_1010 => (Opcode::BSET, Just([OperandSpec::Bit2, OperandSpec::Disp8_SB, OperandSpec::Nothing])),
        0b0100_1011 => (Opcode::BSET, Just([OperandSpec::Bit3, OperandSpec::Disp8_SB, OperandSpec::Nothing])),
        0b0100_1100 => (Opcode::BSET, Just([OperandSpec::Bit4, OperandSpec::Disp8_SB, OperandSpec::Nothing])),
        0b0100_1101 => (Opcode::BSET, Just([OperandSpec::Bit5, OperandSpec::Disp8_SB, OperandSpec::Nothing])),
        0b0100_1110 => (Opcode::BSET, Just([OperandSpec::Bit6, OperandSpec::Disp8_SB, OperandSpec::Nothing])),
        0b0100_1111 => (Opcode::BSET, Just([OperandSpec::Bit7, OperandSpec::Disp8_SB, OperandSpec::Nothing])),
        0b0101_0000 => (Opcode::BNOT, Just([OperandSpec::Bit0, OperandSpec::Disp8_SB, OperandSpec::Nothing])),
        0b0101_0001 => (Opcode::BNOT, Just([OperandSpec::Bit1, OperandSpec::Disp8_SB, OperandSpec::Nothing])),
        0b0101_0010 => (Opcode::BNOT, Just([OperandSpec::Bit2, OperandSpec::Disp8_SB, OperandSpec::Nothing])),
        0b0101_0011 => (Opcode::BNOT, Just([OperandSpec::Bit3, OperandSpec::Disp8_SB, OperandSpec::Nothing])),
        0b0101_0100 => (Opcode::BNOT, Just([OperandSpec::Bit4, OperandSpec::Disp8_SB, OperandSpec::Nothing])),
        0b0101_0101 => (Opcode::BNOT, Just([OperandSpec::Bit5, OperandSpec::Disp8_SB, OperandSpec::Nothing])),
        0b0101_0110 => (Opcode::BNOT, Just([OperandSpec::Bit6, OperandSpec::Disp8_SB, OperandSpec::Nothing])),
        0b0101_0111 => (Opcode::BNOT, Just([OperandSpec::Bit7, OperandSpec::Disp8_SB, OperandSpec::Nothing])),
        0b0101_1000 => (Opcode::BTST, Just([OperandSpec::Bit0, OperandSpec::Disp8_SB, OperandSpec::Nothing])),
        0b0101_1001 => (Opcode::BTST, Just([OperandSpec::Bit1, OperandSpec::Disp8_SB, OperandSpec::Nothing])),
        0b0101_1010 => (Opcode::BTST, Just([OperandSpec::Bit2, OperandSpec::Disp8_SB, OperandSpec::Nothing])),
        0b0101_1011 => (Opcode::BTST, Just([OperandSpec::Bit3, OperandSpec::Disp8_SB, OperandSpec::Nothing])),
        0b0101_1100 => (Opcode::BTST, Just([OperandSpec::Bit4, OperandSpec::Disp8_SB, OperandSpec::Nothing])),
        0b0101_1101 => (Opcode::BTST, Just([OperandSpec::Bit5, OperandSpec::Disp8_SB, OperandSpec::Nothing])),
        0b0101_1110 => (Opcode::BTST, Just([OperandSpec::Bit6, OperandSpec::Disp8_SB, OperandSpec::Nothing])),
        0b0101_1111 => (Opcode::BTST, Just([OperandSpec::Bit7, OperandSpec::Disp8_SB, OperandSpec::Nothing])),
        0b0110_0000 => (Opcode::JMP(Size::S), Reinterpret(OperandCategory::JmpDispOpcodeLow3)),
        0b0110_0001 => (Opcode::JMP(Size::S), Reinterpret(OperandCategory::JmpDispOpcodeLow3)),
        0b0110_0010 => (Opcode::JMP(Size::S), Reinterpret(OperandCategory::JmpDispOpcodeLow3)),
        0b0110_0011 => (Opcode::JMP(Size::S), Reinterpret(OperandCategory::JmpDispOpcodeLow3)),
        0b0110_0100 => (Opcode::JMP(Size::S), Reinterpret(OperandCategory::JmpDispOpcodeLow3)),
        0b0110_0101 => (Opcode::JMP(Size::S), Reinterpret(OperandCategory::JmpDispOpcodeLow3)),
        0b0110_0110 => (Opcode::JMP(Size::S), Reinterpret(OperandCategory::JmpDispOpcodeLow3)),
        0b0110_0111 => (Opcode::JMP(Size::S), Reinterpret(OperandCategory::JmpDispOpcodeLow3)),
        0b0110_1000 => (Opcode::JGEU, Just([OperandSpec::JmpDisp8, OperandSpec::Nothing, OperandSpec::Nothing])),
        0b0110_1001 => (Opcode::JGTU, Just([OperandSpec::JmpDisp8, OperandSpec::Nothing, OperandSpec::Nothing])),
        0b0110_1010 => (Opcode::JEQ, Just([OperandSpec::JmpDisp8, OperandSpec::Nothing, OperandSpec::Nothing])),
        0b0110_1011 => (Opcode::JN, Just([OperandSpec::JmpDisp8, OperandSpec::Nothing, OperandSpec::Nothing])),
        0b0110_1100 => (Opcode::JLTU, Just([OperandSpec::JmpDisp8, OperandSpec::Nothing, OperandSpec::Nothing])),
        0b0110_1101 => (Opcode::JLEU, Just([OperandSpec::JmpDisp8, OperandSpec::Nothing, OperandSpec::Nothing])),
        0b0110_1110 => (Opcode::JNE, Just([OperandSpec::JmpDisp8, OperandSpec::Nothing, OperandSpec::Nothing])),
        0b0110_1111 => (Opcode::JPZ, Just([OperandSpec::JmpDisp8, OperandSpec::Nothing, OperandSpec::Nothing])),
        0b0111_0000 => (Opcode::MULU(size), Reinterpret(OperandCategory::SrcDestRegOrDeref)),
        0b0111_0001 => (Opcode::MULU(size), Reinterpret(OperandCategory::SrcDestRegOrDeref)),
        0b0111_0010 => (Opcode::MOV(size), Reinterpret(OperandCategory::SrcDestRegOrDeref)),
        0b0111_0011 => (Opcode::MOV(size), Reinterpret(OperandCategory::SrcDestRegOrDeref)),
        0b0111_0100 => (Opcode::NOP, Reinterpret(OperandCategory::Op74)),
        0b0111_0101 => (Opcode::NOP, Reinterpret(OperandCategory::Op74)),
        0b0111_0110 => (Opcode::NOP, Reinterpret(OperandCategory::Op76)),
        0b0111_0111 => (Opcode::NOP, Reinterpret(OperandCategory::Op76)),
        0b0111_1000 => (Opcode::NOP, Reinterpret(OperandCategory::Op78)),
        0b0111_1001 => (Opcode::NOP, Reinterpret(OperandCategory::Op78)),
        0b0111_1010 => (Opcode::NOP, Reinterpret(OperandCategory::Op7A)),
        0b0111_1011 => (Opcode::NOP, Reinterpret(OperandCategory::Op7B)),
        0b0111_1100 => (Opcode::NOP, Reinterpret(OperandCategory::Op7C)),
        0b0111_1101 => (Opcode::NOP, Reinterpret(OperandCategory::Op7D)),
        0b0111_1110 => (Opcode::NOP, Reinterpret(OperandCategory::Op7E)),
        0b0111_1111 => { return Err(DecodeError::InvalidOperand); },
        0b1000_0000 => (Opcode::TST(size), Reinterpret(OperandCategory::SrcDestRegOrDeref)),
        0b1000_0001 => (Opcode::TST(size), Reinterpret(OperandCategory::SrcDestRegOrDeref)),
        0b1000_0010 => (Opcode::PUSH(Size::B), Just([OperandSpec::R0L, OperandSpec::Nothing, OperandSpec::Nothing])),
        0b1000_0011 => (Opcode::ADD(Size::B), Just([OperandSpec::Imm8, OperandSpec::R0H, OperandSpec::Nothing])),
        0b1000_0100 => (Opcode::ADD(Size::B), Just([OperandSpec::Imm8, OperandSpec::R0L, OperandSpec::Nothing])),
        0b1000_0101 => (Opcode::ADD(Size::B), Just([OperandSpec::Imm8, OperandSpec::Disp8_SB, OperandSpec::Nothing])),
        0b1000_0110 => (Opcode::ADD(Size::B), Just([OperandSpec::Imm8, OperandSpec::Disp8_FB, OperandSpec::Nothing])),
        0b1000_0111 => (Opcode::ADD(Size::B), Just([OperandSpec::Imm8, OperandSpec::Abs16, OperandSpec::Nothing])),
        0b1000_1000 => (Opcode::XOR(size), Reinterpret(OperandCategory::SrcDestRegOrDeref)),
        0b1000_1001 => (Opcode::XOR(size), Reinterpret(OperandCategory::SrcDestRegOrDeref)),
        0b1000_1010 => (Opcode::PUSH(Size::B), Just([OperandSpec::R0H, OperandSpec::Nothing, OperandSpec::Nothing])),
        0b1000_1011 => (Opcode::SUB(Size::B), Just([OperandSpec::Imm8, OperandSpec::R0H, OperandSpec::Nothing])),
        0b1000_1100 => (Opcode::SUB(Size::B), Just([OperandSpec::Imm8, OperandSpec::R0L, OperandSpec::Nothing])),
        0b1000_1101 => (Opcode::SUB(Size::B), Just([OperandSpec::Imm8, OperandSpec::Disp8_SB, OperandSpec::Nothing])),
        0b1000_1110 => (Opcode::SUB(Size::B), Just([OperandSpec::Imm8, OperandSpec::Disp8_FB, OperandSpec::Nothing])),
        0b1000_1111 => (Opcode::SUB(Size::B), Just([OperandSpec::Imm8, OperandSpec::Abs16, OperandSpec::Nothing])),
        0b1001_0000 => (Opcode::AND(size), Reinterpret(OperandCategory::SrcDestRegOrDeref)),
        0b1001_0001 => (Opcode::AND(size), Reinterpret(OperandCategory::SrcDestRegOrDeref)),
        0b1001_0010 => (Opcode::POP, Just([OperandSpec::R0L, OperandSpec::Nothing, OperandSpec::Nothing])),
        0b1001_0011 => (Opcode::AND(Size::B), Just([OperandSpec::Imm8, OperandSpec::R0H, OperandSpec::Nothing])),
        0b1001_0100 => (Opcode::AND(Size::B), Just([OperandSpec::Imm8, OperandSpec::R0L, OperandSpec::Nothing])),
        0b1001_0101 => (Opcode::AND(Size::B), Just([OperandSpec::Imm8, OperandSpec::Disp8_SB, OperandSpec::Nothing])),
        0b1001_0110 => (Opcode::AND(Size::B), Just([OperandSpec::Imm8, OperandSpec::Disp8_FB, OperandSpec::Nothing])),
        0b1001_0111 => (Opcode::AND(Size::B), Just([OperandSpec::Imm8, OperandSpec::Abs16, OperandSpec::Nothing])),
        0b1001_1000 => (Opcode::OR(size), Reinterpret(OperandCategory::SrcDestRegOrDeref)),
        0b1001_1001 => (Opcode::OR(size), Reinterpret(OperandCategory::SrcDestRegOrDeref)),
        0b1001_1010 => (Opcode::POP, Just([OperandSpec::R0H, OperandSpec::Nothing, OperandSpec::Nothing])),
        0b1001_1011 => (Opcode::OR(Size::B), Just([OperandSpec::Imm8, OperandSpec::R0H, OperandSpec::Nothing])),
        0b1001_1100 => (Opcode::OR(Size::B), Just([OperandSpec::Imm8, OperandSpec::R0L, OperandSpec::Nothing])),
        0b1001_1101 => (Opcode::OR(Size::B), Just([OperandSpec::Imm8, OperandSpec::Disp8_SB, OperandSpec::Nothing])),
        0b1001_1110 => (Opcode::OR(Size::B), Just([OperandSpec::Imm8, OperandSpec::Disp8_FB, OperandSpec::Nothing])),
        0b1001_1111 => (Opcode::OR(Size::B), Just([OperandSpec::Imm8, OperandSpec::Abs16, OperandSpec::Nothing])),
        0b1010_0000 => (Opcode::ADD(size), Reinterpret(OperandCategory::SrcDestRegOrDeref)),
        0b1010_0001 => (Opcode::ADD(size), Reinterpret(OperandCategory::SrcDestRegOrDeref)),
        0b1010_0010 => (Opcode::MOV(Size::W), Just([OperandSpec::Imm16, OperandSpec::A0, OperandSpec::Nothing])),
        0b1010_0011 => (Opcode::INC(Size::B), Just([OperandSpec::R0H, OperandSpec::Nothing, OperandSpec::Nothing])),
        0b1010_0100 => (Opcode::INC(Size::B), Just([OperandSpec::R0L, OperandSpec::Nothing, OperandSpec::Nothing])),
        0b1010_0101 => (Opcode::INC(Size::B), Just([OperandSpec::Disp8_SB, OperandSpec::Nothing, OperandSpec::Nothing])),
        0b1010_0110 => (Opcode::INC(Size::B), Just([OperandSpec::Disp8_FB, OperandSpec::Nothing, OperandSpec::Nothing])),
        0b1010_0111 => (Opcode::INC(Size::B), Just([OperandSpec::Abs16, OperandSpec::Nothing, OperandSpec::Nothing])),
        0b1010_1000 => (Opcode::SUB(size), Reinterpret(OperandCategory::SrcDestRegOrDeref)),
        0b1010_1001 => (Opcode::SUB(size), Reinterpret(OperandCategory::SrcDestRegOrDeref)),
        0b1010_1010 => (Opcode::MOV(Size::W), Just([OperandSpec::Imm16, OperandSpec::A1, OperandSpec::Nothing])),
        0b1010_1011 => (Opcode::DEC(Size::B), Just([OperandSpec::R0H, OperandSpec::Nothing, OperandSpec::Nothing])),
        0b1010_1100 => (Opcode::DEC(Size::B), Just([OperandSpec::R0L, OperandSpec::Nothing, OperandSpec::Nothing])),
        0b1010_1101 => (Opcode::DEC(Size::B), Just([OperandSpec::Disp8_SB, OperandSpec::Nothing, OperandSpec::Nothing])),
        0b1010_1110 => (Opcode::DEC(Size::B), Just([OperandSpec::Disp8_FB, OperandSpec::Nothing, OperandSpec::Nothing])),
        0b1010_1111 => (Opcode::DEC(Size::B), Just([OperandSpec::Abs16, OperandSpec::Nothing, OperandSpec::Nothing])),
        0b1011_0000 => (Opcode::ADC(size), Reinterpret(OperandCategory::SrcDestRegOrDeref)),
        0b1011_0001 => (Opcode::ADC(size), Reinterpret(OperandCategory::SrcDestRegOrDeref)),
        0b1011_0010 => (Opcode::INC(Size::W), Just([OperandSpec::A0, OperandSpec::Nothing, OperandSpec::Nothing])),
        0b1011_0011 => (Opcode::MOV(Size::B), Just([OperandSpec::Zero, OperandSpec::R0H, OperandSpec::Nothing])),
        0b1011_0100 => (Opcode::MOV(Size::B), Just([OperandSpec::Zero, OperandSpec::R0L, OperandSpec::Nothing])),
        0b1011_0101 => (Opcode::MOV(Size::B), Just([OperandSpec::Zero, OperandSpec::Disp8_SB, OperandSpec::Nothing])),
        0b1011_0110 => (Opcode::MOV(Size::B), Just([OperandSpec::Zero, OperandSpec::Disp8_FB, OperandSpec::Nothing])),
        0b1011_0111 => (Opcode::MOV(Size::B), Just([OperandSpec::Zero, OperandSpec::Abs16, OperandSpec::Nothing])),
        0b1011_1000 => (Opcode::SBB(size), Reinterpret(OperandCategory::SrcDestRegOrDeref)),
        0b1011_1001 => (Opcode::SBB(size), Reinterpret(OperandCategory::SrcDestRegOrDeref)),
        0b1011_1010 => (Opcode::INC(Size::W), Just([OperandSpec::A1, OperandSpec::Nothing, OperandSpec::Nothing])),
        0b1011_1011 => (Opcode::NOT(Size::B), Just([OperandSpec::R0H, OperandSpec::Nothing, OperandSpec::Nothing])),
        0b1011_1100 => (Opcode::NOT(Size::B), Just([OperandSpec::R0L, OperandSpec::Nothing, OperandSpec::Nothing])),
        0b1011_1101 => (Opcode::NOT(Size::B), Just([OperandSpec::Disp8_SB, OperandSpec::Nothing, OperandSpec::Nothing])),
        0b1011_1110 => (Opcode::NOT(Size::B), Just([OperandSpec::Disp8_FB, OperandSpec::Nothing, OperandSpec::Nothing])),
        0b1011_1111 => (Opcode::NOT(Size::B), Just([OperandSpec::Abs16, OperandSpec::Nothing, OperandSpec::Nothing])),
        0b1100_0000 => (Opcode::CMP(size), Reinterpret(OperandCategory::SrcDestRegOrDeref)),
        0b1100_0001 => (Opcode::CMP(size), Reinterpret(OperandCategory::SrcDestRegOrDeref)),
        0b1100_0010 => (Opcode::PUSH(Size::W), Just([OperandSpec::A0, OperandSpec::Nothing, OperandSpec::Nothing])),
        0b1100_0011 => (Opcode::MOV(Size::B), Just([OperandSpec::Imm8, OperandSpec::R0H, OperandSpec::Nothing])),
        0b1100_0100 => (Opcode::MOV(Size::B), Just([OperandSpec::Imm8, OperandSpec::R0L, OperandSpec::Nothing])),
        0b1100_0101 => (Opcode::MOV(Size::B), Just([OperandSpec::Imm8, OperandSpec::Disp8_SB, OperandSpec::Nothing])),
        0b1100_0110 => (Opcode::MOV(Size::B), Just([OperandSpec::Imm8, OperandSpec::Disp8_FB, OperandSpec::Nothing])),
        0b1100_0111 => (Opcode::MOV(Size::B), Just([OperandSpec::Imm8, OperandSpec::Abs16, OperandSpec::Nothing])),
        0b1100_1000 => (Opcode::ADD(size), Reinterpret(OperandCategory::Imm4Dest)),
        0b1100_1001 => (Opcode::ADD(size), Reinterpret(OperandCategory::Imm4Dest)),
        0b1100_1010 => (Opcode::PUSH(Size::W), Just([OperandSpec::A1, OperandSpec::Nothing, OperandSpec::Nothing])),
        0b1100_1011 => (Opcode::STZ, Just([OperandSpec::Imm8, OperandSpec::R0H, OperandSpec::Nothing])),
        0b1100_1100 => (Opcode::STZ, Just([OperandSpec::Imm8, OperandSpec::R0L, OperandSpec::Nothing])),
        0b1100_1101 => (Opcode::STZ, Just([OperandSpec::Imm8, OperandSpec::Disp8_SB, OperandSpec::Nothing])),
        0b1100_1110 => (Opcode::STX, Just([OperandSpec::Imm8, OperandSpec::Disp8_FB, OperandSpec::Nothing])),
        0b1100_1111 => (Opcode::STZ, Just([OperandSpec::Imm8, OperandSpec::Abs16, OperandSpec::Nothing])),
        0b1101_0000 => (Opcode::CMP(size), Reinterpret(OperandCategory::Imm4Dest)),
        0b1101_0001 => (Opcode::CMP(size), Reinterpret(OperandCategory::Imm4Dest)),
        0b1101_0010 => (Opcode::POP, Just([OperandSpec::A0, OperandSpec::Nothing, OperandSpec::Nothing])),
        0b1101_0011 => (Opcode::STNZ, Just([OperandSpec::Imm8, OperandSpec::R0H, OperandSpec::Nothing])),
        0b1101_0100 => (Opcode::STNZ, Just([OperandSpec::Imm8, OperandSpec::R0L, OperandSpec::Nothing])),
        0b1101_0101 => (Opcode::STNZ, Just([OperandSpec::Imm8, OperandSpec::Disp8_SB, OperandSpec::Nothing])),
        0b1101_0110 => (Opcode::STNZ, Just([OperandSpec::Imm8, OperandSpec::Disp8_FB, OperandSpec::Nothing])),
        0b1101_0111 => (Opcode::STNZ, Just([OperandSpec::Imm8, OperandSpec::Abs16, OperandSpec::Nothing])),
        0b1101_1000 => (Opcode::MOV(size), Reinterpret(OperandCategory::Imm4Dest)),
        0b1101_1001 => (Opcode::MOV(size), Reinterpret(OperandCategory::Imm4Dest)),
        0b1101_1010 => (Opcode::POP, Just([OperandSpec::A1, OperandSpec::Nothing, OperandSpec::Nothing])),
        0b1101_1011 => (Opcode::STZX, Just([OperandSpec::Imm8, OperandSpec::R0H, OperandSpec::Imm82])),
        0b1101_1100 => (Opcode::STZX, Just([OperandSpec::Imm8, OperandSpec::R0L, OperandSpec::Imm82])),
        0b1101_1101 => (Opcode::STZX, Just([OperandSpec::Imm8, OperandSpec::Disp8_SB, OperandSpec::Imm82])),
        0b1101_1110 => (Opcode::STZX, Just([OperandSpec::Imm8, OperandSpec::Disp8_FB, OperandSpec::Imm82])),
        0b1101_1111 => (Opcode::STZX, Just([OperandSpec::Imm8, OperandSpec::Abs16, OperandSpec::Imm82])),
        0b1110_0000 => (Opcode::ROT(size), Reinterpret(OperandCategory::Imm4Dest)),
        0b1110_0001 => (Opcode::ROT(size), Reinterpret(OperandCategory::Imm4Dest)),
        0b1110_0010 => (Opcode::MOV(Size::B), Just([OperandSpec::Imm8, OperandSpec::A0, OperandSpec::Nothing])),
        0b1110_0011 => (Opcode::CMP(Size::B), Just([OperandSpec::Imm8, OperandSpec::R0H, OperandSpec::Nothing])),
        0b1110_0100 => (Opcode::CMP(Size::B), Just([OperandSpec::Imm8, OperandSpec::R0L, OperandSpec::Nothing])),
        0b1110_0101 => (Opcode::CMP(Size::B), Just([OperandSpec::Imm8, OperandSpec::Disp8_SB, OperandSpec::Nothing])),
        0b1110_0110 => (Opcode::CMP(Size::B), Just([OperandSpec::Imm8, OperandSpec::Disp8_FB, OperandSpec::Nothing])),
        0b1110_0111 => (Opcode::CMP(Size::B), Just([OperandSpec::Imm8, OperandSpec::Abs16, OperandSpec::Nothing])),
        0b1110_1000 => (Opcode::SHA(size), Reinterpret(OperandCategory::Imm4Dest)),
        0b1110_1001 => (Opcode::SHA(size), Reinterpret(OperandCategory::Imm4Dest)),
        0b1110_1010 => (Opcode::MOV(Size::B), Just([OperandSpec::Imm8, OperandSpec::A1, OperandSpec::Nothing])),
        0b1110_1011 => (Opcode::NOP, Reinterpret(OperandCategory::OpEB)), // Opcode will be discarded
        0b1110_1100 => (Opcode::PUSHM, Just([OperandSpec::RegList, OperandSpec::Nothing, OperandSpec::Nothing])),
        0b1110_1101 => (Opcode::POPM, Just([OperandSpec::RegList, OperandSpec::Nothing, OperandSpec::Nothing])),
        0b1110_1110 => (Opcode::JMPS, Just([OperandSpec::Imm8, OperandSpec::Nothing, OperandSpec::Nothing])),
        0b1110_1111 => (Opcode::JSRS, Just([OperandSpec::Imm8, OperandSpec::Nothing, OperandSpec::Nothing])),
        0b1111_0000 => (Opcode::SHA(size), Reinterpret(OperandCategory::Imm4Dest)),
        0b1111_0001 => (Opcode::SHA(size), Reinterpret(OperandCategory::Imm4Dest)),
        0b1111_0010 => (Opcode::DEC(Size::W), Just([OperandSpec::A0, OperandSpec::Nothing, OperandSpec::Nothing])),
        0b1111_0011 => (Opcode::RTS, Just([OperandSpec::Nothing, OperandSpec::Nothing, OperandSpec::Nothing])),
        0b1111_0100 => (Opcode::JMP(Size::W), Just([OperandSpec::JmpDisp16, OperandSpec::Nothing, OperandSpec::Nothing])),
        0b1111_0101 => (Opcode::JSR(Size::W), Just([OperandSpec::JmpDisp16, OperandSpec::Nothing, OperandSpec::Nothing])),
        0b1111_0110 => (Opcode::INTO, Just([OperandSpec::Nothing, OperandSpec::Nothing, OperandSpec::Nothing])),
        0b1111_0111 => { return Err(DecodeError::InvalidOperand); },
        0b1111_1000 => (Opcode::ADJNZ(size), Reinterpret(OperandCategory::ADJNZ)),
        0b1111_1001 => (Opcode::ADJNZ(size), Reinterpret(OperandCategory::ADJNZ)),
        0b1111_1010 => (Opcode::DEC(Size::W), Just([OperandSpec::A1, OperandSpec::Nothing, OperandSpec::Nothing])),
        0b1111_1011 => (Opcode::REIT, Just([OperandSpec::Nothing, OperandSpec::Nothing, OperandSpec::Nothing])),
        0b1111_1100 => (Opcode::JMP(Size::A), Just([OperandSpec::JmpAbs20, OperandSpec::Nothing, OperandSpec::Nothing])),
        0b1111_1101 => (Opcode::JSR(Size::A), Just([OperandSpec::JmpAbs20, OperandSpec::Nothing, OperandSpec::Nothing])),
        0b1111_1110 => (Opcode::JMP(Size::B), Just([OperandSpec::JmpDisp8, OperandSpec::Nothing, OperandSpec::Nothing])),
        0b1111_1111 => (Opcode::UND, Just([OperandSpec::Nothing, OperandSpec::Nothing, OperandSpec::Nothing])),
    };
    inst.opcode = opcode;
    match interpretation {
        Just(operands) => {
            inst.operands = operands;

            // in the base opcode map, if an immediate is specified it comes before bytes for
            // displacement/absolute address that may be part of a destination.
            //
            // additionally, Imm8 or Imm16 is the first operand, if present. so look for that
            // first.
            if let OperandSpec::Imm8 = inst.operands[0] {
                inst.imm_wide = read_imm(bytes, 1)? as u32;
                inst.length += 1;
            }

            // now, read bytes for remaining operands, EXCEPT STZX which we read most but not all
            // of. it specifies a second imm8 after optional disp/abs
            for op in inst.operands.iter() {
                use OperandSpec::*;
                match op {
                    RegList => {
                        inst.imm_wide = read_imm(bytes, 1)? as u32;
                        inst.length += 1;

                        // internally, the register list is given one order.
                        // however, the m16c encoding specifies reverse order for pushm/popm
                        // register lists.. so, reverse it here and make it correct for everyone
                        // else.
                        if let Opcode::POPM = inst.opcode {
                            let mut replacement_imm = 0u8;
                            for i in 0..8 {
                                if inst.imm_wide & (1 << i) != 0 {
                                    replacement_imm |= 1 << (7 - i);
                                }
                            }
                            inst.imm_wide = replacement_imm as u32;
                        }
                    },
                    JmpDisp8 |
                    Disp8 |
                    Disp8_FB |
                    Disp8_SB => {
                        // have to sign extend in case these displacements are negative
                        inst.dispabs = read_imm(bytes, 1)? as i8 as i16 as u16;
                        inst.length += 1;
                    }
                    JmpDisp16 |
                    Abs16 => {
                        inst.dispabs = read_imm(bytes, 2)? as u16;
                        inst.length += 2;
                    }
                    JmpAbs20 |
                    Abs20 => {
                        inst.imm_wide = read_imm(bytes, 3)? as u32 & 0x0f_ff_ff;
                        inst.length += 3;
                    }
                    Disp8_A0 |
                    Disp8_A1 |
                    Disp8_SP |
                    Disp16_A0 |
                    Disp16_A1 |
                    Disp16_SB |
                    Disp20_A0 |
                    Disp20_A1 |
                    Deref_A0 |
                    Deref_A1 |
                    Deref_A1A0 |
                    Disp2_8_A0 |
                    Disp2_8_A1 |
                    Disp2_8_SB |
                    Disp2_8_FB |
                    Disp2_16_A0 |
                    Disp2_16_A1 |
                    Disp2_16_SB |
                    Abs2_16 |
                    Bit_Disp8_A0 |
                    Bit_Disp8_A1 |
                    Bit_Disp8_SB |
                    Bit_Disp8_FB |
                    Bit_Disp16_A0 |
                    Bit_Disp16_A1 |
                    Bit_Disp16_SB |
                    Bit_Abs16 => {
                        panic!("unexpected operand spec for Just set of operands: {:?}", op);
                    }
                    // and other operands? yeah just ignore em
                    _ => {}
                }
            }

            // now for STZX, really,...
            if let OperandSpec::Imm82 = inst.operands[2] {
                inst.imm_wide |= (bytes.next().ok_or(DecodeError::ExhaustedInput)? as u32) << 8;
                inst.length += 1;
            }

            return Ok(());
        },
        Reinterpret(OperandCategory::Op74) => {
            return decode_op74(inst, size, bytes);
        },
        Reinterpret(OperandCategory::Op76) => {
            return decode_op76(inst, size, bytes);
        },
        Reinterpret(OperandCategory::Op78) => {
            return decode_op78(inst, size, bytes);
        },
        Reinterpret(OperandCategory::Op7A) => {
            return decode_op7A(inst, size, bytes);
        },
        Reinterpret(OperandCategory::Op7B) => {
            return decode_op7B(inst, size, bytes);
        },
        Reinterpret(OperandCategory::Op7C) => {
            return decode_op7C(inst, size, bytes);
        },
        Reinterpret(OperandCategory::Op7D) => {
            return decode_op7D(inst, size, bytes);
        },
        Reinterpret(OperandCategory::Op7E) => {
            return decode_op7E(inst, size, bytes);
        },
        Reinterpret(OperandCategory::OpEB) => {
            return decode_opEB(inst, size, bytes);
        },
        Reinterpret(OperandCategory::Imm4Dest) => {
            let operands = bytes.next().ok_or(DecodeError::ExhaustedInput)?;
            inst.length += 1;
            inst.imm_wide = (operands as i8 >> 4) as i32 as u32;
            inst.operands[0] = OperandSpec::Imm4;
            inst.operands[1] = Operand_RegDerefDispAbs(operands & 0b1111, size, inst, bytes)?;
            inst.operands[2] = OperandSpec::Nothing;
        }
        Reinterpret(OperandCategory::JmpDispOpcodeLow3) => {
            inst.dispabs = (byte & 0b111) as u16;
            inst.operands[0] = OperandSpec::Disp8;
            inst.operands[1] = OperandSpec::Nothing;
            inst.operands[2] = OperandSpec::Nothing;
        }
        Reinterpret(OperandCategory::ADJNZ) => {
            // ADJNZ has operands like any other [IMM4|DEST] operand, but takes an additional byte
            // for the destination label.
            let operands = bytes.next().ok_or(DecodeError::ExhaustedInput)?;
            inst.length += 1;
            inst.imm_wide = ((operands as i8) >> 4) as u32;
            inst.operands[0] = OperandSpec::Imm4;
            inst.operands[1] = Operand_RegDerefDispAbs(operands & 0b1111, size, inst, bytes)?;
            inst.operands[2] = OperandSpec::Label8;
        }
        Reinterpret(OperandCategory::SrcDestRegOrDeref) => {
            // these instructions can read two disp8/disp16, so the first one can be handled
            // normally, but the second will need some fixing up...
            let operands = bytes.next().ok_or(DecodeError::ExhaustedInput)?;
            inst.length += 1;
            inst.operands[0] = Operand_RegDerefDispAbs(operands >> 4, size, inst, bytes)?;
            inst.operands[1] = Operand_second_RegDerefDispAbs(operands & 0b1111, size, inst, bytes)?;
            inst.operands[2] = OperandSpec::Nothing;
        }
    }
    Ok(())
}

fn decode_op74<T: Iterator<Item=u8>>(inst: &mut Instruction, size: Size, bytes: &mut T) -> Result<(), DecodeError> {
    let byte = bytes.next().ok_or(DecodeError::ExhaustedInput)?;
    inst.length += 1;
    match byte >> 4 {
        0b0000 => {
            inst.opcode = Opcode::STE;
            inst.operands[0] = Operand_RegDerefDispAbs(byte & 0b1111, size, inst, bytes)?;
            inst.operands[1] = OperandSpec::Abs20;
            inst.imm_wide = read_imm(bytes, 3)? & 0x0f_ff_ff;
            inst.length += 3;
        }
        0b0001 => {
            inst.opcode = Opcode::STE;
            inst.operands[0] = Operand_RegDerefDispAbs(byte & 0b1111, size, inst, bytes)?;
            inst.operands[1] = OperandSpec::Disp20_A0;
            inst.imm_wide = read_imm(bytes, 3)? & 0x0f_ff_ff;
            inst.length += 3;
        }
        0b0010 => {
            inst.opcode = Opcode::STE;
            inst.operands[0] = Operand_RegDerefDispAbs(byte & 0b1111, size, inst, bytes)?;
            inst.operands[1] = OperandSpec::Deref_A1A0;
        }
        0b0011 => {
            inst.opcode = Opcode::MOV(size);
            inst.operands[0] = Operand_RegDerefDispAbs(byte & 0b1111, size, inst, bytes)?;
            inst.operands[1] = OperandSpec::Disp8_SP;
        }
        0b0100 => {
            inst.opcode = Opcode::PUSH(size);
            inst.operands[0] = Operand_RegDerefDispAbs(byte & 0b1111, size, inst, bytes)?;
            inst.operands[1] = OperandSpec::Nothing;
        }
        0b0101 => {
            inst.opcode = Opcode::NEG;
            inst.operands[0] = Operand_RegDerefDispAbs(byte & 0b1111, size, inst, bytes)?;
            inst.operands[1] = OperandSpec::Nothing;
        }
        0b0110 => {
            inst.opcode = Opcode::ROT(size);
            if size == Size::W && byte & 0b1111 == 0b0001 {
                // invalid dest, would be R1
                return Err(DecodeError::InvalidOperand);
            } else if size == Size::B && byte & 0b1111 == 0b0011 {
                // invalid dest, would be R1H
                return Err(DecodeError::InvalidOperand);
            }

            inst.operands[0] = OperandSpec::R1H;
            inst.operands[1] = Operand_RegDerefDispAbs(byte & 0b1111, size, inst, bytes)?;
        }
        0b0111 => {
            inst.opcode = Opcode::NOT(size);
            inst.operands[0] = Operand_RegDerefDispAbs(byte & 0b1111, size, inst, bytes)?;
            inst.operands[1] = OperandSpec::Nothing;
        }
        0b1000 => {
            inst.opcode = Opcode::LDE;
            inst.operands[0] = OperandSpec::Abs20;
            inst.operands[1] = Operand_RegDerefDispAbs(byte & 0b1111, size, inst, bytes)?;
            // careful! abs20 comes after dest disp/abs
            inst.imm_wide = read_imm(bytes, 3)? & 0x0f_ff_ff;
            inst.length += 3;
        }
        0b1001 => {
            inst.opcode = Opcode::LDE;
            inst.operands[0] = OperandSpec::Disp20_A0;
            inst.operands[1] = Operand_RegDerefDispAbs(byte & 0b1111, size, inst, bytes)?;
            // careful! disp20 comes after dest disp/abs
            inst.imm_wide = read_imm(bytes, 3)? & 0x0f_ff_ff;
            inst.length += 3;
        }
        0b1010 => {
            inst.opcode = Opcode::LDE;
            inst.operands[0] = OperandSpec::Deref_A1A0;
            inst.operands[1] = Operand_RegDerefDispAbs(byte & 0b1111, size, inst, bytes)?;
        }
        0b1011 => {
            inst.opcode = Opcode::MOV(size);
            inst.operands[0] = OperandSpec::Disp8_SP;
            inst.operands[1] = Operand_RegDerefDispAbs(byte & 0b1111, size, inst, bytes)?;
            // careful! disp8 comes after dest disp/abs
            inst.dispabs = read_imm(bytes, 1)? as u16;
            inst.length += 1;
        }
        0b1100 => {
            inst.opcode = Opcode::MOV(size);
            inst.operands[0] = if size == Size::W { OperandSpec::Imm16 } else { OperandSpec::Imm8 };
            inst.operands[1] = Operand_RegDerefDispAbs(byte & 0b1111, size, inst, bytes)?;
            // careful! imm comes after dest disp/abs
            inst.imm_wide = read_imm(bytes, size.as_bytes())? as u32;
            inst.length += size.as_bytes();
        }
        0b1101 => {
            inst.opcode = Opcode::POP;
            inst.operands[0] = Operand_RegDerefDispAbs(byte & 0b1111, size, inst, bytes)?;
            inst.operands[1] = OperandSpec::Nothing;
        }
        0b1110 => {
            inst.opcode = Opcode::SHL(size);
            if size == Size::W && byte & 0b1111 == 0b0001 {
                // invalid dest, would be R1
                return Err(DecodeError::InvalidOperand);
            } else if size == Size::B && byte & 0b1111 == 0b0011 {
                // invalid dest, would be R1H
                return Err(DecodeError::InvalidOperand);
            }

            inst.operands[0] = OperandSpec::R1H;
            inst.operands[1] = Operand_RegDerefDispAbs(byte & 0b1111, size, inst, bytes)?;
        }
        0b1111 => {
            inst.opcode = Opcode::SHA(size);
            if size == Size::W && byte & 0b1111 == 0b0001 {
                // invalid dest, would be R1
                return Err(DecodeError::InvalidOperand);
            } else if size == Size::B && byte & 0b1111 == 0b0011 {
                // invalid dest, would be R1H
                return Err(DecodeError::InvalidOperand);
            }

            inst.operands[0] = OperandSpec::R1H;
            inst.operands[1] = Operand_RegDerefDispAbs(byte & 0b1111, size, inst, bytes)?;
        }
        _ => {
            unreachable!("opcode selector is four bits");
        }
    }

    Ok(())
}

fn decode_op76<T: Iterator<Item=u8>>(inst: &mut Instruction, size: Size, bytes: &mut T) -> Result<(), DecodeError> {
    let byte = bytes.next().ok_or(DecodeError::ExhaustedInput)?;
    inst.length += 1;
    let opc_selector = byte >> 4;
    if opc_selector < 0b1001 {
        inst.opcode = [
            Opcode::TST(size), Opcode::XOR(size), Opcode::AND(size),
            Opcode::OR(size), Opcode::ADD(size), Opcode::SUB(size),
            Opcode::ADC(size), Opcode::SBB(size), Opcode::CMP(size)
        ][opc_selector as usize];
        inst.operands[1] = Operand_RegDerefDispAbs(byte & 0b1111, size, inst, bytes)?;
        inst.operands[0] = if size == Size::W { OperandSpec::Imm16 } else { OperandSpec::Imm8 };
        inst.imm_wide = read_imm(bytes, size.as_bytes())?;
        inst.length += size.as_bytes();
    } else {
        match opc_selector {
            0b1001 => {
                inst.opcode = Opcode::DIVX(size);
                inst.operands[0] = if size == Size::W { OperandSpec::Imm16 } else { OperandSpec::Imm8 };
                inst.imm_wide = read_imm(bytes, size.as_bytes())?;
                inst.length += size.as_bytes();
            }
            0b1010 => {
                inst.opcode = Opcode::ROLC;
                inst.operands[0] = Operand_RegDerefDispAbs(byte & 0b1111, size, inst, bytes)?;
                inst.operands[1] = OperandSpec::Nothing;
            }
            0b1011 => {
                inst.opcode = Opcode::RORC;
                inst.operands[0] = Operand_RegDerefDispAbs(byte & 0b1111, size, inst, bytes)?;
                inst.operands[1] = OperandSpec::Nothing;
            }
            0b1100 => {
                inst.opcode = Opcode::DIVU(size);
                inst.operands[0] = if size == Size::W { OperandSpec::Imm16 } else { OperandSpec::Imm8 };
                inst.imm_wide = read_imm(bytes, size.as_bytes())?;
                inst.length += size.as_bytes();
            }
            0b1101 => {
                inst.opcode = Opcode::DIV(size);
                inst.operands[0] = Operand_RegDerefDispAbs(byte & 0b1111, size, inst, bytes)?;
                inst.operands[1] = OperandSpec::Nothing;
            }
            0b1110 => {
                inst.opcode = Opcode::ADCF(size);
                inst.operands[1] = Operand_RegDerefDispAbs(byte & 0b1111, size, inst, bytes)?;
                inst.operands[0] = if size == Size::W { OperandSpec::Imm16 } else { OperandSpec::Imm8 };
                inst.imm_wide = read_imm(bytes, size.as_bytes())?;
                inst.length += size.as_bytes();
            }
            0b1111 => {
                inst.opcode = Opcode::ABS;
                inst.operands[0] = Operand_RegDerefDispAbs(byte & 0b1111, size, inst, bytes)?;
                inst.operands[1] = OperandSpec::Nothing;
            }
            _ => {
                unreachable!("opcode selector is four bits");
            }
        }
    }

    Ok(())
}

fn decode_op78<T: Iterator<Item=u8>>(inst: &mut Instruction, size: Size, bytes: &mut T) -> Result<(), DecodeError> {
    let byte = bytes.next().ok_or(DecodeError::ExhaustedInput)?;
    inst.length += 1;
    inst.opcode = Opcode::MUL;
    inst.operands[0] = Operand_RegDerefDispAbs(byte >> 4, size, inst, bytes)?;
    let dest_code = byte & 0b1111;
    match (size, dest_code) {
        (Size::B, 0b0001) |
        (Size::W, 0b0010) |
        (_, 0b0011) |
        (_, 0b0101) => {
            return Err(DecodeError::InvalidOperand);
        }
        _ => {}
    }
    inst.operands[1] = Operand_RegDerefDispAbs(dest_code, size, inst, bytes)?;

    Ok(())
}

fn decode_op7A<T: Iterator<Item=u8>>(inst: &mut Instruction, size: Size, bytes: &mut T) -> Result<(), DecodeError> {
    let byte = bytes.next().ok_or(DecodeError::ExhaustedInput)?;
    inst.length += 1;
    if byte >= 0b10000000 {
        inst.opcode = Opcode::LDC;
        inst.operands[0] = Operand_RegDerefDispAbs(byte & 0b1111, size, inst, bytes)?;
        inst.operands[1] = Operand_IntFlgSpSbFb((byte >> 4) & 0b111)?;
    } else {
        inst.opcode = Opcode::XCHG(size);
        assert_eq!(size, Size::B);
        if byte >= 0b01000000 {
            return Err(DecodeError::InvalidOperand);
        }
        inst.operands[0] = [
            OperandSpec::R0L, OperandSpec::R0H,
            OperandSpec::R1L, OperandSpec::R1H,
        ][(byte >> 4) as usize];
        inst.operands[1] = Operand_RegDerefDispAbs(byte & 0b1111, size, inst, bytes)?;
    }

    Ok(())
}

fn decode_op7B<T: Iterator<Item=u8>>(inst: &mut Instruction, size: Size, bytes: &mut T) -> Result<(), DecodeError> {
    let byte = bytes.next().ok_or(DecodeError::ExhaustedInput)?;
    inst.length += 1;
    if byte >= 0b10000000 {
        inst.opcode = Opcode::STC;
        inst.operands[1] = Operand_IntFlgSpSbFb((byte >> 4) & 0b111)?;
        inst.operands[0] = Operand_RegDerefDispAbs(byte & 0b1111, size, inst, bytes)?;
    } else {
        inst.opcode = Opcode::XCHG(size);
        assert_eq!(size, Size::W);
        if byte >= 0b01000000 {
            return Err(DecodeError::InvalidOperand);
        }
        inst.operands[0] = [
            OperandSpec::R0, OperandSpec::R1,
            OperandSpec::R2, OperandSpec::R3,
        ][(byte >> 4) as usize];
        inst.operands[1] = Operand_RegDerefDispAbs(byte & 0b1111, size, inst, bytes)?;
    }

    Ok(())
}

fn decode_op7C<T: Iterator<Item=u8>>(inst: &mut Instruction, size: Size, bytes: &mut T) -> Result<(), DecodeError> {
    let byte = bytes.next().ok_or(DecodeError::ExhaustedInput)?;
    inst.length += 1;
    match byte >> 4 {
        op @ 0b0000 |
        op @ 0b0001 |
        op @ 0b0010 |
        op @ 0b0011 => {
            inst.opcode = [
                Opcode::MOVLL, Opcode::MOVLH,
                Opcode::MOVHL, Opcode::MOVHH,
            ][op as usize];
            inst.operands[0] = Operand_RegDerefDispAbs(byte & 0b1111, size, inst, bytes)?;
            inst.operands[1] = OperandSpec::R0L;
        },
        0b0100 => {
            inst.opcode = Opcode::MULU(size);
            inst.operands[0] = if size == Size::W { OperandSpec::Imm16 } else { OperandSpec::Imm8 };
            inst.operands[1] = Operand_RegDerefDispAbs(byte >> 4, size, inst, bytes)?;
            let dest_code = byte & 0b1111;
            match (size, dest_code) {
                (Size::B, 0b0001) |
                (Size::W, 0b0010) |
                (_, 0b0011) |
                (_, 0b0101) => {
                    return Err(DecodeError::InvalidOperand);
                }
                _ => {}
            }
            // careful! imm comes after dest disp/abs
            inst.imm_wide = read_imm(bytes, size.as_bytes())?;
            inst.length += size.as_bytes();
        },
        0b0101 => {
            inst.opcode = Opcode::MUL;
            inst.operands[0] = if size == Size::W { OperandSpec::Imm16 } else { OperandSpec::Imm8 };
            inst.operands[1] = Operand_RegDerefDispAbs(byte >> 4, size, inst, bytes)?;
            let dest_code = byte & 0b1111;
            match (size, dest_code) {
                (Size::B, 0b0001) |
                (Size::W, 0b0010) |
                (_, 0b0011) |
                (_, 0b0101) => {
                    return Err(DecodeError::InvalidOperand);
                }
                _ => {}
            }
            // careful! imm comes after dest disp/abs
            inst.imm_wide = read_imm(bytes, size.as_bytes())?;
            inst.length += size.as_bytes();
        }
        0b0110 => {
            // byte is 0b0110_xxxx
            inst.opcode = Opcode::EXTS;
            match byte & 0b1111 {
                0b0001 |
                0b0011 |
                0b0100 |
                0b0101 => {
                    return Err(DecodeError::InvalidOperand);
                }
                _ => {}
            }
            inst.operands[0] = Operand_RegDerefDispAbs(byte & 0b1111, size, inst, bytes)?;
            inst.operands[1] = OperandSpec::Nothing;
        }
        op @ 0b1000 |
        op @ 0b1001 |
        op @ 0b1010 |
        op @ 0b1011 => {
            inst.opcode = [
                Opcode::MOVLL, Opcode::MOVLH,
                Opcode::MOVHL, Opcode::MOVHH,
            ][(op & 0b11) as usize];
            inst.operands[0] = OperandSpec::R0L;
            inst.operands[1] = Operand_RegDerefDispAbs(byte & 0b1111, size, inst, bytes)?;
        }
        0b1100 => {
            inst.opcode = Opcode::EXTS;
            let dest = match byte & 0b1111 {
                0b0010 |
                0b0011 |
                0b0101 => {
                    return Err(DecodeError::InvalidOperand);
                }
                0b0000 => OperandSpec::R2R0,
                0b0001 => OperandSpec::R3R1,
                0b0100 => OperandSpec::A1A0,
                code => {
                    Operand_RegDerefDispAbs(code, size, inst, bytes)?
                }
            };
            inst.operands[0] = dest;
            inst.operands[1] = OperandSpec::Nothing;
        }
        0b1110 => {
            let code = byte & 0b1111;
            if code & 0b111 < 0b100 {
                match code {
                    op @ 0b0000 |
                    op @ 0b0001 |
                    op @ 0b0010 |
                    op @ 0b0011 => {
                        inst.opcode = [
                            Opcode::DIVU(size), Opcode::DIV(size),
                            Opcode::PUSH(size), Opcode::DIVX(size),
                        ][op as usize];
                        inst.operands[0] = if size == Size::W { OperandSpec::Imm16 } else { OperandSpec::Imm8 };
                        inst.operands[1] = OperandSpec::Nothing;
                        inst.operands[2] = OperandSpec::Nothing;
                        inst.imm_wide = read_imm(bytes, size.as_bytes())?;
                        inst.length += size.as_bytes();
                    }
                    op @ 0b1000 |
                    op @ 0b1001 |
                    op @ 0b1010 => {
                        inst.opcode = [
                            Opcode::SMOVF(size), Opcode::SMOVB(size),
                            Opcode::SSTR(size),
                        ][(op & 0b11) as usize];
                        inst.operands = [OperandSpec::Nothing, OperandSpec::Nothing, OperandSpec::Nothing];
                    }
                    0b1011 => {
                        inst.opcode = Opcode::ADD(size);
                        inst.operands[0] = if size == Size::W { OperandSpec::Imm16 } else { OperandSpec::Imm8 };
                        inst.operands[1] = OperandSpec::SP;
                        inst.operands[2] = OperandSpec::Nothing;
                        inst.imm_wide = read_imm(bytes, size.as_bytes())?;
                        inst.length += size.as_bytes();
                    }
                    _ => { unreachable!("this should be an invalid bit pattern"); }
                }
            } else {
                inst.opcode = [
                    Opcode::DADD, Opcode::DSUB,
                    Opcode::DADC, Opcode::DSBB,
                ][(code & 0b11) as usize];
                if code < 0b1000 {
                    // reg-reg variants of these opcodes
                    inst.operands[0] = OperandSpec::R0H;
                    inst.operands[1] = OperandSpec::R0L;
                } else {
                    // reg-imm8 variants of these opcodes
                    inst.operands[0] = OperandSpec::Imm8;
                    inst.operands[1] = OperandSpec::R0L;
                    assert_eq!(size, Size::B);
                    inst.imm_wide = read_imm(bytes, size.as_bytes())?;
                    inst.length += size.as_bytes();
                }
            }
        }
        0b1111 => {
            match byte & 0b1111 {
                0b0000 => {
                    inst.opcode = Opcode::LDCTX;
                    inst.operands[0] = OperandSpec::Abs16;
                    inst.dispabs = read_imm(bytes, 2)? as u16;
                    inst.length += 2;
                    inst.operands[1] = OperandSpec::Abs20;
                    inst.imm_wide = read_imm(bytes, 3)? & 0x0f_ff_ff;
                    inst.length += 3;
                }
                0b0001 => {
                    inst.opcode = Opcode::RMPA(size);
                    inst.operands[0] = OperandSpec::Nothing;
                    inst.operands[1] = OperandSpec::Nothing;
                }
                0b0010 => {
                    inst.opcode = Opcode::ENTER;
                    inst.operands[0] = OperandSpec::Imm8;
                    inst.imm_wide = read_imm(bytes, 1)?;
                    inst.length += 1;
                    inst.operands[1] = OperandSpec::Nothing;
                }
                0b0011 => {
                    inst.opcode = Opcode::EXTS;
                    inst.operands[0] = OperandSpec::R0;
                    inst.operands[1] = OperandSpec::Nothing;
                }
                _ => {
                    return Err(DecodeError::InvalidOpcode);
                }
            }
        }
        _ => {
            return Err(DecodeError::InvalidOpcode);
        }
    }

    Ok(())
}

fn decode_op7D<T: Iterator<Item=u8>>(inst: &mut Instruction, size: Size, bytes: &mut T) -> Result<(), DecodeError> {
    let byte = bytes.next().ok_or(DecodeError::ExhaustedInput)?;
    inst.length += 1;
    match byte >> 4 {
        op @ 0b0000 |
        op @ 0b0001 |
        op @ 0b0010 |
        op @ 0b0011 => {
            inst.opcode = [
                Opcode::JMPI, Opcode::JSRI,
                Opcode::JMPI, Opcode::JSRI,
            ][op as usize];
            inst.operands[0] = Operand_RegDerefDisp20Abs(byte & 0b1111, inst, bytes)?;
            inst.operands[1] = OperandSpec::Nothing;
        }
        0b0100 => {
            inst.opcode = Opcode::MULU(size);
            inst.operands[0] = if size == Size::W { OperandSpec::Imm16 } else { OperandSpec::Imm8 };
            inst.operands[1] = Operand_RegDerefDispAbs(byte >> 4, size, inst, bytes)?;
            let dest_code = byte & 0b1111;
            match (size, dest_code) {
                (Size::B, 0b0001) |
                (Size::W, 0b0010) |
                (_, 0b0011) |
                (_, 0b0101) => {
                    return Err(DecodeError::InvalidOperand);
                }
                _ => {}
            }
            // careful! imm comes after dest disp/abs
            inst.imm_wide = read_imm(bytes, size.as_bytes())?;
            inst.length += size.as_bytes();
        },
        0b0101 => {
            inst.opcode = Opcode::MUL;
            inst.operands[0] = if size == Size::W { OperandSpec::Imm16 } else { OperandSpec::Imm8 };
            inst.operands[1] = Operand_RegDerefDispAbs(byte >> 4, size, inst, bytes)?;
            let dest_code = byte & 0b1111;
            match (size, dest_code) {
                (Size::B, 0b0001) |
                (Size::W, 0b0010) |
                (_, 0b0011) |
                (_, 0b0101) => {
                    return Err(DecodeError::InvalidOperand);
                }
                _ => {}
            }
            // careful! imm comes after dest disp/abs
            inst.imm_wide = read_imm(bytes, size.as_bytes())?;
            inst.length += size.as_bytes();
        }
        0b1001 => {
            // byte is 0b1001_xxxx
            if byte < 0b1001_1000 {
                return Err(DecodeError::InvalidOperand);
            }
            // byte is 0b1001_1xxx

            inst.opcode = Opcode::PUSHA;
            // while the low eight forms of this encoding are invalid, they would have been rejected above.
            inst.operands[0] = Operand_RegDerefDispAbs(byte & 0b1111, size, inst, bytes)?;
            inst.operands[1] = OperandSpec::Nothing;
        }
        0b1010 => {
            if byte >= 0b1010_1000 {
                return Err(DecodeError::InvalidOperand);
            }
            inst.opcode = Opcode::LDIPL;
            inst.operands[0] = OperandSpec::Imm8;
            inst.operands[1] = OperandSpec::Nothing;
            inst.imm_wide = ((((byte & 0b111) as i8) << 5) >> 5) as i32 as u32;
        }
        0b1011 => {
            // byte is 0b1011_xxxx
            inst.opcode = Opcode::ADD(size);
            inst.operands[0] = OperandSpec::Imm8;
            inst.operands[1] = OperandSpec::SP;
            inst.imm_wide = ((((byte & 0b111) as i8) << 4) >> 4) as i32 as u32;
        }
        0b1100 => {
            inst.opcode = match byte & 0b1111 {
                0b1000 => Opcode::JLE,
                0b1001 => Opcode::JO,
                0b1010 => Opcode::JGE,
                0b1100 => Opcode::JGT,
                0b1101 => Opcode::JNO,
                0b1110 => Opcode::JLT,
                _ => {
                    return Err(DecodeError::InvalidOpcode);
                }
            };
            inst.operands[0] = OperandSpec::Disp8;
            inst.operands[1] = OperandSpec::Nothing;
            inst.imm_wide = read_imm(bytes, 1)?;
            inst.length += 1;
        }
        0b1101 => {
            let code = byte & 0b1111;
            if code == 0b1011 || code == 0b1111 {
                return Err(DecodeError::InvalidOpcode);
            }
            // the NOP here fill gaps that are invalid encodings as tested above.
            inst.opcode = [
                Opcode::BMGEU, Opcode::BMGTU, Opcode::BMEQ, Opcode::BMN,
                Opcode::BMLTU, Opcode::BMLEU, Opcode::BMNE, Opcode::BMPZ,
                Opcode::BMLE,  Opcode::BMO,   Opcode::GE,   Opcode::NOP,
                Opcode::BMGT,  Opcode::NO,    Opcode::LT,   Opcode::NOP,
            ][code as usize];
            inst.operands[0] = OperandSpec::Nothing;
            inst.operands[1] = OperandSpec::Nothing;
        }
        0b1110 => {
            let code = byte & 0b1111;
            if code & 0b111 < 0b100 {
                match code {
                    op @ 0b0000 |
                    op @ 0b0001 |
                    op @ 0b0010 |
                    op @ 0b0011 => {
                        inst.opcode = [
                            Opcode::DIVU(size), Opcode::DIV(size),
                            Opcode::PUSH(size), Opcode::DIVX(size),
                        ][op as usize];
                        inst.operands[0] = if size == Size::W { OperandSpec::Imm16 } else { OperandSpec::Imm8 };
                        inst.operands[1] = OperandSpec::Nothing;
                        inst.operands[2] = OperandSpec::Nothing;
                        inst.imm_wide = read_imm(bytes, size.as_bytes())?;
                        inst.length += size.as_bytes();
                    }
                    op @ 0b1000 |
                    op @ 0b1001 |
                    op @ 0b1010 => {
                        inst.opcode = [
                            Opcode::SMOVF(size), Opcode::SMOVB(size),
                            Opcode::SSTR(size),
                        ][(op & 0b11) as usize];
                        inst.operands = [OperandSpec::Nothing, OperandSpec::Nothing, OperandSpec::Nothing];
                    }
                    0b1011 => {
                        inst.opcode = Opcode::ADD(size);
                        inst.operands[0] = if size == Size::W { OperandSpec::Imm16 } else { OperandSpec::Imm8 };
                        inst.operands[1] = OperandSpec::SP;
                        inst.operands[2] = OperandSpec::Nothing;
                        inst.imm_wide = read_imm(bytes, size.as_bytes())?;
                        inst.length += size.as_bytes();
                    }
                    _ => { unreachable!("this should be an invalid bit pattern"); }
                }
            } else {
                inst.opcode = [
                    Opcode::DADD, Opcode::DSUB,
                    Opcode::DADC, Opcode::DSBB,
                ][(code & 0b11) as usize];
                if code < 0b1000 {
                    // reg-reg variants of these opcodes
                    inst.operands[0] = OperandSpec::R1;
                    inst.operands[1] = OperandSpec::R0;
                } else {
                    // reg-imm16 variants of these opcodes
                    inst.operands[0] = OperandSpec::Imm16;
                    inst.operands[1] = OperandSpec::R0;
                    assert_eq!(size, Size::W);
                    inst.imm_wide = read_imm(bytes, size.as_bytes())?;
                    inst.length += size.as_bytes();
                }
            }
        }
        0b1111 => {
            // byte is 0b1111_xxxx
            match byte & 0b1111 {
                0b0000 => {
                    inst.opcode = Opcode::STCTX;
                    inst.operands[0] = OperandSpec::Abs16;
                    inst.dispabs = read_imm(bytes, 2)? as u16;
                    inst.length += 2;
                    inst.operands[1] = OperandSpec::Abs20;
                    inst.imm_wide = read_imm(bytes, 3)? & 0x0f_ff_ff;
                    inst.length += 3;
                }
                0b0001 => {
                    inst.opcode = Opcode::RMPA(size);
                    inst.operands[0] = OperandSpec::Nothing;
                    inst.operands[1] = OperandSpec::Nothing;
                }
                0b0010 => {
                    inst.opcode = Opcode::EXITD;
                    inst.operands[0] = OperandSpec::Nothing;
                    inst.operands[1] = OperandSpec::Nothing;
                }
                0b0011 => {
                    inst.opcode = Opcode::WAIT;
                    inst.operands[0] = OperandSpec::Nothing;
                    inst.operands[1] = OperandSpec::Nothing;
                }
                _ => {
                    return Err(DecodeError::InvalidOpcode);
                }
            }
        }
        _ => {
            return Err(DecodeError::InvalidOpcode);
        }
    }

    Ok(())
}

fn decode_op7E<T: Iterator<Item=u8>>(inst: &mut Instruction, _size: Size, bytes: &mut T) -> Result<(), DecodeError> {
    let byte = bytes.next().ok_or(DecodeError::ExhaustedInput)?;
    inst.length += 1;
    let op = byte >> 4;
    let code = byte & 0b1111;
    if op > 0b1101 {
        return Err(DecodeError::InvalidOpcode);
    }
    if op == 0b0010 {
        // handle BMCMD specifically
        inst.operands[0] = Operand_BitRegDerefDispAbs(code, inst, bytes)?;
        inst.operands[1] = OperandSpec::Nothing;
        let cnd = read_imm(bytes, 1)? as u8;
        inst.length += 1;
        inst.opcode = match cnd {
            0b0000_0000 => Opcode::BMGEU,
            0b0000_0001 => Opcode::BMGTU,
            0b0000_0010 => Opcode::BMEQ,
            0b0000_0011 => Opcode::BMN,
            0b0000_0100 => Opcode::BMLE,
            0b0000_0101 => Opcode::BMO,
            0b0000_0110 => Opcode::BMGE,
            0b1111_1000 => Opcode::BMLTU,
            0b1111_1001 => Opcode::BMLEU,
            0b1111_1010 => Opcode::BMNE,
            0b1111_1011 => Opcode::BMPZ,
            0b1111_1100 => Opcode::BMGT,
            0b1111_1101 => Opcode::BMNO,
            0b1111_1110 => Opcode::BMLT,
            _ => {
                return Err(DecodeError::InvalidOpcode);
            }
        };
    } else {
        inst.opcode = [
            Opcode::BTSTC, Opcode::BTSTS, Opcode::NOP, // NOP is unreachable, tested above
            Opcode::BNTST, Opcode::BAND,  Opcode::BNAND,
            Opcode::BOR,   Opcode::BNOR,  Opcode::BCLR,
            Opcode::BSET,  Opcode::BNOT,  Opcode::BTST,
            Opcode::BXOR,  Opcode::BNXOR,
        ][op as usize];
        inst.operands[0] = Operand_BitRegDerefDispAbs(code, inst, bytes)?;
        inst.operands[1] = OperandSpec::Nothing;
    }

    Ok(())
}

fn decode_opEB<T: Iterator<Item=u8>>(inst: &mut Instruction, _size: Size, bytes: &mut T) -> Result<(), DecodeError> {
    let byte = bytes.next().ok_or(DecodeError::ExhaustedInput)?;
    inst.length += 1;
    let upper = byte >> 4;
    let lower = byte & 0b1111;
    if upper < 0b1000 {
        // SHL, SHA, LDC, POPC, MOVA, PUSHC, FSET, or FCLR
        match lower {
            0b0000 => {
                // LDC
                inst.opcode = Opcode::LDC;
                inst.operands[0] = OperandSpec::Imm16;
                inst.operands[1] = Operand_IntFlgSpSbFb((byte >> 4) & 0b111)?;
                inst.imm_wide = read_imm(bytes, 2)?;
                inst.length += 2;
            }
            0b0001 => {
                // SHL/SHA
                inst.opcode = if upper & 2 == 0 { Opcode::SHL(Size::L) } else { Opcode::SHA(Size::L) };
                inst.operands[0] = OperandSpec::R1H;
                inst.operands[1] = if upper & 1 == 0 { OperandSpec::R2R0 } else { OperandSpec::R3R1 };
            }
            0b0010 => {
                // PUSHC
                inst.opcode = Opcode::PUSHC;
                inst.operands[0] = Operand_IntFlgSpSbFb((byte >> 4) & 0b111)?;
                inst.operands[1] = OperandSpec::Nothing;
            },
            0b0011 => {
                // POPC
                inst.opcode = Opcode::POPC;
                inst.operands[0] = Operand_IntFlgSpSbFb((byte >> 4) & 0b111)?;
                inst.operands[1] = OperandSpec::Nothing;
            }
            0b0100 => {
                // FSET
            }
            0b0101 => {
                // FCLR
            }
            _ => {
                // MOVA
                inst.opcode = Opcode::MOVA;
                let src = byte & 0b1111;
                if src < 0b1000 {
                    return Err(DecodeError::InvalidOperand);
                }
                inst.operands[0] = Operand_RegDerefDispAbs(src, Size::W, inst, bytes)?;
                let dest = byte >> 4;
                if dest >= 0b1000 {
                    return Err(DecodeError::InvalidOperand);
                }
                inst.operands[1] = Operand_RegDerefDispAbs(dest, Size::W, inst, bytes)?;
            }
        }
    } else {
        // SHL Imm4, SHA Imm4, or INT
        if upper >= 0b1100 {
            inst.opcode = Opcode::INT;
            inst.operands[0] = OperandSpec::Imm8;
            inst.operands[1] = OperandSpec::Nothing;
            inst.imm_wide = (byte & 0b0011_1111) as u32;
        } else {
            inst.opcode = if upper & 2 == 0 { Opcode::SHL(Size::L) } else { Opcode::SHA(Size::L) };
            inst.operands[0] = OperandSpec::Imm4;
            inst.imm_wide = lower as u32;
            inst.operands[1] = if upper & 1 == 0 { OperandSpec::R2R0 } else { OperandSpec::R3R1 };
        }
    }

    Ok(())
}

fn read_imm<T: Iterator<Item=u8>>(bytes: &mut T, mut size: u8) -> Result<u32, DecodeError> {
    let mut imm: u32 = 0;
    let mut offset = 0;

    while size > 0 {
        imm |= (bytes.next().ok_or(DecodeError::ExhaustedInput)? as u32) << (8 * offset);
        offset += 1;
        size -= 1;
    }

    Ok(imm)
}


fn Operand_BitRegDerefDispAbs<T: Iterator<Item=u8>>(code: u8, inst: &mut Instruction, bytes: &mut T) -> Result<OperandSpec, DecodeError> {
    use OperandSpec::*;
    // "bit,<reg>" addressing modes taking their bit selector from a following byte is left mostly
    // unspoken in the manual, aside from the "bit,Rn" and "bit,An" modes taking an extra byte.
    let (imm_size, spec) = match code {
        0b0000 => (1, Bit_R0),
        0b0001 => (1, Bit_R1),
        0b0010 => (1, Bit_R2),
        0b0011 => (1, Bit_R3),
        0b0100 => (1, Bit_A0),
        0b0101 => (1, Bit_A1),
        0b0110 => (0, Bit_Deref_A0),
        0b0111 => (0, Bit_Deref_A1),
        0b1000 => (1, Bit_Disp8_A0),
        0b1001 => (1, Bit_Disp8_A1),
        0b1010 => (1, Bit_Disp8_SB),
        0b1011 => (1, Bit_Disp8_FB),
        0b1100 => (2, Bit_Disp16_A0),
        0b1101 => (2, Bit_Disp16_A1),
        0b1110 => (2, Bit_Disp16_SB),
        0b1111 => (2, Bit_Abs16),
        _ => { unreachable!("invalid code provided, >0b1111"); }
    };

    if imm_size > 0 {
        inst.dispabs = bytes.next().ok_or(DecodeError::ExhaustedInput)? as u16;
        inst.length += 1;

        if imm_size == 2 {
            inst.dispabs |= (bytes.next().ok_or(DecodeError::ExhaustedInput)? as u16) << 8;
            inst.length += 1;
        }
    }

    Ok(spec)

}

fn Operand_RegDerefDispAbs<T: Iterator<Item=u8>>(code: u8, size: Size, inst: &mut Instruction, bytes: &mut T) -> Result<OperandSpec, DecodeError> {
    use OperandSpec::*;
    let (imm_size, spec) = match code {
        0b0000 => (0, { if size == Size::B { R0L } else { R0 } }),
        0b0001 => (0, { if size == Size::B { R0H } else { R1 } }),
        0b0010 => (0, { if size == Size::B { R1L } else { R2 } }),
        0b0011 => (0, { if size == Size::B { R1H } else { R3 } }),
        0b0100 => (0, A0),
        0b0101 => (0, A1),
        0b0110 => (0, Deref_A0),
        0b0111 => (0, Deref_A1),
        0b1000 => (1, Disp8_A0),
        0b1001 => (1, Disp8_A1),
        0b1010 => (1, Disp8_SB),
        0b1011 => (1, Disp8_FB),
        0b1100 => (2, Disp16_A0),
        0b1101 => (2, Disp16_A1),
        0b1110 => (2, Disp16_SB),
        0b1111 => (2, Abs16),
        _ => { unreachable!("invalid code provided, >0b1111"); }
    };

    if imm_size > 0 {
        inst.dispabs = bytes.next().ok_or(DecodeError::ExhaustedInput)? as u16;
        inst.length += 1;

        if imm_size == 2 {
            inst.dispabs |= (bytes.next().ok_or(DecodeError::ExhaustedInput)? as u16) << 8;
            inst.length += 1;
        }
    }

    Ok(spec)
}

fn Operand_second_RegDerefDispAbs<T: Iterator<Item=u8>>(code: u8, size: Size, inst: &mut Instruction, bytes: &mut T) -> Result<OperandSpec, DecodeError> {
    use OperandSpec::*;
    let (imm_size, spec) = match code {
        0b0000 => (0, { if size == Size::B { R0L } else { R0 } }),
        0b0001 => (0, { if size == Size::B { R0H } else { R1 } }),
        0b0010 => (0, { if size == Size::B { R1L } else { R2 } }),
        0b0011 => (0, { if size == Size::B { R1H } else { R3 } }),
        0b0100 => (0, A0),
        0b0101 => (0, A1),
        0b0110 => (0, Deref_A0),
        0b0111 => (0, Deref_A1),
        0b1000 => (1, Disp2_8_A0),
        0b1001 => (1, Disp2_8_A1),
        0b1010 => (1, Disp2_8_SB),
        0b1011 => (1, Disp2_8_FB),
        0b1100 => (2, Disp2_16_A0),
        0b1101 => (2, Disp2_16_A1),
        0b1110 => (2, Disp2_16_SB),
        0b1111 => (2, Abs2_16),
        _ => { unreachable!("invalid code provided, >0b1111"); }
    };

    if imm_size > 0 {
        inst.imm_wide = bytes.next().ok_or(DecodeError::ExhaustedInput)? as u32;
        inst.length += 1;

        if imm_size == 2 {
            inst.imm_wide |= (bytes.next().ok_or(DecodeError::ExhaustedInput)? as u32) << 8;
            inst.length += 1;
        }
    }

    Ok(spec)
}

fn Operand_RegDerefDisp20Abs<T: Iterator<Item=u8>>(code: u8, inst: &mut Instruction, bytes: &mut T) -> Result<OperandSpec, DecodeError> {
    use OperandSpec::*;
    let (imm_size, spec) = match code {
        0b0000 => (0, R2R0),
        0b0001 => (0, R3R1),
        0b0010 => { return Err(DecodeError::InvalidOperand); },
        0b0011 => { return Err(DecodeError::InvalidOperand); },
        0b0100 => (0, A1A0),
        0b0101 => { return Err(DecodeError::InvalidOperand); },
        0b0110 => (0, Deref_A0),
        0b0111 => (0, Deref_A1),
        0b1000 => (1, Disp8_A0),
        0b1001 => (1, Disp8_A1),
        0b1010 => (1, Disp8_SB),
        0b1011 => (1, Disp8_FB),
        0b1100 => (3, Disp20_A0),
        0b1101 => (3, Disp20_A1),
        0b1110 => (2, Disp16_SB),
        0b1111 => (2, Abs16),
        _ => { unreachable!("invalid code provided, >0b1111"); }
    };

    match imm_size {
        0 => {}
        1 => {
            inst.dispabs = bytes.next().ok_or(DecodeError::ExhaustedInput)? as u16;
            inst.length += 1;
        }
        2 => {
            inst.dispabs = bytes.next().ok_or(DecodeError::ExhaustedInput)? as u16;
            inst.dispabs |= (bytes.next().ok_or(DecodeError::ExhaustedInput)? as u16) << 8;
            inst.length += 2;
        }
        3 => {
            inst.imm_wide = bytes.next().ok_or(DecodeError::ExhaustedInput)? as u32;
            inst.imm_wide |= (bytes.next().ok_or(DecodeError::ExhaustedInput)? as u32) << 8;
            inst.imm_wide |= ((bytes.next().ok_or(DecodeError::ExhaustedInput)? & 0x0f) as u32) << 16;
            inst.length += 3;
        }
        _ => {
            unreachable!();
        }
    }

    Ok(spec)
}

fn Operand_IntFlgSpSbFb(code: u8) -> Result<OperandSpec, DecodeError> {
    use OperandSpec::*;

    match code {
        0b001 => Ok(INTBL),
        0b010 => Ok(INTBH),
        0b011 => Ok(FLG),
        0b100 => Ok(ISP),
        0b101 => Ok(SP),
        0b110 => Ok(SB),
        0b111 => Ok(FB),
        _ => Err(DecodeError::InvalidOperand)
    }
}
