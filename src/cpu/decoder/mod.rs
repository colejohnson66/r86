/* ============================================================================
 * File:   mod.rs
 * Author: Cole Johnson
 * ============================================================================
 * Copyright (c) 2020-2021 Cole Johnson
 *
 * This file is part of r86.
 *
 * r86 is free software: you can redistribute it and/or modify it under the
 *   terms of the GNU General Public License as published by the Free Software
 *   Foundation, either version 3 of the License, or (at your option) any later
 *   version.
 *
 * r86 is distributed in the hope that it will be useful, but WITHOUT ANY
 *   WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 *   FOR A PARTICULAR PURPOSE. See the GNU General Public License for more
 *   details.
 *
 * You should have received a copy of the GNU General Public License along with
 *   r86. If not, see <http://www.gnu.org/licenses/>.
 * ============================================================================
 */
pub mod flag;
pub mod opcode;
mod opcode_name;
pub mod opmap;

pub use crate::cpu::decoder::opcode_name::OpcodeName;
use crate::cpu::Cpu;
use std::str;

/// A pointer to an instruction handler.
/// It takes a reference to the CPU being executed on and the decoded instruction
type InstrHandler = fn(&mut Cpu, &Instr) -> u32;

/// A decoded instruction.
pub struct Instr {
    handler: InstrHandler,
    raw_instr: Vec<u8>,
}

pub struct Opcode {
    pub opcode: OpcodeName,
    pub mnemonic_intel: &'static str,
    pub mnemonic_att: &'static str,
    pub handler: InstrHandler,
    pub arguments: [OperandSource; 4],
    pub suffix: OpSuffix,
    pub lockable: bool,
    pub extensions: Vec<IsaExtension>,
}

impl Opcode {
    pub fn new(
        opcode: OpcodeName,
        mnemonic_intel: &'static str,
        mnemonic_att: &'static str,
        handler: InstrHandler,
        arguments: [OperandSource; 4],
        suffix: OpSuffix,
        lockable: bool,
        extensions: Vec<IsaExtension>,
    ) -> Opcode {
        Opcode {
            opcode,
            mnemonic_intel,
            mnemonic_att,
            handler,
            arguments,
            suffix,
            lockable,
            extensions,
        }
    }
}

pub struct OpcodeMapEntry {
    pub handler: InstrHandler,
    pub attr: Vec<&'static dyn flag::Flag>,
}

impl OpcodeMapEntry {
    pub fn new(handler: InstrHandler, attr: Vec<&'static dyn flag::Flag>) -> OpcodeMapEntry {
        OpcodeMapEntry { handler, attr }
    }
}

#[derive(PartialEq)]
pub enum OpSource {
    None,
    A, // direct
    B, // VEX.vvvv is GPR
    C, // reg is control register
    D, // reg is debug register
    E, // ModR/M with possible SIB for GPR
    F, // EFLAGS/RFLAGS
    G, // reg is GPR
    H, // VEX.vvvv is XMM/YMM/ZMM
    I, // immediate
    J, // offset relative to IP/EIP/RIP
    L, // high nibble of 8-bit immediate is XMM/YMM/ZMM
    M, // ModR/M for memory
    N, // r/m is MMX
    P, // reg is MMX
    Q, // ModR/M with possible SIB for MMX
    R, // r/m is GPR
    S, // reg is segment register
    U, // r/m is XMM/YMM
    V, // reg is XMM/YMM
    W, // ModR/M with possible SIB for XMM/YMM/ZMM
    X, // DS:rSI
    Y, // ES:rDI

    // custom addressing modes
    Be, // `E` but for BNDx registers
    Bg, // `G` but for BNDx registers
    Ke, // `E` but for mask registers
    Kg, // `G` but for mask registers
    Kr, // `R` but for mask registers
    O,  // offset (eg. Ob: moffs8)
    Vm, // vm##x

    // extra addressing modes
    EAX, // AL, AX, EAX, RAX (`OpSize` determines which to use)
}

// pd, ps, sd, and ss are not used;
// dq, qq, and dqq are used instead
#[derive(PartialEq)]
pub enum OpSize {
    None, // must only every exist with `OpSource::None`
    A,    // two one-word or two double-word
    B,    // byte
    C,    // byte or word
    D,    // double-word (32-bit)
    Dq,   // double-quad-word (128-bit)
    Dqq,  // double-quad-quad-word (512-bit)
    P,    // 32-, 48-, or 80-bit pointer
    Q,    // quad-word (64-bit)
    Qq,   // quad-quad-word (256-bit)
    S,    // 6- or 10-byte pseudo-descriptor
    T,    // 10-byte (80-bit)
    V,    // word, double-word, or quad-word
    W,    // word
    X,    // double-quad-word or quad-quad-word
    Y,    // double-word or quad-word
    Z,    // word or double-word
    Unknown,
}

#[derive(PartialEq)]
pub enum OpSuffix {
    None,
    Op16,
    Op32,
    Op64,
    V, // Vex LIG
    V128,
    V256,
    E, // EVEX LIG
    E128,
    E256,
    E512,
}

pub struct OperandSource {
    pub src: OpSource,
    pub size: OpSize,
}

impl OperandSource {
    // take a string of the form:
    //   {src}{size?}
    // and return an `OperandSource` with correct values
    //
    // Examples:
    //   "AL" ---> { src: EAX, size: B }
    //   "Vdq" --> { src: V,   size: Dq }
    //   "M" ----> { src: M,   size: Unknown }
    //   "BGdq" -> { src: BG,  size: Dq }
    pub fn from_encoded_str(enc_str: &'static str) -> OperandSource {
        // check for noarg
        if enc_str.len() == 0 {
            return OperandSource {
                src: OpSource::None,
                size: OpSize::None,
            };
        }

        // check for EAX like register
        let possible_eax = match enc_str {
            "AL" => Some(OpSize::B),
            "AX" => Some(OpSize::W),
            "EAX" => Some(OpSize::D),
            "RAX" => Some(OpSize::Q),
            _ => None,
        };
        if possible_eax.is_some() {
            return OperandSource {
                src: OpSource::EAX,
                size: possible_eax.unwrap(),
            };
        }

        // TODO: is this ok? Everything should be ASCII
        let chars = enc_str.as_bytes();
        assert!(chars.len() >= 1);

        // read until unknown character
        let src = match chars[0] as char {
            'A' => OpSource::A,
            'B' if chars.len() == 1 => OpSource::B,
            'B' => {
                // check for known second character
                match chars[1] as char {
                    'E' => OpSource::Be,
                    'G' => OpSource::Bg,
                    _ => OpSource::B,
                }
            }
            'C' => OpSource::C,
            'D' => OpSource::D,
            'E' => OpSource::E,
            'F' => OpSource::F,
            'G' => OpSource::G,
            'H' => OpSource::H,
            'I' => OpSource::I,
            'J' => OpSource::J,
            'K' if chars.len() == 1 => unreachable!(), // No plain 'K' addressing method
            'K' => {
                // check for known second character
                match chars[1] as char {
                    'E' => OpSource::Ke,
                    'G' => OpSource::Kg,
                    'R' => OpSource::Kr,
                    _ => unreachable!(),
                }
            }
            'L' => OpSource::L,
            'M' => OpSource::M,
            'N' => OpSource::N,
            'O' => OpSource::O,
            'P' => OpSource::P,
            'Q' => OpSource::Q,
            'R' => OpSource::R,
            'S' => OpSource::S,
            'U' => OpSource::U,
            'V' if chars.len() == 1 => OpSource::V,
            'V' => {
                // check for known second character
                match chars[1] as char {
                    'M' => OpSource::Vm,
                    _ => OpSource::V,
                }
            }
            'W' => OpSource::W,
            'X' => OpSource::X,
            'Y' => OpSource::Y,
            _ => unreachable!(),
        };
        let read_count = match src {
            OpSource::Be | OpSource::Bg => 2usize,
            OpSource::Ke | OpSource::Kg | OpSource::Kr => 2usize,
            OpSource::Vm => 2usize,
            _ => 1usize,
        };

        // all except `M` MUST have a size; `M` may or may not
        if chars.len() == read_count {
            if src == OpSource::M {
                return OperandSource {
                    src,
                    size: OpSize::Unknown,
                };
            }
            panic!();
        }

        // get the size
        let rest = str::from_utf8(&chars[read_count..]).unwrap();
        let size = match rest {
            "a" => OpSize::A,
            "b" => OpSize::B,
            "c" => OpSize::C,
            "d" => OpSize::D,
            "dq" => OpSize::Dq,
            "dqq" => OpSize::Dqq,
            "p" => OpSize::P,
            "q" => OpSize::Q,
            "qq" => OpSize::Qq,
            "s" => OpSize::S,
            "t" => OpSize::T,
            "v" => OpSize::V,
            "w" => OpSize::W,
            "x" => OpSize::X,
            "y" => OpSize::Y,
            "z" => OpSize::Z,
            _ => unreachable!(), // unknown operand type (size)
        };

        OperandSource { src, size }
    }
}

pub enum IsaExtension {
    I386,
    X87,
    I486,
    Pentium,
    P6,
    Mmx,
    _3dNow,
    Debug,
    Vme,
    Pse,
    Pae,
    Pge,
    Pse36,
    Mtrr,
    Pat,
    SyscallSysretLegacy,
    SysenterSysenter,
    Clflush,
    Clflushopt,
    Clwb,
    Cldemote,
    Sse,
    Sse2,
    Sse3,
    Ssse3,
    Sse41,
    Sse42,
    Popcnt,
    MonitorMwait,
    MonitorxMwaitx,
    Waitpkg,
    Vmx,
    Smx,
    LongMode,
    LmLahfSahf,
    Nx,
    OneGigPages,
    Cmgxchg16b,
    Rdtscp,
    Ffxsr,
    Xsave,
    Xsaveopt,
    Xsavec,
    Xsaves,
    AesPclmulqdq,
    VaesVpclmulqdq,
    Movbe,
    FsGsBase,
    Invpcid,
    Avx,
    Avx2,
    AvxF16c,
    AvxFma,
    AltMoveC8,
    Sse4a,
    MisalignedSse,
    Lzcnt,
    Bmi1,
    Bmi2,
    Fma4,
    Xop,
    Tbm,
    Svm,
    Rdrand,
    Adx,
    Smap,
    Rdseed,
    Sha,
    Gfni,
    Avx512,
    Avx512Cd,
    Avx512Pf,
    Avx512Er,
    Avx512Dq,
    Avx512Bw,
    Avx512Vl,
    Avx512Vbmi,
    Avx512Vbli2,
    Avx512Ifma52,
    Avx512Vpopcntdq,
    Avx512Vnni,
    Avx512Bitalg,
    Avx512Vp2intersect,
    Xapic,
    X2apic,
    XapicExt,
    Pcid,
    Smep,
    TscDeadline,
    FopcodeDeprecation,
    FcsFdsDeprecation,
    FdpDeprecation,
    Pku,
    Umip,
    Rdpid,
    Tce,
    Clzero,
    ScaMitigations,
    Cet,
}
