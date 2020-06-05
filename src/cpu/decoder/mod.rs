/* ============================================================================
 * File:   mod.rs
 * Author: Cole Johnson
 * ============================================================================
 * Copyright (c) 2020 Cole Johnson
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
pub mod instr;
mod opcode_name;

pub use crate::cpu::decoder::opcode_name::OpcodeName;
use crate::cpu::Cpu;

/// A pointer to an instruction handler.
/// It takes a reference to the CPU being executed on and the decoded instruction
type InstrHandler = fn(&mut Cpu, &Instr) -> u32;

/// A decoded instruction.
pub struct Instr {
    handler: InstrHandler,
    opcode: u16,
    raw_instr: Vec<u8>,
}

pub struct Opcode {
    pub opcode: OpcodeName,
    pub opcode_str: &'static str,
    pub extensions: [IsaExtension; 4],
    pub operand_sources: [u8; 4],
    pub special_attr: u32,
}

pub struct OpmapEntry {
    pub handler: InstrHandler,
    pub attr: u64,
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
