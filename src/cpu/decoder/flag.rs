/* ============================================================================
 * File:   flag.rs
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
pub enum FlagType {
    Nnn,
    Rrr,
    SrcEqualsDest,
    Mask,
    NoVexEvexXop,
    VexW,
    VecLength,
    Xop,
    Vex,
    Evex,
    ModC0,
    Lock,
    NoSsePrefixF2F3,
    SsePrefix,
    InstrSet64,
    InstrSet32,
    Addr64,
    Addr32,
    Addr16,
    Operand64,
    Operand32,
    Operand16,
}

pub trait Flag {
    fn flag_type(&self) -> FlagType;
    fn value(&self) -> u8;
}

macro_rules! flag {
    ($name:ident,$type:expr) => {
        pub struct $name {
            val: u8,
        }

        impl $name {
            pub fn new(val: u8) -> $name {
                $name { val }
            }
        }

        impl Flag for $name {
            fn flag_type(&self) -> FlagType {
                $type
            }

            fn value(&self) -> u8 {
                self.val
            }
        }
    };
}

flag!(Nnn, FlagType::Nnn);
flag!(Rrr, FlagType::Rrr);
flag!(SrcEqualsDest, FlagType::SrcEqualsDest);
flag!(Mask, FlagType::Mask); // 0: MaskRequired, 1: MaskK0
flag!(NoVexEvexXop, FlagType::NoVexEvexXop);
flag!(VexW, FlagType::VexW); // 0: W0, 1: W1
flag!(VecLength, FlagType::VecLength); // bit 0: 128, bit 1: 256, bit 2: 512
flag!(Xop, FlagType::Xop);
flag!(Vex, FlagType::Vex);
flag!(Evex, FlagType::Evex);
flag!(ModC0, FlagType::ModC0); // 0: reg, 1: mem
flag!(Lock, FlagType::Lock);
flag!(NoSsePrefixF2F3, FlagType::NoSsePrefixF2F3);
flag!(SsePrefix, FlagType::SsePrefix); // 0: none, 1: 66, 2: F3, 3: F2
flag!(InstrSet64, FlagType::InstrSet64);
flag!(InstrSet32, FlagType::InstrSet32);
flag!(Addr64, FlagType::Addr64);
flag!(Addr32, FlagType::Addr32);
flag!(Addr16, FlagType::Addr16);
flag!(Operand64, FlagType::Operand64);
flag!(Operand32, FlagType::Operand32);
flag!(Operand16, FlagType::Operand16);
