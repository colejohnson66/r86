/* ============================================================================
 * File:   push.rs
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
use crate::cpu::decoder::Instr;
use crate::cpu::Cpu;

pub struct Push;

impl Push {
    fn ew(_cpu: &mut Cpu, _instr: &Instr) -> u32 {
        unimplemented!();
    }

    fn ed(_cpu: &mut Cpu, _instr: &Instr) -> u32 {
        unimplemented!();
    }

    fn eq(_cpu: &mut Cpu, _instr: &Instr) -> u32 {
        unimplemented!();
    }

    fn gw(_cpu: &mut Cpu, _instr: &Instr) -> u32 {
        unimplemented!();
    }

    fn gd(_cpu: &mut Cpu, _instr: &Instr) -> u32 {
        unimplemented!();
    }

    fn gq(_cpu: &mut Cpu, _instr: &Instr) -> u32 {
        unimplemented!();
    }

    fn ib(_cpu: &mut Cpu, _instr: &Instr) -> u32 {
        unimplemented!();
    }

    fn iw(_cpu: &mut Cpu, _instr: &Instr) -> u32 {
        unimplemented!();
    }

    fn id(_cpu: &mut Cpu, _instr: &Instr) -> u32 {
        unimplemented!();
    }

    fn cs(_cpu: &mut Cpu, _instr: &Instr) -> u32 {
        unimplemented!();
    }

    fn ss(_cpu: &mut Cpu, _instr: &Instr) -> u32 {
        unimplemented!();
    }

    fn ds(_cpu: &mut Cpu, _instr: &Instr) -> u32 {
        unimplemented!();
    }

    fn es(_cpu: &mut Cpu, _instr: &Instr) -> u32 {
        unimplemented!();
    }

    fn fd(_cpu: &mut Cpu, _instr: &Instr) -> u32 {
        unimplemented!();
    }

    fn gs(_cpu: &mut Cpu, _instr: &Instr) -> u32 {
        unimplemented!();
    }
}
