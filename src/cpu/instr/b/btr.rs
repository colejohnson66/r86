/* ============================================================================
 * File:   btr.rs
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

pub struct Btr;

impl Btr {
    pub fn ew_gw(_cpu: &mut Cpu, _instr: &Instr) -> u32 {
        unimplemented!();
    }

    pub fn ed_gd(_cpu: &mut Cpu, _instr: &Instr) -> u32 {
        unimplemented!();
    }

    pub fn eq_gq(_cpu: &mut Cpu, _instr: &Instr) -> u32 {
        unimplemented!();
    }

    pub fn ew_ib(_cpu: &mut Cpu, _instr: &Instr) -> u32 {
        unimplemented!();
    }

    pub fn ed_ib(_cpu: &mut Cpu, _instr: &Instr) -> u32 {
        unimplemented!();
    }

    pub fn eq_ib(_cpu: &mut Cpu, _instr: &Instr) -> u32 {
        unimplemented!();
    }
}