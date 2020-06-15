/* ============================================================================
 * File:   pinsr.rs
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

pub struct Pinsrb;
pub struct Pinsrw;
pub struct Pinsrd;
pub struct Pinsrq;

impl Pinsrb {
    fn vdq_eb_ib(_cpu: &mut Cpu, _instr: &Instr) -> u32 {
        unimplemented!();
    }

    fn v_vdq_hdq_eb_ib_v128(_cpu: &mut Cpu, _instr: &Instr) -> u32 {
        unimplemented!();
    }

    fn v_vdq_hdq_eb_ib_e128(_cpu: &mut Cpu, _instr: &Instr) -> u32 {
        unimplemented!();
    }
}

impl Pinsrw {
    fn pq_ew_ib(_cpu: &mut Cpu, _instr: &Instr) -> u32 {
        unimplemented!();
    }

    fn vdq_ew_ib(_cpu: &mut Cpu, _instr: &Instr) -> u32 {
        unimplemented!();
    }

    fn v_vdq_hdq_ew_ib_v128(_cpu: &mut Cpu, _instr: &Instr) -> u32 {
        unimplemented!();
    }

    fn v_vdq_hdq_ew_ib_e128(_cpu: &mut Cpu, _instr: &Instr) -> u32 {
        unimplemented!();
    }
}

impl Pinsrd {
    fn vdq_ed_ib(_cpu: &mut Cpu, _instr: &Instr) -> u32 {
        unimplemented!();
    }

    fn v_vdq_hdq_ed_ib_v128(_cpu: &mut Cpu, _instr: &Instr) -> u32 {
        unimplemented!();
    }

    fn v_vdq_hdq_ed_ib_e128(_cpu: &mut Cpu, _instr: &Instr) -> u32 {
        unimplemented!();
    }
}

impl Pinsrq {
    fn vdq_eq_ib(_cpu: &mut Cpu, _instr: &Instr) -> u32 {
        unimplemented!();
    }

    fn v_vdq_hdq_eq_ib_v128(_cpu: &mut Cpu, _instr: &Instr) -> u32 {
        unimplemented!();
    }

    fn v_vdq_hdq_eq_ib_e128(_cpu: &mut Cpu, _instr: &Instr) -> u32 {
        unimplemented!();
    }
}
