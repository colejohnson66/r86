/* ============================================================================
 * File:   pinsr.rs
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
use crate::cpu::decoder::Instr;
use crate::cpu::Cpu;
use crate::stub_handler;

pub struct Pinsrb;
pub struct Pinsrw;
pub struct Pinsrd;
pub struct Pinsrq;

impl Pinsrb {
    stub_handler!(vdq_eb_ib);

    stub_handler!(v_vdq_hdq_eb_ib_v128);

    stub_handler!(v_vdq_hdq_eb_ib_e128);
}

impl Pinsrw {
    stub_handler!(pq_ew_ib);

    stub_handler!(vdq_ew_ib);

    stub_handler!(v_vdq_hdq_ew_ib_v128);

    stub_handler!(v_vdq_hdq_ew_ib_e128);
}

impl Pinsrd {
    stub_handler!(vdq_ed_ib);

    stub_handler!(v_vdq_hdq_ed_ib_v128);

    stub_handler!(v_vdq_hdq_ed_ib_e128);
}

impl Pinsrq {
    stub_handler!(vdq_eq_ib);

    stub_handler!(v_vdq_hdq_eq_ib_v128);

    stub_handler!(v_vdq_hdq_eq_ib_e128);
}
