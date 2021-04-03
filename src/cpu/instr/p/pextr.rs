/* ============================================================================
 * File:   pextr.rs
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

pub struct Pextrb;
pub struct Pextrw;
pub struct Pextrd;
pub struct Pextrq;

impl Pextrb {
    stub_handler!(ed_vdq_ib);

    stub_handler!(v_eb_vdq_ib_v128);

    stub_handler!(v_eb_vdq_ib_e128);
}

impl Pextrw {
    stub_handler!(gw_nq_ib);

    stub_handler!(gw_udq_ib);

    stub_handler!(ew_vdq_ib);

    stub_handler!(v_gw_udq_ib_v128);

    stub_handler!(v_ew_vdq_ib_v128);

    stub_handler!(v_gw_udq_ib_e128);

    stub_handler!(v_ew_vdq_ib_e128);
}

impl Pextrd {
    stub_handler!(ed_vdq_ib);

    stub_handler!(v_eb_vdq_ib_v128);

    stub_handler!(v_eb_vdq_ib_e128);
}

impl Pextrq {
    stub_handler!(ed_vdq_ib);

    stub_handler!(v_eb_vdq_ib_v128);

    stub_handler!(v_eb_vdq_ib_e128);
}
