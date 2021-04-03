/* ============================================================================
 * File:   imul.rs
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

pub struct Imul;

impl Imul {
    stub_handler!(eb);

    stub_handler!(ew);

    stub_handler!(ed);

    stub_handler!(eq);

    stub_handler!(gb_eb);

    stub_handler!(gw_ew);

    stub_handler!(gq_eq);

    stub_handler!(gw_ew_ib);

    stub_handler!(gd_ed_ib);

    stub_handler!(gq_eq_ib);

    stub_handler!(gw_ew_iw);

    stub_handler!(gd_ed_id);

    stub_handler!(gq_eq_id);
}
