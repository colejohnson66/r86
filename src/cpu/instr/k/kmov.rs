/* ============================================================================
 * File:   kmov.rs
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

pub struct Kmovb;
pub struct Kmovw;
pub struct Kmovd;
pub struct Kmovq;

impl Kmovb {
    stub_handler!(kgb_keb);

    stub_handler!(keb_kgb);

    stub_handler!(keb_gd);

    stub_handler!(gd_keb);
}

impl Kmovw {
    stub_handler!(kgw_kew);

    stub_handler!(kew_kgw);

    stub_handler!(kew_gd);

    stub_handler!(gd_kew);
}

impl Kmovd {
    stub_handler!(kgd_ked);

    stub_handler!(ked_kgd);

    stub_handler!(ked_gd);

    stub_handler!(gd_ked);
}

impl Kmovq {
    stub_handler!(kgq_keq);

    stub_handler!(keq_kgq);

    stub_handler!(keq_gq);

    stub_handler!(gq_keq);
}
