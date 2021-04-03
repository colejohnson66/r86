/* ============================================================================
 * File:   cmovcc.rs
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

pub struct Cmovo;
pub struct Cmovno;
pub struct Cmovb;
pub struct Cmovae;
pub struct Cmove;
pub struct Cmovne;
pub struct Cmovbe;
pub struct Cmova;
pub struct Cmovs;
pub struct Cmovns;
pub struct Cmovp;
pub struct Cmovnp;
pub struct Cmovl;
pub struct Cmovge;
pub struct Cmovle;
pub struct Cmovg;

impl Cmovo {
    stub_handler!(gw_ew);

    stub_handler!(gd_ed);

    stub_handler!(gq_eq);
}

impl Cmovno {
    stub_handler!(gw_ew);

    stub_handler!(gd_ed);

    stub_handler!(gq_eq);
}

impl Cmovb {
    stub_handler!(gw_ew);

    stub_handler!(gd_ed);

    stub_handler!(gq_eq);
}

impl Cmovae {
    stub_handler!(gw_ew);

    stub_handler!(gd_ed);

    stub_handler!(gq_eq);
}

impl Cmove {
    stub_handler!(gw_ew);

    stub_handler!(gd_ed);

    stub_handler!(gq_eq);
}

impl Cmovne {
    stub_handler!(gw_ew);

    stub_handler!(gd_ed);

    stub_handler!(gq_eq);
}

impl Cmovbe {
    stub_handler!(gw_ew);

    stub_handler!(gd_ed);

    stub_handler!(gq_eq);
}

impl Cmova {
    stub_handler!(gw_ew);

    stub_handler!(gd_ed);

    stub_handler!(gq_eq);
}

impl Cmovs {
    stub_handler!(gw_ew);

    stub_handler!(gd_ed);

    stub_handler!(gq_eq);
}

impl Cmovns {
    stub_handler!(gw_ew);

    stub_handler!(gd_ed);

    stub_handler!(gq_eq);
}

impl Cmovp {
    stub_handler!(gw_ew);

    stub_handler!(gd_ed);

    stub_handler!(gq_eq);
}

impl Cmovnp {
    stub_handler!(gw_ew);

    stub_handler!(gd_ed);

    stub_handler!(gq_eq);
}

impl Cmovl {
    stub_handler!(gw_ew);

    stub_handler!(gd_ed);

    stub_handler!(gq_eq);
}

impl Cmovge {
    stub_handler!(gw_ew);

    stub_handler!(gd_ed);

    stub_handler!(gq_eq);
}

impl Cmovle {
    stub_handler!(gw_ew);

    stub_handler!(gd_ed);

    stub_handler!(gq_eq);
}

impl Cmovg {
    stub_handler!(gw_ew);

    stub_handler!(gd_ed);

    stub_handler!(gq_eq);
}
