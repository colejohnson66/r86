/* ============================================================================
 * File:   fcmovcc.rs
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

pub struct Fcmovb;
pub struct Fcmove;
pub struct Fcmovbe;
pub struct Fcmovu;
pub struct Fcmovnb;
pub struct Fcmovne;
pub struct Fcmovnbe;
pub struct Fcmovnu;

impl Fcmovb {
    stub_handler!(st0_sti);
}

impl Fcmove {
    stub_handler!(st0_sti);
}

impl Fcmovbe {
    stub_handler!(st0_sti);
}

impl Fcmovu {
    stub_handler!(st0_sti);
}

impl Fcmovnb {
    stub_handler!(st0_sti);
}

impl Fcmovne {
    stub_handler!(st0_sti);
}

impl Fcmovnbe {
    stub_handler!(st0_sti);
}

impl Fcmovnu {
    stub_handler!(st0_sti);
}
