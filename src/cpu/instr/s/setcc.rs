/* ============================================================================
 * File:   setcc.rs
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

pub struct Seto;
pub struct Setno;
pub struct Setb;
pub struct Setae;
pub struct Sete;
pub struct Setne;
pub struct Setbe;
pub struct Seta;
pub struct Sets;
pub struct Setns;
pub struct Setp;
pub struct Setnp;
pub struct Setl;
pub struct Setge;
pub struct Setle;
pub struct Setg;

impl Seto {
    stub_handler!(eb);
}

impl Setno {
    stub_handler!(eb);
}

impl Setb {
    stub_handler!(eb);
}

impl Setae {
    stub_handler!(eb);
}

impl Sete {
    stub_handler!(eb);
}

impl Setne {
    stub_handler!(eb);
}

impl Setbe {
    stub_handler!(eb);
}

impl Seta {
    stub_handler!(eb);
}

impl Sets {
    stub_handler!(eb);
}

impl Setns {
    stub_handler!(eb);
}

impl Setp {
    stub_handler!(eb);
}

impl Setnp {
    stub_handler!(eb);
}

impl Setl {
    stub_handler!(eb);
}

impl Setge {
    stub_handler!(eb);
}

impl Setle {
    stub_handler!(eb);
}

impl Setg {
    stub_handler!(eb);
}
