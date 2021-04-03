/* ============================================================================
 * File:   fldc.rs
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

pub struct Fld1;
pub struct Fldl2t;
pub struct Fldl2e;
pub struct Fldpi;
pub struct Fldlg2;
pub struct Fldln2;
pub struct Fldz;

impl Fld1 {
    stub_handler!(noarg);
}

impl Fldl2t {
    stub_handler!(noarg);
}

impl Fldl2e {
    stub_handler!(noarg);
}

impl Fldpi {
    stub_handler!(noarg);
}

impl Fldlg2 {
    stub_handler!(noarg);
}

impl Fldln2 {
    stub_handler!(noarg);
}

impl Fldz {
    stub_handler!(noarg);
}
