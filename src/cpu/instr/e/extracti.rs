/* ============================================================================
 * File:   extracti.rs
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

pub struct Extracti128;
pub struct Extracti32x4;
pub struct Extracti64x2;
pub struct Extracti32x8;
pub struct Extracti64x4;

impl Extracti128 {
    stub_handler!(v_wdq_vqq_ib_e256);
}

impl Extracti32x4 {
    stub_handler!(v_wdq_vqq_ib_e256);

    stub_handler!(v_wdq_vdqq_ib_e512);
}

impl Extracti64x2 {
    stub_handler!(v_wdq_vqq_ib_e256);

    stub_handler!(v_wdq_vdqq_ib_e512);
}

impl Extracti32x8 {
    stub_handler!(v_wqq_vdqq_ib_e512);
}

impl Extracti64x4 {
    stub_handler!(v_wqq_vdqq_ib_e512);
}
