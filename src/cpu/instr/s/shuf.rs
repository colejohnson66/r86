/* ============================================================================
 * File:   shuf.rs
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

pub struct Shuff32x4;
pub struct Shuff64x2;
pub struct Shufi32x4;
pub struct Shufi64x2;

impl Shuff32x4 {
    stub_handler!(v_vqq_hqq_wqq_ib_e256);

    stub_handler!(v_vdqq_hdqq_wdqq_ib_e512);
}

impl Shuff64x2 {
    stub_handler!(v_vqq_hqq_wqq_ib_e256);

    stub_handler!(v_vdqq_hdqq_wdqq_ib_e512);
}

impl Shufi32x4 {
    stub_handler!(v_vqq_hqq_wqq_ib_e256);

    stub_handler!(v_vdqq_hdqq_wdqq_ib_e512);
}

impl Shufi64x2 {
    stub_handler!(v_vqq_hqq_wqq_ib_e256);

    stub_handler!(v_vdqq_hdqq_wdqq_ib_e512);
}
