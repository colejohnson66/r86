/* ============================================================================
 * File:   movdqu.rs
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

pub struct Movdqu;
pub struct Movdqu8;
pub struct Movdqu16;
pub struct Movdqu32;
pub struct Movdqu64;

impl Movdqu {
    stub_handler!(vdq_wdq);

    stub_handler!(wdq_vdq);

    stub_handler!(v_vdq_wdq_v128);

    stub_handler!(v_wdq_vdq_v128);

    stub_handler!(v_vqq_wqq_v256);

    stub_handler!(v_wqq_vqq_v256);
}

impl Movdqu8 {
    stub_handler!(v_vdq_wdq_e128);

    stub_handler!(v_wdq_vdq_e128);

    stub_handler!(v_vqq_wqq_e256);

    stub_handler!(v_wqq_vqq_e256);

    stub_handler!(v_wdqq_vdqq_e512);

    stub_handler!(v_vdqq_wdqq_e512);
}

impl Movdqu16 {
    stub_handler!(v_vdq_wdq_e128);

    stub_handler!(v_wdq_vdq_e128);

    stub_handler!(v_vqq_wqq_e256);

    stub_handler!(v_wqq_vqq_e256);

    stub_handler!(v_wdqq_vdqq_e512);

    stub_handler!(v_vdqq_wdqq_e512);
}

impl Movdqu32 {
    stub_handler!(v_vdq_wdq_e128);

    stub_handler!(v_wdq_vdq_e128);

    stub_handler!(v_vqq_wqq_e256);

    stub_handler!(v_wqq_vqq_e256);

    stub_handler!(v_wdqq_vdqq_e512);

    stub_handler!(v_vdqq_wdqq_e512);
}

impl Movdqu64 {
    stub_handler!(v_vdq_wdq_e128);

    stub_handler!(v_wdq_vdq_e128);

    stub_handler!(v_vqq_wqq_e256);

    stub_handler!(v_wqq_vqq_e256);

    stub_handler!(v_wdqq_vdqq_e512);

    stub_handler!(v_vdqq_wdqq_e512);
}
