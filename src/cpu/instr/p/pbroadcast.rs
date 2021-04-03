/* ============================================================================
 * File:   pbroadcast.rs
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

pub struct Pbroadcastb;
pub struct Pbroadcastw;
pub struct Pbroadcastd;
pub struct Pbroadcastq;
pub struct Pbroadcasti32x2;
pub struct Pbroadcasti128;
pub struct Pbroadcasti32x4;
pub struct Pbroadcasti64x2;
pub struct Pbroadcasti32x8;
pub struct Pbroadcasti64x4;

impl Pbroadcastb {
    stub_handler!(v_vdq_wb_v128);

    stub_handler!(v_vqq_wb_v256);

    stub_handler!(v_vdq_wb_e128);

    stub_handler!(v_vqq_wb_e256);

    stub_handler!(v_vdqq_wb_e512);

    stub_handler!(v_vdq_eb_e128);

    stub_handler!(v_vqq_eb_e256);

    stub_handler!(v_vdqq_eb_e512);
}

impl Pbroadcastw {
    stub_handler!(v_vdq_wb_v128);

    stub_handler!(v_vqq_wb_v256);

    stub_handler!(v_vdq_wb_e128);

    stub_handler!(v_vqq_wb_e256);

    stub_handler!(v_vdqq_wb_e512);

    stub_handler!(v_vdq_eb_e128);

    stub_handler!(v_vqq_eb_e256);

    stub_handler!(v_vdqq_eb_e512);
}

impl Pbroadcastd {
    stub_handler!(v_vdq_wb_v128);

    stub_handler!(v_vqq_wb_v256);

    stub_handler!(v_vdq_wb_e128);

    stub_handler!(v_vqq_wb_e256);

    stub_handler!(v_vdqq_wb_e512);

    stub_handler!(v_vdq_eb_e128);

    stub_handler!(v_vqq_eb_e256);

    stub_handler!(v_vdqq_eb_e512);
}

impl Pbroadcastq {
    stub_handler!(v_vdq_wb_v128);

    stub_handler!(v_vqq_wb_v256);

    stub_handler!(v_vdq_wb_e128);

    stub_handler!(v_vqq_wb_e256);

    stub_handler!(v_vdqq_wb_e512);

    stub_handler!(v_vdq_eb_e128);

    stub_handler!(v_vqq_eb_e256);

    stub_handler!(v_vdqq_eb_e512);
}

impl Pbroadcasti32x2 {
    stub_handler!(v_vdq_wq_e128);

    stub_handler!(v_vqq_wq_e256);

    stub_handler!(v_vdqq_wq_e512);
}

impl Pbroadcasti128 {
    stub_handler!(v_vqq_mdq_v256);
}

impl Pbroadcasti32x4 {
    stub_handler!(v_vqq_mdq_e256);

    stub_handler!(v_vdqq_mdq_e512);
}

impl Pbroadcasti64x2 {
    stub_handler!(v_vqq_mdq_e256);

    stub_handler!(v_vdqq_mdq_e512);
}

impl Pbroadcasti32x8 {
    stub_handler!(v_vdqq_mdq_e512);
}

impl Pbroadcasti64x4 {
    stub_handler!(v_vdqq_mdq_e512);
}
